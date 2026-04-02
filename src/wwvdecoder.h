#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <vector>

/**
 * WwvTime — decoded UTC time from one complete WWV frame.
 *
 * WWV signal format (NIST SP 432):
 *   Every second begins with a 5 ms 1 kHz reference tick (on-time point).
 *   Following each tick, a 100 Hz subcarrier encodes one BCD bit by duration:
 *     200 ms → BIT_ZERO
 *     500 ms → BIT_ONE
 *     800 ms → BIT_MARKER  (position markers at seconds 0,9,19,29,39,49)
 *   60 bits per frame (one per second, one frame per minute).
 */
struct WwvTime {
    int  minute         = 0;
    int  hour           = 0;
    int  dayOfYear      = 0;   // 1–366
    int  year2digit     = 0;   // last 2 digits, e.g. 26 for 2026
    double ut1Seconds   = 0.0; // UT1-UTC correction in seconds (±0.9 s)
    int  dstCode        = 0;   // 0=standard, 2=begins today, 3=in effect, 1=ends today
    bool leapSecondWarning = false;
    bool valid          = false;
    int  confidence     = 0;   // consecutive valid frames decoded
};

/**
 * WwvDecoder — real-time WWV time code decoder.
 *
 * Feed mono float audio samples (normalised –1..+1).  Uses dual Goertzel
 * filters: 1 kHz to detect the 5 ms reference tick at the start of each
 * second, and 100 Hz to measure the BCD data bit duration that follows.
 *
 * Thread safety: pushSamples() is typically called from a PortAudio
 * callback; currentUtc() may be called from any thread (no internal mutex —
 * the caller must serialise if needed).
 */
class WwvDecoder {
public:
    explicit WwvDecoder(float sampleRate);

    // Push audio samples; call from audio callback.
    void pushSamples(const float* samples, int count);

    // Estimate of current UTC as a UNIX timestamp (seconds since epoch).
    // Returns 0 if not yet synced.
    time_t currentUtc() const;

    // Most recently decoded frame.
    WwvTime lastFrame() const;

    // Instantaneous signal level, 0–1 based on 1 kHz tick SNR.
    float signalLevel() const { return m_signalLevel; }

    // Total bits classified so far (useful for "searching" display).
    int   bitsReceived() const { return m_bitCount; }

    // Callback invoked (from audio thread) each time a new frame decodes.
    using FrameCallback = std::function<void(const WwvTime&)>;
    void setFrameCallback(FrameCallback cb) { m_frameCb = std::move(cb); }

    // Callback invoked (from audio thread) each time a bit is classified.
    // type: 0=zero, 1=one, 2=marker.  ms: measured 100 Hz pulse duration.
    using BitCallback = std::function<void(int type, int ms)>;
    void setBitCallback(BitCallback cb) { m_bitCb = std::move(cb); }

private:
    // --- Goertzel tone power at freqHz over a block of n samples ---
    static float goertzel(const float* buf, int n, float freqHz, float sampleRate);

    // Process one complete Goertzel block (m_blockSize samples).
    void processBlock(const float* block);

    // Called when a 100 Hz data pulse ends; durationBlocks = measured length.
    // tickTime = audio timestamp of the preceding 1 kHz tick (second boundary).
    void onPulse(int durationBlocks,
                 std::chrono::steady_clock::time_point tickTime);

    // Scan m_bits ring buffer for a valid 60-bit frame; decode if found.
    bool trySyncAndDecode();

    // Validate and decode a candidate frame starting at ring-buffer offset.
    bool decodeFrame(int startIdx);

    // Append one bit to the ring buffer (used by onPulse and gap-fill).
    void appendBit(int type, std::chrono::steady_clock::time_point t);

    // --- configuration ---
    float m_sampleRate;
    int   m_blockSize;          // samples per Goertzel block (~10 ms)

    // --- sample accumulator ---
    std::vector<float> m_blockBuf;
    int   m_blockBufFill = 0;

    // --- 1 kHz tick detection (second boundary) ---
    // Warmup: wait k_warmup blocks for EMA to converge from initial value 1.0.
    static constexpr int k_warmup = 200; // ~2 s at 100 blocks/s
    int   m_totalBlocks  = 0;
    float m_smoothPower  = 0.0f;   // EMA of 1 kHz Goertzel power
    float m_noisePower   = 1.0f;   // 1 kHz noise floor EMA (for SNR display)
    bool  m_tickLastBlock = false;  // was power above tick threshold last block?
    int   m_lastTickBlock = 0;      // block index of most recent rising-edge tick
    std::chrono::steady_clock::time_point m_lastTickTime; // audio time of last tick

    float m_signalLevel  = 0.0f;   // 0–1 display value based on 1 kHz SNR

    // --- 100 Hz data bit detection ---
    // Per-second thresholds are derived from smooth100 captured at the tick
    // (second boundary).  At tick time the data bit from the previous second
    // has fully decayed (EMA τ ≈ 20 ms, inter-bit silence ≥ 150 ms), so
    // smooth100 reflects only background noise — exactly the right reference
    // for distinguishing the next second's data bit from background.
    float m_preTick100   = 1e-5f;  // smooth100 at the most recent tick

    float m_smooth100    = 0.0f;   // EMA of 100 Hz power (for edge smoothing)
    float m_background100 = 1e-5f; // slow EMA: 100 Hz noise floor (synthetic tick ref)
    int   m_synthDebounce = 0;     // consecutive blocks above synthetic-tick threshold
    bool  m_inDataWindow = false;  // currently within the post-tick data window
    int   m_dataWindowEnd = 0;     // block at which data window closes
    int   m_dataWindowTick = 0;    // tick block that opened the current/last window
    std::chrono::steady_clock::time_point m_dataWindowTickTime; // tick time for open window
    bool  m_in100Hz      = false;  // currently measuring a 100 Hz data bit
    int   m_debounce100  = 0;      // consecutive blocks at current level
    int   m_data100StartBlock = 0; // block index when current data bit began

    // --- bit ring buffer ---
    static constexpr int k_bufSize  = 120; // 2 minutes of bits
    static constexpr int BIT_ZERO   = 0;
    static constexpr int BIT_ONE    = 1;
    static constexpr int BIT_MARKER = 2;
    static constexpr int BIT_MISSING = 3;  // gap-fill placeholder for lost seconds

    int   m_bits[k_bufSize];
    std::chrono::steady_clock::time_point m_bitTimes[k_bufSize]; // tick timestamps
    int   m_bitHead  = 0;   // write index (mod k_bufSize)
    int   m_bitCount = 0;   // total bits ever written

    // --- decoded result ---
    WwvTime m_frame;
    std::chrono::steady_clock::time_point m_audioEpoch; // audio time at block 0
    std::chrono::steady_clock::time_point m_p0Time;     // audio time at decoded P0 tick
    bool  m_p0Valid  = false;

    FrameCallback m_frameCb;
    BitCallback   m_bitCb;
};
