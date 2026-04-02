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
 * second, and 100 Hz to classify the BCD data bit that follows.
 *
 * Data bit classification uses energy integration rather than edge detection:
 * raw 100 Hz Goertzel power is accumulated into three sub-windows after each
 * tick (early 30–230 ms, mid 230–530 ms, late 530–830 ms).  The ratio of
 * incremental energies determines the bit type without requiring a sustained
 * above-threshold tone, making the decoder robust to HF flutter fading where
 * the subcarrier arrives as short incoherent fragments.
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

    // Estimate of current UTC with sub-second precision.
    // Returns the epoch if not yet synced.
    std::chrono::system_clock::time_point currentUtcPoint() const;

    // Compensate for audio pipeline latency (PortAudio inputLatency, in seconds).
    // Call once after Pa_OpenStream with Pa_GetStreamInfo(stream)->inputLatency.
    // Shifts m_audioEpoch backward so that audio timestamps reflect when sound
    // actually entered the microphone rather than when samples arrived in the
    // callback.
    void setAudioLatency(double latencySeconds);

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
    // type: 0=zero, 1=one, 2=marker, 3=missing.  ms: nominal pulse duration.
    using BitCallback = std::function<void(int type, int ms)>;
    void setBitCallback(BitCallback cb) { m_bitCb = std::move(cb); }

private:
    // --- Goertzel tone power at freqHz over a block of n samples ---
    static float goertzel(const float* buf, int n, float freqHz, float sampleRate);

    // Process one complete Goertzel block (m_blockSize samples).
    void processBlock(const float* block);

    // Called at 830 ms after each tick: classify the accumulated 100 Hz energy
    // and push the result into the bit ring buffer.
    void onWindowClose(std::chrono::steady_clock::time_point tickTime);

    // Scan m_bits ring buffer for a valid 60-bit frame; decode if found.
    bool trySyncAndDecode();

    // Validate and decode a candidate frame starting at ring-buffer offset.
    bool decodeFrame(int startIdx);

    // Append one bit to the ring buffer (used by onWindowClose and gap-fill).
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

    // --- 100 Hz background floor (for tick anti-spoof gate) -----------------
    // Slow EMA of raw 100 Hz Goertzel power.  Converges to the true 100 Hz
    // background level and is used to reject false 1 kHz tick detections that
    // are actually caused by 100 Hz MARKER harmonics: genuine ticks have p100
    // near background; false triggers from harmonics have p100 >> background.
    float m_background100 = 1e-5f; // slow EMA of 100 Hz noise floor

    // --- Energy integration: per-second 100 Hz power accumulators ----------
    // Three fixed sub-windows after each tick, filled with raw Goertzel power:
    //   early : blocks  3–22  =  30–230 ms  (20 blocks)
    //   mid   : blocks 23–52  = 230–530 ms  (30 blocks)
    //   late  : blocks 53–82  = 530–830 ms  (30 blocks)
    // Classification fires at block 83 (830 ms post-tick).
    // A noise tail window at blocks 92–97 (920–980 ms) updates the rolling
    // noise floor, which is used as the SNR reference at classification time.
    float m_accEarly      = 0.0f;  // accumulated 100 Hz energy: 30–230 ms
    float m_accMid        = 0.0f;  // accumulated 100 Hz energy: 230–530 ms
    float m_accLate       = 0.0f;  // accumulated 100 Hz energy: 530–830 ms
    float m_accNoise      = 0.0f;  // accumulated 100 Hz energy: noise tail
    float m_noiseFloor100 = 1e-5f; // rolling EMA of per-second noise tail energy

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
