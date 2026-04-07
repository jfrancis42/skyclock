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
 * DigitField — per-BCD-digit EMA accumulator (refclock_wwv.c technique 1.4).
 *
 * For each of the 9 BCD digit fields (minute-tens, minute-units, hour-tens,
 * hour-units, day-hundreds, day-tens, day-units, year-tens, year-units),
 * maintains a 10-element array of EMA correlation scores.  When a decoded
 * frame assigns value v to the field, like[v] is boosted by kBcdAlpha and all
 * others are decayed.  A field "converges" when one entry dominates and has
 * sustained for kBcdMinStreak consecutive frames.
 */
struct DigitField {
    float like[10] = {};  // EMA score per digit value 0–9
    int   value    = -1;  // winning value, or -1 if not yet converged
    int   streak   = 0;   // consecutive frames where same value won
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

    // Returns the number of BCD digit fields (out of 9) that have converged
    // to a stable value (streak >= kBcdMinStreak consecutive frames).
    // Range 0–9; useful as a "lock quality" indicator.
    int digitFieldsConverged() const;

    // Returns per-digit state for field i (0–8):
    //   0=minute-tens, 1=minute-units, 2=hour-tens, 3=hour-units,
    //   4=day-hundreds, 5=day-tens, 6=day-units, 7=year-tens, 8=year-units
    // df.value is the current winner (-1 before any frame), df.streak >= kBcdMinStreak
    // means stable.  Caller may check stable with (df.streak >= WwvDecoder::kMinStreak).
    const DigitField& digitField(int i) const { return m_digitFields[i]; }
    static constexpr int kMinStreak = 3; // same as kBcdMinStreak

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

    // Callback invoked (from audio thread) on each genuine 1 kHz tick rising
    // edge.  tickTime is the audio-derived steady_clock timestamp of the tick,
    // already compensated for audio pipeline latency via setAudioLatency().
    // This fires only on real second boundaries (not free-running predictions).
    using TickCallback = std::function<void(std::chrono::steady_clock::time_point)>;
    void setTickCallback(TickCallback cb) { m_tickCb = std::move(cb); }

    // Enable I/Q frequency-shift demodulation for SSB reception tuned offsetHz
    // below the WWV carrier.  When enabled, pushSamples() mixes the input with
    // a complex oscillator at offsetHz, low-pass filters I and Q, then feeds
    // the I (in-phase) channel to the Goertzel detectors.  This shifts the
    // 100 Hz subcarrier (at offsetHz+100 Hz in SSB audio) and the 1 kHz tick
    // (at offsetHz+1000 Hz) back to their nominal baseband frequencies.
    // Note: only the I channel is used (not sqrt(I²+Q²)).  The magnitude would
    // give a constant envelope for a single tone (DC), defeating the Goertzel.
    // Typical use: setIqOffset(500.0f) when radio is tuned 500 Hz low in USB.
    // Default: 0 (disabled — direct AM audio).
    void setIqOffset(float offsetHz);

    // When enabled (default: true), decoded frames are rejected unless the
    // decoded UTC time is within 25 hours of the system clock.  Disable for
    // file-based debugging where the recording may be from a different date.
    void setTimePlausibilityCheck(bool enable) { m_timePlausibility = enable; }

    // Minimum consecutive valid frames before emitting via the frame callback.
    // Default: k_minConsecutiveGood (2).  Set to 1 in file/debug mode to see
    // single-frame decodes without requiring temporal confirmation.
    void setMinConsecutiveFrames(int n) { m_minConsecutiveGood = n; }

private:
    // --- Goertzel tone power at freqHz over a block of n samples ---
    static float goertzel(const float* buf, int n, float freqHz, float sampleRate);

    // --- Complex Goertzel: returns {I, Q} at freqHz (for Q-channel coherence) ---
    struct IQ { float I, Q; };
    static IQ goertzelComplex(const float* buf, int n, float freqHz, float sampleRate);

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
    float m_background100 = 1e-3f; // slow EMA of 100 Hz noise floor

    // --- Comb filter: 100-slot per-epoch 1 kHz EMA (refclock_wwv.c 1.1) ------
    // Slot m_combSlot accumulates the 1 kHz Goertzel power from the block that
    // lands on that 10 ms epoch within each second.  argmax gives the tick epoch
    // integrated over many seconds — far more stable than rising-edge detection.
    float m_combBuf[100] = {};     // EMA of 1 kHz power per 10 ms epoch slot
    int   m_combSlot     = 0;      // current epoch index 0–99

    // --- Median filter on comb peak (refclock_wwv.c 1.2) ---------------------
    int   m_epochHist[3]   = {};   // ring buffer: last 3 comb-argmax positions
    int   m_epochHistIdx   = 0;    // next write index into m_epochHist
    int   m_combPeakMedian = 0;    // current 3-sample median (blocks after last tick)

    // --- Q-channel subcarrier phase EMA (refclock_wwv.c 1.3) -----------------
    float m_subcarrierPhaseEma = 0.0f; // slow EMA of Q_early from 100 Hz Goertzel

    // --- Energy integration: full-window coherent Goertzel ------------------
    // Raw audio samples for each sub-window after each tick:
    //   early : blocks  3–22  =  30–230 ms  (20 blocks = 20×blockSize samples)
    //   mid   : blocks 23–52  = 230–530 ms  (30 blocks)
    //   late  : blocks 53–82  = 530–830 ms  (30 blocks)
    //   noise : blocks 92–97  = 920–980 ms  (6 blocks)
    // At each window boundary a single coherent Goertzel is run over the entire
    // accumulated buffer.  This narrows the effective 100 Hz bin from ±50 Hz
    // (10 ms block) to ±2.5 Hz (200 ms window), reducing broadband-noise
    // contamination by ~20× and improving SNR by the same factor.
    //
    // Coherent energy: E = goertzel_power × n_samples.
    //   For a pure tone (amplitude A):  E = n × A²/4   (grows with window)
    //   For broadband noise (variance σ²): E ≈ σ²       (window-independent)
    // This makes E an unbiased estimator of σ² for pure noise and an
    // amplitude-independent SNR reference for the noise floor EMA.
    std::vector<float> m_earlyRaw;  // raw samples: bat  3–22
    std::vector<float> m_midRaw;    // raw samples: bat 23–52
    std::vector<float> m_lateRaw;   // raw samples: bat 53–82
    std::vector<float> m_noiseRaw;  // raw samples: bat 92–97
    int   m_earlyFill = 0;
    int   m_midFill   = 0;
    int   m_lateFill  = 0;
    int   m_noiseFill = 0;
    // Rolling EMA of coherent noise energy E_noise (≈ σ² for broadband noise).
    float m_noiseFloor100 = 1e-5f;

    // --- bit ring buffer ---
    static constexpr int k_bufSize  = 120; // 2 minutes of bits
    static constexpr int BIT_ZERO   = 0;
    static constexpr int BIT_ONE    = 1;
    static constexpr int BIT_MARKER = 2;
    static constexpr int BIT_MISSING = 3;  // gap-fill placeholder for lost seconds

    // --- free-running prediction and consistency ---
    // Prediction fires at bat=k_predDelay rather than bat=100, giving real ticks
    // at bat≈100 a chance to fire before the fallback kicks in.
    //
    // IMPORTANT — anchor correction: when a prediction fires, the anchor is set
    // to (currentBlock − (predTarget−100)) rather than currentBlock.  This keeps
    // the sub-windows aligned with the actual second boundary (which was expected
    // at bat=100 but missed) rather than shifting them forward by predTarget−100
    // blocks.  As a result the next real tick arrives at bat=100 (≥95 lockout),
    // resyncing immediately rather than staying blocked for ~17 seconds.
    static constexpr int k_predDelay = 106;
    // Require this many temporally-consistent consecutive frames before emitting
    // a result via m_frameCb.  One stray match passes BCD range checks by chance;
    // two frames 60 s apart cannot.
    static constexpr int k_minConsecutiveGood = 2;

    int   m_bits[k_bufSize];
    std::chrono::steady_clock::time_point m_bitTimes[k_bufSize]; // tick timestamps
    int   m_bitHead  = 0;   // write index (mod k_bufSize)
    int   m_bitCount = 0;   // total bits ever written

    // --- decoded result ---
    WwvTime m_frame;
    std::chrono::steady_clock::time_point m_audioEpoch; // audio time at block 0
    std::chrono::steady_clock::time_point m_p0Time;     // audio time at decoded P0 tick
    bool  m_p0Valid  = false;

    // --- temporal consistency state ---
    std::chrono::steady_clock::time_point m_prevP0Time;  // P0 timestamp of previous decoded frame
    int   m_consecutiveGood = 0; // consecutive frames that passed all checks + timing

    bool  m_timePlausibility = true; // reject frames >25 h from system clock
    int   m_minConsecutiveGood = k_minConsecutiveGood;

    // --- Per-digit BCD accumulator (refclock_wwv.c 1.4) ----------------------
    // Fields: 0=minute-tens, 1=minute-units, 2=hour-tens, 3=hour-units,
    //         4=day-hundreds, 5=day-tens, 6=day-units, 7=year-tens, 8=year-units
    static constexpr float kBcdAlpha    = 0.25f; // EMA weight per frame
    static constexpr int   kBcdMinStreak = 3;    // frames to declare convergence
    DigitField m_digitFields[9];

    FrameCallback m_frameCb;
    BitCallback   m_bitCb;
    TickCallback  m_tickCb;

    // --- I/Q envelope demodulation (setIqOffset) ----------------------------
    // Rotates a complex phasor at m_iqOffset Hz, mixes input, two-stage IIR
    // low-pass filters the I and Q channels, then feeds sqrt(I²+Q²) to the
    // existing block processor.  Disabled when m_iqOffset == 0.
    float m_iqOffset    = 0.0f;
    float m_iqCosStep   = 1.0f;  // cos of phase increment per sample
    float m_iqSinStep   = 0.0f;  // sin of phase increment per sample
    float m_iqCosState  = 1.0f;  // current phasor real part (unit circle)
    float m_iqSinState  = 0.0f;  // current phasor imaginary part
    float m_iLpf1       = 0.0f;  // I channel first IIR stage state
    float m_iLpf2       = 0.0f;  // I channel second IIR stage state
    float m_qLpf1       = 0.0f;  // Q channel first IIR stage state
    float m_qLpf2       = 0.0f;  // Q channel second IIR stage state
    std::vector<float> m_iqEnvBuf; // scratch buffer for envelope samples
    int   m_iqRenorm    = 0;       // counter for phasor renormalisation
};
