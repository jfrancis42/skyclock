/**
 * wwvdecoder.cpp — WWV time code decoder
 *
 * Signal format (NIST SP 432):
 *   Every second begins with a 5 ms 1 kHz reference tick (the on-time point).
 *   Following the tick (~30 ms later), a 100 Hz subcarrier encodes a BCD bit
 *   by its duration:
 *     200 ms  →  BIT_ZERO
 *     500 ms  →  BIT_ONE
 *     800 ms  →  BIT_MARKER  (position markers at seconds 0,9,19,29,39,49)
 *
 *   Detection strategy:
 *     • 1 kHz Goertzel detects the 5 ms tick (one 10 ms block is enough).
 *       The rising edge marks the precise second boundary.
 *     • 100 Hz Goertzel raw power is accumulated into three fixed sub-windows
 *       after each tick and classified by the ratio of incremental energies:
 *
 *       early (30–230 ms): 20 blocks  →  e_early
 *       mid  (230–530 ms): 30 blocks  →  e_mid
 *       late (530–830 ms): 30 blocks  →  e_late
 *
 *       Classification at 830 ms:
 *         total = e_early + e_mid + e_late
 *         if total < SNR_MIN × noise_800 → MISSING
 *         elif e_late / total > 0.28     → MARKER  (energy persists to 830 ms)
 *         elif e_mid  / total > 0.33     → ONE     (energy persists to 530 ms)
 *         else                           → ZERO    (energy ends by 230 ms)
 *
 *       This approach accumulates evidence across the full bit period, so HF
 *       flutter fading (10–70 ms incoherent fragments) sums to the same
 *       result as a sustained tone of the same total duration.
 *
 * BCD frame layout (60 bits, one per second):
 *   P0  : second 0   (reference marker / start of minute)
 *   1-8 : minutes BCD  (weights: 40,20,10,_,8,4,2,1)
 *   P1  : second 9
 *   10-18: hours BCD  (weights: _,_,20,10,_,8,4,2,1)
 *   P2  : second 19
 *   20-28: day-of-year BCD hi  (weights: _,_,200,100,_,80,40,20,10)
 *   P3  : second 29
 *   30-33: day-of-year BCD lo  (weights: 8,4,2,1)
 *   34-38: UT1 correction  (38=sign, 34-37 unused)
 *   P4  : second 39
 *   40-48: UT1 magnitude + year BCD hi  (40=0.8s,41=0.4s,42=0.2s,43=0.1s,44=_,45-48=year tens)
 *   P5  : second 49
 *   50-53: year BCD lo  (weights: 8,4,2,1)
 *   54  : unused
 *   55  : DST1
 *   56  : DST2
 *   57  : leap second warning
 *   58  : unused
 *   (P0 at second 0 of next minute)
 */

#include "wwvdecoder.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Position marker seconds within the frame ───────────────────────────── */
static const int k_markerSecs[] = {0, 9, 19, 29, 39, 49};

/* ── "Always zero" unweighted bit positions ─────────────────────────────── */
static const int k_zeroSecs[] = {4, 10, 11, 14, 20, 21, 24, 34, 35, 36, 37, 44, 54, 58};

/* ── Classification parameters ───────────────────────────────────────────── */
// Minimum total-window SNR to produce any bit; below this → MISSING.
// noise_800 = m_noiseFloor100 × (80 blocks / 6 blocks).
static constexpr float k_snrMin      = 2.0f;
// Fraction of noise-subtracted energy in the late or mid sub-window that
// divides the three bit types.  These thresholds apply to noise-subtracted
// fractions (E_late_s / E_total_s) rather than raw fractions.
//
// For noise-subtracted fractions, a pure tone gives the same fractions as
// the raw case (signal dominates), but purely-noise bits produce E_total_s ≈ 0
// which triggers the E_total_s guard rather than a false-MARKER classification.
//
// True MARKER:  late_frac_s = 30/(20+30+30) = 0.375  → must be > k_lateFrac
// True ONE:     mid_frac_s  = 30/(20+30)    = 0.600  → must be > k_midFrac
// True ZERO:    late_frac_s ≈ 0, mid_frac_s ≈ 0      → neither threshold met
//
// HF sky-wave delay can put ZERO/ONE energy in the late window, yielding
// late_frac_s ≈ 0.1–0.25.  k_lateFrac = 0.28 keeps a comfortable margin
// below the true-MARKER value (0.375) while tolerating this multipath.
static constexpr float k_lateFrac    = 0.28f; // late_frac_s > this → MARKER
static constexpr float k_midFrac     = 0.33f; // mid_frac_s  > this → ONE

/* ── Constructor ─────────────────────────────────────────────────────────── */
WwvDecoder::WwvDecoder(float sampleRate)
    : m_sampleRate(sampleRate)
    , m_blockSize(static_cast<int>(sampleRate * 0.010f)) // 10 ms blocks
    , m_blockBuf(m_blockSize, 0.0f)
    , m_lastTickTime(std::chrono::steady_clock::now())
    , m_earlyRaw(20 * static_cast<int>(sampleRate * 0.010f), 0.0f)
    , m_midRaw  (30 * static_cast<int>(sampleRate * 0.010f), 0.0f)
    , m_lateRaw (30 * static_cast<int>(sampleRate * 0.010f), 0.0f)
    , m_noiseRaw( 6 * static_cast<int>(sampleRate * 0.010f), 0.0f)
    , m_audioEpoch(std::chrono::steady_clock::now())
{
    memset(m_bits, 0, sizeof(m_bits));
}

/* ── Goertzel power at a single frequency ───────────────────────────────── */
float WwvDecoder::goertzel(const float* buf, int n, float freqHz, float sampleRate)
{
    float k     = freqHz / sampleRate * (float)n;
    float omega = 2.0f * (float)M_PI * k / (float)n;
    float coeff = 2.0f * cosf(omega);
    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
    for (int i = 0; i < n; ++i) {
        q2 = q1; q1 = q0;
        q0 = coeff * q1 - q2 + buf[i];
    }
    float power = q1*q1 + q2*q2 - coeff*q1*q2;
    return power / ((float)n * (float)n); // normalise to per-sample²
}

/* ── Complex Goertzel: returns {I, Q} components ────────────────────────── */
WwvDecoder::IQ WwvDecoder::goertzelComplex(const float* buf, int n,
                                           float freqHz, float sampleRate)
{
    float k     = freqHz / sampleRate * (float)n;
    float omega = 2.0f * (float)M_PI * k / (float)n;
    float coeff = 2.0f * cosf(omega);
    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
    for (int i = 0; i < n; ++i) {
        q2 = q1; q1 = q0;
        q0 = coeff * q1 - q2 + buf[i];
    }
    // I = real part, Q = imaginary part of the DFT bin.
    float I = (q1 * cosf(omega) - q2) / (float)n;
    float Q = (q1 * sinf(omega))       / (float)n;
    return {I, Q};
}

/* ── setIqOffset ─────────────────────────────────────────────────────────── */
void WwvDecoder::setIqOffset(float offsetHz)
{
    m_iqOffset   = offsetHz;
    float phase  = 2.0f * (float)M_PI * offsetHz / m_sampleRate;
    m_iqCosStep  = cosf(phase);
    m_iqSinStep  = sinf(phase);
    m_iqCosState = 1.0f;
    m_iqSinState = 0.0f;
    m_iLpf1 = m_iLpf2 = m_qLpf1 = m_qLpf2 = 0.0f;
    m_iqRenorm = 0;
}

/* ── pushSamples ─────────────────────────────────────────────────────────── */
void WwvDecoder::pushSamples(const float* samples, int count)
{
    if (m_iqOffset > 0.0f) {
        // ── I/Q envelope demodulation ─────────────────────────────────────
        // Mix input with complex oscillator at m_iqOffset Hz, low-pass filter
        // I and Q, then feed the envelope sqrt(I²+Q²) to the block processor.
        //
        // LPF: two cascaded single-pole IIRs, each with cutoff 1100 Hz.
        // This gives ~7 dB additional rejection at 1500 Hz (the image of the
        // 1 kHz tick shifted up by the offset) relative to a single stage.
        //
        //   α = 1 − exp(−2π × 1100 / sampleRate)
        //
        // The envelope contains:
        //   • DC         — from the carrier (Goertzel ignores DC)
        //   • 100 Hz     — from the BCD subcarrier at (offset+100) Hz in audio
        //   • 1000 Hz    — from the 1 kHz tick at (offset+1000) Hz in audio
        // Both match what the existing Goertzel detectors expect.
        const float alpha = 1.0f - expf(-2.0f * (float)M_PI * 1100.0f / m_sampleRate);
        const float beta  = 1.0f - alpha;

        if ((int)m_iqEnvBuf.size() < count)
            m_iqEnvBuf.resize(count);

        for (int i = 0; i < count; ++i) {
            // Advance complex phasor by one step (cos/sin rotation).
            float nc = m_iqCosState * m_iqCosStep - m_iqSinState * m_iqSinStep;
            float ns = m_iqCosState * m_iqSinStep + m_iqSinState * m_iqCosStep;
            m_iqCosState = nc;
            m_iqSinState = ns;

            // Renormalise every 4096 samples to prevent rounding drift.
            if (++m_iqRenorm >= 4096) {
                m_iqRenorm = 0;
                float mag = sqrtf(nc * nc + ns * ns);
                if (mag > 1e-10f) {
                    m_iqCosState /= mag;
                    m_iqSinState /= mag;
                }
            }

            // Mix
            float x    = samples[i];
            float iSig = x * m_iqCosState;
            float qSig = x * m_iqSinState;

            // Two-stage IIR LPF
            m_iLpf1 = beta * m_iLpf1 + alpha * iSig;
            m_iLpf2 = beta * m_iLpf2 + alpha * m_iLpf1;
            m_qLpf1 = beta * m_qLpf1 + alpha * qSig;
            m_qLpf2 = beta * m_qLpf2 + alpha * m_qLpf1;

            // Use the I (in-phase) channel directly — NOT sqrt(I²+Q²).
            // sqrt(I²+Q²) computes the amplitude envelope.  For a single tone
            // at frequency f shifted to baseband, the envelope is constant DC;
            // the Goertzel at f would see zero.  The I channel alone gives
            // a×cos(2πft), which the Goertzel detects with normal power a²/4.
            m_iqEnvBuf[i] = m_iLpf2;
        }

        // Feed envelope samples into the existing block processor.
        for (int s = 0; s < count; ++s) {
            m_blockBuf[m_blockBufFill++] = m_iqEnvBuf[s];
            if (m_blockBufFill == m_blockSize) {
                processBlock(m_blockBuf.data());
                m_blockBufFill = 0;
            }
        }
        return;
    }

    for (int s = 0; s < count; ++s) {
        m_blockBuf[m_blockBufFill++] = samples[s];
        if (m_blockBufFill == m_blockSize) {
            processBlock(m_blockBuf.data());
            m_blockBufFill = 0;
        }
    }
}

/* ── processBlock ────────────────────────────────────────────────────────── */
void WwvDecoder::processBlock(const float* block)
{
    ++m_totalBlocks;

    // Audio-based timestamp for this block: derived from block count so that
    // file playback at any speed produces correctly-spaced timestamps.
    auto blockTime = m_audioEpoch +
        std::chrono::milliseconds(static_cast<long long>(m_totalBlocks) * 10LL);

    // ── 1 kHz Goertzel — tick detection and signal-level display ─────────────
    float p1k = goertzel(block, m_blockSize, 1000.0f, m_sampleRate);

    // Smooth 1 kHz power for noise floor tracking (fast α=0.5).
    m_smoothPower = 0.5f * p1k + 0.5f * m_smoothPower;

    // Asymmetric EMA noise floor: fast downward convergence from initial 1.0,
    // very slow upward drift during tones so ticks don't contaminate floor.
    float ad1k = (m_totalBlocks < k_warmup) ? 0.1f : 0.01f;
    if (m_smoothPower < m_noisePower * 1.5f)
        m_noisePower = (1.0f - ad1k) * m_noisePower + ad1k * m_smoothPower;
    else
        m_noisePower = 0.9999f * m_noisePower + 0.0001f * m_smoothPower;
    if (m_noisePower < 1e-12f) m_noisePower = 1e-12f;

    // Signal level for display (1 kHz SNR, normalised 0–1).
    float snr1k   = m_smoothPower / m_noisePower;
    m_signalLevel = (snr1k > 1.0f) ? (1.0f - 1.0f / snr1k) : 0.0f;
    if (m_signalLevel > 1.0f) m_signalLevel = 1.0f;

    // ── Comb filter update (refclock_wwv.c 1.1) ─────────────────────────────
    // Update the comb slot for the current 10 ms epoch within the second.
    // After many seconds the comb peak converges to the true tick position.
    {
        int slot = (m_totalBlocks - 1) % 100;
        m_combBuf[slot] = 0.9f * m_combBuf[slot] + 0.1f * p1k;
        m_combSlot = (slot + 1) % 100;
    }

    // ── 100 Hz Goertzel — raw power for accumulation ─────────────────────────
    float p100 = goertzel(block, m_blockSize, 100.0f, m_sampleRate);

    // Track 100 Hz background for tick anti-spoof gate.
    // Update downward quickly when p100 is near floor; admit only a trickle
    // from active-subcarrier blocks so the floor isn't pulled up to signal level.
    {
        float ad100bg = (p100 < m_background100 * 3.0f) ? 0.005f : 0.0001f;
        m_background100 = (1.0f - ad100bg) * m_background100 + ad100bg * p100;
        if (m_background100 < 1e-12f) m_background100 = 1e-12f;
    }

    // Wait for noise floor EMA to converge before attempting detection.
    if (m_totalBlocks < k_warmup) return;

    // ── 1 kHz tick rising-edge detection ────────────────────────────────────
    // The on-time tick is 5 ms — it fits inside a single 10 ms block and rises
    // sharply.  Threshold at 100× noise floor catches even low-SNR ticks while
    // rejecting 100 Hz data-bit leakage (which is at ~0 in the 1 kHz bin).
    // Lockout of 95 blocks (950 ms) prevents false re-triggers from 100 Hz
    // MARKER harmonics or multi-path echoes.
    //
    // Anti-spoof gate: genuine 1 kHz ticks occur at the very start of each
    // second, before the 100 Hz subcarrier begins (~30 ms later), so p100 is
    // near the background floor.  MARKER harmonics (10th harmonic of 100 Hz)
    // produce elevated p1k AND elevated p100 simultaneously.  Gate on p100 to
    // reject these false triggers.
    //
    // Threshold: 15× noise floor.  Live HF measurements on a clean IC-7300
    // signal into a USB audio device show tick SNR of 18–61× (vs. 100× in the
    // original recordings used for development).  100× was too aggressive for
    // real-world AM and SSB reception; 15× retains the anti-spoof gate as the
    // primary false-trigger rejection mechanism.
    bool tickNow = (p1k > m_noisePower * 15.0f)
                && (p100 < m_background100 * 10.0f);
    if (static_cast<int>(m_totalBlocks) - m_lastTickBlock < 95) tickNow = false;
    bool risingEdge = (tickNow && !m_tickLastBlock);
    if (risingEdge) {
        // Rising edge: reset raw sample buffers for the new second.
        m_earlyFill = m_midFill = m_lateFill = m_noiseFill = 0;
        m_lastTickBlock = m_totalBlocks;
        m_lastTickTime  = blockTime;
        if (m_tickCb) m_tickCb(blockTime);
    }
    m_tickLastBlock = tickNow;

    // ── Free-running 1-second prediction with comb+median correction (1.1/1.2) ─
    // On each rising edge, update the comb argmax → 3-sample median to estimate
    // the stable tick epoch offset within the second.  The prediction fires at
    // that offset rather than the fixed k_predDelay, adapting to audio-clock
    // drift and HF propagation delay.
    if (risingEdge) {
        // Find the comb slot with peak accumulated 1 kHz power.
        int peakSlot = 0;
        float peakVal = -1.0f;
        for (int i = 0; i < 100; ++i) {
            if (m_combBuf[i] > peakVal) { peakVal = m_combBuf[i]; peakSlot = i; }
        }
        // rawOffset = how many blocks from the last anchor to the next expected tick.
        // In steady state this ≈ 100.  Clamp to [95 .. k_predDelay=106].
        //
        // The comb peakSlot is the 10 ms epoch (0–99) where 1 kHz energy is
        // highest.  We just fired a risingEdge at the current epoch
        //   currentSlot = (m_totalBlocks - 1) % 100.
        // The next occurrence of peakSlot is (peakSlot - currentSlot + 100) % 100
        // blocks away; if that is 0 (we are exactly AT the peak) the tick just
        // fired so the next occurrence is a full 100 blocks away.  Add 6 blocks
        // of margin so real ticks at bat ≈ 100 have time to arrive before the
        // prediction fires.  The anchor-correction in the prediction branch then
        // backs the anchor up by (predTarget−100) so sub-windows stay aligned.
        {
            int currentSlot = (int)((m_totalBlocks - 1) % 100);
            int rawOffset = ((peakSlot - currentSlot) % 100 + 100) % 100;
            if (rawOffset == 0) rawOffset = 100;
            rawOffset += 6;  // fire prediction 60 ms after expected tick
            // Lower clamp is 100, not 95: allowing predTarget < 100 causes the
            // prediction to fire BEFORE the expected real tick (at bat < 100).
            // The real tick then arrives at bat = 100 - (100 - predTarget) < 95
            // and is blocked by the lockout forever.  With clamp [100, 106],
            // real ticks always arrive at bat=100 from the corrected anchor.
            if (rawOffset < 100)         rawOffset = 100;
            if (rawOffset > k_predDelay) rawOffset = k_predDelay;

            m_epochHist[m_epochHistIdx] = rawOffset;
            m_epochHistIdx = (m_epochHistIdx + 1) % 3;

            // 3-sample median (sort copy).
            int h[3] = {m_epochHist[0], m_epochHist[1], m_epochHist[2]};
            if (h[0] > h[1]) { int t = h[0]; h[0] = h[1]; h[1] = t; }
            if (h[1] > h[2]) { int t = h[1]; h[1] = h[2]; h[2] = t; }
            if (h[0] > h[1]) { int t = h[0]; h[0] = h[1]; h[1] = t; }
            m_combPeakMedian = h[1];
        }
    }

    int bat_predict = m_totalBlocks - m_lastTickBlock;
    int predTarget  = (m_combPeakMedian > 0) ? m_combPeakMedian : k_predDelay;
    if (m_lastTickBlock > 0 && bat_predict == predTarget && !tickNow) {
        // Real tick didn't arrive — fire the free-running prediction.
        //
        // ANCHOR CORRECTION: set the anchor to where the tick *should* have
        // been (bat=100 from the previous anchor, i.e. predTarget−100 blocks
        // before now) rather than to the current block.  This keeps the early/
        // mid/late sub-windows aligned with the actual second boundary.
        //
        // Without this, each missed prediction shifts the anchor forward by
        // (predTarget−100)=6 blocks, causing the late sub-window to drift into
        // the NEXT second's subcarrier onset (~9 predictions later), producing
        // a cascade of false MARKER classifications.
        //
        // With the correction the next real tick arrives at bat=100 from the
        // corrected anchor (≥ the 95-block lockout), so it resyncs immediately
        // when the signal recovers rather than staying blocked for 17 seconds.
        //
        // Guard: only correct when predTarget > 100 (the common case); if
        // predTarget ≤ 100 the prediction is already firing at or before the
        // expected tick and no forward shift has occurred.
        int correction = (predTarget > 100) ? (predTarget - 100) : 0;
        m_earlyFill = m_midFill = m_lateFill = m_noiseFill = 0;
        m_lastTickBlock = m_totalBlocks - correction;
        m_lastTickTime  = blockTime - std::chrono::milliseconds(
            static_cast<long long>(correction) * 10LL);
    }

    // Note: SYNTHK (100 Hz onset detection for initial lock) is intentionally
    // omitted.  In flutter-faded HF conditions the first detectable 100 Hz
    // fragment may arrive 200–500 ms after the true second boundary; using it
    // as the phase origin shifts every subsequent prediction by that offset,
    // causing ZERO bits (30–230 ms window) to be entirely missed.  Waiting for
    // the first real 1 kHz tick is slower to acquire but produces an accurate
    // phase reference that the free-running prediction then extrapolates correctly.

    // ── Raw sample accumulation into sub-windows ─────────────────────────────
    // bat = blocks after tick.  Raw audio samples are copied into per-window
    // buffers so that a single coherent Goertzel can be computed over the
    // entire window at the window boundary.  Coherent integration narrows the
    // effective 100 Hz DFT bin from ±50 Hz (10 ms) to ±2.5 Hz (200 ms) or
    // ±1.7 Hz (300 ms), reducing broadband-audio noise contamination by ~20×.
    int bat = m_totalBlocks - m_lastTickBlock;

    if (bat >= 3 && bat <= 22) {
        std::copy(block, block + m_blockSize, m_earlyRaw.data() + m_earlyFill);
        m_earlyFill += m_blockSize;
    } else if (bat >= 23 && bat <= 52) {
        std::copy(block, block + m_blockSize, m_midRaw.data() + m_midFill);
        m_midFill += m_blockSize;
    } else if (bat >= 53 && bat <= 82) {
        std::copy(block, block + m_blockSize, m_lateRaw.data() + m_lateFill);
        m_lateFill += m_blockSize;
    } else if (bat >= 92 && bat <= 97) {
        std::copy(block, block + m_blockSize, m_noiseRaw.data() + m_noiseFill);
        m_noiseFill += m_blockSize;
    }

    // ── Classify at 830 ms ───────────────────────────────────────────────────
    if (bat == 83) {
        onWindowClose(m_lastTickTime);
    }

    // ── Update noise floor at end of noise tail window (980 ms) ─────────────
    // Compute a single coherent Goertzel over the 60 ms noise window.
    // m_noiseFloor100 is an EMA of E_noise = goertzel_power × n_samples.
    // For broadband noise (variance σ²): E_noise ≈ σ² (window-length independent).
    // α = 0.1 → ~10 s settling time.
    if (bat == 98) {
        float e_noise = goertzel(m_noiseRaw.data(), m_noiseFill, 100.0f, m_sampleRate);
        float E_noise = (m_noiseFill > 0) ? e_noise * (float)m_noiseFill : 0.0f;
        const float noiseAlpha = 0.1f;
        m_noiseFloor100 = (1.0f - noiseAlpha) * m_noiseFloor100 + noiseAlpha * E_noise;
        if (m_noiseFloor100 < 1e-15f) m_noiseFloor100 = 1e-15f;
        m_noiseFill = 0;
    }
}

/* ── appendBit ───────────────────────────────────────────────────────────── */
void WwvDecoder::appendBit(int type, std::chrono::steady_clock::time_point t)
{
    m_bits[m_bitHead]     = type;
    m_bitTimes[m_bitHead] = t;
    m_bitHead = (m_bitHead + 1) % k_bufSize;
    ++m_bitCount;
}

/* ── onWindowClose ───────────────────────────────────────────────────────── */
void WwvDecoder::onWindowClose(std::chrono::steady_clock::time_point tickTime)
{
    // Compute coherent full-window Goertzel for each sub-window.
    // Goertzel power is normalised to per-sample² (divided by n²), so:
    //   pure tone (amplitude A): power ≈ A²/4   (independent of window length)
    //   broadband noise (σ²):    power ≈ σ²/n   (falls as window grows)
    //
    // Coherent energy E = power × n:
    //   signal:  E ≈ n × A²/4    (grows with window — longer window captures more)
    //   noise:   E ≈ σ²          (window-length independent — always ≈ σ²)
    // This makes the fractions E_late/E_total, E_mid/E_total signal-ratio
    // quantities with the same thresholds as the old per-block approach while
    // providing ~20× better SNR from the narrower effective DFT bandwidth.
    float e_early_g = goertzel(m_earlyRaw.data(), m_earlyFill, 100.0f, m_sampleRate);
    float e_mid_g   = goertzel(m_midRaw.data(),   m_midFill,   100.0f, m_sampleRate);
    float e_late_g  = goertzel(m_lateRaw.data(),  m_lateFill,  100.0f, m_sampleRate);

    float E_early = e_early_g * (float)m_earlyFill;
    float E_mid   = e_mid_g   * (float)m_midFill;
    float E_late  = e_late_g  * (float)m_lateFill;
    float E_total = E_early + E_mid + E_late;

    // ── Q-channel coherence (refclock_wwv.c 1.3) ─────────────────────────────
    // Compute the complex Goertzel over the early window.  The I component is
    // the in-phase projection; Q is 90° ahead.  The ratio |I|/magnitude gives
    // the coherence of the subcarrier: 1.0 for a pure steady tone, ~0 for
    // incoherent flutter fragments.
    //
    // Coherence is tracked via a phase EMA (Doppler tracking) but is NOT applied
    // to the MISSING threshold — doing so would unfairly penalise marginal but
    // real signals on weak HF paths.  The coherence value is available for
    // future use (e.g. per-bit confidence scoring for the BCD accumulator).
    if (m_earlyFill > 0) {
        IQ iq = goertzelComplex(m_earlyRaw.data(), m_earlyFill, 100.0f, m_sampleRate);
        float mag = sqrtf(iq.I * iq.I + iq.Q * iq.Q);
        // Update subcarrier phase EMA (slow — tracks ionospheric Doppler drift).
        const float kPhaseAlpha = 0.05f;
        m_subcarrierPhaseEma = (1.0f - kPhaseAlpha) * m_subcarrierPhaseEma
                             + kPhaseAlpha * ((mag > 1e-10f) ? iq.Q : 0.0f);
    }

    // Noise reference: m_noiseFloor100 is the EMA of E_noise ≈ σ².
    // Each of the three sub-windows contributes ~σ² noise, so total noise ≈ 3σ².
    float noise_ref = m_noiseFloor100 * 3.0f;
    float snr_total = (noise_ref > 1e-15f) ? (E_total / noise_ref) : 0.0f;

    // ── Noise-subtracted energy fractions ────────────────────────────────────
    // Raw fractions (E_late/E_total) are biased toward MARKER for pure noise
    // because the late and mid windows are 30 blocks each vs. 20 for early.
    // For equal broadband noise: late/total = 30/80 = 0.375 > k_lateFrac=0.28,
    // which would misclassify pure-noise bits as MARKER.  To safely lower the
    // MISSING threshold we subtract the expected per-window noise floor (σ²)
    // from each window before computing fractions.  For pure noise each window
    // measures ≈ σ²; after subtraction all are ≈ 0, so fractions are undefined
    // and the E_total_s guard forces MISSING.  For a real signal the window
    // containing the subcarrier has E >> σ², giving unbiased fractions.
    float N = m_noiseFloor100;  // expected noise energy per window ≈ σ²
    float E_early_s = (E_early > N) ? E_early - N : 0.0f;
    float E_mid_s   = (E_mid   > N) ? E_mid   - N : 0.0f;
    float E_late_s  = (E_late  > N) ? E_late  - N : 0.0f;
    float E_total_s = E_early_s + E_mid_s + E_late_s;

    // Hard MISSING: below k_snrSoft (1.5), or no window significantly above
    // the noise floor (guards against pure-noise fluctuations at borderline SNR).
    // At k_snrSoft=1.5 vs. the old k_snrMin=2.0, borderline bits from a
    // flutter-faded HF path get a second chance via the noise-subtracted fractions.
    static constexpr float k_snrSoft = 1.5f;

    float late_frac = 0.0f, mid_frac = 0.0f;
    int type;
    if (snr_total < k_snrSoft || E_total_s < N * 0.2f) {
        type = BIT_MISSING;
    } else {
        late_frac = (E_total_s > 0.0f) ? E_late_s / E_total_s : 0.0f;
        mid_frac  = (E_total_s > 0.0f) ? E_mid_s  / E_total_s : 0.0f;

        if      (late_frac > k_lateFrac) type = BIT_MARKER;
        else if (mid_frac  > k_midFrac)  type = BIT_ONE;
        else                             type = BIT_ZERO;
    }

    // Approximate nominal duration for the bit callback.
    int ms = (type == BIT_ZERO)   ? 200 :
             (type == BIT_ONE)    ? 500 :
             (type == BIT_MARKER) ? 800 : 0;

    // ── Gap-fill: insert BIT_MISSING placeholders for skipped seconds ─────────
    if (m_bitCount > 0) {
        int prevIdx = (m_bitHead - 1 + k_bufSize) % k_bufSize;
        auto prevTime = m_bitTimes[prevIdx];
        double gapSec = std::chrono::duration<double>(tickTime - prevTime).count();

        // Dedup: if the previous bit fired less than 600 ms ago, something
        // fired twice for the same second (e.g. SYNTHK then real tick).
        // Keep whichever bit has the higher-priority type (MARKER > ONE > ZERO).
        if (gapSec < 0.600 && type != BIT_MISSING) {
            int prevType = m_bits[prevIdx];
            if (type > prevType) {
                m_bits[prevIdx]     = type;
                m_bitTimes[prevIdx] = tickTime;
            }
            if (m_bitCb) m_bitCb(type, ms);
            return;
        }

        // Gap-fill: more than 1.5 s since last bit → insert placeholders.
        // Cap at 20 so a long silence doesn't flood the buffer.
        int missed = static_cast<int>(std::round(gapSec)) - 1;
        if (missed > 0 && missed <= 20) {
            for (int i = 0; i < missed; ++i) {
                auto fillTime = prevTime + std::chrono::milliseconds(
                    static_cast<long long>((i + 1) * 1000));
                appendBit(BIT_MISSING, fillTime);
                if (m_bitCb) m_bitCb(BIT_MISSING, 0);
            }
        }
    }

    appendBit(type, tickTime);
    if (m_bitCb) m_bitCb(type, ms);

    // Once we have at least 60 bits, attempt frame sync.
    if (m_bitCount >= 60)
        trySyncAndDecode();
}

/* ── trySyncAndDecode ────────────────────────────────────────────────────── */
bool WwvDecoder::trySyncAndDecode()
{
    int avail = (m_bitCount < k_bufSize) ? m_bitCount : k_bufSize;
    if (avail < 60) return false;

    // Scan newest-to-oldest so the most recent alignment wins.
    int maxStart = avail - 60;
    for (int start = maxStart; start >= 0; --start) {
        if (decodeFrame(start)) return true;
    }

    return false;
}

/* ── decodeFrame ─────────────────────────────────────────────────────────── */
bool WwvDecoder::decodeFrame(int startOffset)
{
    // startOffset is relative to (m_bitHead - avail) in the ring buffer.
    int avail = (m_bitCount < k_bufSize) ? m_bitCount : k_bufSize;
    int base  = (m_bitHead - avail + k_bufSize) % k_bufSize;

    // Helper: get raw bit value at position pos within this candidate frame
    auto bit = [&](int pos) -> int {
        return m_bits[(base + startOffset + pos) % k_bufSize];
    };
    auto ts = [&](int pos) -> std::chrono::steady_clock::time_point {
        return m_bitTimes[(base + startOffset + pos) % k_bufSize];
    };
    // BCD helper: BIT_ONE→1, BIT_ZERO/BIT_MISSING→0.
    auto bval = [&](int pos) -> int {
        return (bit(pos) == BIT_ONE) ? 1 : 0;
    };

    // ── BCD digit accumulator helper (refclock_wwv.c 1.4) ────────────────────
    // fieldIdx: 0=min-tens,1=min-units,2=hr-tens,3=hr-units,
    //           4=day-hund,5=day-tens,6=day-units,7=yr-tens,8=yr-units
    // Decay all entries, then boost the decoded value.
    // Updates m_digitFields[fieldIdx] and returns the new winning value (-1 if
    // not yet converged to kBcdMinStreak consecutive frames).
    auto updateDigit = [&](int fieldIdx, int decodedVal) {
        DigitField& df = m_digitFields[fieldIdx];
        for (int v = 0; v < 10; ++v)
            df.like[v] *= (1.0f - kBcdAlpha);
        if (decodedVal >= 0 && decodedVal <= 9)
            df.like[decodedVal] += kBcdAlpha;
        // Find winner.
        int winner = 0;
        for (int v = 1; v < 10; ++v)
            if (df.like[v] > df.like[winner]) winner = v;
        if (winner == df.value) {
            ++df.streak;
        } else {
            df.value  = winner;
            df.streak = 1;
        }
    };

    // Reject frames with too many missing bits — BCD decode becomes unreliable.
    int missingCount = 0;
    for (int i = 0; i < 60; ++i)
        if (bit(i) == BIT_MISSING) ++missingCount;
    if (missingCount > 40) return false;

    // 1. Verify markers at expected positions.
    // Under HF flutter fading, position markers (P1–P5) sometimes appear one
    // second late: the 100 Hz subcarrier fades through the late sub-window of
    // the true second, then recovers and fires strongly in the early+mid windows
    // of the following second.  Accept a MARKER at mp+1 as evidence of mp.
    // MISSING (HF dropout on e.g. P0) is tolerated up to once per frame;
    // the BCD range checks and temporal consistency gate provide additional
    // false-positive rejection.
    int  markerMissing = 0;
    bool lateMarker[60] = {};   // lateMarker[mp+1] = true when mp arrived late
    for (int mp : k_markerSecs) {
        int b = bit(mp);
        if (b == BIT_MARKER) continue;
        // Late-marker: the MARKER arrived one second after the expected position.
        if (mp + 1 < 60 && bit(mp + 1) == BIT_MARKER) {
            lateMarker[mp + 1] = true;
            continue;
        }
        if (b == BIT_MISSING) { ++markerMissing; continue; }
        return false; // ZERO or ONE at marker position with no late evidence
    }
    if (markerMissing > 2) return false;

    // 2. Reject frames where "always-zero" positions are markers.
    // Late-marker positions (e.g. position 10 when P1 arrived late) are exempt.
    // Allow at most one unexplained violation for robustness under heavy fading.
    int zeroViolations = 0;
    for (int zp : k_zeroSecs) {
        if (lateMarker[zp]) continue;
        if (bit(zp) == BIT_MARKER) {
            if (++zeroViolations > 1) return false;
        }
    }

    // ── Decode minutes ─────────────────────────────────────────────────────
    int tens  = bval(1)*40 + bval(2)*20 + bval(3)*10;
    int units = bval(5)*8  + bval(6)*4  + bval(7)*2  + bval(8)*1;
    int minute = tens + units;
    if (minute < 0 || minute > 59) return false;
    updateDigit(0, tens / 10);
    updateDigit(1, units);

    // ── Decode hours ───────────────────────────────────────────────────────
    int htens  = bval(12)*20 + bval(13)*10;
    int hunits = bval(15)*8  + bval(16)*4  + bval(17)*2 + bval(18)*1;
    int hour   = htens + hunits;
    if (hour < 0 || hour > 23) return false;
    updateDigit(2, htens / 10);
    updateDigit(3, hunits);

    // ── Decode day of year ─────────────────────────────────────────────────
    int doy = bval(22)*200 + bval(23)*100
            + bval(25)*80  + bval(26)*40 + bval(27)*20 + bval(28)*10
            + bval(30)*8   + bval(31)*4  + bval(32)*2  + bval(33)*1;
    if (doy < 1 || doy > 366) return false;
    updateDigit(4, (doy / 100));
    updateDigit(5, (doy % 100) / 10);
    updateDigit(6, doy % 10);

    // ── Decode year (last 2 digits) ────────────────────────────────────────
    int ytens  = bval(45)*8 + bval(46)*4 + bval(47)*2 + bval(48)*1;
    int yunits = bval(50)*8 + bval(51)*4 + bval(52)*2 + bval(53)*1;
    // Reject invalid BCD digits individually (each must be 0–9).
    if (ytens > 9 || yunits > 9) return false;
    int year2  = ytens * 10 + yunits;
    updateDigit(7, ytens);
    updateDigit(8, yunits);

    // ── Time plausibility gate ─────────────────────────────────────────────
    // Reject frames whose decoded UTC is more than 25 hours from the system
    // clock.  Catches noise frames that pass BCD range checks but decode to
    // implausible dates (e.g. year 2001 when the system is in 2025).
    // Disable via setTimePlausibilityCheck(false) when replaying old recordings.
    if (m_timePlausibility) {
        // Build a UTC unix timestamp from the decoded BCD fields.
        int fullYear = 2000 + year2;
        static const int days_norm[] = {31,28,31,30,31,30,31,31,30,31,30,31};
        static const int days_leap[] = {31,29,31,30,31,30,31,31,30,31,30,31};
        bool leap = (fullYear % 4 == 0 && fullYear % 100 != 0) || (fullYear % 400 == 0);
        const int* dim = leap ? days_leap : days_norm;
        int month = 1, dayOfMonth = doy;
        for (int m2 = 0; m2 < 12 && dayOfMonth > dim[m2]; ++m2) {
            dayOfMonth -= dim[m2]; month = m2 + 2;
        }
        struct tm tmv = {};
        tmv.tm_year = fullYear - 1900; tmv.tm_mon = month - 1;
        tmv.tm_mday = dayOfMonth; tmv.tm_hour = hour; tmv.tm_min = minute;
#ifdef _WIN32
        time_t decoded = _mkgmtime(&tmv);
#else
        time_t decoded = timegm(&tmv);
#endif
        time_t now = std::time(nullptr);
        if (std::abs(static_cast<double>(decoded) - static_cast<double>(now))
                > 25.0 * 3600.0)
            return false;
    }

    // ── Decode UT1 correction ──────────────────────────────────────────────
    bool   ut1Positive = (bit(38) == BIT_ONE);
    double ut1 = bval(40)*0.8 + bval(41)*0.4 + bval(42)*0.2 + bval(43)*0.1;
    if (!ut1Positive) ut1 = -ut1;

    // ── Decode DST and leap second ─────────────────────────────────────────
    int  dst = bval(55)*2 + bval(56)*1;
    bool lsw = (bit(57) == BIT_ONE);

    // ── All checks passed — temporal consistency gate ──────────────────────
    // A single frame that clears BCD range checks is still fallible (random
    // noise can produce plausible-looking BCD).  Require that successive frames
    // are ~60 s apart (as measured by the audio-clock P0 timestamps) before
    // we trust the decode and invoke the frame callback.
    auto currentP0 = ts(0);
    bool temporalOk = false;
    double dt = 0.0;
    if (m_consecutiveGood > 0) {
        dt = std::chrono::duration<double>(currentP0 - m_prevP0Time).count();
        // Accept ±2.5 s around the expected 60-second frame interval.
        temporalOk = (dt > 57.5 && dt < 62.5);
    }

    if (temporalOk) {
        ++m_consecutiveGood;
    } else {
        m_consecutiveGood = 1; // restart streak; this frame is the new candidate
    }

    // ── Record the decoded frame ───────────────────────────────────────────
    WwvTime t;
    t.minute              = minute;
    t.hour                = hour;
    t.dayOfYear           = doy;
    t.year2digit          = year2;
    t.ut1Seconds          = ut1;
    t.dstCode             = dst;
    t.leapSecondWarning   = lsw;
    t.valid               = true;
    t.confidence          = m_consecutiveGood;

    // Only advance the P0 anchor when the timing is trustworthy:
    //   • first decode ever (!m_p0Valid)
    //   • temporal consistency confirmed (dt ≈ 60 s) — normal minute advance
    //   • anchor is stale (dt > 65 s or negative) — signal came back after dropout
    // Leaving the anchor fixed on small-dt failures (false-positive shifted
    // frames with dt = 0..57 s) means the genuine 60 s gap at the real
    // minute boundary is still measurable even after a string of false positives.
    bool anchorStale = m_p0Valid && (dt < -5.0 || dt > 65.0);
    if (!m_p0Valid || temporalOk || anchorStale) {
        m_prevP0Time = currentP0;
    }
    m_frame      = t;
    m_p0Time     = currentP0;
    m_p0Valid    = true;

    // Only emit via callback once the streak meets the minimum.
    if (m_consecutiveGood >= m_minConsecutiveGood) {
        if (m_frameCb) m_frameCb(m_frame);
    }
    return true;
}

/* ── p0UnixTime (internal helper) ────────────────────────────────────────── */
// Returns the UNIX timestamp of P0 (second 0 of the most recently decoded
// minute) from the BCD fields in m_frame.
static time_t p0UnixTime(const WwvTime& frame)
{
    int year = 2000 + frame.year2digit;
    int doy  = frame.dayOfYear;
    int hour = frame.hour;
    int min  = frame.minute;

    static const int days_norm[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    static const int days_leap[] = {31,29,31,30,31,30,31,31,30,31,30,31};
    bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    const int* dim = leap ? days_leap : days_norm;
    int month = 1, day = doy;
    for (int m2 = 0; m2 < 12 && day > dim[m2]; ++m2) {
        day -= dim[m2];
        month = m2 + 2;
    }

    struct tm t = {};
    t.tm_year  = year - 1900;
    t.tm_mon   = month - 1;
    t.tm_mday  = day;
    t.tm_hour  = hour;
    t.tm_min   = min;
    t.tm_sec   = 0;
    t.tm_isdst = 0;

#ifdef _WIN32
    return _mkgmtime(&t);
#else
    return timegm(&t);
#endif
}

/* ── currentUtc ──────────────────────────────────────────────────────────── */
time_t WwvDecoder::currentUtc() const
{
    if (!m_p0Valid || !m_frame.valid) return 0;

    auto currentAudioTime = m_audioEpoch +
        std::chrono::milliseconds(static_cast<long long>(m_totalBlocks) * 10LL);
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        currentAudioTime - m_p0Time);
    return p0UnixTime(m_frame) + elapsed.count();
}

/* ── currentUtcPoint ─────────────────────────────────────────────────────── */
std::chrono::system_clock::time_point WwvDecoder::currentUtcPoint() const
{
    if (!m_p0Valid || !m_frame.valid)
        return std::chrono::system_clock::time_point{};

    // P0 as a system_clock time_point (whole-second resolution from BCD, which
    // is fine — P0 is always at the start of a UTC minute, no sub-second part).
    auto p0 = std::chrono::system_clock::from_time_t(p0UnixTime(m_frame));

    // Audio-time elapsed since P0, at full steady_clock precision (nanoseconds
    // on most platforms; at worst microseconds).  This carries the sub-second
    // component that currentUtc() discards.
    auto currentAudioTime = m_audioEpoch +
        std::chrono::milliseconds(static_cast<long long>(m_totalBlocks) * 10LL);
    auto elapsed = currentAudioTime - m_p0Time;

    return p0 + std::chrono::duration_cast<std::chrono::system_clock::duration>(elapsed);
}

/* ── setAudioLatency ─────────────────────────────────────────────────────── */
void WwvDecoder::setAudioLatency(double latencySeconds)
{
    // Shift the audio epoch backward by the pipeline latency so that all
    // block timestamps reflect when sound entered the microphone rather than
    // when the samples arrived in the callback.
    auto correction = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(latencySeconds));
    m_audioEpoch -= correction;
}

/* ── lastFrame ───────────────────────────────────────────────────────────── */
WwvTime WwvDecoder::lastFrame() const
{
    return m_frame;
}

/* ── digitFieldsConverged ────────────────────────────────────────────────── */
int WwvDecoder::digitFieldsConverged() const
{
    int count = 0;
    for (int i = 0; i < 9; ++i)
        if (m_digitFields[i].streak >= kBcdMinStreak) ++count;
    return count;
}
