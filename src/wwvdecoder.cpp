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
 *         elif e_mid  / total > 0.28     → ONE     (energy persists to 530 ms)
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
// Fraction of total energy in the late or mid sub-window that divides the
// three bit types.  Derived from Python energy analysis on wwv-noisy.opus.
static constexpr float k_lateFrac    = 0.28f; // late_frac > this → MARKER
static constexpr float k_midFrac     = 0.28f; // mid_frac  > this → ONE

/* ── Constructor ─────────────────────────────────────────────────────────── */
WwvDecoder::WwvDecoder(float sampleRate)
    : m_sampleRate(sampleRate)
    , m_blockSize(static_cast<int>(sampleRate * 0.010f)) // 10 ms blocks
    , m_blockBuf(m_blockSize, 0.0f)
    , m_lastTickTime(std::chrono::steady_clock::now())
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

/* ── pushSamples ─────────────────────────────────────────────────────────── */
void WwvDecoder::pushSamples(const float* samples, int count)
{
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
    bool tickNow = (p1k > m_noisePower * 100.0f)
                && (p100 < m_background100 * 10.0f);
    if (static_cast<int>(m_totalBlocks) - m_lastTickBlock < 95) tickNow = false;
    if (tickNow && !m_tickLastBlock) {
        // Rising edge: reset energy accumulators for the new second.
        m_accEarly = 0.0f;
        m_accMid   = 0.0f;
        m_accLate  = 0.0f;
        m_accNoise = 0.0f;

        m_lastTickBlock = m_totalBlocks;
        m_lastTickTime  = blockTime;
    }
    m_tickLastBlock = tickNow;

    // ── Free-running 1-second prediction (primary fallback) ──────────────────
    // Once a real tick has been detected, advance the second boundary by
    // exactly 100 blocks (1000 ms) when no real tick fires.  WWV's cesium
    // clock makes the actual second boundaries accurate to microseconds, so
    // the predicted position is correct to within audio-clock jitter (~1 ms
    // per minute for a typical consumer sound card at 50 ppm).
    //
    // This replaces SYNTHK for steady-state operation: SYNTHK fires on 100 Hz
    // onset, which arrives late under flutter fading, shifting the accumulation
    // windows away from the true second boundary.  The prediction uses the last
    // confirmed tick as a phase anchor and drifts only with the audio clock.
    int bat_predict = m_totalBlocks - m_lastTickBlock;
    if (m_lastTickBlock > 0 && bat_predict == 100 && !tickNow) {
        // Real tick didn't arrive — advance prediction.
        m_accEarly = 0.0f;
        m_accMid   = 0.0f;
        m_accLate  = 0.0f;
        m_accNoise = 0.0f;

        m_lastTickBlock = m_totalBlocks;
        m_lastTickTime  = blockTime;
    }

    // Note: SYNTHK (100 Hz onset detection for initial lock) is intentionally
    // omitted.  In flutter-faded HF conditions the first detectable 100 Hz
    // fragment may arrive 200–500 ms after the true second boundary; using it
    // as the phase origin shifts every subsequent prediction by that offset,
    // causing ZERO bits (30–230 ms window) to be entirely missed.  Waiting for
    // the first real 1 kHz tick is slower to acquire but produces an accurate
    // phase reference that the free-running prediction then extrapolates correctly.

    // ── Energy accumulation into sub-windows ─────────────────────────────────
    // bat = blocks after tick.  Each block's raw 100 Hz Goertzel power is added
    // to the appropriate window.  Raw power (not the EMA) accumulates correctly
    // from incoherent flutter fragments — brief bursts simply add their energy
    // to whichever window they land in.
    int bat = m_totalBlocks - m_lastTickBlock;

    if      (bat >= 3  && bat <= 22) m_accEarly += p100;
    else if (bat >= 23 && bat <= 52) m_accMid   += p100;
    else if (bat >= 53 && bat <= 82) m_accLate  += p100;
    else if (bat >= 92 && bat <= 97) m_accNoise += p100;

    // ── Classify at 830 ms ───────────────────────────────────────────────────
    if (bat == 83) {
        onWindowClose(m_lastTickTime);
    }

    // ── Update noise floor at end of noise tail window ───────────────────────
    // At bat == 98 (980 ms) the 6-block noise measurement is complete.
    // Update the rolling EMA.  α = 0.1 gives ~10 second settling time.
    if (bat == 98) {
        const float noiseAlpha = 0.1f;
        m_noiseFloor100 = (1.0f - noiseAlpha) * m_noiseFloor100
                        + noiseAlpha * m_accNoise;
        if (m_noiseFloor100 < 1e-12f) m_noiseFloor100 = 1e-12f;
        m_accNoise = 0.0f;
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
    // Classify the accumulated sub-window energies.
    float e_early = m_accEarly;   // 30–230 ms (20 blocks)
    float e_mid   = m_accMid;     // 230–530 ms (30 blocks)
    float e_late  = m_accLate;    // 530–830 ms (30 blocks)
    float e_total = e_early + e_mid + e_late;

    // Noise reference: scale the 6-block tail measurement to an 80-block window.
    float noise_800 = m_noiseFloor100 * (80.0f / 6.0f);
    float snr_total = (noise_800 > 1e-12f) ? (e_total / noise_800) : 0.0f;

    int type;
    if (snr_total < k_snrMin) {
        type = BIT_MISSING;
    } else {
        float late_frac = e_late / e_total;
        float mid_frac  = e_mid  / e_total;

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
    // Try each of the 60 possible frame starting positions in the ring buffer.
    // The buffer holds k_bufSize bits; we only have m_bitCount bits total but
    // at most k_bufSize stored.  Use min(m_bitCount, k_bufSize) as the window.
    int avail = (m_bitCount < k_bufSize) ? m_bitCount : k_bufSize;
    if (avail < 60) return false;

    // Only try starting positions where a full 60-bit frame fits in the buffer.
    int maxStart = avail - 60;
    for (int start = 0; start <= maxStart; ++start) {
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

    // Reject frames with too many missing bits — BCD decode becomes unreliable.
    int missingCount = 0;
    for (int i = 0; i < 60; ++i)
        if (bit(i) == BIT_MISSING) ++missingCount;
    if (missingCount > 30) return false;

    // 1. Verify markers at expected positions.
    // ZERO or ONE at a marker position is a definitive alignment failure.
    // MISSING (HF dropout on e.g. P0) is tolerated up to once per frame;
    // the BCD range checks below provide additional false-positive rejection.
    int markerMissing = 0;
    for (int mp : k_markerSecs) {
        int b = bit(mp);
        if (b == BIT_MARKER) continue;
        if (b == BIT_MISSING) { ++markerMissing; continue; }
        return false; // ZERO or ONE at a marker position
    }
    if (markerMissing > 1) return false;

    // 2. Reject frames where "always-zero" positions are markers
    for (int zp : k_zeroSecs) {
        if (bit(zp) == BIT_MARKER) return false;
    }

    // ── Decode minutes ─────────────────────────────────────────────────────
    int tens  = bval(1)*40 + bval(2)*20 + bval(3)*10;
    int units = bval(5)*8  + bval(6)*4  + bval(7)*2  + bval(8)*1;
    int minute = tens + units;
    if (minute < 0 || minute > 59) return false;

    // ── Decode hours ───────────────────────────────────────────────────────
    int htens  = bval(12)*20 + bval(13)*10;
    int hunits = bval(15)*8  + bval(16)*4  + bval(17)*2 + bval(18)*1;
    int hour   = htens + hunits;
    if (hour < 0 || hour > 23) return false;

    // ── Decode day of year ─────────────────────────────────────────────────
    int doy = bval(22)*200 + bval(23)*100
            + bval(25)*80  + bval(26)*40 + bval(27)*20 + bval(28)*10
            + bval(30)*8   + bval(31)*4  + bval(32)*2  + bval(33)*1;
    if (doy < 1 || doy > 366) return false;

    // ── Decode year (last 2 digits) ────────────────────────────────────────
    int ytens  = bval(45)*8 + bval(46)*4 + bval(47)*2 + bval(48)*1;
    int yunits = bval(50)*8 + bval(51)*4 + bval(52)*2 + bval(53)*1;
    int year2  = ytens * 10 + yunits;
    if (year2 < 0 || year2 > 99) return false;

    // ── Decode UT1 correction ──────────────────────────────────────────────
    bool   ut1Positive = (bit(38) == BIT_ONE);
    double ut1 = bval(40)*0.8 + bval(41)*0.4 + bval(42)*0.2 + bval(43)*0.1;
    if (!ut1Positive) ut1 = -ut1;

    // ── Decode DST and leap second ─────────────────────────────────────────
    int  dst = bval(55)*2 + bval(56)*1;
    bool lsw = (bit(57) == BIT_ONE);

    // ── All checks passed — record the decoded frame ───────────────────────
    WwvTime t;
    t.minute              = minute;
    t.hour                = hour;
    t.dayOfYear           = doy;
    t.year2digit          = year2;
    t.ut1Seconds          = ut1;
    t.dstCode             = dst;
    t.leapSecondWarning   = lsw;
    t.valid               = true;
    t.confidence          = m_frame.valid ? m_frame.confidence + 1 : 1;

    m_frame  = t;
    m_p0Time = ts(0);  // system timestamp at P0 (start of this decoded minute)
    m_p0Valid = true;

    if (m_frameCb) m_frameCb(m_frame);
    return true;
}

/* ── currentUtc ──────────────────────────────────────────────────────────── */
time_t WwvDecoder::currentUtc() const
{
    if (!m_p0Valid || !m_frame.valid) return 0;

    // Convert decoded minute (year/doy/hour/minute) to UNIX timestamp
    // P0 is at second 0 of the decoded minute.
    int year = 2000 + m_frame.year2digit;
    int doy  = m_frame.dayOfYear;
    int hour = m_frame.hour;
    int min  = m_frame.minute;

    // Day-of-year → month/day
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
    t.tm_year = year - 1900;
    t.tm_mon  = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min  = min;
    t.tm_sec  = 0;
    t.tm_isdst = 0;

#ifdef _WIN32
    time_t p0_unix = _mkgmtime(&t);
#else
    time_t p0_unix = timegm(&t);
#endif

    // Add elapsed audio-time seconds since P0.
    // m_p0Time and current audio time are both derived from m_audioEpoch + block
    // count, so this works correctly for both file and live-audio modes.
    auto currentAudioTime = m_audioEpoch +
        std::chrono::milliseconds(static_cast<long long>(m_totalBlocks) * 10LL);
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        currentAudioTime - m_p0Time);
    return p0_unix + elapsed.count();
}

/* ── lastFrame ───────────────────────────────────────────────────────────── */
WwvTime WwvDecoder::lastFrame() const
{
    return m_frame;
}
