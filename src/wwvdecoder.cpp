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
 *     • 100 Hz Goertzel measures the data-bit pulse that follows.
 *       The pulse duration is classified as ZERO/ONE/MARKER.
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

    // ── 100 Hz Goertzel — smooth on every block (including warmup) ───────────
    {
        float p100w = goertzel(block, m_blockSize, 100.0f, m_sampleRate);
        m_smooth100 = 0.5f * p100w + 0.5f * m_smooth100;
    }

    // Wait for noise floor EMA to converge before attempting detection.
    if (m_totalBlocks < k_warmup) return;

    // ── 1 kHz tick rising-edge detection ────────────────────────────────────
    // The on-time tick is 5 ms — it fits inside a single 10 ms block and rises
    // sharply.  Threshold at 100× noise floor catches even low-SNR ticks while
    // rejecting 100 Hz data-bit leakage (which is at ~0 in the 1 kHz bin).
    // No debounce for onset — the tick is intentionally brief; we want exactly
    // one event per second.  A lockout of 10 blocks (100 ms) prevents the same
    // tick from re-triggering on adjacent blocks.
    bool tickNow = (p1k > m_noisePower * 100.0f);
    // Lockout: suppress false re-triggers within 950 ms of the last tick.
    // Real ticks are exactly 1000 ms apart (atomic clock); any tick within
    // 950 ms is false — typically 100 Hz MARKER harmonics exciting the 1 kHz
    // bin, or multi-path echoes.  950 ms is safely below the 1000 ms period
    // while leaving room for the ±50 ms timing jitter possible in HF recordings.
    if (static_cast<int>(m_totalBlocks) - m_lastTickBlock < 95) tickNow = false;
    if (tickNow && !m_tickLastBlock) {
        // Rising edge: each tick is a hard second boundary.  Force-close any
        // active data window from the *previous* second so the measurement is
        // not abandoned silently.  This is the correct path when a MARKER
        // bit's EMA tail hasn't decayed to thresh_off before the next tick
        // arrives — the tick itself provides a clean fence.
        if (m_inDataWindow) {
            if (m_in100Hz) {
                int durBlocks = m_totalBlocks - m_data100StartBlock;
                onPulse(durBlocks, m_dataWindowTickTime);
                // Reset the 100 Hz EMA to background so the decaying tail of
                // the just-closed bit does not contaminate the next data window
                // (which opens only 30 ms later).  Without this, a low
                // preTick100 from the background guard would cause thresh100_on
                // to be so low that the EMA decay spike triggers a spurious
                // short BIT_ZERO, closing the window before the real data bit
                // is measured.
                m_smooth100 = m_background100;
            }
            m_inDataWindow = false;
            m_in100Hz      = false;
            m_debounce100  = 0;
        }
        // Capture the pre-tick smooth100 as the per-second threshold reference.
        // Normally the previous second's data bit has fully decayed before this
        // tick arrives (EMA τ ≈ 20 ms; shortest inter-bit gap ≥ 150 ms).
        // Exception: position-marker seconds (P0/P3/P4/P5) sometimes have a
        // weak 1 kHz tick that is just above the detection threshold AND a 100 Hz
        // subcarrier onset that coincides with the same 10 ms block.  In that
        // case smooth100 is already elevated — using it as preTick100 would make
        // thresh100_on/off too high and cause the MARKER pulse to go undetected.
        // Guard: if smooth100 is already elevated above the tracked background,
        // use the background level instead so thresholds stay noise-referenced.
        m_preTick100 = (m_smooth100 > m_background100 * 5.0f)
                       ? m_background100 : m_smooth100;

        // Record the new second boundary.
        m_lastTickBlock = m_totalBlocks;
        m_lastTickTime  = blockTime;
    }
    m_tickLastBlock = tickNow;

    // ── 100 Hz background floor — updated only in quiet periods ──────────────
    // Mirrors the 1 kHz noise-floor EMA; used by the synthetic tick below.
    // Only moves downward when smooth100 is below 3× current background so that
    // active data bits (100–1000× above background) never corrupt the estimate.
    {
        float ad100bg = 0.01f;
        if (m_smooth100 < m_background100 * 3.0f)
            m_background100 = (1.0f - ad100bg) * m_background100 + ad100bg * m_smooth100;
        else
            m_background100 = 0.9999f * m_background100 + 0.0001f * m_smooth100;
        if (m_background100 < 1e-12f) m_background100 = 1e-12f;
    }

    // ── Synthetic tick: strong 100 Hz onset without a preceding 1 kHz tick ──
    // WWV position-marker seconds (P0, P3, P4, P5) can have a 1 kHz reference
    // tick that is 100–165× weaker than the nominal level, falling below the
    // 100× noise-floor detection threshold.  In that case the 100 Hz subcarrier
    // rises simultaneously with the weak tick.  Detect the 100 Hz onset as a
    // fallback second-boundary indicator so the 800 ms MARKER pulse is not lost.
    //
    // Conditions: no data window already open; at least 950 ms since last tick
    // (real or synthetic); smooth100 rises above 20× background for ≥2 blocks.
    // 950 ms matches the 1 kHz tick lockout — prevents SYNTHK from firing on
    // the same 100 Hz MARKER onset that would trigger a false real tick.
    if (!m_inDataWindow && !m_in100Hz) {
        int bstS = m_totalBlocks - m_lastTickBlock;
        if (bstS >= 95) {
            if (m_smooth100 > m_background100 * 20.0f) {
                if (++m_synthDebounce >= 2) {
                    // Place synthetic tick 2 blocks before this one (the onset).
                    m_lastTickBlock = m_totalBlocks - 2;
                    m_lastTickTime  = m_audioEpoch +
                        std::chrono::milliseconds(
                            static_cast<long long>(m_lastTickBlock) * 10LL);
                    // Use the pre-onset background as preTick100 so that
                    // thresh100_on/off are relative to the true noise floor,
                    // not the already-elevated smooth100 at this moment.
                    m_preTick100    = m_background100;
                    m_synthDebounce = 0;
                }
            } else {
                m_synthDebounce = 0;
            }
        } else {
            m_synthDebounce = 0;
        }
    } else {
        m_synthDebounce = 0;
    }

    // ── Tick-gated data bit detection state machine ───────────────────────────
    // Detection is restricted to a window [+30 ms, +1150 ms] after each tick
    // so the high pre-tick 100 Hz background cannot cause false triggers.
    //
    // Thresholds are set relative to m_preTick100 (smooth100 at last tick):
    //   background within window ≈ m_preTick100; data bits ≈ 10–1000×.
    // thresh_on at 8× leaves comfortable margin above background fluctuations
    // while being reachable even for moderate flutter-faded bits.
    // thresh_off at 3× lets the EMA tail decay past the offset threshold
    // within ~40 ms of the bit ending (≈ 4 EMA time-constants of 10 ms each).
    float thresh100_on  = m_preTick100 * 8.0f;
    float thresh100_off = m_preTick100 * 3.0f;

    // Manage data window open/close based on the most-recent tick.
    if (m_lastTickBlock > 0) {
        int bst = m_totalBlocks - m_lastTickBlock; // blocks since tick
        if (!m_inDataWindow && !m_in100Hz && bst >= 3 && bst <= 115
                && m_lastTickBlock != m_dataWindowTick) {
            m_inDataWindow       = true;
            m_dataWindowEnd      = m_lastTickBlock + 115;  // 1150 ms: 800ms MARKER + EMA decay room
            m_dataWindowTick     = m_lastTickBlock;
            m_dataWindowTickTime = m_lastTickTime;         // snapshot tick time at window open
        }
        if (m_inDataWindow && m_totalBlocks > m_dataWindowEnd) {
            // Window expired — if a bit was in progress, close it.
            if (m_in100Hz) {
                int durBlocks = m_dataWindowEnd - m_data100StartBlock;
                onPulse(durBlocks, m_dataWindowTickTime);
                m_in100Hz     = false;
                m_debounce100 = 0;
            }
            m_inDataWindow = false;
        }
    }

    if (m_inDataWindow) {
        if (!m_in100Hz) {
            if (m_smooth100 > thresh100_on) {
                if (++m_debounce100 >= 2) { // 20 ms onset confirmation
                    m_in100Hz           = true;
                    m_debounce100       = 0;
                    m_data100StartBlock = m_totalBlocks - 2;
                }
            } else {
                m_debounce100 = 0;
            }
        } else {
            if (m_smooth100 < thresh100_off) {
                if (++m_debounce100 >= 5) { // 50 ms offset confirmation — prevents brief dips from closing MARKER windows
                    m_in100Hz      = false;
                    m_debounce100  = 0;
                    m_inDataWindow = false;
                    int durBlocks  = (m_totalBlocks - 5) - m_data100StartBlock;
                    onPulse(durBlocks, m_dataWindowTickTime);
                }
            } else {
                m_debounce100 = 0;
            }
        }
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

/* ── onPulse ─────────────────────────────────────────────────────────────── */
void WwvDecoder::onPulse(int durationBlocks,
                          std::chrono::steady_clock::time_point pulseStart)
{
    // 10 ms per block
    int ms = durationBlocks * 10;

    // Classify the pulse duration.  Nominal widths: 200 ms (0), 500 ms (1), 800 ms (marker).
    // Lower minimum reduced from 150 ms to 80 ms to handle HF flutter-fading, where
    // multi-path interference breaks a sustained tone into brief high-power bursts; the
    // EMA tail and 150 ms fall-debounce mean even a heavy faded 200 ms pulse can measure
    // as short as 110 ms.  Upper bound for MARKER raised to 1100 ms for the same reason.
    int type;
    if      (ms >=  80 && ms < 350) type = BIT_ZERO;
    else if (ms >= 350 && ms < 650) type = BIT_ONE;
    else if (ms >= 650 && ms < 1100) type = BIT_MARKER;
    else return; // not a valid WWV pulse

    if (m_bitCount > 0) {
        int prevIdx = (m_bitHead - 1 + k_bufSize) % k_bufSize;
        auto prevTime = m_bitTimes[prevIdx];
        double gapSec = std::chrono::duration<double>(pulseStart - prevTime).count();

        // Deduplication: if the previous bit fired less than 600 ms ago, this is
        // a second burst within the same second (flutter fading broke the pulse into
        // multiple clusters).  Keep whichever bit has the longer/more-reliable type
        // (prefer MARKER > ONE > ZERO) and discard the other.
        if (gapSec < 0.600) {
            int prevType = m_bits[prevIdx];
            if (type > prevType) {
                // New bit is "higher priority" — replace the previous one.
                m_bits[prevIdx] = type;
                m_bitTimes[prevIdx] = pulseStart;
            }
            // Either way, don't add a second ring-buffer entry for this second.
            if (m_bitCb) m_bitCb(type, ms);
            return;
        }

        // Gap-fill: if more than 1.5 s has elapsed since the last detected bit,
        // one or more seconds were missed (HF fading dropout).  Insert BIT_MISSING
        // placeholders so the ring buffer retains the correct 1-second cadence and
        // trySyncAndDecode() can still find a valid 60-bit frame when only isolated
        // seconds are lost.  Cap at 20 so a long silence doesn't flood the buffer.
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

    appendBit(type, pulseStart);
    if (m_bitCb) m_bitCb(type, ms);

    // Once we have at least 60 bits, attempt frame sync
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
