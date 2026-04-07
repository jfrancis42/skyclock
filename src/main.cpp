/**
 * skyclock — WWV time code decoder and clock synchroniser.
 *
 * Receives the NIST WWV time signal via a connected radio (hamlib optional),
 * decodes the BCD time code from audio, and displays current UTC time.
 * Optionally sets the system clock once enough frames have been verified.
 *
 * Configuration: ~/.skyclock/settings.json
 *
 * Usage:
 *   skyclock                         # decode from default audio device
 *   skyclock --device <name>         # use named PortAudio device
 *   skyclock --file <path>           # fast decode from audio file (debug)
 *   skyclock --file <path> --realtime  # real-time file simulation
 *   skyclock --list-devices          # list audio devices and exit
 *   skyclock --list-rigs             # list hamlib rig models and exit
 *   skyclock --version               # print version and exit
 *   skyclock --help                  # this help
 */

#include "settings.h"
#include "rigcontrol.h"
#include "wwvdecoder.h"
#include "clocksync.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <csignal>
#include <mutex>
#include <string>

#include <portaudio.h>

#ifdef _WIN32
#  include <io.h>
#  include <windows.h>
#  define isatty _isatty
#  define fileno _fileno
static void sleepMs(int ms) { Sleep((DWORD)ms); }
static void enableAnsiConsole() {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD  m = 0;
    if (GetConsoleMode(h, &m))
        SetConsoleMode(h, m | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}
#else
#  include <unistd.h>
static void sleepMs(int ms) { usleep((useconds_t)ms * 1000u); }
static void enableAnsiConsole() {}
#endif

#ifndef SC_VERSION
#  define SC_VERSION "0.1.0-ALPHA"
#endif

/* ── Signal handling ─────────────────────────────────────────────────────── */
static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running.store(false); }

/* ── Shared decoder state (audio thread → main thread) ──────────────────── */
static WwvDecoder* g_decoder    = nullptr;
static std::mutex  g_frameMutex;
static WwvTime     g_latestFrame;
static bool        g_frameUpdated = false;

/* ── Tick-sync state (audio thread → main thread) ────────────────────────── */
struct TickSnap {
    bool   hasAnchor       = false;
    std::chrono::steady_clock::time_point anchorSteady; // steady time of first tick
    time_t anchorUtc       = 0;    // UTC assigned to anchorSteady
    std::chrono::steady_clock::time_point lastTick;
    time_t lastTickUtc     = 0;    // estimated UTC at lastTick
    int    tickCount       = 0;
    bool   pendingSet      = false; // minute boundary identified, pending clock set
    time_t pendingUtc      = 0;
    std::chrono::steady_clock::time_point pendingTick;
    bool   boundaryFound   = false; // minute boundary has been identified (latches)
    time_t boundaryUtc     = 0;
    bool   clockSet        = false; // system clock was set successfully
    bool   useSystemAnchor = false; // anchor from system clock, not --time
};
static TickSnap   g_tickSnap;
static std::mutex g_tickMutex;

/* ── TTY display state ───────────────────────────────────────────────────── */
// Sliding window of the last 60 classified bits as printable chars.
// Written only by the audio-thread bit callback; memmove of 59 bytes and
// single char writes are safe without a mutex on all supported platforms.
static char  g_bitRow[61];     // display chars (0, 1, |, ?) + NUL; space-padded
static int   g_bitPos = 0;     // total bits appended (for count label)
static float g_smoothSig = 0.0f; // slow EMA of signal level (~3 s time constant)

static constexpr int kDisplayLines = 4;

// Append one classified bit to the sliding display window.
static void appendBitDisplay(int type)
{
    char c = (type == 0) ? '0' : (type == 1) ? '1' : (type == 2) ? '|' : '?';
    if (g_bitPos < 60) {
        g_bitRow[g_bitPos] = c;
    } else {
        memmove(g_bitRow, g_bitRow + 1, 59);
        g_bitRow[59] = c;
    }
    ++g_bitPos;
}

// Print a 20-char bar coloured by SNR: red < 3 dB, yellow 3–10 dB, green ≥ 10 dB.
static void printBar(float sig, float snrDb)
{
    int filled = (int)(sig * 20.0f + 0.5f);
    if (filled > 20) filled = 20;
    const char* color = (snrDb >= 10.0f) ? "\033[32m" :  // green  — good
                        (snrDb >=  3.0f) ? "\033[33m" :  // yellow — marginal
                                           "\033[31m";   // red    — poor
    if (filled > 0) printf("%s", color);
    for (int i = 0; i < filled; ++i)
        printf("\xe2\x96\x88");           // █
    printf("\033[2m");                    // dim for empty portion
    for (int i = filled; i < 20; ++i)
        printf("\xe2\x96\x91");           // ░
    printf("\033[0m");
}

// Redraw (or initially draw) the 4-line status panel.
// Uses \033[NA to move the cursor up and overwrite previous content.
// Each line ends with \033[K to clear any leftover chars from a longer previous value.
static void drawDisplay(bool first, float sig, time_t utc, const WwvTime& f,
                        bool clockSet, long long freqKhz, int minConf)
{
    if (!first)
        printf("\033[%dA\r", kDisplayLines);

    // Line 1 — title, signal bar, frequency
    float snrDb = (sig < 0.9999f) ? -10.0f * log10f(1.0f - sig) : 30.0f;
    printf("\033[1mskyclock " SC_VERSION "\033[0m   ");
    printBar(sig, snrDb);
    printf(" SNR:%4.1f dB   %lld kHz\033[K\n", snrDb, freqKhz);

    // Line 2 — bit stream (searching) or decoded UTC time
    if (utc == 0) {
        // Show the last 60 bits and total count.
        printf("\033[2m[%s]\033[0m  %d bit%s received\033[K\n",
               g_bitRow, g_bitPos, g_bitPos == 1 ? "" : "s");
    } else {
        struct tm t;
#ifdef _WIN32
        gmtime_s(&t, &utc);
#else
        gmtime_r(&utc, &t);
#endif
        bool confirmed = (f.confidence >= minConf);
        // Confirmed: bold white.  Candidate: dim yellow to signal uncertainty.
        const char* timeColor = confirmed ? "\033[1m" : "\033[33m";
        printf("%s%04d-%02d-%02d %02d:%02d:%02d UTC\033[0m"
               "  Day %03d  UT1%+.1fs\033[K\n",
               timeColor,
               t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
               t.tm_hour, t.tm_min, t.tm_sec,
               f.dayOfYear, f.ut1Seconds);
    }

    // Line 3 — phase / lock status
    if (utc == 0) {
        int bits = g_decoder ? g_decoder->bitsReceived() : 0;
        if (bits == 0) {
            printf("\033[33mWaiting for signal — no ticks detected yet\033[0m\033[K\n");
        } else if (bits < 60) {
            printf("\033[33mReceiving bits (%d/60 for first frame attempt)\033[0m\033[K\n",
                   bits);
        } else {
            printf("\033[33mDecoding — frame sync not yet achieved\033[0m\033[K\n");
        }
    } else if (clockSet) {
        printf("\033[1m\033[32mLOCKED\033[0m  — clock set successfully\033[K\n");
    } else if (f.confidence >= minConf) {
        printf("\033[1m\033[32mLOCKED\033[0m"
               "  — %d consecutive frames confirmed\033[K\n", f.confidence);
    } else {
        printf("\033[33mCandidate frame\033[0m"
               "  — confirming (%d/%d frames needed)\033[K\n",
               f.confidence, minConf);
    }

    // Line 4 — decoder detail: digit convergence + DST/leap
    // Build a partial time string from the BCD accumulator.
    // Fields: 0=min-tens, 1=min-units, 2=hr-tens, 3=hr-units,
    //         4=day-hund, 5=day-tens, 6=day-units, 7=yr-tens, 8=yr-units
    // Each digit is shown as its value when stable, '-' when not yet verified.
    auto dchar = [](const WwvDecoder* dec, int fieldIdx) -> char {
        if (!dec) return '-';
        const DigitField& df = dec->digitField(fieldIdx);
        if (df.value >= 0 && df.streak >= WwvDecoder::kMinStreak)
            return (char)('0' + df.value);
        return '-';
    };
    int converged = g_decoder ? g_decoder->digitFieldsConverged() : 0;
    // Format: "HH:MM  Day DDD  Year 20YY"
    // Field order: hr-tens=2, hr-units=3, min-tens=0, min-units=1,
    //              day-hundreds=4, day-tens=5, day-units=6,
    //              year-tens=7, year-units=8
    char partialBuf[32];
    snprintf(partialBuf, sizeof(partialBuf), "%c%c:%c%c  Day %c%c%c  Year 20%c%c",
             dchar(g_decoder, 2), dchar(g_decoder, 3),
             dchar(g_decoder, 0), dchar(g_decoder, 1),
             dchar(g_decoder, 4), dchar(g_decoder, 5), dchar(g_decoder, 6),
             dchar(g_decoder, 7), dchar(g_decoder, 8));
    if (utc == 0) {
        printf("BCD: \033[1m%s\033[0m  (%d/9 stable)\033[K\n",
               partialBuf, converged);
    } else {
        // After lock: show DST, leap-second warning, and partial BCD for confirmation.
        const char* dstStr = (f.dstCode == 0) ? "Standard time"
                           : (f.dstCode == 3) ? "DST in effect"
                           : (f.dstCode == 2) ? "DST begins today"
                           :                    "DST ends today";
        const char* lswStr = f.leapSecondWarning ? "  \033[33m⚠ Leap second\033[0m" : "";
        printf("%s%s  BCD: %s  [conf:%d]\033[K\n",
               dstStr, lswStr, partialBuf, f.confidence);
    }

    fflush(stdout);
}

/* ── PortAudio callback ──────────────────────────────────────────────────── */
static int pa_callback(const void* input, void* /*output*/,
                       unsigned long frames,
                       const PaStreamCallbackTimeInfo* /*ti*/,
                       PaStreamCallbackFlags /*flags*/,
                       void* /*userdata*/)
{
    if (input && g_decoder)
        g_decoder->pushSamples(static_cast<const float*>(input),
                               static_cast<int>(frames));
    return g_running.load() ? paContinue : paComplete;
}

/* ── Device listing ──────────────────────────────────────────────────────── */
static void listDevices()
{
    Pa_Initialize();
    int count = Pa_GetDeviceCount();
    printf("Available audio input devices:\n");
    for (int i = 0; i < count; ++i) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (info && info->maxInputChannels > 0)
            printf("  [%d] %s  (%.0f Hz, %d ch)\n",
                   i, info->name,
                   info->defaultSampleRate,
                   info->maxInputChannels);
    }
    Pa_Terminate();
}

/* ── Tick-sync helpers ───────────────────────────────────────────────────── */

// Parse "HH:MM" or "HH:MM:SS" (UTC) into a time_t using today's UTC date.
// If seconds are omitted, falls back to the current system clock's second.
// Returns 0 on failure.
static time_t parseUtcTime(const char* str)
{
    int h = 0, m = 0, s = -1;
    if (sscanf(str, "%d:%d:%d", &h, &m, &s) < 2
        || h < 0 || h > 23 || m < 0 || m > 59 || (s >= 0 && s > 59))
        return 0;
    time_t now = std::time(nullptr);
    struct tm t = {};
#ifdef _WIN32
    gmtime_s(&t, &now);
#else
    gmtime_r(&now, &t);
#endif
    t.tm_hour = h;
    t.tm_min  = m;
    t.tm_sec  = (s >= 0) ? s : (int)(now % 60); // fall back to system seconds
#ifdef _WIN32
    return _mkgmtime(&t);
#else
    return timegm(&t);
#endif
}

static constexpr int kTickDisplayLines = 3;

// Redraw the tick-sync 3-line status panel.
static void drawTickDisplay(bool first, float sig, long long freqKhz,
                            const TickSnap& ts)
{
    if (!first)
        printf("\033[%dA\r", kTickDisplayLines);

    float snrDb = (sig < 0.9999f) ? -10.0f * log10f(1.0f - sig) : 30.0f;
    printf("\033[1mskyclock " SC_VERSION "\033[0m   ");
    printBar(sig, snrDb);
    printf(" SNR:%4.1f dB   %lld kHz\033[K\n", snrDb, freqKhz);

    if (!ts.hasAnchor) {
        printf("\033[33mNo ticks detected yet...\033[0m\033[K\n");
        printf("\033[33mSearching for WWV 1 kHz ticks...\033[0m\033[K\n");
    } else {
        // Line 2: current estimated UTC
        double secSince = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ts.lastTick).count();
        time_t estNow = ts.lastTickUtc + (time_t)secSince;
        struct tm t = {};
#ifdef _WIN32
        gmtime_s(&t, &estNow);
#else
        gmtime_r(&estNow, &t);
#endif
        const char* bold  = ts.clockSet ? "\033[1m" : "";
        const char* label = ts.useSystemAnchor
                          ? "  \033[2m(system-clock anchor)\033[0m" : "";
        printf("%s%04d-%02d-%02d %02d:%02d:%02d UTC\033[0m"
               "  [ticks: %d]%s\033[K\n",
               bold,
               t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
               t.tm_hour, t.tm_min, t.tm_sec,
               ts.tickCount, label);

        // Line 3: countdown or locked status
        if (ts.clockSet) {
            struct tm tb = {};
            time_t bu = ts.boundaryUtc;
#ifdef _WIN32
            gmtime_s(&tb, &bu);
#else
            gmtime_r(&bu, &tb);
#endif
            printf("\033[1m\033[32mLOCKED\033[0m"
                   "  — clock set to %02d:%02d:%02d UTC"
                   " (tick-sync)\033[K\n",
                   tb.tm_hour, tb.tm_min, tb.tm_sec);
        } else if (ts.boundaryFound) {
            // Compute the precise UTC the clock would have been set to right now
            // (boundary UTC + elapsed since tick).  Updates every redraw so the
            // user can see the running timestamp even without root privileges.
            auto elaps   = std::chrono::steady_clock::now() - ts.pendingTick;
            auto wh      = std::chrono::system_clock::from_time_t(ts.boundaryUtc)
                         + std::chrono::duration_cast<
                               std::chrono::system_clock::duration>(elaps);
            time_t whSec = std::chrono::system_clock::to_time_t(wh);
            int    whMs  = (int)(std::chrono::duration_cast<std::chrono::milliseconds>(
                               wh.time_since_epoch()).count() % 1000);
            struct tm tw = {};
#ifdef _WIN32
            gmtime_s(&tw, &whSec);
#else
            gmtime_r(&whSec, &tw);
#endif
            printf("\033[33mWould set: %02d:%02d:%02d.%03d UTC\033[0m"
                   "  (run as root, or enable setSystemClock)\033[K\n",
                   tw.tm_hour, tw.tm_min, tw.tm_sec, whMs);
        } else {
            int secsToNext = (int)(60 - ts.lastTickUtc % 60);
            if (secsToNext == 60) secsToNext = 0;
            time_t nxt = (ts.lastTickUtc / 60 + 1) * 60;
            struct tm tnm = {};
#ifdef _WIN32
            gmtime_s(&tnm, &nxt);
#else
            gmtime_r(&nxt, &tnm);
#endif
            const char* need = (ts.tickCount < 3) ? "need 3 ticks" : "waiting...";
            printf("Next minute: %02d:%02d:00 UTC  (%d s)  [%s]\033[K\n",
                   tnm.tm_hour, tnm.tm_min, secsToNext, need);
        }
    }
    fflush(stdout);
}

/* ── Help ────────────────────────────────────────────────────────────────── */
static void printHelp(const char* argv0)
{
    printf("skyclock %s — WWV time code decoder\n\n", SC_VERSION);
    printf("Usage: %s [OPTIONS]\n\n", argv0);
    printf("  --device <name>        Use named PortAudio audio input device\n");
    printf("  --file <path>          Fast decode from audio file (debug mode)\n");
    printf("  --file <path> --realtime\n");
    printf("                         Decode audio file at real-time speed\n");
    printf("                         (simulates a live radio; shows status panel)\n");
    printf("  --time <HH:MM[:SS]>    Current UTC time (enables tick-sync clock set)\n");
    printf("                         Include seconds for best accuracy.\n");
    printf("                         Omit seconds to use the system clock's second.\n");
    printf("  --full-decode          Full BCD time-code decode mode (requires a clean,\n");
    printf("                         low-noise signal and two consecutive clean minutes)\n");
    printf("                         --== NOTE THAT THIS FUNCTION IS CURRENTLY BROKEN ==--\n");
    printf("  --mode <mode>          Override radio mode from config (e.g. AM, USB, LSB, FM)\n");
    printf("  --iq-offset <hz>       I/Q envelope demodulation offset in Hz (default: 0).\n");
    printf("                         Use with a radio tuned <hz> below WWV in USB mode.\n");
    printf("                         Shifts the 100 Hz subcarrier and 1 kHz tick up by\n");
    printf("                         <hz> into the radio's audio passband, then recovers\n");
    printf("                         them via I/Q demodulation. Typical: --iq-offset 500\n");
    printf("  --rigctld              Connect to rigctld to tune the radio\n");
    printf("  --rigctld-host <host>  rigctld hostname or IP  (default: localhost)\n");
    printf("  --rigctld-port <port>  rigctld TCP port         (default: 4532)\n");
    printf("  --freq-khz <khz>       Override WWV receive frequency in kHz\n");
    printf("  --list-devices         List available audio input devices and exit\n");
    printf("  --list-rigs            List available hamlib rig models and exit\n");
    printf("  --version              Print version and exit\n");
    printf("  --help                 This help\n\n");
    printf("Configuration: ~/.skyclock/settings.json\n");
    printf("  Key settings:\n");
    printf("    rigctldEnabled     true/false  connect via rigctld (default: false)\n");
    printf("    rigctldHost        rigctld hostname or IP  (default: \"localhost\")\n");
    printf("    rigctldPort        rigctld TCP port        (default: 4532)\n");
    printf("    rigEnabled         true/false  direct hamlib (default: false)\n");
    printf("    rigModel           hamlib model number\n");
    printf("    rigPort            serial port path\n");
    printf("    freqKhz            WWV frequency in kHz (2500/5000/10000/15000/20000)\n");
    printf("    rigMode            radio mode string, e.g. \"AM\"\n");
    printf("    audioDevice        audio device name substring (empty = default)\n");
    printf("    setSystemClock     true/false (requires root/admin)\n");
    printf("    minConfidence      consecutive frames required before setting clock\n\n");
    printf("WWV frequencies (kHz): 2500, 5000, 10000, 15000, 20000\n");
    printf("Recommended: tune to 10000 kHz, mode AM.\n");
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(int argc, char* argv[])
{
    bool listDev  = false;
    bool listRigs = false;
    bool realtime = false;
    bool fullDecode = false;          // --full-decode: BCD mode instead of tick-sync
    std::string deviceName;
    std::string filePath;
    std::string timeArg;              // --time HH:MM[:SS]
    bool        rigctldOverride = false;  // --rigctld flag seen on CLI
    std::string rigctldHostArg;           // --rigctld-host <h>
    int         rigctldPortArg  = 0;      // --rigctld-port <p>
    long long   freqKhzArg      = 0;      // --freq-khz <f>  (0 = use settings)
    float       iqOffsetHz      = 0.0f;   // --iq-offset <hz> (0 = disabled)
    std::string modeArg;                  // --mode <mode>    (empty = use config)

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printHelp(argv[0]); return 0;
        } else if (!strcmp(argv[i], "--version")) {
            printf("skyclock %s\n", SC_VERSION); return 0;
        } else if (!strcmp(argv[i], "--list-devices")) {
            listDev = true;
        } else if (!strcmp(argv[i], "--list-rigs")) {
            listRigs = true;
        } else if (!strcmp(argv[i], "--device") && i + 1 < argc) {
            deviceName = argv[++i];
        } else if (!strcmp(argv[i], "--file") && i + 1 < argc) {
            filePath = argv[++i];
        } else if (!strcmp(argv[i], "--realtime")) {
            realtime = true;
        } else if (!strcmp(argv[i], "--full-decode")) {
            fullDecode = true;
        } else if (!strcmp(argv[i], "--time") && i + 1 < argc) {
            timeArg = argv[++i];
        } else if (!strcmp(argv[i], "--rigctld")) {
            rigctldOverride = true;
        } else if (!strcmp(argv[i], "--rigctld-host") && i + 1 < argc) {
            rigctldHostArg = argv[++i];
        } else if (!strcmp(argv[i], "--rigctld-port") && i + 1 < argc) {
            rigctldPortArg = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--freq-khz") && i + 1 < argc) {
            freqKhzArg = atoll(argv[++i]);
        } else if (!strcmp(argv[i], "--iq-offset") && i + 1 < argc) {
            iqOffsetHz = (float)atof(argv[++i]);
        } else if (!strcmp(argv[i], "--mode") && i + 1 < argc) {
            modeArg = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 1;
        }
    }

    if (listDev)  { listDevices(); return 0; }

    if (listRigs) {
        auto rigs = RigControl::availableRigs();
        if (rigs.empty()) {
            printf("No rigs found (hamlib not compiled in, or no backends loaded).\n");
        } else {
            printf("Available hamlib rig models:\n");
            for (auto& r : rigs)
                printf("  %s\n", r.c_str());
        }
        return 0;
    }

    Settings& cfg = Settings::instance();

    // Apply CLI overrides (these win over settings.json values).
    if (rigctldOverride)      cfg.rigctldEnabled = true;
    if (!rigctldHostArg.empty()) cfg.rigctldHost = rigctldHostArg;
    if (rigctldPortArg > 0)   cfg.rigctldPort   = rigctldPortArg;
    if (freqKhzArg > 0)       cfg.freqKhz       = freqKhzArg;

    cfg.save();

    bool isTty = isatty(fileno(stdout)) != 0;
    if (isTty) enableAnsiConsole();

    memset(g_bitRow, ' ', 60);
    g_bitRow[60] = '\0';

    printf("skyclock %s\n", SC_VERSION);

    // ── Fast file mode (--file, no --realtime) ─────────────────────────────
    // Decodes as fast as possible; prints bits and status as plain scrolling
    // text.  Useful for debugging signal files.
    if (!filePath.empty() && !realtime) {
        static constexpr float kSR    = 48000.0f;
        static constexpr int   kChunk = 512;
        // Status line every ~2 s of audio time
        static constexpr int   kStatusEvery = (int)(kSR * 2.0f / kChunk);

        WwvDecoder decoder(kSR);
        g_decoder = &decoder;
        // File mode: recording may be from a different date — skip time check.
        // Also lower the consecutive-frame requirement to 1 so single-frame
        // decodes are visible (useful for signal analysis).
        decoder.setTimePlausibilityCheck(false);
        decoder.setMinConsecutiveFrames(1);
        if (iqOffsetHz > 0.0f) {
            decoder.setIqOffset(iqOffsetHz);
            printf("I/Q offset: %.0f Hz (SSB carrier at %.0f Hz in audio)\n",
                   iqOffsetHz, iqOffsetHz);
        }

        decoder.setFrameCallback([&](const WwvTime& frame) {
            std::lock_guard<std::mutex> lk(g_frameMutex);
            g_latestFrame  = frame;
            g_frameUpdated = true;
        });

        bool needNewline = false;
        decoder.setBitCallback([&needNewline](int type, int ms) {
            if (needNewline) { printf("\n"); needNewline = false; }
            if (type == 0)      printf("0(%d) ", ms);
            else if (type == 1) printf("1(%d) ", ms);
            else if (type == 2) printf("|  (%d ms)\n", ms);
            else                printf("? ");
            fflush(stdout);
        });

        std::string cmd = std::string("ffmpeg -i \"") + filePath +
                          "\" -f f32le -ar 48000 -ac 1 -loglevel quiet -";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            fprintf(stderr, "Failed to launch ffmpeg: %s\n", filePath.c_str());
            return 1;
        }
        printf("File (fast): %s\nPress Ctrl+C to stop.\n\n", filePath.c_str());

        signal(SIGINT, on_signal);
#ifndef _WIN32
        signal(SIGTERM, on_signal);
#endif

        float buf[kChunk];
        int   statusTick = 0;

        while (g_running.load()) {
            size_t got = fread(buf, sizeof(float), kChunk, pipe);
            if (got == 0) break;
            decoder.pushSamples(buf, (int)got);

            WwvTime frame;
            bool updated = false;
            {
                std::lock_guard<std::mutex> lk(g_frameMutex);
                if (g_frameUpdated) {
                    frame = g_latestFrame; updated = true; g_frameUpdated = false;
                }
            }

            time_t utc = decoder.currentUtc();
            float  sig = decoder.signalLevel();

            if (updated) {
                if (isTty && needNewline) { printf("\n"); needNewline = false; }
                struct tm t;
#ifdef _WIN32
                gmtime_s(&t, &utc);
#else
                gmtime_r(&utc, &t);
#endif
                printf("\n==> %04d-%02d-%02d %02d:%02d:%02d UTC"
                       "  Day %03d  UT1%+.1fs  [conf:%d]\n\n",
                       t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                       t.tm_hour, t.tm_min, t.tm_sec,
                       frame.dayOfYear, frame.ut1Seconds, frame.confidence);
                needNewline = false;
            } else if (utc == 0 && ++statusTick % kStatusEvery == 0) {
                if (needNewline) { printf("\n"); needNewline = false; }
                // Show partial BCD digits as they stabilize.
                auto dc = [&](int fi) -> char {
                    const DigitField& d = decoder.digitField(fi);
                    return (d.value >= 0 && d.streak >= WwvDecoder::kMinStreak)
                           ? (char)('0' + d.value) : '-';
                };
                printf("-- Signal: %3.0f%%  Bits: %3d  BCD: %c%c:%c%c  Day %c%c%c  Year 20%c%c --\n",
                       sig * 100.0f, decoder.bitsReceived(),
                       dc(2), dc(3), dc(0), dc(1),
                       dc(4), dc(5), dc(6), dc(7), dc(8));
            }
        }

        if (isTty && needNewline) printf("\n");
        pclose(pipe);
        return 0;
    }

    // ── Real-time file mode (--file --realtime) ────────────────────────────
    // Feeds audio at 48 kHz pace to simulate a live radio, with the same
    // status panel as live mode.
    if (!filePath.empty() && realtime) {
        static constexpr float kSR        = 48000.0f;
        static constexpr int   kRtChunk   = 480;   // 10 ms per chunk at 48 kHz
        static constexpr int   kRtSleepMs = 10;
        static constexpr int   kDrawEvery = 20;    // redraw every 200 ms

        WwvDecoder decoder(kSR);
        g_decoder = &decoder;
        // File mode: recording may be from a different date — skip time check.
        // Lower consecutive-frame requirement to 1 for single-frame visibility.
        decoder.setTimePlausibilityCheck(false);
        decoder.setMinConsecutiveFrames(1);
        if (iqOffsetHz > 0.0f) {
            decoder.setIqOffset(iqOffsetHz);
            printf("I/Q offset: %.0f Hz (SSB carrier at %.0f Hz in audio)\n",
                   iqOffsetHz, iqOffsetHz);
        }

        decoder.setFrameCallback([&](const WwvTime& frame) {
            std::lock_guard<std::mutex> lk(g_frameMutex);
            g_latestFrame  = frame;
            g_frameUpdated = true;
        });

        decoder.setBitCallback([](int type, int /*ms*/) {
            appendBitDisplay(type);
        });

        std::string cmd = std::string("ffmpeg -i \"") + filePath +
                          "\" -f f32le -ar 48000 -ac 1 -loglevel quiet -";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            fprintf(stderr, "Failed to launch ffmpeg: %s\n", filePath.c_str());
            return 1;
        }
        printf("File (real-time): %s\nPress Ctrl+C to stop.\n", filePath.c_str());

        signal(SIGINT, on_signal);
#ifndef _WIN32
        signal(SIGTERM, on_signal);
#endif

        float buf[kRtChunk];
        int   chunkCount = 0;
        bool  clockSet   = false;

        // Print initial display block
        drawDisplay(true, 0.0f, 0, WwvTime{}, false, cfg.freqKhz, cfg.minConfidence);

        while (g_running.load()) {
            size_t got = fread(buf, sizeof(float), kRtChunk, pipe);
            if (got == 0) break;
            decoder.pushSamples(buf, (int)got);
            sleepMs(kRtSleepMs);

            if (++chunkCount % kDrawEvery != 0) continue;

            {
                std::lock_guard<std::mutex> lk(g_frameMutex);
                g_frameUpdated = false;
            }

            time_t utc = decoder.currentUtc();
            float  sig = decoder.signalLevel();
            WwvTime f  = decoder.lastFrame();
            g_smoothSig = 0.94f * g_smoothSig + 0.06f * sig;

            drawDisplay(false, g_smoothSig, utc, f, clockSet, cfg.freqKhz, cfg.minConfidence);

            if (cfg.setSystemClock && !clockSet && utc != 0 &&
                f.confidence >= cfg.minConfidence) {
                std::string clockErr;
                if (setSystemClock(decoder.currentUtcPoint(), clockErr))
                    clockSet = true;
                else
                    fprintf(stderr, "\nClock sync failed: %s\n", clockErr.c_str());
                // Redraw immediately to show updated status
                drawDisplay(false, g_smoothSig, utc, f, clockSet, cfg.freqKhz, cfg.minConfidence);
            }
        }

        printf("\n");
        pclose(pipe);
        return 0;
    }

    // ── Live mode (PortAudio) ──────────────────────────────────────────────
    // When I/Q offset mode is active, the radio must be in USB and tuned
    // (iqOffsetHz) below the WWV centre frequency so that the carrier appears
    // at iqOffsetHz in the audio passband.
    long long   tuneHz  = cfg.freqKhz * 1000LL - (long long)iqOffsetHz;
    std::string tuneMode = !modeArg.empty() ? modeArg
                         : (iqOffsetHz > 0.0f) ? "USB"
                         : cfg.rigMode;

    printf("Config: %s\n", cfg.path().c_str());
    if (iqOffsetHz > 0.0f) {
        printf("WWV centre: %lld kHz  I/Q offset: %.0f Hz  → tune to %.4f kHz %s\n",
               cfg.freqKhz, iqOffsetHz, tuneHz / 1000.0, tuneMode.c_str());
    } else {
        printf("Frequency: %lld kHz  Mode: %s\n", cfg.freqKhz, cfg.rigMode.c_str());
    }

    // ── Hamlib radio setup ─────────────────────────────────────────────────
    RigControl rig;
    rig.setErrorCallback([](const std::string& e) {
        fprintf(stderr, "Radio: %s\n", e.c_str());
    });

    if (cfg.rigctldEnabled) {
        // Connect via rigctld (hamlib network daemon).
        printf("Radio: connecting to rigctld at %s:%d...\n",
               cfg.rigctldHost.c_str(), cfg.rigctldPort);
        if (rig.connectRigctld(cfg.rigctldHost, cfg.rigctldPort)) {
            printf("Radio: connected to rigctld\n");
            rig.setMode(tuneMode);
            rig.setFreqHz(tuneHz);
            printf("Radio: tuned to %.4f kHz %s\n",
                   tuneHz / 1000.0, tuneMode.c_str());
        }
    } else if (cfg.rigEnabled) {
        // Connect via direct hamlib (serial/USB).
        RigConfig rcfg;
        rcfg.model     = cfg.rigModel;
        rcfg.port      = cfg.rigPort;
        rcfg.baudRate  = cfg.rigBaud;
        rcfg.dataBits  = cfg.rigDataBits;
        rcfg.stopBits  = cfg.rigStopBits;
        rcfg.parity    = cfg.rigParity;
        rcfg.handshake = cfg.rigHandshake;
        rcfg.dtrState  = cfg.rigDtrState;
        rcfg.rtsState  = cfg.rigRtsState;

        if (rig.connect(rcfg)) {
            printf("Radio: connected (model %d on %s)\n",
                   cfg.rigModel, cfg.rigPort.c_str());
            rig.setMode(tuneMode);
            rig.setFreqHz(tuneHz);
            printf("Radio: tuned to %.4f kHz %s\n",
                   tuneHz / 1000.0, tuneMode.c_str());
        }
    } else {
        printf("Radio: not configured (tune manually to %.4f kHz %s)\n",
               tuneHz / 1000.0, tuneMode.c_str());
    }

    // ── PortAudio setup ────────────────────────────────────────────────────
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        fprintf(stderr, "PortAudio init failed: %s\n", Pa_GetErrorText(err));
        return 1;
    }

    std::string devMatch = deviceName.empty() ? cfg.audioDevice : deviceName;

    PaDeviceIndex devIdx = Pa_GetDefaultInputDevice();
    if (!devMatch.empty()) {
        int n = Pa_GetDeviceCount();
        for (int i = 0; i < n; ++i) {
            const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
            if (info && info->maxInputChannels > 0 &&
                strstr(info->name, devMatch.c_str())) {
                devIdx = i;
                break;
            }
        }
    }
    if (devIdx == paNoDevice) {
        fprintf(stderr, "No suitable audio input device found\n");
        Pa_Terminate(); return 1;
    }

    const PaDeviceInfo* devInfo = Pa_GetDeviceInfo(devIdx);
    float sampleRate = devInfo ? (float)devInfo->defaultSampleRate : 48000.0f;

    printf("Audio: %s @ %.0f Hz\n",
           devInfo ? devInfo->name : "default", sampleRate);

    // ── Decoder ────────────────────────────────────────────────────────────
    WwvDecoder decoder(sampleRate);
    g_decoder = &decoder;
    if (iqOffsetHz > 0.0f) {
        decoder.setIqOffset(iqOffsetHz);
        printf("I/Q offset: %.0f Hz  (tune radio %.3f kHz below WWV in USB mode)\n",
               iqOffsetHz, iqOffsetHz / 1000.0);
    }

    // ── Mode-specific setup ────────────────────────────────────────────────
    if (fullDecode) {
        // BCD decode mode: full time-code decode from the 100 Hz subcarrier.
        decoder.setFrameCallback([&](const WwvTime& frame) {
            std::lock_guard<std::mutex> lk(g_frameMutex);
            g_latestFrame  = frame;
            g_frameUpdated = true;
        });
        decoder.setBitCallback([](int type, int /*ms*/) {
            appendBitDisplay(type);
        });
    } else {
        // Tick-sync mode: use 1 kHz tick rising edges to identify the minute
        // boundary, then set the clock with sub-second precision.
        //
        // UTC anchor: from --time HH:MM[:SS] if provided, otherwise from the
        // system clock at the moment of the first tick (works if the system
        // clock is within ±29 s of real UTC).
        time_t anchorUtc = 0;
        if (!timeArg.empty()) {
            anchorUtc = parseUtcTime(timeArg.c_str());
            if (anchorUtc == 0) {
                fprintf(stderr,
                        "Invalid --time value '%s'. Use HH:MM or HH:MM:SS (UTC).\n",
                        timeArg.c_str());
                Pa_Terminate(); return 1;
            }
            printf("Tick-sync anchor: %s UTC\n", timeArg.c_str());
        } else {
            printf("Tick-sync mode — no --time given; anchoring to system clock.\n"
                   "  For accuracy, provide --time HH:MM:SS (current UTC).\n");
        }

        decoder.setTickCallback(
            [anchorUtc](std::chrono::steady_clock::time_point tickTime) {
                std::lock_guard<std::mutex> lk(g_tickMutex);
                TickSnap& ts = g_tickSnap;
                if (ts.clockSet || ts.boundaryFound) return;

                if (!ts.hasAnchor) {
                    ts.anchorSteady    = tickTime;
                    ts.useSystemAnchor = (anchorUtc == 0);
                    ts.anchorUtc       = (anchorUtc != 0)
                                       ? anchorUtc
                                       : std::time(nullptr);
                    ts.hasAnchor = true;
                }

                double elapsed = std::chrono::duration<double>(
                    tickTime - ts.anchorSteady).count();
                time_t estUtc = ts.anchorUtc +
                                static_cast<time_t>(std::round(elapsed));

                ++ts.tickCount;
                ts.lastTick    = tickTime;
                ts.lastTickUtc = estUtc;

                // After ≥3 ticks, trigger on the first minute boundary.
                if (ts.tickCount >= 3 && estUtc % 60 == 0) {
                    ts.pendingSet    = true;
                    ts.boundaryFound = true;
                    ts.pendingUtc    = estUtc;
                    ts.boundaryUtc   = estUtc;
                    ts.pendingTick   = tickTime;
                }
            });
    }

    // ── Open audio stream ──────────────────────────────────────────────────
    PaStreamParameters inParams{};
    inParams.device           = devIdx;
    inParams.channelCount     = 1;
    inParams.sampleFormat     = paFloat32;
    inParams.suggestedLatency = devInfo ? devInfo->defaultLowInputLatency : 0.1;

    PaStream* stream = nullptr;
    err = Pa_OpenStream(&stream, &inParams, nullptr,
                        sampleRate, 512, paClipOff, pa_callback, nullptr);
    if (err != paNoError) {
        fprintf(stderr, "Pa_OpenStream failed: %s\n", Pa_GetErrorText(err));
        Pa_Terminate(); return 1;
    }

    // Compensate for audio pipeline latency so that decoder timestamps reflect
    // when sound entered the microphone, not when samples arrived in the callback.
    if (const PaStreamInfo* si = Pa_GetStreamInfo(stream)) {
        decoder.setAudioLatency(si->inputLatency);
        printf("Audio latency: %.1f ms (compensated)\n", si->inputLatency * 1000.0);
    }

    signal(SIGINT, on_signal);
#ifndef _WIN32
    signal(SIGTERM, on_signal);
#endif

    Pa_StartStream(stream);
    printf("Listening. Press Ctrl+C to stop.\n");

    // ── Main display loop ──────────────────────────────────────────────────
    if (fullDecode) {
        // BCD decode display loop
        bool clockSet = false;
        drawDisplay(true, 0.0f, 0, WwvTime{}, false, cfg.freqKhz, cfg.minConfidence);

        while (g_running.load()) {
            Pa_Sleep(200);

            {
                std::lock_guard<std::mutex> lk(g_frameMutex);
                g_frameUpdated = false;  // consumed
            }

            time_t  utc = decoder.currentUtc();
            float   sig = decoder.signalLevel();
            WwvTime f   = decoder.lastFrame();
            g_smoothSig = 0.94f * g_smoothSig + 0.06f * sig;

            drawDisplay(false, g_smoothSig, utc, f, clockSet,
                        cfg.freqKhz, cfg.minConfidence);

            if (cfg.setSystemClock && !clockSet && utc != 0 &&
                f.confidence >= cfg.minConfidence) {
                std::string clockErr;
                if (setSystemClock(decoder.currentUtcPoint(), clockErr))
                    clockSet = true;
                else
                    fprintf(stderr, "\nClock sync failed: %s\n", clockErr.c_str());
                drawDisplay(false, g_smoothSig, utc, f, clockSet,
                            cfg.freqKhz, cfg.minConfidence);
            }
        }
    } else {
        // Tick-sync display loop
        drawTickDisplay(true, 0.0f, cfg.freqKhz, TickSnap{});

        while (g_running.load()) {
            Pa_Sleep(200);

            // Drain any pending clock-set request (minute boundary identified
            // by the audio thread).
            bool   doSet   = false;
            time_t setUtc  = 0;
            std::chrono::steady_clock::time_point setTick;
            {
                std::lock_guard<std::mutex> lk(g_tickMutex);
                if (g_tickSnap.pendingSet && !g_tickSnap.clockSet) {
                    doSet              = true;
                    setUtc             = g_tickSnap.pendingUtc;
                    setTick            = g_tickSnap.pendingTick;
                    g_tickSnap.pendingSet = false;
                }
            }
            if (doSet) {
                if (cfg.setSystemClock) {
                    auto elapsed = std::chrono::steady_clock::now() - setTick;
                    auto utcNow  = std::chrono::system_clock::from_time_t(setUtc)
                                 + std::chrono::duration_cast<
                                       std::chrono::system_clock::duration>(elapsed);
                    std::string clockErr;
                    if (setSystemClock(utcNow, clockErr)) {
                        {
                            std::lock_guard<std::mutex> lk(g_tickMutex);
                            g_tickSnap.clockSet = true;
                        }
                        // Redraw once to show LOCKED, then exit.
                        TickSnap tsFinal;
                        {
                            std::lock_guard<std::mutex> lk(g_tickMutex);
                            tsFinal = g_tickSnap;
                        }
                        g_smoothSig = 0.94f * g_smoothSig + 0.06f * decoder.signalLevel();
                        drawTickDisplay(false, g_smoothSig, cfg.freqKhz, tsFinal);
                        printf("\n");
                        g_running.store(false);
                        break;
                    } else {
                        fprintf(stderr, "\nClock sync failed: %s\n", clockErr.c_str());
                    }
                } else {
                    // setSystemClock disabled — boundary is already recorded;
                    // display will show the running "would set to" timestamp.
                }
            }

            // Prediction-based boundary fallback: if ≥3 ticks have been seen
            // but the exact :00 tick was missed (signal fade), fire the clock
            // set once the predicted minute boundary has passed by 500 ms.
            // The 500 ms grace period lets a real :00 tick arrive first (more
            // accurate); only the prediction fires if it doesn't come.
            {
                std::lock_guard<std::mutex> lk(g_tickMutex);
                TickSnap& ts = g_tickSnap;
                if (ts.tickCount >= 3 && !ts.boundaryFound && ts.hasAnchor) {
                    int secsIn   = static_cast<int>(ts.lastTickUtc % 60);
                    int secsLeft = (secsIn == 0) ? 60 : (60 - secsIn);
                    auto predBoundary = ts.lastTick +
                                        std::chrono::seconds(secsLeft);
                    // Only act on a recent-enough tick (≤30 s old at boundary)
                    // to keep the UTC estimate valid.
                    auto tickAge = predBoundary - ts.lastTick;
                    if (tickAge <= std::chrono::seconds(30) &&
                        std::chrono::steady_clock::now() >=
                            predBoundary + std::chrono::milliseconds(500)) {
                        ts.boundaryFound = true;
                        ts.boundaryUtc   = ts.lastTickUtc + secsLeft;
                        ts.boundaryUtc  -= ts.boundaryUtc % 60;
                        ts.pendingTick   = predBoundary;
                        ts.pendingSet    = true;
                        ts.pendingUtc    = ts.boundaryUtc;
                    }
                }
            }

            float sig = decoder.signalLevel();
            g_smoothSig = 0.94f * g_smoothSig + 0.06f * sig;

            TickSnap tsCopy;
            {
                std::lock_guard<std::mutex> lk(g_tickMutex);
                tsCopy = g_tickSnap;
            }
            drawTickDisplay(false, g_smoothSig, cfg.freqKhz, tsCopy);
        }
    }

    // ── Cleanup ────────────────────────────────────────────────────────────
    printf("\n");
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    rig.disconnect();
    return 0;
}
