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

/* ── TTY display state ───────────────────────────────────────────────────── */
// Sliding window of the last 60 classified bits as printable chars.
// Written only by the audio-thread bit callback; memmove of 59 bytes and
// single char writes are safe without a mutex on all supported platforms.
static char  g_bitRow[61];     // display chars (0, 1, |, ?) + NUL; space-padded
static int   g_bitPos = 0;     // total bits appended (for count label)
static float g_smoothSig = 0.0f; // slow EMA of signal level (~3 s time constant)

static constexpr int kDisplayLines = 3;

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

// Print a 20-char signal-strength bar using Unicode block elements and ANSI colour.
static void printBar(float sig)
{
    int filled = (int)(sig * 20.0f + 0.5f);
    if (filled > 20) filled = 20;
    if (filled > 0) printf("\033[32m");   // green for filled portion
    for (int i = 0; i < filled; ++i)
        printf("\xe2\x96\x88");           // █
    printf("\033[2m");                    // dim for empty portion
    for (int i = filled; i < 20; ++i)
        printf("\xe2\x96\x91");           // ░
    printf("\033[0m");
}

// Redraw (or initially draw) the 3-line status panel.
// Uses \033[NA to move the cursor up and overwrite previous content.
// Each line ends with \033[K to clear any leftover chars from a longer previous value.
static void drawDisplay(bool first, float sig, time_t utc, const WwvTime& f,
                        bool clockSet, long long freqKhz, int minConf)
{
    if (!first)
        printf("\033[%dA\r", kDisplayLines);

    // Line 1 — title, signal bar, frequency
    printf("\033[1mskyclock " SC_VERSION "\033[0m   ");
    printBar(sig);
    printf(" %3.0f%%   %lld kHz\033[K\n", sig * 100.0f, freqKhz);

    // Line 2 — bit stream (searching) or decoded UTC time (locked)
    if (utc == 0) {
        printf("\033[33m[%s]\033[0m  %d bit%s\033[K\n",
               g_bitRow, g_bitPos, g_bitPos == 1 ? "" : "s");
    } else {
        struct tm t;
#ifdef _WIN32
        gmtime_s(&t, &utc);
#else
        gmtime_r(&utc, &t);
#endif
        printf("\033[1m%04d-%02d-%02d %02d:%02d:%02d UTC\033[0m"
               "  Day %03d  UT1%+.1fs  [conf:%d]\033[K\n",
               t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
               t.tm_hour, t.tm_min, t.tm_sec,
               f.dayOfYear, f.ut1Seconds, f.confidence);
    }

    // Line 3 — status
    if (utc == 0) {
        printf("\033[33mSearching for WWV signal...\033[0m\033[K\n");
    } else if (clockSet) {
        printf("\033[1m\033[32mLOCKED\033[0m  — clock set successfully\033[K\n");
    } else {
        printf("\033[1m\033[32mLOCKED\033[0m"
               "  [conf: %d / %d needed]\033[K\n", f.confidence, minConf);
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

/* ── Help ────────────────────────────────────────────────────────────────── */
static void printHelp(const char* argv0)
{
    printf("skyclock %s — WWV time code decoder\n\n", SC_VERSION);
    printf("Usage: %s [OPTIONS]\n\n", argv0);
    printf("  --device <name>      Use named PortAudio audio input device\n");
    printf("  --file <path>        Fast decode from audio file (debug mode)\n");
    printf("  --file <path> --realtime\n");
    printf("                       Decode audio file at real-time speed\n");
    printf("                       (simulates a live radio; shows status panel)\n");
    printf("  --list-devices       List available audio input devices and exit\n");
    printf("  --list-rigs          List available hamlib rig models and exit\n");
    printf("  --version            Print version and exit\n");
    printf("  --help               This help\n\n");
    printf("Configuration: ~/.skyclock/settings.json\n");
    printf("  Key settings:\n");
    printf("    rigEnabled         true/false  (default: false)\n");
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
    std::string deviceName;
    std::string filePath;

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
                printf("-- Signal: %3.0f%%  Bits: %d --\n",
                       sig * 100.0f, decoder.bitsReceived());
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
    printf("Config: %s\n", cfg.path().c_str());
    printf("Frequency: %lld kHz  Mode: %s\n", cfg.freqKhz, cfg.rigMode.c_str());

    // ── Hamlib radio setup ─────────────────────────────────────────────────
    RigControl rig;
    rig.setErrorCallback([](const std::string& e) {
        fprintf(stderr, "Radio: %s\n", e.c_str());
    });

    if (cfg.rigEnabled) {
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
            rig.setFreqHz(cfg.freqKhz * 1000LL);
            rig.setMode(cfg.rigMode);
            printf("Radio: tuned to %lld kHz %s\n",
                   cfg.freqKhz, cfg.rigMode.c_str());
        }
    } else {
        printf("Radio: not configured (tune manually to %lld kHz %s)\n",
               cfg.freqKhz, cfg.rigMode.c_str());
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

    decoder.setFrameCallback([&](const WwvTime& frame) {
        std::lock_guard<std::mutex> lk(g_frameMutex);
        g_latestFrame  = frame;
        g_frameUpdated = true;
    });

    // Bit callback: update the sliding display row (audio thread).
    decoder.setBitCallback([](int type, int /*ms*/) {
        appendBitDisplay(type);
    });

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
    bool clockSet = false;

    // Print initial display block
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

        drawDisplay(false, g_smoothSig, utc, f, clockSet, cfg.freqKhz, cfg.minConfidence);

        if (cfg.setSystemClock && !clockSet && utc != 0 &&
            f.confidence >= cfg.minConfidence) {
            std::string clockErr;
            if (setSystemClock(decoder.currentUtcPoint(), clockErr))
                clockSet = true;
            else
                fprintf(stderr, "\nClock sync failed: %s\n", clockErr.c_str());
            // Redraw immediately to reflect updated clock-set status
            drawDisplay(false, g_smoothSig, utc, f, clockSet, cfg.freqKhz, cfg.minConfidence);
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
