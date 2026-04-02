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
 *   skyclock                    # decode from default audio device
 *   skyclock --device <name>    # use named PortAudio device
 *   skyclock --file <path>      # decode from audio file via ffmpeg
 *   skyclock --list-devices     # list audio devices and exit
 *   skyclock --list-rigs        # list hamlib rig models and exit
 *   skyclock --version          # print version and exit
 *   skyclock --help             # this help
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
#  define isatty _isatty
#  define fileno _fileno
#else
#  include <unistd.h>
#endif

#ifndef SC_VERSION
#  define SC_VERSION "0.1.0-ALPHA"
#endif

/* ── Signal handling ─────────────────────────────────────────────────────── */
static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running.store(false); }

/* ── Shared decoder state (audio thread → main thread) ──────────────────── */
static WwvDecoder* g_decoder = nullptr;
static std::mutex  g_frameMutex;
static WwvTime     g_latestFrame;
static bool        g_frameUpdated = false;

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

/* ── Time display helpers ────────────────────────────────────────────────── */
static void printTime(time_t utc, const WwvTime& f, float sigLevel, bool tty)
{
    struct tm t;
#ifdef _WIN32
    gmtime_s(&t, &utc);
#else
    gmtime_r(&utc, &t);
#endif
    if (tty) printf("\r");
    printf("%04d-%02d-%02d %02d:%02d:%02d UTC  "
           "Day %03d  UT1%+.1fs  Signal: %3.0f%%  [conf: %d]   ",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec,
           f.dayOfYear,
           f.ut1Seconds,
           sigLevel * 100.0f,
           f.confidence);
    if (!tty) printf("\n");
    fflush(stdout);
}

/* ── Help ────────────────────────────────────────────────────────────────── */
static void printHelp(const char* argv0)
{
    printf("skyclock %s — WWV time code decoder\n\n", SC_VERSION);
    printf("Usage: %s [OPTIONS]\n\n", argv0);
    printf("  --device <name>    Use named PortAudio audio input device\n");
    printf("  --file <path>      Decode from audio file via ffmpeg (no radio/PortAudio)\n");
    printf("  --list-devices     List available audio input devices and exit\n");
    printf("  --list-rigs        List available hamlib rig models and exit\n");
    printf("  --version          Print version and exit\n");
    printf("  --help             This help\n\n");
    printf("Configuration: ~/.skyclock/settings.json\n");
    printf("  Key settings:\n");
    printf("    rigEnabled       true/false  (default: false)\n");
    printf("    rigModel         hamlib model number\n");
    printf("    rigPort          serial port path\n");
    printf("    freqKhz          WWV frequency in kHz (2500/5000/10000/15000/20000)\n");
    printf("    rigMode          radio mode string, e.g. \"AM\"\n");
    printf("    audioDevice      audio device name substring (empty = default)\n");
    printf("    setSystemClock   true/false (requires root/admin)\n");
    printf("    minConfidence    consecutive frames required before setting clock\n\n");
    printf("WWV frequencies (kHz): 2500, 5000, 10000, 15000, 20000\n");
    printf("Recommended: tune to 10000 kHz, mode AM.\n");
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(int argc, char* argv[])
{
    bool listDev  = false;
    bool listRigs = false;
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

    // Load settings (creates default file on first run)
    Settings& cfg = Settings::instance();
    cfg.save(); // write defaults if file missing

    bool isTty = isatty(fileno(stdout)) != 0;

    printf("skyclock %s\n", SC_VERSION);

    // ── File mode (--file) ─────────────────────────────────────────────────
    // Bypass PortAudio and radio; decode directly from an audio file via ffmpeg.
    if (!filePath.empty()) {
        static constexpr float kFileSampleRate = 48000.0f;
        static constexpr int   kChunk = 512;
        // Display update every ~200 ms of audio time
        static constexpr int   kDisplayEvery =
            (int)(kFileSampleRate * 0.2f / kChunk); // ~18 chunks

        WwvDecoder decoder(kFileSampleRate);
        g_decoder = &decoder;

        decoder.setFrameCallback([&](const WwvTime& frame) {
            std::lock_guard<std::mutex> lk(g_frameMutex);
            g_latestFrame  = frame;
            g_frameUpdated = true;
        });

        bool needNewline = false;
        decoder.setBitCallback([&needNewline](int type, int ms) {
            if (needNewline) { printf("\n"); needNewline = false; }
            if (type == 0)      printf(".(%d) ", ms);
            else if (type == 1) printf("#(%d) ", ms);
            else if (type == 2) printf("|  (%d ms)\n", ms);
            else                printf("? ");
            fflush(stdout);
        });

        // ffmpeg decodes any format to raw mono float32 at 48 kHz on stdout.
        // Double-quote the path to handle spaces; paths with embedded " not supported.
        std::string cmd = std::string("ffmpeg -i \"") + filePath +
                          "\" -f f32le -ar 48000 -ac 1 -loglevel quiet -";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            fprintf(stderr, "Failed to launch ffmpeg for: %s\n", filePath.c_str());
            return 1;
        }

        printf("File: %s  (ffmpeg → %.0f Hz mono f32le)\n"
               "Press Ctrl+C to stop.\n\n",
               filePath.c_str(), kFileSampleRate);

        signal(SIGINT,  on_signal);
#ifndef _WIN32
        signal(SIGTERM, on_signal);
#endif

        float buf[kChunk];
        int   displayCounter = 0;
        bool  clockSet = false;
        int   statusTick = 0;

        while (g_running.load()) {
            size_t got = fread(buf, sizeof(float), kChunk, pipe);
            if (got == 0) break; // EOF or error

            decoder.pushSamples(buf, (int)got);

            if (++displayCounter < kDisplayEvery) continue;
            displayCounter = 0;

            WwvTime frame;
            bool updated = false;
            {
                std::lock_guard<std::mutex> lk(g_frameMutex);
                if (g_frameUpdated) {
                    frame          = g_latestFrame;
                    updated        = true;
                    g_frameUpdated = false;
                }
            }
            if (updated && isTty) { printf("\n"); needNewline = false; }

            time_t utc = decoder.currentUtc();
            float  sig = decoder.signalLevel();

            if (utc == 0) {
                if (++statusTick % 10 == 0)
                    printf("\n-- Signal: %3.0f%%  Bits: %d --\n",
                           sig * 100.0f, decoder.bitsReceived());
            } else {
                WwvTime f = decoder.lastFrame();
                printTime(utc, f, sig, isTty);
                needNewline = isTty; // \r was written; next bit needs a newline

                if (cfg.setSystemClock && !clockSet &&
                    f.confidence >= cfg.minConfidence) {
                    std::string clockErr;
                    if (setSystemClock(utc, clockErr)) {
                        printf("\nClock set to %lld UTC\n", (long long)utc);
                        clockSet = true;
                    } else {
                        fprintf(stderr, "\nClock sync failed: %s\n", clockErr.c_str());
                    }
                }
            }
        }

        if (isTty) printf("\n");
        pclose(pipe);
        return 0;
    }

    // ── Live mode ──────────────────────────────────────────────────────────
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

    // Use command-line --device if given, else fall back to settings, else default
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
        g_latestFrame   = frame;
        g_frameUpdated  = true;
    });

    // Print each classified bit as it arrives so the operator can see the
    // signal structure before a full frame is decoded.
    //   '.' = 0 (200 ms)   '#' = 1 (500 ms)   '|' = marker (800 ms)   '?' = missing
    // A newline is printed after each marker to visually separate the
    // 9-bit groups (P0..P5) that make up a WWV frame.
    bool needNewline = false; // true if searching \r line is on screen
    decoder.setBitCallback([&needNewline, isTty](int type, int ms) {
        if (needNewline) { printf("\n"); needNewline = false; }
        if (type == 0)      printf(".(%d) ", ms);
        else if (type == 1) printf("#(%d) ", ms);
        else if (type == 2) printf("|  (%d ms)\n", ms);  // marker: newline after each group
        else                printf("? ");                 // BIT_MISSING gap-fill
        fflush(stdout);
    });

    // ── Open audio stream ──────────────────────────────────────────────────
    PaStreamParameters inParams{};
    inParams.device                    = devIdx;
    inParams.channelCount              = 1;
    inParams.sampleFormat              = paFloat32;
    inParams.suggestedLatency          = devInfo ? devInfo->defaultLowInputLatency
                                                 : 0.1;

    PaStream* stream = nullptr;
    err = Pa_OpenStream(&stream, &inParams, nullptr,
                        sampleRate, 512, paClipOff, pa_callback, nullptr);
    if (err != paNoError) {
        fprintf(stderr, "Pa_OpenStream failed: %s\n", Pa_GetErrorText(err));
        Pa_Terminate(); return 1;
    }

    signal(SIGINT,  on_signal);
#ifndef _WIN32
    signal(SIGTERM, on_signal);
#endif

    Pa_StartStream(stream);
    printf("Listening. Press Ctrl+C to stop.\n\n");

    // ── Main display loop ──────────────────────────────────────────────────
    bool clockSet = false;
    while (g_running.load()) {
        Pa_Sleep(200); // 200 ms update interval

        // Check for a newly decoded frame
        WwvTime frame;
        bool updated = false;
        {
            std::lock_guard<std::mutex> lk(g_frameMutex);
            if (g_frameUpdated) {
                frame = g_latestFrame;
                updated = true;
                g_frameUpdated = false;
            }
        }
        if (updated && isTty) {
            printf("\n");
        }

        time_t utc = decoder.currentUtc();
        float sig  = decoder.signalLevel();

        if (utc == 0) {
            // Bit stream is printed inline by the bit callback.
            // Print a periodic signal-level status line (not \r, so it
            // doesn't stomp the bit stream).
            static int statusTick = 0;
            if (++statusTick % 10 == 0) { // every ~2 s
                printf("\n-- Signal: %3.0f%%  Bits: %d --\n",
                       sig * 100.0f, decoder.bitsReceived());
            }
        } else {
            WwvTime f = decoder.lastFrame();
            printTime(utc, f, sig, isTty);

            // Optionally sync the system clock
            if (cfg.setSystemClock && !clockSet &&
                f.confidence >= cfg.minConfidence) {
                std::string clockErr;
                if (setSystemClock(utc, clockErr)) {
                    printf("\nClock set to %lld UTC\n", (long long)utc);
                    clockSet = true;
                } else {
                    fprintf(stderr, "\nClock sync failed: %s\n", clockErr.c_str());
                }
            }
        }
    }

    // ── Cleanup ────────────────────────────────────────────────────────────
    if (isTty) printf("\n");
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    rig.disconnect();
    return 0;
}
