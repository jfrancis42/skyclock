# skyclock

A command-line WWV time code decoder and system clock synchroniser written in C++17.

Receives the NIST WWV shortwave time signal through a connected radio (or any audio
input), decodes the BCD time code from audio in real time, displays current UTC, and
optionally sets the system clock once a configurable number of consecutive frames have
been verified.

**Status: early alpha.** Decodes reliably from clean recordings and live reception
under good propagation. Fails on signals with heavy HF flutter fading (see [Known
Limitations](#known-limitations)).

---

## Table of Contents

1. [Signal format — WWV](#signal-format--wwv)
2. [Theory of operation](#theory-of-operation)
3. [Building](#building)
4. [Usage](#usage)
5. [Configuration](#configuration)
6. [Reading audio files (ffmpeg)](#reading-audio-files-ffmpeg)
7. [What works](#what-works)
8. [Known limitations](#known-limitations)
9. [Future work](#future-work)

---

## Signal format — WWV

WWV (Fort Collins, Colorado) transmits on 2.5, 5, 10, 15, and 20 MHz. Every second
begins with a 5 ms 1 kHz sine-wave "tick" at the precise on-time point, followed by a
100 Hz subcarrier burst whose duration encodes one BCD bit:

| Duration | Bit type    | Meaning                        |
|----------|-------------|--------------------------------|
| 200 ms   | `0` (ZERO)  | BCD data zero                  |
| 500 ms   | `1` (ONE)   | BCD data one                   |
| 800 ms   | `M` (MARKER)| Position marker                |

One complete frame is 60 bits (one per second, one frame per minute). Position markers
occur at seconds 0, 9, 19, 29, 39, and 49 within each frame and are used by the
decoder for frame alignment.

The BCD fields encoded in the 60 bits are (by second within the frame):

| Bits      | Field                        |
|-----------|------------------------------|
| 1–8       | Minutes (BCD, tens then units) |
| 10–18     | Hours (BCD)                  |
| 20–28     | Day of year (hundreds)       |
| 25–33     | Day of year (tens + units)   |
| 38, 40–43 | UT1 correction (±0.1 s steps) |
| 45–48     | Year tens (BCD)              |
| 50–53     | Year units (BCD)             |
| 55–56     | DST status                   |
| 57        | Leap-second warning          |

See [NIST SP 432](https://www.nist.gov/system/files/documents/2017/04/28/SP-432-NIST-Time-and-Frequency-Services-2012-02-13.pdf) for the complete specification.

---

## Theory of operation

### Goertzel filters

Rather than FFT, the decoder uses **Goertzel filters** — single-frequency DFT
evaluations — at exactly 1 kHz and 100 Hz over 10 ms blocks (480 samples at 48 kHz).
This gives a power measurement at each target frequency every 10 ms without the
overhead of a full FFT.

### 1 kHz tick detection (second boundary)

Each 10 ms block's 1 kHz Goertzel power is smoothed with an exponential moving
average. A rising edge that exceeds 100× the tracked noise floor is classified as a
tick (the on-time second marker). A 950 ms lockout after each tick prevents false
re-triggers from MARKER harmonics or multipath echoes.

If the 1 kHz tick at a particular second is too weak to cross threshold (common on
position-marker seconds where the signal structure differs), the decoder falls back to
a **synthetic tick**: it detects a sharp onset of the 100 Hz subcarrier rising more than
20× above the background noise floor and treats that onset as the second boundary.

### 100 Hz data-bit detection

After each tick (real or synthetic), a 1 150 ms data window opens. Inside the window
the decoder tracks an exponential moving average of 100 Hz Goertzel power and looks
for:

- **Onset**: `smooth100` rises above `preTick100 × 8` for two consecutive 10 ms
  blocks.  `preTick100` is the 100 Hz level captured at tick time, giving a per-second
  noise reference that automatically adapts to varying conditions.
- **Offset**: `smooth100` falls below `preTick100 × 3` for five consecutive blocks
  (50 ms confirmation debounce prevents brief signal dips from prematurely closing a
  MARKER measurement).

The measured pulse duration classifies the bit:

```
 80–349 ms → BIT_ZERO
350–649 ms → BIT_ONE
650–1099 ms → BIT_MARKER
```

### Ring buffer and gap-fill

Decoded bits are written into a 120-entry ring buffer (two minutes of history). When
a second is skipped due to HF fading, a `BIT_MISSING` placeholder is inserted so the
ring buffer keeps correct 1-second cadence. The frame decoder tolerates up to 30
missing bits per frame and up to one missing *marker* position (HF dropouts on P0 are
common in real recordings).

### Frame sync and BCD decode

After 60 bits are accumulated the decoder tries all possible 60-bit starting positions
in the ring buffer. A candidate frame is accepted when:

1. All six marker positions contain `BIT_MARKER` (or at most one contains
   `BIT_MISSING`; `BIT_ZERO` or `BIT_ONE` at a marker position is a hard alignment
   failure).
2. "Always-zero" positions defined by the NIST spec do not contain `BIT_MARKER`.
3. The decoded minute (0–59), hour (0–23), day of year (1–366), and year (0–99) are
   all in valid range.

Once a valid frame is decoded, `currentUtc()` extrapolates forward from the P0 tick
timestamp using `std::chrono::steady_clock` to give a running real-time UTC estimate
accurate to roughly ±50 ms under typical conditions.

---

## Building

### Dependencies

| Library    | Purpose              | Package (Debian/Ubuntu)         |
|------------|----------------------|---------------------------------|
| PortAudio  | Live audio capture   | `libportaudio19-dev`            |
| Hamlib     | Radio CAT control    | `libhamlib-dev` *(optional)*    |

```sh
sudo apt install cmake build-essential libportaudio19-dev
sudo apt install libhamlib-dev   # optional, enables --list-rigs and rig control
```

### Compile

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

The binary is `build/skyclock`. No install step is required to run it.

---

## Usage

```
skyclock                        # decode from default audio device
skyclock --device <name>        # use a named PortAudio input device
skyclock --file <path>          # decode from an audio file via ffmpeg
skyclock --list-devices         # list audio input devices and exit
skyclock --list-rigs            # list hamlib rig models and exit (requires hamlib)
skyclock --version              # print version and exit
skyclock --help                 # this help
```

### Live reception example

Tune your radio to **10 000 kHz AM**, connect audio output to the PC sound card input,
then:

```sh
./build/skyclock
```

As bits are classified they are printed inline:
```
.  = BIT_ZERO  (200 ms)
#  = BIT_ONE   (500 ms)
|  = MARKER    (800 ms)  — newline after each 9-bit group
?  = MISSING   (no signal detected)
```

Once a frame decodes, the running time line replaces the bit stream:

```
2024-10-12 14:32:07 UTC  Day 286  UT1+0.2s  Signal:  87%  [conf: 3]
```

---

## Configuration

Settings are stored in `~/.skyclock/settings.json`, created with defaults on first
run.

| Key              | Type    | Default        | Description                                  |
|------------------|---------|----------------|----------------------------------------------|
| `rigEnabled`     | bool    | `false`        | Enable hamlib radio control                  |
| `rigModel`       | int     | `1`            | Hamlib rig model number                      |
| `rigPort`        | string  | `/dev/ttyUSB0` | Serial port for CAT control                  |
| `freqKhz`        | int     | `10000`        | WWV frequency to tune (kHz)                  |
| `rigMode`        | string  | `"AM"`         | Radio mode string                            |
| `audioDevice`    | string  | `""`           | Audio device name substring (empty = default)|
| `setSystemClock` | bool    | `false`        | Set system clock after decode (needs root)   |
| `minConfidence`  | int     | `2`            | Consecutive valid frames before setting clock|

WWV transmits on 2 500, 5 000, 10 000, 15 000, and 20 000 kHz. 10 000 kHz is the most
reliable frequency across North America during daylight hours.

---

## Reading audio files (ffmpeg)

The `--file` mode pipes the audio file through **ffmpeg**, which handles any format
ffmpeg understands (MP3, OGG, Opus, FLAC, WAV, …) and resamples to 48 kHz mono
float32 on the fly. ffmpeg must be installed and in `PATH`:

```sh
sudo apt install ffmpeg
```

Any sample rate and channel count is accepted; ffmpeg resamples internally. The decoder
itself always operates at whatever sample rate the audio arrives at (48 kHz in file
mode, the device native rate in live mode).

---

## What works

- **Clean recordings and strong live signals** decode reliably within 1–2 minutes.
  The decoder reaches a stable time display with confidence climbing each frame.
- **Position-marker seconds with weak 1 kHz ticks** are handled via the synthetic
  tick fallback, so MARKER bits at P0/P3/P4/P5 are still captured.
- **Isolated HF dropouts** (single missed seconds) are filled with `BIT_MISSING`
  placeholders; the frame decoder tolerates up to 30 per frame and 1 missing marker
  position, so moderate fading does not prevent a decode.
- **False ticks** from 100 Hz MARKER harmonics at 820–880 ms after a real tick are
  suppressed by a 950 ms lockout.
- **Brief signal dips** within a MARKER pulse (common multipath artefact) are
  bridged by a 50 ms offset debounce.

---

## Known limitations

### Heavy HF flutter fading (wwv-noisy.opus and similar)

Under severe HF flutter fading conditions the 100 Hz subcarrier arrives as a rapid
sequence of 10–70 ms fragments rather than a single continuous burst. The current
threshold-based burst detector requires a *continuous* pulse of at least 80 ms and
cannot accumulate evidence across gaps, so almost all data bits are reported as
`BIT_MISSING`.

This is not an edge-case tuning issue — it is a fundamental architectural limitation:
the detector asks "is there a continuous burst right now?" when the correct question
under flutter is "has enough 100 Hz energy arrived in this 200/500/800 ms window?"

### P0 dropout

In some recordings the 1 kHz tick and/or the 100 Hz subcarrier at second 0 of each
minute (P0) is too weak to detect. The decoder handles a single missing marker per
frame, but if P0 is consistently absent across multiple minutes, cross-minute
confidence cannot build through the P0 position.

---

## Future work

### Energy integration for flutter-faded signals (highest priority)

Replace the continuous-burst detector with an **energy integrator**:

Instead of detecting a sustained onset/offset edge pair, accumulate the 100 Hz
Goertzel power across the entire post-tick data window and compare against sub-window
integrals:

- Integrate `smooth100` over `[30 ms, 230 ms]` → ZERO-window energy
- Integrate over `[30 ms, 530 ms]` → ONE-window energy
- Integrate over `[30 ms, 830 ms]` → MARKER-window energy

Classify the bit by which window's accumulated energy exceeds the expected noise-only
energy (= `background100 × window_duration`) by the largest margin. This approach is
insensitive to brief fades mid-pulse because it accumulates evidence rather than
requiring continuity.

The per-second tick-gated window structure stays the same; only the within-window
classification logic changes.

### Improved P0 recovery

Track the inter-marker interval precisely: once P1–P5 are established, P0 can be
inferred to within ±10 ms from the known 60 s frame period. Synthesize a P0 timestamp
from this estimate rather than requiring a detected signal at exactly second 0.

### Cross-frequency diversity

When two WWV frequencies are received simultaneously (e.g., 5 MHz and 10 MHz via two
radios or a dual-band receiver), combine their bit streams. A bit confirmed on both
paths can be trusted with much higher confidence; a bit that disagrees or is missing
on one path can be taken from the other. This would make the decoder significantly more
robust under selective HF fading.

### Hamlib integration testing

The hamlib rig-control path exists in the code but has not been tested end-to-end with
a physical radio. Contributions of tested rig configurations are welcome.
