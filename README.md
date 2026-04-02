# skyclock

A command-line WWV time code decoder and system clock synchroniser written in C++17.

Receives the NIST WWV shortwave time signal through a connected radio (or any audio
input), decodes the BCD time code from audio in real time, displays current UTC, and
optionally sets the system clock once a configurable number of consecutive frames have
been verified.

**Status: early alpha.** Decodes reliably from clean recordings, live reception under
good propagation, and HF signals degraded by severe flutter fading.

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

| Bits      | Field                          |
|-----------|--------------------------------|
| 1–8       | Minutes (BCD, tens then units) |
| 10–18     | Hours (BCD)                    |
| 20–28     | Day of year (hundreds)         |
| 25–33     | Day of year (tens + units)     |
| 38, 40–43 | UT1 correction (±0.1 s steps)  |
| 45–48     | Year tens (BCD)                |
| 50–53     | Year units (BCD)               |
| 55–56     | DST status                     |
| 57        | Leap-second warning            |

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
tick (the on-time second marker), subject to an anti-spoof gate: genuine 1 kHz ticks
occur before the 100 Hz subcarrier begins (~30 ms later), so the 100 Hz Goertzel
power at tick time is near the background floor. MARKER-subcarrier harmonics at 1 kHz
(10th harmonic of 100 Hz) are rejected because they occur simultaneously with elevated
100 Hz power. A 950 ms lockout after each tick also prevents false re-triggers from
multipath echoes.

Once a real tick is detected, subsequent second boundaries are **free-running
predictions**: the decoder advances the phase by exactly 100 blocks (1000 ms) each
second regardless of whether a real tick arrives. WWV's cesium clock is accurate to
microseconds; the audio clock drifts less than 1 ms per minute at 50 ppm, so
predictions remain accurate across many missed ticks.

### 100 Hz data-bit classification via energy integration

Rather than detecting a continuous burst onset/offset, the decoder **accumulates raw
100 Hz Goertzel power** across three fixed sub-windows after each tick:

| Window | Blocks after tick | Audio time | Blocks |
|--------|-------------------|------------|--------|
| Early  | 3–22              | 30–230 ms  | 20     |
| Mid    | 23–52             | 230–530 ms | 30     |
| Late   | 53–82             | 530–830 ms | 30     |

Classification fires at block 83 (830 ms post-tick):

```
total = e_early + e_mid + e_late

if total < 2.0 × noise_ref   → BIT_MISSING  (no detectable signal)
elif e_late / total > 0.28   → BIT_MARKER   (energy extends to 830 ms)
elif e_mid  / total > 0.28   → BIT_ONE      (energy extends to 530 ms)
else                         → BIT_ZERO     (energy confined to 230 ms)
```

The noise reference is a rolling EMA of the 100 Hz power measured in a quiet tail
window at 920–970 ms after each tick, scaled to the 80-block measurement span.

This approach is robust to **HF flutter fading**: when the subcarrier arrives as
10–70 ms incoherent fragments rather than a sustained burst, the fragments still
deposit energy into the correct sub-window. Brief gaps between fragments do not reset
the accumulator, so the full bit period contributes to the classification.

### Ring buffer and gap-fill

Decoded bits are written into a 120-entry ring buffer (two minutes of history). When
a second is skipped due to HF fading, a `BIT_MISSING` placeholder is inserted so the
ring buffer keeps correct 1-second cadence. The frame decoder tolerates up to 30
missing bits per frame and up to one missing *marker* position.

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

---

## What works

- **Clean recordings and strong live signals** decode within 1–2 minutes of
  accumulated bits.
- **HF flutter fading** — signals that arrive as 10–70 ms incoherent fragments due to
  severe ionospheric fading — are handled by the energy integration classifier. Each
  fragment deposits its 100 Hz energy into the correct sub-window; the total still
  classifies the bit correctly even when no individual fragment is long enough to
  trigger a threshold-based detector.
- **Isolated HF dropouts** (single missed seconds) are filled with `BIT_MISSING`
  placeholders; the frame decoder tolerates up to 30 per frame and 1 missing marker
  position.
- **False ticks** from 100 Hz MARKER harmonics are rejected by the anti-spoof gate:
  genuine ticks have near-floor p100 (subcarrier hasn't started yet); harmonic false
  triggers have simultaneous elevated p100 and p1k. The 950 ms lockout provides
  additional protection.

---

## Known limitations

### P0 dropout

In some recordings the 1 kHz tick at second 0 of each minute (P0) is too weak to
detect. The decoder allows one missing marker per frame, but confidence cannot build
through P0 if it is consistently absent. Future work: infer P0 from the established
P1–P5 inter-marker interval.

### WWVH / overlapping transmissions

WWVH (Hawaii) transmits on the same frequencies as WWV. Both stations are receivable
across much of North America on 10 and 15 MHz. The decoder does not distinguish
between them; mixed reception can produce garbled frames. Tuning to 5 MHz (WWV only)
typically gives the cleanest single-station decode.

### Linux only (for now)

The code compiles on Linux. Windows support is structurally present (PortAudio is
cross-platform; clock-setting code is conditionally compiled), but has not been tested
on a live Windows system. Cross-compile toolchain files are included in the repository.

### Hamlib integration untested end-to-end

The hamlib rig-control path exists in the code but has not been exercised with a
physical radio. Contributions of tested rig configurations are welcome.
