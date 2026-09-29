# DGPS Beacon Decoder (for ATmega328P)

A from-scratch, bare-metal RTCM SC-104 / MSK decoder for maritime DGPS
longwave beacons, written in plain **avr-gcc / avr-libc** — no Arduino
core, no `printf`, no floating point, no external libraries. It runs
directly on a $3 ATmega328P board (e.g. an Arduino Nano) and prints
decoded correction messages as plain text over the serial port.

Feed it the audio output of any SSB/CW receiver tuned to a DGPS beacon
(283.5–325 kHz in Europe) and it will lock on, auto-detect the beacon's
baud rate, and start printing GPS/GLONASS correction data, station
health, and reference station position.

```
DGPS-Decoder (RTCM SC-104/MSK) ATmega328P, 4 kHz ADC, 50/100/200 Bit/s
Erwarte NF-Signal um 750 Hz an ADC0 ...
*** SYNC: 100 Bit/s, NF-Mitte 743.7 Hz ***
[452] Typ 9 GPS-Teilkorrekturen, Z=51:00.6, Nr 6, 5 Wo, Stat: UDRE-Skala 1
  G04  PRC -5.72 m  RRC 0.000 m/s  IOD 193  UDRE <=1 m
  G06  PRC -9.46 m  RRC 0.004 m/s  IOD 11  UDRE <=1 m
  G09  PRC -17.64 m  RRC 0.010 m/s  IOD 45  UDRE <=4 m
[452] Typ 3 Referenzstation-Position, Z=51:03.0, Nr 7, 4 Wo, Stat: UDRE-Skala 1
  Referenzposition ECEF  X 3579683.85  Y 508399.47  Z 5236838.40 m
[452] Typ 9 GPS-Teilkorrekturen, Z=51:06.6, Nr 1, 5 Wo, Stat: UDRE-Skala 1
  G19  PRC -6.88 m  RRC 0.000 m/s  IOD 81  UDRE <=1 m
  G28  PRC -7.86 m  RRC 0.000 m/s  IOD 36  UDRE <=1 m
  G31  PRC -9.02 m  RRC 0.002 m/s  IOD 38  UDRE <=1 m
[452] Typ 9 GPS-Teilkorrekturen, Z=51:09.0, Nr 2, 5 Wo, Stat: UDRE-Skala 1
  G01  PRC -4.98 m  RRC 0.000 m/s  IOD 177  UDRE <=1 m
  G02  PRC -8.72 m  RRC -0.002 m/s  IOD 47  UDRE <=1 m
  G32  PRC -23.20 m  RRC -0.016 m/s  IOD 41  UDRE <=1 m
[452] Typ 9 GPS-Teilkorrekturen, Z=51:10.8, Nr 3, 5 Wo, Stat: UDRE-Skala 1
  G03  PRC -2.84 m  RRC 0.000 m/s  IOD 55  UDRE <=1 m
  G04  PRC -5.72 m  RRC 0.000 m/s  IOD 193  UDRE <=1 m
  G06  PRC -9.42 m  RRC 0.004 m/s  IOD 11  UDRE <=1 m
[452] Typ 9 GPS-Teilkorrekturen, Z=51:13.2, Nr 4, 5 Wo, Stat: UDRE-Skala 1
  G09  PRC -17.52 m  RRC 0.010 m/s  IOD 45  UDRE <=4 m
  G12  PRC -19.84 m  RRC 0.004 m/s  IOD 111  UDRE <=4 m
  G13  PRC -18.60 m  RRC 0.000 m/s  IOD 232  UDRE <=4 m
  ...
```

This example is a live off-air capture of **Blåvandshuk, Denmark**
(290 kHz, station ID 452), decoded on real hardware. It has also been
verified against live recordings of Helgoland (Germany), Gilze-Rijen,
Hoek van Holland and Vlieland (Netherlands), and Oostende (Belgium) —
see [Testing](#testing) below.

This project is the DGPS-specific sibling of an earlier from-scratch
EWBS (Japanese Emergency Warning Broadcast System) decoder for the
same chip — same "no shortcuts, no black boxes" philosophy, same
comment-heavy teaching style, different signal.

## What is DGPS, and what does this decoder actually receive?

**Differential GPS (DGPS)** beacons are shore-based longwave radio
transmitters (283.5–325 kHz, MF/LF band) operated by coast guards and
maritime authorities around the world. Each beacon sits at a precisely
surveyed reference location, continuously computes the error between
its known position and what a GPS receiver would calculate, and
broadcasts that error as a correction stream. A DGPS-capable GPS
receiver within range (typically 100–300 km) applies these corrections
and improves its fix from ~5–10 m down to sub-meter accuracy — long
before satellite-based augmentation systems (WAAS/EGNOS) existed, this
was *the* way to get precise GPS, and it is still broadcast
today.

The correction data itself is standardized as **RTCM SC-104** (Radio
Technical Commission for Maritime Services, Special Committee 104),
originally version 2, the same message format historically used over
FM subcarriers and other correction links. On longwave, it is
transmitted using **MSK (Minimum Shift Keying)** — a form of FSK where
the phase stays continuous across bit transitions, which keeps the
transmitted spectrum narrow enough to fit in a few hundred hertz of
channel. Each beacon independently chooses a bit rate of **50, 100, or
200 bit/s**; a receiver has to figure out which one is in use before
it can even start decoding bits, let alone frames.

A typical SSB/CW receiver, tuned a few hundred hertz off the beacon's
carrier, turns this radio signal into an audio tone pair somewhere
around 600–900 Hz (exact frequency depends entirely on how the
receiver's BFO is set) — and that audio is exactly what this decoder
expects on its ADC input.

### RTCM SC-104 message structure

- A **word** is 30 bits: 24 data bits + 6 parity bits, using the same
  Hamming-style parity scheme as GPS navigation data (including the
  D29\*/D30\* polarity-inversion rule — the receiver does not need to
  know which MSK tone represents a "1", since the parity check fixes
  the polarity automatically).
- **Word 1** of every frame: preamble `01100110`, message type (6
  bits), station ID (10 bits), parity.
- **Word 2**: Z-count / time tag (13 bits, 0.6 s resolution), sequence
  number (3 bits), frame length in words (5 bits), station health (3
  bits), parity.
- Followed by "length" data words, whose content depends on the
  message type. This decoder implements:
  - **Type 1 / 9** — GPS pseudorange corrections (full / partial set)
  - **Type 31 / 34** — GLONASS pseudorange corrections (full / partial)
  - **Type 3** — reference station position (ECEF X/Y/Z)
  - **Type 16** — plain-text station message
  - All other types are still frame-synced and their header is
    printed, but the payload is not decoded.

## Signal processing pipeline

Everything below runs as fixed-point integer arithmetic — no floating
point, no external DSP library, nothing that needs more than the
ATmega328P's native 8-bit ALU (with the compiler emitting the
occasional 16×16 or 32-bit multiply/shift sequence where needed).

```
ADC0 --4 kHz--> [DC removal] --> [NCO mixer cos/sin] --> [narrowband LPF] --> [CIC3 / 4]
                                        ^                                          |
                                        |                                    1 kHz complex (I,Q)
                     carrier tracking (FLL)                                        |
                     steers the NCO frequency                                      v
                                        ^                     [limiter: |z| = constant]
                                        |                                          |
                                        +---- discriminator  Im(z[n] * conj(z[n-1]))
                                                                                    |
                            +-----------------------+-----------+-----------+
                            |                       |                       |
                        50 bit/s lane          100 bit/s lane         200 bit/s lane
                      (zero-crossing DPLL  +  integrate & dump  +  RTCM frame sync)
                                                    |
                        (whichever lane first produces valid RTCM frames
                         wins -> baud rate has been detected)
                                                    |
                                       RTCM parser --> plain-text output
```

1. **4 kHz sampling.** The signal of interest sits at 500–1000 Hz;
   Timer1 divides the 16 MHz system clock exactly by 4000, so the ADC
   sample rate has zero rounding error. A simple analog low-pass
   around 1.5 kHz ahead of the ADC is sufficient anti-aliasing.
2. **DC removal**, then a **complex NCO mixer** (16-bit phase
   accumulator + an 8-bit sine table in flash) shifts the audio tone
   pair down to baseband.
3. A **3rd-order CIC filter with decimation by 4** brings the rate
   down to 1 kHz complex — using only additions, no filter
   coefficients at all. 1000 Hz was chosen specifically because
   1000/50, 1000/100 and 1000/200 are all integers (20/10/5 samples
   per bit), so the bit-timing accumulator never accumulates rounding
   error regardless of which baud rate is in use.
4. A short **narrowband follow-up filter** (two cascaded one-pole
   low-passes, ~110 Hz corner) further cleans up the CIC's still-wide
   output before the discriminator, which otherwise amplifies noise
   aggressively. This one stage made the difference between "occasional
   frame" and "100 out of 100 frames decoded" on a weak, 100 bit/s
   signal during testing.
5. A **limiter** normalizes the I/Q magnitude, so only phase (not
   amplitude/fading) matters from here on — classic FM-receiver style.
6. A **frequency discriminator**, `Im(z[n] * conj(z[n-1]))`, turns
   instantaneous phase change into an instantaneous frequency estimate.
7. **Carrier tracking (FLL).** Because MSK carries no residual carrier,
   a classic Costas loop has nothing to lock onto. Instead, since the
   two MSK tones sit symmetrically around the true center frequency,
   the *average* of the discriminator output over time converges to
   the receiver's actual BFO offset, even while data is present — this
   average becomes the NCO frequency-correction signal. Capture range
   is ±100 Hz around a 750 Hz nominal center; loop bandwidth is wider
   while searching, narrower once locked.
8. **Bit timing recovery.** MSK's instantaneous frequency only changes
   at bit boundaries, so the discriminator output crosses zero at
   (fractional) bit boundaries. Three independent zero-crossing DPLLs
   — one per candidate baud rate — each nudge their own bit-phase
   accumulator toward every such crossing.
9. **Baud rate detection.** All three bit "lanes" run in parallel
   against the very same discriminator stream, each attempting RTCM
   frame sync (preamble + two consecutive parity-valid words, a
   1-in-2^26 chance of a false lock). Whichever lane achieves this
   first is declared the active lane; the others are ignored until
   sync is lost.
10. If no valid frame arrives for **8 seconds**, the decoder drops back
    to searching all three lanes simultaneously (beacon changed baud
    rate, went off air, or the receiver was retuned). After **40
    seconds** without any lock, the NCO is reset to its nominal center
    frequency.

## Console output format

Every line the decoder prints falls into one of a handful of
categories:

**Startup banner**
```
DGPS-Decoder (RTCM SC-104/MSK) ATmega328P, 4 kHz ADC, 50/100/200 Bit/s
Erwarte NF-Signal um 750 Hz an ADC0 ...
```
Printed once at boot. ("Erwarte NF-Signal um 750 Hz an ADC0" = "expecting
audio signal around 750 Hz on ADC0" — the firmware's log/status strings
are currently German; message *types* and *field names* below apply
regardless of language.)

**Sync acquired**
```
*** SYNC: 100 Bit/s, NF-Mitte 738.9 Hz ***
```
The decoder has locked onto a beacon: detected baud rate, and the
current audio center frequency ("NF-Mitte" = "AF center") as tracked
by the FLL — useful for checking how far your receiver's BFO is off
from the nominal 750 Hz.

**Sync lost**
```
*** Sync verloren - suche 50/100/200 Bit/s ***
```
("Sync lost — searching 50/100/200 bit/s.") No valid frame for 8 s;
the decoder is back to trying all three baud rates.

**Periodic status line** (every 30 s by default)
```
--- Status: NF-Mitte 746.0 Hz, Pegel ~39, Rahmen ok 5, fehlerhaft 6, uebersprungen 0 ---
```
| Field | Meaning |
|---|---|
| `NF-Mitte` | Current tracked audio center frequency (Hz) |
| `Pegel` | Rolling-average signal level ahead of the limiter (arbitrary units, roughly proportional to ADC swing — useful as a relative signal-strength indicator, not calibrated to any absolute unit) |
| `Rahmen ok` | Count of frames that passed RTCM parity and were fully decoded |
| `fehlerhaft` | Count of frames that failed parity (noise, fading, adjacent-channel interference) |
| `uebersprungen` | Frames that decoded correctly but were dropped because the UART/printer was still busy with a previous frame (never lost — always because output couldn't keep up, not a receive error) |
| `KEIN SYNC` | Appended instead of the trailing `---` when no beacon is currently locked |

**Frame header line** (one per decoded RTCM frame)
```
[452] Typ 9 GPS-Teilkorrekturen, Z=51:04.8, Nr 0, 5 Wo, Stat: UDRE-Skala 1
```
| Field | Meaning |
|---|---|
| `[452]` | Station ID (10-bit field from word 1) |
| `Typ 9` | RTCM message type number |
| `GPS-Teilkorrekturen` | Human-readable type name ("GPS partial corrections"; see table below for all recognized types) |
| `Z=51:04.8` | Z-count / time tag, decoded as `MM:SS.s` (minutes:seconds, 0.6 s resolution — this is *not* wall-clock time, it's the beacon's internal modulo-3600s frame counter) |
| `Nr 0` | Sequence number (3-bit, wraps 0–7) |
| `5 Wo` | Frame length in RTCM words (excluding the two header words) |
| `Stat: ...` | Station health: `ok`, `nicht ueberwacht` ("not monitored"), `AUSSER BETRIEB` ("out of service"), or `UDRE-Skala N` (health code doubles as a UDRE scale factor hint) |

Recognized message types:

| Type | Name | Content |
|---|---|---|
| 1 | DGPS-Korrekturen | Full GPS corrections |
| 2 | Delta-Korrekturen | Delta corrections |
| 3 | Referenzstation-Position | Reference station ECEF position |
| 5 | Satelliten-Zustand | Satellite health |
| 6 | Fuellrahmen | Filler frame |
| 7 | Baken-Almanach | Beacon almanac |
| 9 | GPS-Teilkorrekturen | Partial GPS corrections |
| 16 | Textnachricht | Plain-text station message |
| 31 | GLONASS-Korrekturen | Full GLONASS corrections |
| 34 | GLONASS-Teilkorrekturen | Partial GLONASS corrections |
| *other* | "(Typ nicht dekodiert)" | Frame is still parity-checked and counted, only the header is printed |

**Per-satellite correction line** (types 1/9/31/34, one line per
satellite in the frame)
```
  G12  PRC -19.88 m  RRC 0.004 m/s  IOD 111  UDRE <=4 m
```
| Field | Meaning |
|---|---|
| `G` / `R` | Constellation: GPS or GLONASS |
| `12` | Satellite PRN (GPS) or slot number (GLONASS) |
| `PRC` | Pseudorange correction, in meters |
| `RRC` | Range-rate correction, in meters/second |
| `IOD` | Issue-of-data, used by the receiver to match this correction to the right ephemeris |
| `UDRE` | User Differential Range Error class — a coarse confidence bound on the correction (`<=1 m`, `<=4 m`, `<=8 m`, or "`>8 m (nicht nutzen)`" = "do not use") |

**Reference position line** (type 3)
```
  Referenzposition ECEF  X 3579683.85  Y 508399.47  Z 5236838.40 m
```
The beacon's own surveyed position, in Earth-Centered-Earth-Fixed
Cartesian coordinates, centimeter resolution.

**Text message line** (type 16)
```
  Text: Some station announcement here
```
Raw 8-bit ASCII, printed until a null byte or 90 characters.

## Building

Standard avr-gcc toolchain, no Arduino IDE required:

```sh
make          # builds main.hex
make size     # shows flash/RAM usage
make flash    # flashes via avrdude (edit PORT/PROGRAMMER/BAUD in the Makefile first)
```

Flash/RAM footprint (avr-gcc 7.3, `-Os`):

```
   text    data     bss     dec     hex filename
   7058      10     654    7722    1e2a main.elf
```

I.e. about 7 KB of the ATmega328P's 32 KB flash, and well under 1 KB
of its 2 KB of RAM.

### Hardware

- Audio input on **ADC0** (pin A0 on an Arduino Nano), AC-coupled and
  biased to roughly Vcc/2 by a resistor divider, preceded by a simple
  1–2 pole analog low-pass around 1.5 kHz.
- **USART0** (the Nano's onboard USB-serial), 9600 baud 8N1, for the
  decoded text output.
- Feed it the demodulated SSB/CW audio output of any longwave-capable
  receiver (a standalone LF/MF receiver, an SDR's audio output, or a
  standalone Si4732/Si4735-based front end) tuned a few hundred hertz
  off a DGPS beacon's published frequency.

## Testing

There is no dependency on real hardware to try this out: `main.c`
also compiles for the host PC.

```sh
make test RAW=hel.raw     # or: make host_sim && ./host_sim yourfile.raw
```

`host_sim.c` `#include`s `main.c` directly with `-DHOST_SIM` defined,
which swaps out all AVR-specific register access, interrupt vectors,
and `PROGMEM` handling for plain host equivalents — the DSP and RTCM
code itself is completely unmodified between the AVR build and the
host build. `RAW` files are raw, headerless, little-endian `uint16_t`
samples at 4000 Hz, DC-biased around 512 (i.e. exactly what the ADC
ISR would otherwise hand to `dsp_sample()`) — easy to generate from
any WAV recording of a DGPS beacon's SSB/CW audio with a short Python
script (resample to 4 kHz, normalize, offset by 512).

This decoder has been validated this way against off-air recordings
of six different European DGPS beacons, covering all three baud
rates, weak/noisy signals, and receiver BFO offset/drift:

| Station | Country | Frequency | Baud rate | Station ID |
|---|---|---|---|---|
| Helgoland | Germany | — | 100 bit/s | 762 |
| Gilze-Rijen | Netherlands | — | 200 bit/s | 652 |
| Vlieland | Netherlands | 294 kHz | 100 bit/s | 655 |
| Hoek van Holland | Netherlands | 312.5 kHz | 200 bit/s | 650 |
| Oostende | Belgium | 312 kHz | 100 bit/s | 640 |
| Blåvandshuk | Denmark | 290 kHz | 100 bit/s | 452 |

...and, on real ATmega328P hardware fed live off-air audio, decoded
correctly across a range of receive conditions from clean to
noisy/fading (see the Blåvandshuk console capture above — a real
serial-port log, unedited).

If you record your own test files: note that **not every audio player
resamples correctly**. These WAV files carry an unusual 7119 Hz sample
rate in their header (an artifact of the original recording setup);
`aplay` and VLC play them back at the correct speed by reading that
header properly, while some builds of `mplayer` were observed
mis-resampling on startup, producing a noticeably wrong pitch/timing
for the first second or two of playback. If you see the decoder
reporting a wildly wrong "NF-Mitte" and no sync, check your playback
chain before assuming the decoder is at fault.

## Possible next steps

- Pair this with an **Si4732/Si4735** longwave module (or any other
  chip/radio that outputs demodulated SSB/CW audio) for a fully
  standalone DGPS correction receiver with no PC involved.
- Feed the decoded corrections into a GPS receiver's RTCM input for
  an actual sub-meter differential fix.
- Add an OLED/LCD status display (station ID, baud rate, lock status)
  driven from the same data this firmware already parses.
- Port the frame parser to decode additional message types (5, 6, 7)
  if you have a use for satellite health / beacon almanac data.
