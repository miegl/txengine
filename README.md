# txengine

An FM and AM transmitter for the Raspberry Pi, written in Rust. It turns a
GPIO pin into a wide-band FM broadcast transmitter with a
standards-conformant stereo multiplex (ITU-R BS.450) and RDS (EN 50067), a
narrow-band FM voice transmitter from short wave and CB to UHF, or an AM
broadcast transmitter on long, medium and short wave. It runs on the
Raspberry Pi 2, 3, 4, 5 and Zero 2 W, as 32-bit (armv7) or 64-bit (aarch64)
static binaries, and through a Raspberry Pi Pico or Pico 2 with txengine's
firmware on USB, from any Linux computer.

* **FM**: stereo with RDS (PS, RadioText, PTY, alternative frequencies,
  clock time, ...), pre-emphasis, a look-ahead limiter on the true peaks,
  and the multiplex power held to ITU-R BS.412.
* **NFM**: voice in 12.5 or 25 kHz channels with CTCSS or DCS squelch
  signalling, pre-emphasis, band-limited and peak-limited to the channel,
  on any carrier from 1.5 MHz (short wave, CB, 10 m) through VHF to UHF
  (PMR446, 70 cm): up to 1 GHz on the Pi 5 and the Pico, 400 MHz on the
  other Raspberry Pis, beyond on a harmonic.
* **AM**: double sideband with full carrier, 100 kHz to 6.2 MHz (Pi 5),
  4.2 MHz (other Raspberry Pis), 1.76 MHz (Pico) or 1.71 MHz (Pico 2), with
  the programme band-limited to the channel and peak-limited.
* **Audio** from a file, standard input (a network stream, another program)
  or a test tone; RDS, gain, depth and the squelch signalling can be
  changed while transmitting.
* **On frequency**: the board's crystal error is measured against network
  time and corrected while transmitting.
* **Robust**: the output is timed by hardware (DMA), and every register
  txengine touched is restored when it stops, even after `kill -9`.

> **Transmitting without a licence is illegal in most countries.** The output
> is a square wave or a pulse train, rich in harmonics. Connect it to a
> receiver with a shielded cable and an attenuator, or to a dummy load, and
> always filter it before anything that can radiate.

* [Usage](#usage): FM, NFM, AM, control commands, frequency accuracy
* [Hardware](#hardware): the boards and their outputs
* [Building](#building)
* [Measurements](#measurements): FM, NFM and AM on every board, CPU
* [How it works](#how-it-works): the signal processing, and how each board makes FM and AM
* [Hardware notes](#hardware-notes): what each board requires, and how txengine handles it

## Usage

txengine is one static binary ([Building](#building)); on a Raspberry Pi it
runs as root, for a Pico it does not need to. The output pin and the
filtering it needs are in [Hardware](#hardware).

Commands are grouped by transmission mode: `txengine wfm tx` transmits
wide-band FM, `txengine wfm render` renders it to a file and
`txengine wfm analyze` measures a multiplex recording; `txengine nfm tx`
and `txengine nfm render` do the same for narrow-band FM, `txengine am tx`
and `txengine am render` for AM. `txengine probe`, `txengine devices` and
`txengine tone` need no mode. Every command lists its options with
`--help`. Frequencies are in MHz, or carry a unit: `-f 87.6`, `-f 999k`,
`-f 6.1M`.

`-d`/`--device` picks the transmitter: `pi` (the Raspberry Pi txengine runs
on), `pico` (the only Pico connected), `pico:SERIAL` or `pico:PORT` (its USB
port, such as `3-1.2`). Without it, txengine uses the only one it finds.

### FM

```sh
# Stereo file with RDS on 87.6 MHz
sudo txengine wfm tx -f 87.6 -a music.flac --ps "MY RADIO" --rt "Now playing ..."

# From another program (WAV on stdin, or raw PCM with --raw-rate/--raw-channels)
ffmpeg -i stream.m3u8 -f wav - | sudo txengine wfm tx -f 87.6 -a -
arecord -f S16_LE -r 48000 -c 2 -t raw | sudo txengine wfm tx -f 87.6 -a - --raw-rate 48000 --raw-channels 2

# Carrier + RDS only, with clock time and alternative frequencies
sudo txengine wfm tx -f 87.6 --ps STATION --af 87.6 --af 99.5 --pty "Pop music"

# Change RDS while transmitting
sudo txengine wfm tx -f 87.6 -a music.mp3 --ctl /tmp/rds &
echo "RT Live from the studio" > /tmp/rds
echo "PS ONAIR" > /tmp/rds

# Through a Pico on USB, from any Linux computer
txengine devices
txengine wfm tx -f 87.6 -d pico:5054316550E3291C -a music.flac
txengine wfm tx -f 87.6 -d pico --synthesis pio -a music.flac   # the PIO method
```

When standard input ends (the stream's server closed the connection), the
transmission goes on with the carrier and RDS but no programme
(`--on-end wait`, the default for standard input). For a station fed by a
network stream, `--on-end stop` ends txengine instead, so that the service
manager can restart the whole pipeline and with it the connection.

### NFM

```sh
# PMR446 channel 1 from a Raspberry Pi 5, CTCSS 88.5 Hz
sudo txengine nfm tx -f 446.00625 -a voice.wav --ctcss 88.5

# 2 m through a Pico on USB
txengine nfm tx -d pico -f 145.5125 -a voice.wav --ctcss 88.5

# 2 m from a Raspberry Pi in a 25 kHz channel (±5 kHz), DCS 023
sudo txengine nfm tx -f 145.5 --deviation 5000 -a voice.flac --dcs 023

# CB channel 19 (FM, Europe: ±2 kHz), from a microphone
arecord -f S16_LE -r 48000 -c 1 -t raw | sudo txengine nfm tx -f 27.185 --deviation 2000 -a - --raw-rate 48000 --raw-channels 1

# 70 cm from a Pi 2-4 or Zero 2 W: the third harmonic of 144.8333 MHz
sudo txengine nfm tx -f 434.5 --harmonic 3 -a voice.wav

# Change the squelch signalling while transmitting
sudo txengine nfm tx -f 145.5 -a voice.wav --ctl /tmp/nfm &
echo "CTCSS 123.0" > /tmp/nfm
echo "DCS 754I" > /tmp/nfm

# The signal as complex baseband, for an SDR program
txengine nfm render -a voice.wav --ctcss 88.5 --format iq --duration 10 -o nfm.cf32
```

The programme is mixed to mono and band-limited to the voice band, 300 Hz
up to `--bandwidth` (3 kHz; 60 dB down from 10 % above it, and from 254 Hz
down, where the squelch signalling lives). The pre-emphasis rises by 6 dB
per octave across it (750 µs, as receivers de-emphasise it; `--preemph off`
for flat FM) and is normalised at 1 kHz, the reference of these services:
a full-scale 1 kHz tone deviates the whole voice deviation, and a
look-ahead limiter keeps the voice's true peaks within it. A CTCSS tone
(`--ctcss`, 67.0–254.1 Hz) or DCS code (`--dcs`, three octal digits, `I`
for inverted polarity) takes 15 % of `--deviation`
(`--subaudio-deviation`), the voice the rest, so the peak deviation never
exceeds `--deviation` (2.5 kHz for 12.5 kHz channels, 5 kHz for 25 kHz
ones). In DCS a 1 bit deviates the carrier upwards; whether a receiver
calls that normal or inverted polarity is not standardised, so if it does
not open on `023`, try `023I`.

Every board makes carriers from 1.5 MHz up: the Pi 5 and the Pico to
1 GHz, the other Raspberry Pis to 400 MHz, above which their clock
generators' output fades. Higher carriers go out on a harmonic:
`--harmonic 3` makes a third of the frequency, with a third of the
deviation, and its third harmonic carries the signal at the full
deviation. The fundamental then is the strongest emission, so a band-pass
filter at the carrier is mandatory (as is a low-pass for the fundamental
itself: the pin's square wave has strong odd harmonics). A Pico's
modulator moves a UHF carrier by 3 MHz per divider step, and its noise
reaches the neighbouring channels there (adjacent-channel power −24 to
−31 dB, see [Measurements](#nfm-1)); the Raspberry Pis stay below −44 dB.
On a few carriers per band a board's PLL makes a spur right next to the
carrier, which distorts quiet passages: txengine warns of them (see
[How the carrier is made](#how-the-carrier-is-made)), and a neighbouring
channel is usually clear.

### AM

```sh
sudo txengine am tx -f 999k -a music.flac                      # medium wave
sudo txengine am tx -f 6.1M -a talk.mp3 --bandwidth 10000      # short wave (49 m, Pi 5), 10 kHz audio
sudo txengine am tx -f 999k --tone 1000 --gain 6               # 1 kHz at the full depth (90 %)
sudo txengine probe --mode am -f 999k                          # the clock plan
txengine am tx -d pico -f 1503k -a music.flac                  # through a Pico on USB
txengine am render -a music.flac --duration 10 -o envelope.wav # the envelope as WAV
```

The programme is mixed to mono, low-passed at the audio bandwidth
(`--bandwidth`, 4.5 kHz for the 9 kHz channels of Europe, Africa and Asia,
up to 10 kHz; 60 dB from 10 % above it) and peak-limited with look-ahead on
the true (interpolated) peaks. The carrier is `1 / (1 + m)` of the full
output (−5.6 dB at the default depth `--depth 90`), so that the positive
peaks reach the full output and the negative ones never cut the carrier
off, which would splatter into the neighbouring channels. While the output
waits for samples (an underrun), the carrier stays on, unmodulated, at its
recent mean level.

### Control commands

One command per line on `--ctl` (FIFO) or `--ctl-socket` (Unix socket, which
answers with `OK ...` or `ERR ...`). txengine removes the socket, and a FIFO
it created, when it stops:

| Command | Effect |
|---|---|
| `PS text`, `RT text`, `PTYN text\|OFF` | FM: programme service name, RadioText, PTY name |
| `PTY n\|name`, `PI hex`, `DI hex`, `ECC hex`, `LIC hex` | FM: codes |
| `TA 0\|1`, `TP 0\|1`, `MS 0\|1`, `CT 0\|1` | FM: flags |
| `AB` | FM: toggle the RadioText A/B flag (receivers clear the display) |
| `AF 87.6,99.5 \| OFF` | FM: alternative frequencies |
| `RDS 0\|1`, `RDSLEVEL hz` | FM: RDS on/off, injection level |
| `GAIN db` | programme gain |
| `CTCSS hz\|OFF`, `DCS code\|OFF` | NFM: squelch signalling (tone, or code such as `023` or `754I`) |
| `DEPTH percent` | AM: modulation depth of the peaks |
| `PPM`, `PPM auto\|ppm\|off` | report the crystal error correction; measure, fix or turn it off |
| `STATUS` | current settings and counters |
| `QUIT` | stop transmitting |

### Frequency accuracy

Every clock of the transmitter comes from the board's crystal, which is a
few ppm to a few tens of ppm off and drifts with temperature (10 ppm is
876 Hz at 87.6 MHz). That one error shifts the carrier, and it makes the
hardware emit the samples slightly too fast or too slow, which shifts the
pilot and the subcarriers (in NFM the CTCSS tone and the DCS bit rate) and
changes the playback speed. With the default
`--ppm auto`, txengine times the hardware's sample counter against the
system clock while transmitting, fits the crystal error (exponentially
weighted over about two minutes, so it follows a warming board) and
corrects both: the carrier synthesiser is retuned, and the modulation is
rendered for the actual sample rate (pilot, subcarriers, RDS bit rate,
CTCSS tone, DCS bit rate and the programme's playback speed). The hardware
sample clock itself is never
changed, so the correction cannot fill or drain any buffer; a live input
keeps following its own clock through its drift compensation. In AM the
carrier itself paces the samples (each lasts whole carrier cycles), so
correcting the carrier corrects the sample rate as well; txengine measures
the crystal through the corrections.

The system clock has to be synchronised by an NTP daemon (chrony,
systemd-timesyncd, ...); until it is, txengine transmits uncorrected and
says so (`txengine probe` shows the state). The reference is the frequency
the daemon has measured for the system clock, so the result is as good as
that. With chrony (polling a LAN server every 16–64 s) the carriers, pilots
and playback speeds of the Raspberry Pis came within 0.4 ppm (35 Hz at
87.6 MHz); a Pi 4 whose chrony polls internet servers came within 0.1 and
1.0 ppm in two measurements, the latter while chrony removed a time offset
of 0.3 ms by running the clock slightly fast. With systemd-timesyncd, which
polls only every 34 minutes, the error can reach a few ppm (the Picos below,
on a computer with it, came within 1 ppm). Use chrony. The first measurement
is used after about 25 s (on a Pico 25–50 s). The status line shows the
correction in effect, and `PPM` on the control interface reports the
measurement in detail:

```
crystal error +6.73 ppm (measured against the system clock): correcting the carrier by -590 Hz and the sample rate
```

Without network time, give the error (`--ppm 6.44`, positive when the
crystal runs fast; every transmission, and `txengine tone`, prints the
measured value on a board that has NTP); txengine keeps measuring and
reports it, but
applies the given value. `--ppm off` turns the correction off (no
measurement either). For GPS accuracy, let chrony discipline the system
clock from a GPS receiver's PPS signal (gpsd + chrony's `refclock`); txengine
then measures against GPS time.

### Testing without hardware

```sh
txengine wfm render -a music.flac --duration 10 -o mpx.wav   # the multiplex as WAV
txengine wfm analyze mpx.wav                                 # levels, pilot phase, RDS
```

`analyze` accepts any multiplex recording (for example from an SDR; see
`tools/`) and decodes RDS with an independent decoder.

## Hardware

### Boards

| Board | SoC | FM | NFM | AM |
|---|---|---|---|---|
| Raspberry Pi 5 | BCM2712 + RP1 | 4.2 MHz – 1 GHz, tested (64-bit build) | 1.2 MHz – 1 GHz, tested | 100 kHz – 6.2 MHz, tested |
| Raspberry Pi 4, 400 | BCM2711 | 9.1 – 300 MHz, tested on a Pi 4 (64- and 32-bit builds) | 0.8 – 300 MHz, tested on a Pi 4 | 100 kHz – 2.2 MHz, tested on a Pi 4 |
| Raspberry Pi 2 v1.2, 3, 3+ | BCM2837 | 11 – 400 MHz, supported, not tested | 1.1 – 400 MHz, not tested | 100 kHz – 4.2 MHz, not tested |
| Raspberry Pi 2 v1.1 | BCM2836 | 11 – 400 MHz, tested | 1.1 – 400 MHz, tested | 100 kHz – 4.2 MHz, not tested |
| Raspberry Pi 1 | BCM2835 (ARMv6) | not supported | not supported | not supported |
| Raspberry Pi Zero 2 W | BCM2837 (RP3A0) | 11 – 400 MHz, tested (64- and 32-bit builds) | 1.1 – 400 MHz, tested | 100 kHz – 4.2 MHz, tested |
| Raspberry Pi Zero, Zero W | BCM2835 (ARMv6) | not supported | not supported | not supported |
| Raspberry Pi Pico 2 (on USB) | RP2350A | 18.5 MHz – 1 GHz (PIO method: up to 132 MHz), tested (both methods) | 1.5 MHz – 1 GHz (PIO method: up to 132 MHz), tested (both methods) | 64 kHz – 1.71 MHz, tested |
| Raspberry Pi Pico (on USB) | RP2040 | 18.5 MHz – 1 GHz (PIO method: up to 132 MHz), tested (both methods) | 1.5 MHz – 1 GHz, tested | 64 kHz – 1.76 MHz, tested |

The FM and NFM ranges are the carriers the board makes itself, with room
for the deviation (narrow-band FM reaches further down) and for correcting
the crystal; a few spots inside them cannot be modulated (see [Hardware
notes](#hardware-notes)), and higher carriers go out on a harmonic
(`--harmonic`). The Pi 4's FM and NFM ranges count on its firmware leaving
PLLC to the GPU, as it did on the board tested; `txengine probe` shows the
plans for a board. The AM ranges are at the default 100 kS/s; `txengine
probe --mode am` shows the limit for another sample rate.

### Outputs

| | FM, NFM | AM |
|---|---|---|
| Raspberry Pi | GPIO4 (pin 7, ground on pin 9) or GPIO20 (pin 38): GPCLK0; GPIO5 (pin 29) or GPIO21 (pin 40): GPCLK1; GPIO6 (pin 31): GPCLK2; on the Pi 5 also GPIO18 (pin 12): GPCLK1 | GPIO18 (pin 12, ground on pin 14), or GPIO12 (pin 32), GPIO13 (pin 33), GPIO19 (pin 35) |
| Pico, Pico 2 | GP21 (pin 27, ground on pin 28), the only clock output on the header | any GPIO, GP21 unless given |

Several pins of one clock generator transmit together (`-g 4 -g 20`). On
the Pi 5 the audio PLL reaches GPCLK0 and GPCLK1, the video PLL GPCLK2
(GPIO6 needs `--pll video`). On the Pi 2–4 and Zero 2 W a clock generator
may already be in use: on the Pi 2 GPCLK1 clocks the Ethernet and USB chip
(25 MHz), on the Zero 2 W GPCLK2 the WiFi chip's sleep clock (32.768 kHz).
txengine refuses a clock generator that is already running (`txengine
probe` shows it), so use GPCLK0 there; `--force-gpclk` takes it over until
txengine stops, which cuts that connection meanwhile. Their GPCLK2 divides
by whole numbers only: `--pll clkdiv` needs GPCLK0 or GPCLK1, and on the
Pi 4 is not available at all (see [Hardware
notes](#raspberry-pi-24-and-zero-2-w)).

`txengine devices` lists the transmitters found, and `txengine probe -f 87.6`
(or `--mode nfm -f 446.00625`, `--mode am -f 999k`) shows what will be used,
without transmitting.

FM and NFM are a square wave at the carrier: its odd harmonics are strong,
so put a low-pass at the carrier between the pin and anything that can
radiate (on a harmonic, `--harmonic`, a band-pass at the carrier). AM
is a pulse train: at full output the third harmonic is 9.5 dB below the
carrier, and narrower pulses carry even harmonics as well (at the carrier
of 90 % depth the second harmonic is 1.4 dB below the carrier on the Pi 5
and the Pico, and 2.8 dB above it on the Pi 2–4 and Zero 2 W, which make two
pulses per period). The noise-shaped quantisation error is strongest half
the carrier frequency away (simulated −40 to −56 dBc in 9 kHz at 999 kHz).
Put a band-pass, at least a low-pass, at the carrier.

## Building

Static binaries (musl) are cross-compiled with Rust's own linker; no C
toolchain is needed:

```sh
rustup target add aarch64-unknown-linux-musl armv7-unknown-linux-musleabihf
cargo build --release --target aarch64-unknown-linux-musl       # 64-bit OS
cargo build --release --target armv7-unknown-linux-musleabihf   # 32-bit OS
cargo build --release                                            # this computer (for a Pico)
```

The Raspberry Pi 5 needs the 64-bit build: the RP1's DMA engine reads the
samples from main memory over PCIe without cache coherency, and only 64-bit
user space can perform the cache maintenance this requires (the 32-bit build
says so and stops). All other boards work with either build.

The binary is `target/<target>/release/txengine`. Copy it to the Pi; it needs
root to reach the hardware (`/dev/mem`, the VideoCore mailbox, or the RP1
PCIe resources). A Pico needs no root (see below).

On the Pi 2, 3, 4 and Zero 2 W, txengine programs the clock manager, DMA and
PWM registers directly. Kernels built with `CONFIG_IO_STRICT_DEVMEM` (NixOS,
for example; not Raspberry Pi OS) deny `/dev/mem` access to registers that a
kernel driver owns, even to root. On those kernels add `iomem=relaxed` to the
kernel command line (on NixOS: `boot.kernelParams = [ "iomem=relaxed" ];`).
`txengine probe` reports when it is missing.

`txengine -V` prints the version: `git describe --tags --always --dirty` of
the source tree it was built from, that is the latest tag, the number of
commits since it and the commit (before the first tag the commit alone),
with `-dirty` for uncommitted changes. Without `-dirty`, git accepts it as
a revision. Transmissions log it at start-up. A build without git (a
source archive, a sandboxed package build) can be given it in
`TXENGINE_VERSION`; otherwise it is the version in Cargo.toml.

### Pico firmware

The firmware (`firmware/pico`, a cargo workspace of its own) is built for
the Pico or the Pico 2 and loaded with
[picotool](https://github.com/raspberrypi/picotool):

```sh
rustup target add thumbv6m-none-eabi thumbv8m.main-none-eabihf
cd firmware/pico
cargo rp2040 && picotool load -x -t elf target/thumbv6m-none-eabi/release/txengine-pico         # Pico
cargo rp2350 && picotool load -x -t elf target/thumbv8m.main-none-eabihf/release/txengine-pico  # Pico 2
```

The first time, hold the BOOTSEL button while connecting the Pico; after
that `picotool reboot -u -f` (with `--ser SERIAL` if several are connected)
puts it back into its bootloader, and a firmware fault does so by itself.
The firmware uses the Pico SDK's USB IDs (2e8a:000a and 2e8a:0009), to which
picotool's udev rules already give the logged-in user access: txengine needs
no root for a Pico.

The firmware's version has the same form as txengine's. `txengine probe`
shows it, and `picotool info` reads it from the board even in its
bootloader. A txengine refuses firmware that speaks another version of the
USB protocol and asks for an update.

## Measurements

For FM and AM all boards ran txengine and firmware c595393, on 2026-10-19
and 20; the Raspberry Pi 4 ran this version, on 2026-10-23. FM was
received over the air by a USRP B210 with a short antenna, the Pi 4's
through a wire (`tools/sdr_capture.py`, demodulated at 4 MS/s, then a
pilot-locked stereo decoder with 50 µs de-emphasis). AM went through an RC
tap (1 kΩ, 470 pF, 10 kΩ, 1000 pF ∥ 50 Ω) from the pin into a LimeSDR
Mini, detected synchronously.

### FM

| | Raspberry Pi 5 | Raspberry Pi 4 | Raspberry Pi 2 | Raspberry Pi Zero 2 W | Pico 2, PLL | Pico 2, PIO | Pico, PLL | Pico, PIO |
|---|---|---|---|---|---|---|---|---|
| Sample rate | 250 kS/s | 228 kS/s | 190 kS/s | 190 kS/s | 166.7 kS/s | 190.2 kS/s | 166.7 kS/s | 190.2 kS/s |
| Pilot (target 6.75 kHz) | 6.76 kHz | 6.76 kHz | 6.76 kHz | 6.76 kHz | 6.75 kHz | 6.75 kHz | 6.75 kHz | 6.75 kHz |
| 38 kHz carrier residual | 0.001 % | 0.003 % | 0.003 % | 0.003 % | 0.003 % | 0.001 % | 0.002 % | 0.005 % |
| Stereo separation (left only, 1 kHz) | 51 dB | 53 dB | 51 dB | 50 dB | 65 dB | 85 dB | 64 dB | 77–87 dB |
| RDS injection (target 2.0 kHz) | 2.04 kHz | 2.03 kHz | 2.06 kHz | 2.04 kHz | 2.03 kHz | 2.01 kHz | 2.04 kHz | 2.05 kHz |
| RDS block errors (stereo music) | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % |
| Crystal error, uncorrected | −55.5 ppm | +18.9 ppm | +8.1 ppm | +3.7 ppm | −4.5 ppm | −4.7 ppm | +9.9 ppm | +9.5 ppm |
| … with `--ppm auto` | −0.3 ppm | +0.1 ppm | (reference) | −0.4 ppm | −0.8 ppm | −0.4 ppm | −1.0 ppm | −0.5 ppm |
| Audio THD (1 kHz, 0 dBFS) | −97 dB | −99 dB | −89 dB | −85 dB | −94 dB | −88 dB | −98 dB | −85 dB |
| Audio frequency response (31.5 Hz – 15 kHz) | ±0.04 dB | ±0.06 dB | ±0.04 dB | ±0.04 dB | ±0.02 dB | ±0.03 dB | ±0.03 dB | ±0.05 dB |
| Audio SNR, mono / stereo (unweighted) | 71 / 53 dB | 68–74 / 50–59 dB | 74 / 56 dB | 70–73 / 52–55 dB | 68 / 50 dB | 71 / 52 dB | 71 / 53 dB | 71 / 54 dB |
| Audio SNR, mono / stereo (ITU-R 468 weighting, rms) | 64 / 45 dB | 62–68 / 40–52 dB | 66 / 48 dB | 63–66 / 45–47 dB | 61 / 42 dB | 64 / 45 dB | 64 / 45 dB | 64 / 46 dB |
| Effective bit depth, mono / stereo | 10.5 / 7.6 bits | 10.1–11.1 / 7.0–8.7 bits | 11.1 / 8.1 bits | 10.5–11.0 / 7.5–8.0 bits | 10.1 / 7.1 bits | 10.6 / 7.4 bits | 10.6 / 7.5 bits | 10.6 / 7.7 bits |
| Phase noise at 10 kHz / 100 kHz / 1 MHz | −91 / −95 / < −101 dBc/Hz | −98 / −97 / −101 dBc/Hz | < −94 / < −91 / < −93 dBc/Hz | −93 / −96 / < −102 dBc/Hz | −85 / −88 / −96 dBc/Hz | −95 / −95 / < −99 dBc/Hz | −92 / −86 / −95 dBc/Hz | −91 / −91 / < −99 dBc/Hz |
| Modulation THD (10 kHz at 75 kHz deviation) | −76 dB | −77 dB | −62 dB | −60 dB | −62 dB | −57 dB | −65 dB | −51 dB |
| Strongest line more than 1 MHz from the carrier (82–112 MHz) | none above −60 dBc | none above −55 dBc | none above −43 dBc | −57 dBc (85.0 MHz) | −41 dBc (±1.2 MHz) | −28 dBc (97.2 MHz) | −34 dBc (94.8 MHz) | −22 dBc (92.4 MHz) |
| CPU (stereo + RDS, network stream) | 3.3 % | 6.6 % at 1.5 GHz | 53 % at 600 MHz (throttled) | 18 % at 1 GHz | 2 % on the computer | 2 % | 2 % | 2 % |

* **Audio figures**: a pilot-locked stereo decoder with 50 µs de-emphasis,
  unweighted, 20 Hz – 15 kHz. A 0 dBFS tone deviates 40 kHz, not 65 kHz,
  because of the −4.3 dB pre-emphasis drive correction. The frequency steps
  of the hardware limit the modulation to about 20 bits on the Pi 5, 17 on
  the BCM boards and 15.6 on the Pico (16-bit samples of about 3 Hz); the
  multiplex as computed (`txengine wfm render`) has a noise floor of
  −128 dBFS (21 bits).
* **Noise**: the decoded silence of a stereo transmission (two rounds; the
  Pi 4 three), re a sine at 75 kHz deviation, without local interference:
  the spectrum at 0.5 Hz resolution, its median over 2 s segments (drops
  bursts), then its running median over 300 Hz (drops narrow lines),
  calibrated on white noise. The weighted row applies the ITU-R BS.468
  weighting and reads rms, not quasi-peak. The effective bit depth follows
  from the unweighted SNR: (SNR re 0 dBFS − 1.76 dB) / 6.02 dB, a 0 dBFS
  tone deviating 40 kHz.
* **Phase noise**: an unmodulated carrier, the median of both sidebands over
  ±5 % of the offset; at 10 kHz also from the noise floor of the decoded
  silence above. The lowest of the readings (an evening and two morning
  rounds; the Pi 4 three evening rounds), as interference only adds; "<"
  where the receiver's own floor was reached.
* **Over the air**, the reception limits the noise figures: the boards
  measured in the morning read about 71 dB mono and 53 dB stereo, though
  their carriers are made in entirely different ways, while the Pi 2,
  measured later with the antenna placed differently, reads 74 dB. The
  hardware notes' phase noise figures, from earlier measurements under
  better reception, are lower. In the other measurements the Pi 2 arrived
  15 dB below the others. The Pi 4, through a wire at the level of the
  others (−31 dBFS), read 68–69 dB mono and 50–53 dB stereo in two rounds
  and 74 / 59 dB an hour later, when its phase noise at 10 kHz was 7 dB
  lower too: most of the noise comes in with the reception.
* **Modulation THD**: `txengine tone --tones 10000 --deviation 75000`, the
  harmonics up to 100 kHz in the demodulated signal, without any of the
  programme's processing.
* **Lines**: unmodulated carriers, 82–112 MHz at 20 MS/s, kept only where a
  line is at least 10 dB above the spectrum with the transmitter off, at the
  same frequency for two tunings of the receiver (not an image) and at the
  same level re the carrier at 10 dB less receiver gain (not
  intermodulation in it); channels with a station are skipped.
* **Crystal**: the receiver's own error calibrated against the Pi 2 with
  `--ppm auto` (its chrony keeps its clock within 0.03 ppm). The Picos run
  from a laptop with systemd-timesyncd, which leaves them within about a
  ppm.
* **CPU**: of one core, all of txengine's threads, the audio input's
  resampling (from the stream's 48 kHz, with the drift correction) included:
  on the Pi 2 that is 18 % of the 53 %. For a Pico, txengine runs on the
  computer (an Intel Core Ultra 7 155H here).

### NFM

Measured on 2026-10-23 with this version and firmware c595393, by the same
receiver: a ±8 kHz channel
demodulated at 48 kHz, 750 µs de-emphasis for the audio figures, all at
±2.5 kHz deviation (CB ±2 kHz), compared with `txengine nfm render` of the
same input. Raspberry Pis transmitted from GPIO4, the Picos from GP21 with
the PLL method.

| | Raspberry Pi 5 | Raspberry Pi 4 | Raspberry Pi 2 | Raspberry Pi Zero 2 W | Pico 2 | Pico |
|---|---|---|---|---|---|---|
| Sample rate | 250 kS/s | 228 kS/s | 190 kS/s | 190 kS/s | 166.7 kS/s | 166.7 kS/s |
| 1013 Hz tone at half scale (render: 1263.9 Hz) | 1264.1 Hz | 1264.2 Hz | 1263.9 Hz | 1263.9 Hz | 1263.8 Hz | 1263.9 Hz |
| Audio frequency response (310 Hz – 2.99 kHz), re the render | ±0.004 dB | ±0.009 dB | ±0.006 dB | ±0.007 dB | ±0.009 dB | ±0.012 dB |
| Audio THD (1013 Hz at half scale) | −81 dB | −74 dB | −75 dB | −73 dB | −78 dB | −75 dB |
| Audio SNR (300 Hz – 3 kHz, re ±2.5 kHz) | 63 dB | 63 dB | 59 dB | 60 dB | 63 dB | 59 dB |
| Phase noise at 1 kHz / 10 kHz | −88 / −98 dBc/Hz | −90 / −94 dBc/Hz | −87 / −89 dBc/Hz | −87 / −94 dBc/Hz | −90 / −85 dBc/Hz | −79 / −84 dBc/Hz |
| Adjacent channel (±12.5 kHz, in 8.5 kHz), unmodulated | −56 dB | −53 dB | −49 dB | −50 dB | −43 dB | −41 dB |
| CTCSS 88.5 Hz (target ±375 Hz) | 374.9 Hz | 374.8 Hz | 374.8 Hz | 374.8 Hz | 374.8 Hz | 374.8 Hz |
| DCS 023 | 32 of 32 words, no bit errors | 32 of 32, none | 32 of 32, none | 32 of 32, none | 32 of 32, none | 32 of 32, none |
| Peak deviation, pink noise 12 dB over full scale (limit 2.5 kHz) | 2.49 kHz | 2.49 kHz | 2.50 kHz | 2.51 kHz | 2.49 kHz | 2.53 kHz |
| Occupied bandwidth (99 %), the same | 6.1 kHz | 6.1 kHz | 6.1 kHz | 6.1 kHz | 6.1 kHz | 6.2 kHz |
| Carrier with `--ppm auto` | −0.4 ppm | +1.0 ppm | (reference) | 0.0 ppm | +0.7 ppm | +1.2 ppm |
| 446.00625 MHz (PMR446 channel 1): audio SNR / adjacent channel | 60 / −54 dB | 57 / −47 dB (`--harmonic 3`) | 53 / −44 dB (`--harmonic 3`) | 56 / −49 dB (`--harmonic 3`) | 48 / −31 dB | 42 / −24 dB |
| 27.185 MHz (CB channel 19, ±2 kHz): audio SNR / THD | 66 / −81 dB | 58 / −75 dB | 60 / −76 dB | 58 / −77 dB | 65 / −84 dB | 64 / −79 dB |
| CPU (48 kHz stereo stream, CTCSS) | 1.3 % | 2.8 % at 1.5 GHz | 22 % at 900 MHz | 6.8 % at 1 GHz | 1.4 % on the computer | as the Pico 2 |

* **Carrier**: 145.5125 MHz unless given, a channel whose plan is clear of
  the PLLs' spurs on every board (see [How the carrier is
  made](#how-the-carrier-is-made); at 145.5 MHz the Picos use the 12 MHz
  reference).
* **Audio figures** measure the received deviation against the render's,
  which the hardware reproduces to within 0.02 %. The SNR is that of
  silence re a ±2.5 kHz sine, from the spectrum at 1 Hz resolution (median
  over time, then a running median over 51 Hz): the received carrier is
  frequency-modulated on the 100 Hz mains grid by the local environment, on
  every board and on unmodulated carriers alike, which puts lines up to
  60 dB above the noise floor (in 1 Hz bins) into the band; with them the
  SNR reads 33–50 dB. THD at 1013 Hz, whose harmonics stay off that grid.
* **Peak deviation**: the 99.99th percentile of the received deviation,
  its noise included; as rendered the limited signal never exceeds
  ±2.5 kHz.
* **Phase noise and adjacent channel**: the unmodulated carrier, line-free
  as above; the adjacent-channel power comes from the phase noise 8–17 kHz
  out. A mains-locked cluster of lines around ±16 kHz, from outside, puts
  the figures with the lines 1–16 dB higher.
* **UHF**: the Picos' modulator moves the carrier by 3 MHz per divider step
  at 446 MHz (division 2), so its noise spreads over the neighbouring
  channels (99 % bandwidth with a tone: 55 kHz on the Pico 2, 80 kHz on
  the Pico). On the third harmonic (`--harmonic 3`, division 10) the Pico 2
  reads 52 / −34 dB with 4.2 kHz of bandwidth, the Pico 47 / −27 dB with
  30 kHz. The Raspberry Pi 2–4 and Zero 2 W reach 446 MHz only on a
  harmonic.
* **CB** was received on the third harmonic (81.555 MHz, below the B210's
  range at the fundamental), with the deviation scaled back. The Pico 2's
  PIO method measures 64 dB and −82 dB there.
* **PLLA far down**: 172.0 MHz takes PLLA to −23.6 % (1720 MHz); the Zero
  2 W measures there as at 145.5 MHz. The Pi 2 read −79 dBc/Hz at 1 and
  10 kHz there, about as at 156–168 MHz with PLLA anywhere from +2.7 % to
  −16.6 % (−73 to −84 dBc/Hz): its reception limits these figures.
* **`--ppm auto`** follows the frequency the NTP daemon sets for the system
  clock, and chrony removes a time offset by running the clock slightly fast
  or slow for a while: on the Pi 4, catching up 0.2–0.3 ms, its frequency
  was 0.5–1.3 ppm away from chrony's own estimate of the crystal, which put
  the carrier +1.0 ppm off here (+0.1 ppm in the FM measurement).
* **CPU**: of one core, the audio input's resampling from 48 kHz included.

### AM

| | Raspberry Pi 5 | Raspberry Pi 4 | Raspberry Pi Zero 2 W | Pico 2 | Pico |
|---|---|---|---|---|---|
| Carriers | 100 kHz – 6.2 MHz | 100 kHz – 2.2 MHz | 100 kHz – 4.2 MHz | 64 kHz – 1.71 MHz | 64 kHz – 1.76 MHz |
| Width steps up to the full output (999 kHz) | 100 | 62 | 62 | edges placed to the clock cycle (264 MHz) | edges placed to the clock cycle (264 MHz) |
| Crystal error, uncorrected (re the Lime) | −54.8 ppm | +18.8 ppm | +3.7 ppm | −4.4 ppm | +9.7 ppm |
| Carrier with `--ppm auto` (re the Lime) | within 0.3 Hz | within 1 Hz | within 0.2 Hz (2.5 Hz at 3.5 MHz) | within 1.5 Hz | within 0.3 Hz |
| 1 kHz at 90 % reads | 89.6 % (999 kHz), 90.0 % (1.602 MHz), 89.5 % (3.5 MHz), 88.8 % (6.1 MHz) | 89.9 % (999 kHz), 89.7 % (1.602 MHz), 89.5 % (2.2 MHz) | 90.4 % (999 kHz), 89.2 % (1.602 MHz), 88.8 % (3.5 MHz) | 90.6 % (999 kHz), 89.6 % (1.602 MHz) | 91.4 % (999 kHz), 91.2 % (1.602 MHz), 90.8 % (1.71 MHz) |
| Harmonics of the tone | below −34 dB | below −41 dB | below −37 dB | below −40 dB | below −39 dB |
| Phase modulation (1 kHz at 90 %) | < 0.13° | < 0.15° | < 0.15° | < 0.08° | < 0.36° |
| CPU (network stream) | 1.9 % (999 kHz), 5.3 % (6.1 MHz) | 4.3 % (999 kHz), 7.3 % (2.2 MHz) at 1.5 GHz; 32-bit 6.0 %, 8.8 % | 10.1 % (999 kHz), 18.8 % (3.5 MHz); 32-bit 15.5 %, 24.6 % | as the Pico | 1.4 % on the computer |

* **The receive path** limits what these show. Through the tap the Lime gets
  the carrier at about −90 dBFS against a noise floor that leaves a
  carrier-to-noise density of only about 40 dBHz: enough for the carrier and
  the depth over a 12 s average, not for the audio noise (it reads 2–4 dB
  below 100 % modulation, the Lime's own floor) or for harmonics below about
  −35 dB. The Lime's input also distorts with its gain: at its lowest gain
  (−12 dB) a 90 % tone reads within 1.5 % of 90 %, at 8 dB up to 120 % on
  the Pico and 99 % on the Pico 2; the transmitters' own distortion
  simulates below −100 dB. On long wave (153 kHz) the Lime is too
  insensitive to judge the modulation.
* **Underruns and faults**: while the output waits for samples the carrier
  stays on at its recent mean level, unmodulated; a 0.3 s stall of txengine
  was survived on every board. After `kill -9` the guardian restores a
  Raspberry Pi's registers, and a Pico's firmware switches the carrier off
  one second later; the next transmission starts normally.

## How it works

### Features in detail

* **Hardware-timed output.** Every sample reaches the carrier synthesiser by
  DMA (on the Pico by its PIO and DMA), paced by hardware: the CPU only has
  to keep a buffer of about 60 ms filled. Register writes are checked (lock,
  busy and status bits, read-back, frequency counter); nothing relies on
  sleeping.
* **FM multiplex** (all derived from one phase accumulator, so pilot, 38 kHz,
  57 kHz and the RDS bit clock are exactly locked): 50/75 µs pre-emphasis,
  15 kHz programme low-pass, look-ahead limiter that bounds the true
  (inter-sample) composite peak, 19 kHz pilot, DSB-SC stereo subcarrier,
  RDS, and pre-compensation of the output stage's measured frequency
  response (sample-and-hold, interpolation and, on the Pi 2, the Zero 2 W
  and the Pico, the PLL loop).
* **FM levels**: the drive is lowered by the rise of the multiplex power the
  pre-emphasis causes for pink noise (4.3 dB at 50 µs), so a programme keeps
  about the MPX power (ITU-R BS.412) it has without pre-emphasis; the status
  line meters it over the last 60 s. The limiter is pre-emphasis-aware: a
  high-frequency overload only reduces the pre-emphasis boost for a moment
  (less treble), and the whole programme is only turned down when it is
  over full scale without pre-emphasis. A programme at 0 dBr is normally
  transmitted unlimited.
* **RDS**: PS, RadioText (A/B flag, 0x0D termination), PTY, PTYN, PI, TP/TA,
  MS, DI, alternative frequencies (method A), clock time (placed at the
  minute edge using the actual emission time), ECC/LIC; EBU Latin character
  set (e.g. Czech diacritics); data-channel shaping exactly as specified.
* **NFM**: voice band 300 Hz to 3 kHz (elliptic high-pass, linear-phase
  low-pass), 750 µs pre-emphasis normalised at 1 kHz, a look-ahead limiter
  on the true peaks, and a CTCSS tone (EIA/TIA-603) or DCS code (Golay
  23-bit words at 134.4 bit/s, raised-cosine bit transitions that keep the
  code out of the voice band) at its own deviation; the same carrier
  synthesis as FM, from 1.5 MHz to 1 GHz (400 MHz on the Pi 2–4 and Zero
  2 W), and on a harmonic beyond.
* **AM**: double sideband with full carrier (A3E), made by pulse widths that
  keep the carrier's phase, one noise-shaped width for every carrier cycle;
  programme low-pass at the channel's audio bandwidth (4.5 kHz, up to
  10 kHz) and a look-ahead limiter on the true peaks; 100 kHz to 6.2 MHz on
  the Pi 5, to 4.2 MHz on the other Raspberry Pis, to 1.76 MHz on the Pico
  and 1.71 MHz on the Pico 2.
* **Crystal error correction** (all modes): the board's crystal error is
  measured continuously against the system clock (NTP, or GPS through
  chrony) and corrected in the carrier frequency, the subcarriers (FM), the
  CTCSS tone and DCS bit rate (NFM) and the playback speed; no calibration,
  and it follows temperature drift. A known
  error can be given instead (`--ppm`).
* **Audio input**: WAV, FLAC, MP3, OGG/Vorbis, AAC/MP4 (symphonia), raw PCM
  on standard input, or a test tone; resampled with rubato; clock-drift
  compensation for live input. A WAV stream on standard input plays until
  it ends, whatever length its header declares.
* **Run-time control** through a FIFO or a Unix socket (PS, RT, PTY, TA, AF,
  gain, CTCSS/DCS, depth, ...), status reporting, clean shutdown and a
  guardian process that restores every touched register even after
  `kill -9`.
* **Testing tools**: render to a WAV/IQ file, analyse a recorded multiplex
  (levels, pilot phase, RDS decode), transmit raw test tones, probe the
  frequency plan.

### How the carrier is made

**FM** moves the frequency of a clock that drives the pin, one value per
sample. **AM** keeps the frequency and varies the waveform: a GPIO can only
switch, so the carrier's amplitude comes from the width of one pulse per
carrier period, centred on the same instant of every period. Its
fundamental is `sin(π w / P)` of the full square wave's (width `w`, period
`P`), and its phase does not move with the width (a pulse starting at the
period's start would move it by up to 90°). The widths the hardware has are
far too coarse by themselves, so they are noise-shaped (second-order error
feedback), which moves the quantisation error from the carrier towards
±fc/2: within ±5 kHz of a 999 kHz carrier it is 89 dB below the carrier
(simulated, Pi 5). Each sample lasts a whole number of carrier cycles, so
in AM the carrier paces the samples.

**Spurs of the PLLs.** A fractional PLL makes a spur next to its output
when its VCO is close to a half multiple of the frequency its phase detector
compares at (19.2 MHz on the Pi 2, 3 and Zero 2 W, 50 MHz on the Pi 5; on
the Pico, at 6 MHz, also when the carrier is on a multiple of 2 or
1.5 MHz), offset from the carrier by the distance from that point. The
modulation drags it through the audio band, where it distorts quiet
passages by some 10–30 Hz of deviation: near 145 MHz the harmonics of a
1 kHz tone at 1.25 kHz deviation reached −33 to −42 dB on a Pico, −42 dB on
a Zero 2 W and −51 dB on a Pi 5, and stayed below −66 dB one 12.5 kHz
channel away. In narrow-band FM that is far above the noise; in wide-band
FM, which deviates thirty times more, it is about at the noise floor (at
90.0 MHz a Pico 2 put the second harmonic of a tone 30 dB below full scale
at −39 dB, −69 dB re full scale). txengine therefore chooses the PLL's
division to keep the carrier clear of these points, by an eighth of the largest
deviation plus 30 ppm for the crystal correction, and on a Pico in
narrow-band FM, if only that helps, the 12 MHz reference (twice the
modulator's steps: 6 dB more phase noise; at 145.5 MHz −59 dB instead of
−41 dB). Where neither helps txengine warns: in the FM band 62 of 2051
carriers on a Pico (next to multiples of 1.5 and 2 MHz), 6 on the Pi 2, 3
and Zero 2 W (next to multiples of 9.6 MHz), none on the Pi 5; in the 2 m
band carriers next to multiples of 2 and 3 MHz on a Pico, of 9.6 MHz on
the other Raspberry Pis. Measured with the planner's choice, the Pi 5 at
147.5 MHz went from −51 to −73 dB. Thirds and quarters of the comparison
frequency are milder (−50 to −60 dB) and not avoided.

#### Raspberry Pi 5 (RP1)

* **FM**: fractional-N modulation of the RP1 audio or video PLL (24-bit
  fraction, < 0.1 Hz steps), at 250 kS/s. Carriers up to 1 GHz, the PLL
  itself at the top, from 4.2 MHz in wide-band FM and 1.2 MHz in
  narrow-band FM (0.6 MHz on GPIO 5, 18 and 21), except within a few
  hundred kHz of multiples of 50 MHz (see [Hardware notes](#raspberry-pi-5)).
* **AM**: PWM0 runs in phase-correct mode from `pll_audio` at up to
  400 MHz, beyond the datasheet's 150 MHz (the counter and the pin follow),
  and the DMA engine gives it one width per carrier cycle, in steps of two
  clocks: 100 steps up to the full square wave at 999 kHz, 16 at 6.1 MHz.
  The carriers run from 100 kHz to 6.2 MHz (the DMA engine moves at most
  6.25 million words per second into the PWM).

#### Raspberry Pi 2–4 and Zero 2 W (BCM283x, BCM2711)

* **FM**: fractional-N modulation of PLLA, PLLC or PLLD (20-bit fraction,
  1–3 Hz steps), or a dithered fractional clock divider (`--pll clkdiv`), at
  190 kS/s (Pi 4: 228 kS/s). The PLL is chosen automatically: on the Pi 2/3
  and Zero 2 W PLLA, on the Pi 4 PLLC, which only clock the GPU's video and
  3D blocks there and may therefore move down by up to 25 % (12 % reach the
  whole FM band on the Pi 2/3 and Zero 2 W); PLLs that clock the rest of the
  system (VPU, SD card, WiFi, UARTs) are only retuned within ±3 %. On the
  Pi 2/3 and Zero 2 W, PLLA and PLLC run with a wider loop than the firmware
  gives them (loop gain KI 5 instead of 2), which removes most of their
  phase noise from the multiplex band. Carriers up to 400 MHz, above which
  the clock generators' output fades, from 11 MHz in wide-band FM and
  1.1 MHz in narrow-band FM; on the Pi 4, whose clock generators take at
  most 600 MHz, up to 300 MHz, from 9.1 and 0.8 MHz (see [Hardware
  notes](#raspberry-pi-24-and-zero-2-w)).
* **AM**: this PWM has no phase-correct mode. In mark-space mode it puts out
  one pulse at the start of each range period, so it runs two periods per
  carrier cycle with pulses of `P/4 − u` and `P/4 + u` clocks: their
  fundamental is `sin(2π u / P)`, the law of one centred pulse of width
  `2u`, again with a constant phase. The PWM clock is a PLL (an idle one,
  else PLLA, on the Pi 4 PLLC) divided by an integer at up to 250 MHz,
  retuned so that the carrier period is a whole number of clocks: 62 steps
  up to the full output at 999 kHz, 27 at 2.3 MHz. Carriers run from 100 kHz
  to 4.2 MHz (two words per cycle: 8.4 million words per second; on a
  Zero 2 W the DMA engine kept up to 10), on the Pi 4, whose DMA engine is
  slower, to 2.2 MHz.

#### Pico and Pico 2 (RP2040, RP2350) on USB

txengine renders the samples on the computer; the firmware puts them out.

* **FM, PLL method** (the default): the RP2's PLLs have integer feedback
  dividers only, so the firmware switches `PLL_USB`'s divider between
  neighbouring values with a second-order sigma-delta modulator, 3 million
  times a second, by DMA paced by a PIO state machine that is aligned to the
  crystal; 6 MHz reference (crystal / 2; in narrow-band FM 12 MHz where
  only that keeps the carrier clear of the PLL's spurs), 166.7 kS/s (18
  updates per sample). Carriers up to 1 GHz (above 160 MHz the clock
  output passes `PLL_USB`'s post dividers on undivided), from 18.5 MHz in
  wide-band FM and 1.5 MHz in narrow-band FM.
* **FM, PIO method** (`--synthesis pio`): a PIO state machine puts the
  carrier out as a bit stream at the system clock, which runs from
  `PLL_USB` at the bit rate (240–264 Mbit/s) and 1.20 V core voltage while
  it transmits (both are restored afterwards); 190.2 kS/s (41 words per
  sample). Each bit is the square wave's sign at that instant, so the
  carrier has no dithering noise and the PLL's loop is out of the modulation
  path (the response is exact), but there are lines all over the FM band
  (see [Hardware notes](#pico-and-pico-2)). Carriers below half the bit
  rate. For a dummy load or a cable to a receiver.
* **AM**: a PIO state machine puts out one pulse per carrier cycle, low for
  one count of system clock cycles and high for another, at the raised
  system clock of the PIO method (264 MHz from `PLL_USB`, 1.20 V). The
  carrier period is not a whole number of clocks (999 kHz is 264.26), so
  the firmware places every pulse's edges itself: the carrier is an exact
  fraction of the clock (`f · k / 2³²`, 0.06 Hz steps, which is also how
  the crystal is corrected), and each pulse is chosen so that the error of
  the carrier's complex envelope, computed from the pulse's integer edges,
  is noise-shaped (second-order error feedback on the phasor; shaping the
  edges' times instead lets the shaped noise intermodulate back into the
  audio band). Simulated, the error within ±5 kHz is 98 dB below the
  carrier at 153 kHz, 106 dB at 999 kHz and 111 dB at 1.6 MHz, for a 1 kHz
  tone at 90 %; a steady carrier has no line above −100 dBc within 20 kHz.
  The full output is the square wave less 10·(π/P)², 0.01 dB at 999 kHz,
  the headroom the noise shaping needs. Core 1 of the RP2040 runs this in
  assembly, with its interpolators as single-cycle table lookup and scratch
  registers: about 120 clock cycles per carrier cycle and 280 per sample,
  so carriers up to 1.76 MHz at the default 100 kS/s. The RP2350 runs the
  Rust version, which its Cortex-M33 gets through in about 131 cycles per
  carrier cycle and 157 per sample: carriers up to 1.71 MHz. The PLL method
  can only modulate the frequency, so `--synthesis pll` is refused for AM.

## Hardware notes

### Raspberry Pi 5

* **PCIe link power saving** (ASPM L1) occasionally delays the DMA enough to
  lose a timer tick, shifting the output timeline by one sample. txengine
  disables ASPM on the RP1 link while transmitting and restores it
  afterwards.
* **Noise** (FM): about −104 dBc/Hz at 10 kHz, with equal AM noise: noise
  added after the PLL, not phase noise of it. It is the same for every
  `pll_audio` setting (VCO 1050–1750 MHz, integer or fractional,
  dividers), for `pll_sys`, and for any pad drive or slew.
* **Carriers near multiples of 50 MHz** cannot be modulated: the PLL's
  VCO is a whole multiple of the carrier, so at a multiple of the 50 MHz
  reference every division puts the feedback divider's fraction at a whole
  number, which the modulation would have to cross (it only writes the
  fraction). Around 450 MHz that excludes ±270 kHz, mostly the room
  reserved for correcting the crystal (±200 ppm).
* **Up to 1 GHz:** the PLL itself drives the pin at the top of the range
  (measured at 1001 MHz); above, the frequency counter that verifies the
  PLL no longer counts it.

### Raspberry Pi 2–4 and Zero 2 W

* **DMA write rate:** a DMA write to a PLL's fractional register takes about
  4.3 µs (Pi 2, Zero 2 W, probably other BCM283x boards), which caps the
  sample rate at about 231 kS/s. txengine measures the capacity at start-up
  and picks the highest even multiple of 19 kHz within 85 % of it, 190 kS/s
  (`--sample-rate` above the capacity is refused), and it watches the
  achieved rate while running. The Pi 4's engine writes in about 1.5 µs
  (646 kS/s), so it runs at 228 kS/s, the highest such rate up to the
  250 kS/s txengine plans by default.
* **Sample rate and stereo decoders:** the output holds each sample, so the
  multiplex is repeated around multiples of the sample rate, attenuated
  only by the hold: the pilot's images at `fs ± 19 kHz` are 19 dB below it.
  An analogue stereo decoder switches with a 38 kHz square wave, whose 3rd,
  5th and 7th harmonics (114, 190, 266 kHz) demodulate whatever the
  receiver's IF passes there. At the former rates (193–196 kS/s, chosen
  with 1 kHz steps) the pilot's image at 174–177 kHz beat with 190 kHz into
  a tone: for the Zero 2 W at 195.9 kS/s (176.9 kHz against 190 kHz) a
  model of such a decoder, fed with the received signal, gives 13.1 kHz at
  −71 dB re 75 kHz (−82 dB behind a ±150 kHz IF). At an even multiple of
  19 kHz every such product falls on 19 kHz or onto the programme's own
  frequencies, so 190 kS/s it is: the images then lie exactly on the 9th
  and 11th harmonic of the received pilot, and the model's tone is gone
  (−96 dB on the Zero 2 W, its noise floor). From PLLD_PER (500 MHz) the
  nearest integer divider gives 189.97 kS/s, so the pacer uses a fractional
  (MASH 1) one: it moves each sample by at most 2 ns, and interleaved runs
  on a Pi 2 measured the same noise (within 0.3 dB) as with the integer
  divider. The Pi 5 (250 kS/s; its pacer divides 50 MHz by an integer,
  which gives no multiple of 19 kHz near it) has its images at 231 and
  269 kHz: only the 7th harmonic reaches one (3 kHz, −67 dB in the model
  with an unlimited IF), and a ±150 kHz IF removes it (−90 dB).
* **PLLA's and PLLC's loop:** with the firmware's loop gain (KI 2) the loop
  bandwidth is about 125 kHz. That leaves the VCO's phase noise around
  10–30 kHz offset, in the audio and stereo bands, poorly suppressed
  (Pi 2: 44 dB stereo SNR), and makes the modulation response rise by
  0.57 dB with 6° of lag at 67 kHz (35 dB separation uncompensated).
  txengine raises the gain to KI 5 while modulating (about 600 kHz
  bandwidth): on the Pi 2 the stereo SNR rises to 54 dB, the response is
  flat within 0.12 dB (the rest is measured and compensated) and the
  separation is 55 dB from 100 Hz to 14 kHz (Zero 2 W: 51–53 dB at 5–14 kHz
  instead of 37–44 dB; its noise does not change). Costs: the wider loop's
  noise peak at 300–600 kHz offset (+1 dB in the neighbouring 200 kHz
  channels), and the 3rd harmonic of a full-scale 1 kHz tone at −80
  instead of −86 dB. The firmware's gain is restored afterwards. PLLA and
  PLLC are the same design: at the same VCO frequency their responses agree
  within 0.01 dB, at KI 2 and at KI 5 (Pi 2, PLLC: stereo SNR 53.2 dB and
  separation 58–60 dB at KI 5, 44.6 dB and 33 dB at KI 2 uncompensated).
  The response is not perfectly stable: between two sessions it moved by
  0.15 dB at 67 kHz on both boards, and by 0.05 dB under CPU and network
  load, which leaves the separation unchanged.
* **Noise** (FM): phase noise of the PLLs (PM 8–16 dB above AM). PLLA, PLLC
  and PLLD measure the same on both boards, and neither the GPCLK divider,
  the pad drive nor the slew rate changes it; integer-N mode is 6 dB worse
  than fractional. With the firmware's loop gain it was −91 dBc/Hz (Pi 2)
  and −86 dBc/Hz (Zero 2 W) at 10 kHz; txengine's wider PLLA loop brings the
  Pi 2 to −101 dBc/Hz (stereo 44 → 54 dB).
* **Choosing the PLL:** retuning a PLL moves every clock derived from it.
  PLLC clocks the VPU, from which the SD card host, SPI, I2C and the mini
  UART (the serial console on boards with WiFi) take their clocks, and the
  WiFi's SDIO host. The SD and SDIO buses run at their 50 MHz maximum and a
  UART tolerates about ±3 % of baud rate error, so PLLC (like PLLD: PL011
  UART, HDMI) stays within ±3 %, which reaches only half of the band. PLLA
  only clocks the GPU's video and 3D blocks, idle on a transmitter, so
  txengine prefers it and lets it move down by up to 25 % (the blocks run
  slower; upwards the 3 % limit stays), retuning it as little as keeps the
  carrier clear of the PLL's spurs (see [How the carrier is
  made](#how-the-carrier-is-made)). −12 % reach every carrier in
  87.5–108 MHz on a 10 kHz raster from the firmware's 2250 MHz (21 of them
  go further, to −22 %, to stay clear), and the response,
  noise and separation measure the same at −14 % (VCO 1927 MHz) as at
  +1.2 %. For narrow-band FM the full −25 % reaches every channel of CB,
  the 10, 6, 4, 2 and 1.25 m bands and marine VHF, and of 148–174 MHz all
  but those within 50 kHz of 153.6, 166.4 and 172.8 MHz: a fractional PLL
  cannot be modulated with its fraction near a whole number, and at these
  carriers every division the PLL can make puts it there. PLLC moved by
  −3.6 % caused no SD or WiFi errors and no lost packets on either board.
* **Clock generators above 400 MHz:** a GPCLK passes 380 MHz like any
  lower carrier, but above about 400 MHz its output fades. Measured against
  the third harmonic of a carrier at a third of the frequency (a square
  wave's third harmonic is 9.5 dB below its fundamental), a Zero 2 W's
  fundamental was 8 dB short at 400 MHz and 29 dB short at 446 MHz, a
  Pi 2's 14 dB at 446 MHz. txengine plans carriers up to 400 MHz and
  transmits higher ones on a harmonic (`--harmonic 3`).
* **GPCLK2 has no MASH control and no frequency counter input** (Pi 2):
  its MASH field takes no writes and reads 1 while it runs, and the
  counter's input for it (22, as for GPCLK0 and 1 at 20 and 21) counts
  another clock, 163.68 MHz, whatever GPCLK2 does. txengine divides it by
  whole numbers only (`--pll clkdiv` is refused there) and cannot count its
  carrier, so on GPIO 6 the start-up check is skipped and a PLL changed
  behind its back is noticed by the lock bit alone. On a Pi 2, GPIO 6
  carried the carrier and the modulation as GPIO 4 does (the same crystal
  offset, the 1 kHz tone within 0.02 %).
* **A clock divider of 1 divides by 4096:** set to an integer divider of
  1, GPCLK0 and the PWM clock put out a 4096th of their input, from any
  source and with or without MASH (counted by the frequency counter on a
  Pi 2 and a Zero 2 W, from 19.2 MHz to 1.125 GHz). txengine divides by
  at least 2, as Linux does.
* **CPU clock changes:** the firmware changes the ARM clock over the same
  slow PLL register bus that the DMA engine keeps busy. When the two
  collide, the firmware's write lands in the modulated PLL instead: on the
  Zero 2 W the carrier vanished a few seconds into a transmission. txengine
  holds the CPU clock at its maximum (cpufreq governor `performance`) while
  it modulates a PLL and restores the previous governor afterwards, even
  after `kill -9`. Should the PLL still be changed (e.g. by a thermal
  throttle), it is repaired, typically within 20–40 ms (a 12–36 ms dropout
  in tests that forced collisions or changed its dividers). It is watched
  through its lock bit and the carrier frequency (the clock manager's
  frequency counter), not by reading its registers: every read on that bus
  delays the DMA engine's next write by about 3 µs, which is an audible
  click. In AM the same clock changes stall the DMA engine for longer than
  the PWM's FIFO lasts, and the PWM drops carrier cycles (0.4–1 % with the
  `schedutil` governor, which also shifts the carrier's phase); the CPU
  clock is held there too, and txengine warns if the PWM misses cycles.
* **Pi 4: the clock generators' input.** The BCM2711's GPCLKs divide wrongly
  when fed faster than about 670 MHz, even divisors first: GPCLK0 counted
  exact up to 662.5 MHz, at 675 MHz exact or 0.015 % slow, at 725 MHz 0.5 %
  slow; from 750 MHz the divisor 4 gave nothing, and from 864 MHz every
  divisor beyond 3 about 0.22 MHz (the carrier check stopped the first
  transmissions there). txengine feeds them at most 600 MHz, so carriers
  reach 300 MHz (higher ones go out on a harmonic), and `--pll clkdiv` is
  refused: it would divide PLLD_PER, which runs at 750 MHz. The PWM and PCM
  clocks take 1.3 GHz.
* **Pi 4: the PLLs.** The firmware runs the VCOs at 2.6–3.25 GHz, without
  the BCM283x's feedback pre-divider (its control bits select a VCO range
  here, set differently on each PLL), so txengine moves a running VCO only
  within the limits above. On the board tested the VPU, and with it the
  UARTs, SPI and I2C, runs from PLLA, and PLLC clocks nothing: txengine uses
  PLLC and lets it move down by 25 %. It locked and divided exactly from
  −25 % to +3 % of its 2592 MHz, and a tone at 145.5125 MHz measured the
  same from −21 % to +1 % (harmonics below −74 dB). That reaches every FM
  carrier but those within 140 kHz of 108 MHz, and every narrow-band channel
  of the bands above but those within 50 kHz of 54 and 162 MHz (multiples of
  the 54 MHz crystal, where every division puts the fraction at a whole
  number). Its PLLs make none of the BCM283x's spurs (below −76 dB with the
  VCO at a half or a quarter of a multiple of 54 MHz, or 1.4 MHz from one).
  They keep the firmware's loop gain, whose modulation response is flat
  within 0.04 dB up to 45 kHz (stereo separation 53 dB).
* **Pi 4: AM.** The BCM2711's DMA engine is slower to write to the PWM: fed
  as on the BCM283x (each burst waiting for its writes' responses, requested
  from 8 words in the FIFO on), the PWM missed carrier cycles from 1.8 MHz
  on (0.06 % at 2.5 MHz, 2 % at 3.5 MHz, where a 1 kHz tone at 90 % read
  10 %), from a lite channel or a full one, at the same 250 MHz core clock
  as a Zero 2 W. txengine sends the bursts without waiting and requests them
  from 12 words on, which kept every cycle up to 2.2 MHz (and all but 30 ppm
  at 3 MHz, 0.5 % at 4.2 MHz): the Pi 4 makes AM carriers up to 2.2 MHz.
* **Zero 2 W (Cortex-A53):** a floating-point register load from a
  peripheral register at an address ≡ 4 (mod 8) returns the register below
  it. All register accesses are explicit 32-bit integer loads and stores.
* A weak power supply throttles the CPU; txengine reports the firmware's
  under-voltage and throttling flags at start-up (`txengine probe` shows
  them too).

### Pico and Pico 2

**PLL method.** What the RP2's PLLs (VCO 750–1600 MHz, integer feedback
divider) require:

* **A divider value must stay for at least four crystal cycles.** With a
  new one every two (6 MHz) or three (4 MHz) cycles the carrier dissolves
  into noise at any write timing, on both chips; every four (3 MHz) it is
  clean. The value apparently crosses into the PLL's reference clock domain
  through a synchroniser that needs that long.
* **Writes must avoid a narrow window of the reference cycle**, about three
  system clock cycles (21 ns at 144 MHz) wide: a write there smears the
  carrier, its neighbours degrade it. The firmware therefore synchronises its
  PIO state machine to a crystal edge once at start (the crystal clock is put
  on GPIO 25, the LED, for that moment) and writes in the middle of the
  good range. The phase was measured on both chips: 1–4 cycles after the
  edge is bad, 5–11 and 0 are clean.
* **A lower comparison frequency is quieter.** With the crystal divided by
  two (6 MHz) a divider step moves the carrier half as far as with 12 MHz,
  and the modulator's noise next to the carrier drops by 7–12 dB; every FM
  channel can be planned with it.
* **The PLL's loop is wide**: it hardly filters the modulator's noise,
  which spreads up to ±1.5 MHz around the carrier and peaks near 300 kHz,
  where the loop amplifies it. On an exact raster frequency it also forms
  tones, e.g. at multiples of 600 kHz at 87.6 MHz (about −32 dBc on an
  unmodulated carrier; with modulation they become noise). Emissions into
  neighbouring channels are much higher than from a Raspberry Pi. A
  one-bit second-order modulator, which moves the divider by one step at
  most, and a MASH 1-1-1 were both worse.
* **The loop lifts the multiplex band** slightly, as f²: +0.12 dB at 38 kHz
  on the RP2040, +0.15 dB on the RP2350, half that with a 12 MHz reference,
  the same over the band. Uncorrected that limits the separation to
  41–44 dB; txengine pre-compensates it.
* **RP2350: the DMA may not access `PLL_USB`** until ACCESSCTRL allows it;
  the firmware does.
* **UHF:** a clock output that passes `PLL_USB`'s post dividers on
  undivided puts out carriers up to 1 GHz on both chips (at 1.3 GHz
  nothing came out). The modulator's noise grows with the carrier movement
  of one divider step, the reference over the division from the VCO, which
  is 2 or 3 at UHF (9–18 in the FM band): at 446 MHz the phase noise 1 to
  10 kHz out measured −72 dBc/Hz on the RP2350 and −65 dBc/Hz on the
  RP2040 (at 145.5 MHz −85 to −90), with the modulator's pattern tones at
  ±1 MHz (−23 dBc), and it reaches the neighbouring channels (see
  [Measurements](#nfm-1)); on the third harmonic of a third of the
  frequency (division 10) it is lower.

**PIO method.** A square wave whose edges are rounded to 4 ns bits contains
its harmonics folded back by multiples of the bit rate, and some land in the
FM band: txengine chooses, per carrier, the bit rate whose strongest such
alias is weakest (about −25 dBc at worst for an ideal bit stream; for a
carrier outside the FM band, within ±10 % of it). The pin's output
measured 6–17 dB stronger on many of these lines through a cable, up to
about −15 dBc, on both chips, at 4 and 12 mA and at 240 as well as
249.6 Mbit/s; the cause is not known yet. On the RP2350 the
double-rate HSTX output could make the same stream without the higher
clock and voltage (not done). The raised system clock needs the higher core
voltage: on the RP2350 the regulator takes a new setting only once its
software control is unlocked, without which the cores crash at these clocks.

**AM.** The quantiser needs about 120 (RP2040, assembly) or 131 (RP2350,
Rust) clock cycles per carrier cycle and 280 or 157 per sample, which the
firmware reports (with a margin) and txengine plans with; beyond, it
refuses the carrier. A starved state machine ends the transmission with a
fault rather than a gap.

**Safety.** If txengine dies, the firmware switches the carrier off after
one second without samples. A watchdog (one second) reboots a hung
firmware, which also stops the carrier; `txengine probe` then shows where
it had stopped. Panics and hard faults, on either core, reboot into the USB
bootloader. Over USB a status request takes 0.1–0.15 ms, and a request the
host gives up does not block the device (txengine repeats it once).

## Documentation

* [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): crates, signal flow, the
  hardware backends, how to add transmission modes and hardware, and the
  design for GPS-synchronised single frequency networks.

## License

GPL-3.0-or-later.
