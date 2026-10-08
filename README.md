# txengine

An FM broadcast transmitter for the Raspberry Pi, written in Rust. It turns a
GPIO pin into a wide-band FM transmitter with a standards-conformant stereo
multiplex (ITU-R BS.450) and RDS (EN 50067), on the Raspberry Pi 2, 3, 4, 5 and
Zero 2 W, as 32-bit (armv7) or 64-bit (aarch64) static binaries.

> **Transmitting without a licence is illegal in most countries.** The output
> is a square wave rich in harmonics. Connect it to a receiver with a shielded
> cable and an attenuator, or to a dummy load, and always use a low-pass
> filter before anything that can radiate.

## Features

* **Hardware-timed output.** Every sample is written to the carrier
  synthesiser by DMA, paced by a hardware timer: the CPU only has to keep a
  60 ms buffer filled. Register writes are checked (lock, busy and status
  bits, read-back, frequency counter); nothing relies on sleeping.
* **Modulation methods**
  * Pi 1-4 (BCM2835/6/7, BCM2711): fractional-N modulation of PLLA, PLLC or
    PLLD (20-bit fraction, 1-3 Hz steps), or a dithered fractional clock
    divider (`--pll clkdiv`). The PLL is chosen automatically: on the
    Pi 2/3 and Zero 2 W PLLA, which only clocks the GPU's video and 3D
    blocks and may therefore move down by up to 15 %, which reaches the
    whole band; PLLs that clock the rest of the system (VPU, SD card, WiFi,
    UARTs) are only retuned within ±3 %. PLLA and PLLC run with a wider
    loop than the firmware gives them (loop gain KI 5 instead of 2), which
    removes most of their phase noise from the multiplex band.
  * Pi 5 (RP1): fractional-N modulation of the RP1 audio or video PLL
    (24-bit fraction, < 0.1 Hz steps).
* **Crystal error correction**: the board's crystal error is measured
  continuously against the system clock (NTP, or GPS through chrony) and
  corrected in the carrier frequency, the subcarriers and the playback
  speed; no calibration, and it follows temperature drift. A known error
  can be given instead (`--ppm`).
* **Multiplex** (all derived from one phase accumulator, so pilot, 38 kHz,
  57 kHz and the RDS bit clock are exactly locked): 50/75 µs pre-emphasis,
  15 kHz programme low-pass, look-ahead limiter that bounds the true
  (inter-sample) composite peak, 19 kHz pilot, DSB-SC stereo subcarrier,
  RDS, and pre-compensation of the output stage's measured frequency
  response (sample-and-hold and, on the Pi 2 and Zero 2 W, the PLL loop).
* **Levels**: the drive is lowered by the rise of the multiplex power the
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
* **Audio input**: WAV, FLAC, MP3, OGG/Vorbis, AAC/MP4 (symphonia), raw PCM on
  standard input, or a test tone; resampled with rubato; clock-drift
  compensation for live input.
* **Run-time control** through a FIFO or a Unix socket (PS, RT, PTY, TA, AF,
  gain, ...), status reporting, clean shutdown and a guardian process that
  restores every touched register even after `kill -9`.
* **Testing tools**: render to a WAV/IQ file, analyse a recorded multiplex
  (levels, pilot phase, RDS decode), transmit raw test tones, probe the
  frequency plan.

## Supported hardware

| Board | SoC | Status |
|---|---|---|
| Raspberry Pi 5 | BCM2712 + RP1 | tested over the air (64-bit build) |
| Raspberry Pi 2 v1.1 | BCM2836 | tested over the air |
| Raspberry Pi Zero 2 W | BCM2837 (RP3A0) | tested over the air (64-bit and 32-bit builds) |
| Raspberry Pi 2 v1.2, 3, 3+ | BCM2837 | supported, not yet tested on hardware |
| Raspberry Pi 4, 400 | BCM2711 | supported, not yet tested on hardware |
| Raspberry Pi 1, Zero, Zero W | BCM2835 (ARMv6) | not supported |

The default output is GPIO4 (pin 7). `txengine probe` shows what will be used
on a given board without transmitting.

## Building

Static binaries (musl) are cross-compiled with Rust's own linker; no C
toolchain is needed:

```sh
rustup target add aarch64-unknown-linux-musl armv7-unknown-linux-musleabihf
cargo build --release --target aarch64-unknown-linux-musl       # 64-bit OS
cargo build --release --target armv7-unknown-linux-musleabihf   # 32-bit OS
```

The Raspberry Pi 5 needs the 64-bit build: the RP1's DMA engine reads the
samples from main memory over PCIe without cache coherency, and only 64-bit
user space can perform the cache maintenance this requires (the 32-bit build
says so and stops). All other boards work with either build.

The binary is `target/<target>/release/txengine`. Copy it to the Pi; it needs
root to reach the hardware (`/dev/mem`, the VideoCore mailbox, or the RP1 PCIe resources).

On the Pi 2, 3, 4 and Zero 2 W, txengine programs the clock manager, DMA and
PWM registers directly. Kernels built with `CONFIG_IO_STRICT_DEVMEM` (NixOS,
for example; not Raspberry Pi OS) deny `/dev/mem` access to registers that a
kernel driver owns, even to root. On those kernels add `iomem=relaxed` to the
kernel command line (on NixOS: `boot.kernelParams = [ "iomem=relaxed" ];`).
`txengine probe` reports when it is missing.

## Usage

```sh
# Stereo file with RDS on 87.6 MHz
sudo txengine tx -f 87.6 -a music.flac --ps "MY RADIO" --rt "Now playing ..."

# From another program (WAV on stdin, or raw PCM with --raw-rate/--raw-channels)
ffmpeg -i stream.m3u8 -f wav - | sudo txengine tx -f 87.6 -a -
arecord -f S16_LE -r 48000 -c 2 -t raw | sudo txengine tx -f 87.6 -a - --raw-rate 48000 --raw-channels 2

# Carrier + RDS only, with clock time and alternative frequencies
sudo txengine tx -f 87.6 --ps STATION --af 87.6 --af 99.5 --pty "Pop music"

# Change RDS while transmitting
sudo txengine tx -f 87.6 -a music.mp3 --ctl /tmp/rds &
echo "RT Live from the studio" > /tmp/rds
echo "PS ONAIR" > /tmp/rds
```

### Control commands

One command per line on `--ctl` (FIFO) or `--ctl-socket` (Unix socket, which
answers with `OK ...` or `ERR ...`):

| Command | Effect |
|---|---|
| `PS text`, `RT text`, `PTYN text\|OFF` | programme service name, RadioText, PTY name |
| `PTY n\|name`, `PI hex`, `DI hex`, `ECC hex`, `LIC hex` | codes |
| `TA 0\|1`, `TP 0\|1`, `MS 0\|1`, `CT 0\|1` | flags |
| `AB` | toggle the RadioText A/B flag (receivers clear the display) |
| `AF 87.6,99.5 \| OFF` | alternative frequencies |
| `RDS 0\|1`, `RDSLEVEL hz` | RDS on/off, injection level |
| `GAIN db` | programme gain |
| `PPM`, `PPM auto\|ppm\|off` | report the crystal error correction; measure, fix or turn it off |
| `STATUS` | current settings and counters |
| `QUIT` | stop transmitting |

### Frequency accuracy

Every clock of the transmitter comes from the board's crystal, which is a
few ppm to a few tens of ppm off and drifts with temperature (10 ppm is
876 Hz at 87.6 MHz). That one error shifts the carrier, and it makes the
hardware emit the samples slightly too fast or too slow, which shifts the
pilot and the subcarriers and changes the playback speed. With the default
`--ppm auto`, txengine times the hardware's sample counter against the
system clock while transmitting, fits the crystal error (exponentially
weighted over about two minutes, so it follows a warming board) and
corrects both: the carrier synthesiser is retuned, and the multiplex is
rendered for the actual sample rate (pilot, subcarriers, RDS bit rate and
the programme's playback speed). The hardware sample clock itself is never
changed, so the correction cannot fill or drain any buffer; a live input
keeps following its own clock through its drift compensation.

The system clock has to be synchronised by an NTP daemon (chrony,
systemd-timesyncd, ...); until it is, txengine transmits uncorrected and
says so (`txengine probe` shows the state). The reference is the frequency
the daemon has measured for the system clock, so the result is as good as
that. With chrony (polling a LAN server every 16-64 s) the carrier, pilot
and playback speed of our three boards came within 0.3 ppm (26 Hz at
87.6 MHz); with systemd-timesyncd, which polls only every 34 minutes, the
Pi 5 was 2.7 ppm off. Use chrony. The first measurement is used after about 25 s. The
status line shows the correction in effect, and `PPM` on the control
interface reports the measurement in detail:

```
crystal error +6.73 ppm (measured against the system clock): correcting the carrier by -590 Hz and the sample rate
```

Without network time, give the error (`--ppm 6.44`, positive when the
crystal runs fast; `txengine tx` and `txengine tone` print the measured value
on a board that has NTP); txengine keeps measuring and reports it, but
applies the given value. `--ppm off` turns the correction off (no
measurement either). For GPS accuracy, let chrony discipline the system
clock from a GPS receiver's PPS signal (gpsd + chrony's `refclock`); txengine
then measures against GPS time.

### Testing without hardware

```sh
txengine render -a music.flac --duration 10 --format mpx -o mpx.wav   # the multiplex as WAV
txengine analyze mpx.wav                                             # levels, pilot phase, RDS
```

`analyze` accepts any multiplex recording (for example from an SDR; see
`tools/`) and decodes RDS with an independent decoder.

## Signal quality

Measured over the air with a USRP B210 (`tools/sdr_capture.py`,
`tools/fm_analyze.py`, `txengine analyze`):

| | Raspberry Pi 5 | Raspberry Pi 2 | Raspberry Pi Zero 2 W |
|---|---|---|---|
| Sample rate | 250 kS/s | 194.9 kS/s (measured DMA capacity, see below) | 195.9 kS/s |
| Pilot | 6.749 kHz (target 6.75) | 6.74-6.76 kHz | 6.75-6.76 kHz |
| 38 kHz carrier residual | 0.001 % | 0.005 % | 0.006 % |
| Stereo separation (left only) | > 50 dB (1 kHz) | 55–58 dB (100 Hz – 14 kHz) | 51–56 dB (1–14 kHz) |
| RDS injection | 2.02-2.06 kHz (target 2.0) | 2.03 kHz | 2.04 kHz |
| RDS block errors | 0 % | 0 % | 0 % (also at the end of a 10 min run) |
| Crystal error (carrier, pilot, playback speed) | −54 ppm; −0.3 ppm with `--ppm auto` | +7.3 ppm; +0.1 ppm with `--ppm auto` | +4.7 ppm; < 0.1 ppm with `--ppm auto` |
| Timing | no sample slips in 25 s tone tests | no sample slips | no sample slips |
| Modulation THD (10 kHz) | −71 dB | | |
| Audio SNR, mono / stereo (re 75 kHz) | ≥ 69.5 / ≥ 56.5 dB | 71.2 / 59.8 dB | 56.0 / 43.9 dB |
| Audio SNR, mono / stereo (re 0 dBFS) | ≥ 64 / ≥ 51 dB (65–70 / 53–56 dB at higher receiver gain) | 65.7 / 54.4 dB | 50.5 / 38.5 dB |
| Audio THD (1 kHz, 0 dBFS) | −91 dB | −72 dB | −69 dB |
| Audio frequency response (31.5 Hz – 15 kHz) | ±0.03 dB | ±0.04 dB | ±0.04 dB |
| CPU (stereo + RDS, one core) | | 24 % at 600 MHz (throttled) | 11 % at 1 GHz, plus 4 % audio decoding |

Audio figures: a reference stereo decoder (pilot-locked, 50 µs
de-emphasis), unweighted 20 Hz – 15 kHz. A 0 dBFS tone deviates 40 kHz,
not 65 kHz, because of the −4.3 dB pre-emphasis drive correction. The
noise comes from the carrier, not from the signal processing: an
unmodulated carrier has the same floor. In stereo the noise from the
23–53 kHz subband adds to it. Measured per clock path (unmodulated
carriers, AM and PM noise separated):

* **Pi 2, Zero 2 W:** phase noise of the PLLs (PM 8–16 dB above AM).
  PLLA, PLLC and PLLD measure the same on both boards, and neither the
  GPCLK divider, the pad drive nor the slew rate changes it; integer-N
  mode is 6 dB worse than fractional. With the firmware's loop gain it
  was −91 dBc/Hz (Pi 2) and −86 dBc/Hz (Zero 2 W) at 10 kHz; txengine's
  wider PLLA loop (see below) brings the Pi 2 to −101 dBc/Hz (stereo
  44 → 54 dB). The Zero 2 W does not change with the loop gain: its
  noise has another source (it is worse under CPU load and has a
  −21 dBc 100 Hz spur, i.e. its supply). On the Pi 2 the mono SNR is now
  limited by noise that only appears with modulation (72 dB for an
  unmodulated carrier, 66 dB transmitting).
* **Pi 5:** about −104 dBc/Hz at 10 kHz, with equal AM noise: noise
  added after the PLL, not phase noise of it. It is the same for every
  `pll_audio` setting (VCO 1050–1750 MHz, integer or fractional,
  dividers), for `pll_sys`, and for any pad drive or slew. At a B210
  gain of 20 dB the receiver contributes about half of it, so the table
  gives lower bounds.

The multiplex as computed (`txengine render`) has a noise floor of
−128 dBFS (21 bits); quantising it to the frequency steps of the hardware
gives −123 dBFS on the Pi 5 (0.17 Hz steps) and −106 dBFS on the BCM
boards (1.41 Hz steps). Both are far below the RF noise.

Hardware-specific findings that txengine handles:

* **Pi 2 and Zero 2 W (and probably other BCM283x boards):** a DMA write to
  a PLL's fractional register takes about 4.3 µs, which caps the sample rate
  at about 231 kS/s. txengine measures the capacity at start-up and picks a
  rate with 15 % margin (`--sample-rate` above the capacity is refused), and
  it watches the achieved rate while running.
* **Pi 2 and Zero 2 W, PLLA's and PLLC's loop:** with the firmware's loop gain
  (KI 2) the loop bandwidth is about 125 kHz. That leaves the VCO's
  phase noise around 10–30 kHz offset, in the audio and stereo bands,
  poorly suppressed (Pi 2: 44 dB stereo SNR), and makes the modulation
  response rise by 0.57 dB with 6° of lag at 67 kHz (35 dB separation
  uncompensated). txengine raises the gain to KI 5 while modulating
  (about 600 kHz bandwidth): on the Pi 2 the stereo SNR rises to 54 dB,
  the response is flat within 0.12 dB (the rest is measured and
  compensated) and the separation is 55 dB from 100 Hz to 14 kHz (Zero 2 W:
  51–53 dB at 5–14 kHz instead of 37–44 dB; its noise does not change). Costs:
  the wider loop's noise peak at 300–600 kHz offset (+1 dB in the
  neighbouring 200 kHz channels), and the 3rd harmonic of a full-scale
  1 kHz tone at −80 instead of −86 dB. The firmware's gain is restored
  afterwards. PLLA and PLLC are the same design: at the same VCO frequency
  their responses agree within 0.01 dB, at KI 2 and at KI 5 (Pi 2, PLLC:
  stereo SNR 53.2 dB and separation 58–60 dB at KI 5, 44.6 dB and 33 dB at
  KI 2 uncompensated). The response is not perfectly stable: between two
  sessions it moved by 0.15 dB at 67 kHz on both boards, and by 0.05 dB
  under CPU and network load, which leaves the separation unchanged.
* **Pi 2 and Zero 2 W, choosing the PLL:** retuning a PLL moves every
  clock derived from it. PLLC clocks the VPU, from which the SD card host,
  SPI, I2C and the mini UART (the serial console on boards with WiFi) take
  their clocks, and the WiFi's SDIO host. The SD and SDIO buses run at
  their 50 MHz maximum and a UART tolerates about ±3 % of baud rate error,
  so PLLC (like PLLD: PL011 UART, HDMI) stays within ±3 %, which reaches
  only half of the band. PLLA only clocks the GPU's video and 3D blocks,
  idle on a transmitter, so txengine prefers it and lets it move down by up
  to 15 % (the blocks run slower; upwards the 3 % limit stays). That
  reaches every carrier in 87.5–108 MHz on a 10 kHz raster (from the
  firmware's 2250 MHz at most −12 %), and the response, noise and separation measure the same
  at −14 % (VCO 1927 MHz) as at +1.2 %. PLLC moved by −3.6 % caused no SD
  or WiFi errors and no lost packets on either board.
* **BCM283x/BCM2711, CPU clock changes:** the firmware changes the ARM clock
  over the same slow PLL register bus that the DMA engine keeps busy. When
  the two collide, the firmware's write lands in the modulated PLL instead:
  on the Zero 2 W the carrier vanished a few seconds into a transmission.
  txengine holds the CPU clock at its maximum (cpufreq governor
  `performance`) while it modulates a PLL and restores the previous governor
  afterwards, even after `kill -9`. Should the PLL still be changed (e.g. by
  a thermal throttle), it is repaired, typically within 20 ms (a 12-16 ms
  dropout in tests that forced collisions).
* **Zero 2 W (Cortex-A53):** a floating-point register load from a
  peripheral register at an address ≡ 4 (mod 8) returns the register below
  it. All register accesses are explicit 32-bit integer loads and stores.
* **Pi 5:** PCIe link power saving (ASPM L1) occasionally delays the DMA
  enough to lose a timer tick, shifting the output timeline by one sample.
  txengine disables ASPM on the RP1 link while transmitting and restores it
  afterwards.
* A weak power supply throttles the CPU; txengine reports the firmware's
  under-voltage and throttling flags at start-up (`txengine probe` shows
  them too).

## Documentation

* [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): crates, signal flow, the
  hardware backends, how to add transmission modes and hardware, and the
  design for GPS-synchronised single frequency networks.

## License

GPL-3.0-or-later.
