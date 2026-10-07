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
    divider (`--pll clkdiv`). The PLL is chosen automatically; PLLs used by
    other parts of the system are only retuned within ±3 %.
  * Pi 5 (RP1): fractional-N modulation of the RP1 audio or video PLL
    (24-bit fraction, < 0.1 Hz steps).
* **Multiplex** (all derived from one phase accumulator, so pilot, 38 kHz,
  57 kHz and the RDS bit clock are exactly locked): 50/75 µs pre-emphasis,
  15 kHz programme low-pass, look-ahead peak limiter that bounds the true
  (inter-sample) composite peak, 19 kHz pilot, DSB-SC stereo subcarrier,
  RDS, and pre-compensation of the output stage's measured frequency
  response (sample-and-hold and, on the Pi 2, the PLL loop).
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
| Raspberry Pi 2 v1.2, 3, 3+, Zero 2 W | BCM2837 | supported, not yet tested on hardware |
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
| `STATUS` | current settings and counters |
| `QUIT` | stop transmitting |

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

| | Raspberry Pi 5 | Raspberry Pi 2 |
|---|---|---|
| Sample rate | 250 kS/s | 194.9 kS/s (measured DMA capacity, see below) |
| Pilot | 6.749 kHz (target 6.75) | 6.757 kHz |
| 38 kHz carrier residual | 0.001 % | 0.004 % |
| Stereo separation (1 kHz, left only) | > 60 dB | > 50 dB |
| RDS injection | 2.02 kHz (target 2.0) | 2.05 kHz |
| RDS block errors | 0 % | 0 % |
| Timing | no sample slips in 25 s tone tests | no sample slips |
| Modulation THD (10 kHz) | −71 dB | |

Hardware-specific findings that txengine handles:

* **Pi 2 (and probably other BCM283x boards):** a DMA write to a PLL's
  fractional register takes about 4.3 µs, which caps the sample rate at
  about 231 kS/s. txengine measures the capacity at start-up and picks a
  rate with 15 % margin (`--sample-rate` above the capacity is refused), and
  it watches the achieved rate while running. The PLL's loop response is
  measured and compensated, which is what brings the separation above 50 dB.
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
