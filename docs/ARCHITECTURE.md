# txengine architecture

txengine is split into layers that meet only at two small interfaces in
`txengine-core`: a **modulator** produces RF control samples, an **RF sink**
emits them. The engine in the binary connects one to the other. Adding a
transmission mode or a hardware platform means implementing one side.

```text
 audio file / stdin                                         GPIO pin
        │                                                      ▲
 ┌──────▼───────┐  ring   ┌──────────────┐    f32    ┌────────┴───────┐
 │ txengine-    │ ──────► │  Modulator   │ ────────► │    RfSink      │
 │ audio thread │ (rtrb)  │ (WFM: MPX +  │  samples  │ (RP1, BCM, or  │
 │ decode+resamp│         │  RDS)        │           │  file)         │
 └──────────────┘         └──────▲───────┘           └────────▲───────┘
                                 │ RenderContext              │ poll / write
                          ┌──────┴────────────────────────────┴──────┐
                          │  engine (SCHED_FIFO thread): keeps the   │
                          │  sink's buffer full, handles control     │
                          └──────────────────────────────────────────┘
```

## Crates

| Crate | Contents |
|---|---|
| `txengine-core` | `Modulator` and `RfSink` traits, `SinkInfo`, `RenderContext`. No dependencies. |
| `txengine-dsp` | Allocation-free DSP blocks: Kaiser FIR design, linear-phase block FIRs, polyphase interpolator, an elliptic high-pass (second-order sections), pre-emphasis, pre-emphasis-aware look-ahead limiter, phase accumulator and sine table, small optimiser. |
| `txengine-rds` | EN 50067: group coding and CRC, character set, scheduler/encoder, the physical layer (`RdsModulator`), and an independent decoder used by tests and `analyze`. |
| `txengine-fm` | The wide-band FM mode: `MpxGenerator` (stereo multiplex) and `WfmModulator` (the `Modulator` implementation, control commands). |
| `txengine-nfm` | The narrow-band FM mode: `BasebandGenerator` (voice and squelch signalling to frequency deviation), CTCSS tones and DCS words (`subaudio`), and `NfmModulator`. |
| `txengine-am` | The AM broadcast mode: `EnvelopeGenerator` (programme to carrier envelope) and `AmModulator`. |
| `txengine-audio` | Input decoding (symphonia), resampling (rubato), the lock-free ring to the transmit thread, clock-drift steering for live input; the programme the modes share (`Programme`, `AudioFeed`). |
| `txengine-hal` | Hardware: platform detection, MMIO, DMA memory providers, the BCM283x/BCM2711 backends and the RP1 (Pi 5) backends (FM and AM each), amplitude by pulse width (`am`), the shutdown guardian; usbfs (USB without libusb) and the Pico backend (planning, `RfSink`). |
| `txengine-pico-protocol` | The USB protocol between txengine and the Pico firmware, and the arithmetic both sides share: the sigma-delta modulator of FM, the pulse quantiser of AM (`am`). `no_std`, no dependencies. |
| `txengine` | The command-line program: engine loop, control FIFO/socket, file sinks, the modes' `probe` and `tone`, `devices`, the hardware backends and their selection (`hw/`, also transmitting on a harmonic), the programme input options every mode shares, and per mode its options, set-up and tools (WFM: `analyze`). |
| `firmware/pico` | The RP2040/RP2350 firmware (embassy-rp), a cargo workspace of its own for the bare-metal targets. |

## The two interfaces

### `RfSink`

A sink accepts a stream of `f32` samples of one kind (`SampleFormat`): the
instantaneous carrier frequency offset in Hz, held for one sample period
(`FrequencyOffset`, FM), or the carrier's amplitude, 0 to 1 of the
strongest carrier it makes, interpolated between samples (`Amplitude`,
AM). It describes itself once in `SinkInfo`:

* `sample_rate`: the exact nominal rate the hardware runs at (derived from
  its dividers and the nominal crystal frequency, e.g. 50 MHz / 200 on the
  Pi 5).
* `buffer_len`, `realtime`.
* `zero_order_hold` and `response_db`: the output stage's frequency response
  (a hold, plus any measured correction such as the PLL loop of the Pi 2 and
  Zero 2 W).
  Modulators may equalise it.
* `max_offset_hz`, `resolution_hz`, `carrier_hz`.
* `max_clock_error_ppm`: how large a crystal error the sink can correct.
* `paced_by_carrier`: the carrier itself paces the samples (each lasts a
  whole number of carrier cycles: AM), so correcting the carrier corrects
  the sample clock too (see "Crystal error").

`poll()` reports how many samples can be written, how many the hardware has
consumed (`consumed`, an absolute sample index) and the underrun count;
`write()` appends; `start()`/`stop()`. `set_clock_error_ppm()` tells the
sink how far its crystal is off, and the sink retunes the carrier for it
(see "Crystal error" below).

### `Modulator`

`configure(&SinkInfo)` adapts to the sink, `render(&RenderContext, &mut
[f32])` produces samples, `command()` handles control lines, and
`take_meters()` reports levels and limiting for the status line. The
`RenderContext` carries the absolute index of the first sample, the rate at
which the samples actually leave the transmitter (`sample_rate`: the sink's
nominal rate corrected for its crystal error, so it can change slowly while
transmitting) and, for real-time sinks, the wall-clock time at which the
first sample will leave the antenna (`emit_time_unix`). Everything
time-related builds on that mapping: the RDS clock-time group is scheduled
so that it *ends* at the minute edge of the emission time, not the render
time, and the WFM modulator keeps its subcarriers and the programme's
playback speed true to the actual rate.

A new transmission mode (e.g. a digital mode) implements `Modulator`. A
mode that needs a different kind of sample (I/Q, say) adds a `SampleFormat`
variant, and sinks that can produce it advertise it.

On the command line every mode is a command with its own subcommands
(`txengine wfm tx | render | analyze | probe | tone`,
`txengine nfm tx | render | probe | tone`,
`txengine am tx | render | probe`); `probe` shows the plan for the mode
and `tone` transmits raw tones from it (both in `mode.rs`), and only
`devices` needs no mode. A mode's options are a clap `Args` struct that
implements `Mode` (`txengine/src/mode.rs`): what it needs from the
transmitter (`hw::Output`: frequency modulation over a range, or
amplitude), a check of the options, and building its modulator for the
opened output (its `SinkInfo`: rate, real time or not, and the buffer the
programme must fill at the start). The generic `Tx<M>` and `Render<M>`
(`txengine/src/cli.rs`) add the options every mode shares (hardware,
control, file output), the programme input options are one flattened
struct (`txengine/src/programme.rs`), and `mode::transmit` and
`mode::render` run any mode. The modes live in `txengine/src/wfm/`,
`txengine/src/nfm/` and `txengine/src/am/`.

## Signal processing (wide-band FM)

All per-sample work happens in blocks of up to 512 output samples, one stage
at a time; this keeps the inner loops short, which matters on the in-order
Cortex-A7/A53 cores. On a Pi 2 at 900 MHz, transmitting stereo + RDS at
195 kS/s from a FLAC file takes about 33 % of one core for the transmit
thread (28 % without the separate interpolation of the pre-emphasis boost
the limiter needs) and 23 % for decoding and resampling the audio.

1. **Audio rate** `fa = fs / round(fs / 48 kHz)`: gain, mono mix (if not
   stereo), 15 kHz low-pass (≥ 80 dB from 18.5 kHz), pre-emphasis
   (first-order shelf fitted minimax to the analogue curve, ≤ 0.06 dB), M/S.
   With the limiter on, the flat programme `x` and the boost the
   pre-emphasis adds, `d` (pre-emphasised = `x + d`), are kept apart.
   The gain includes a drive correction: minus the rise of the power of
   pink noise (20 Hz – 15 kHz) through the pre-emphasis, 4.3 dB at 50 µs,
   6.8 dB at 75 µs. BS.412 limits the power of the pre-emphasised
   multiplex; with the correction a programme no brighter than pink noise
   (music rarely is) keeps the MPX power it has without pre-emphasis.
2. **Interpolation** to `fs`: polyphase FIR, ≥ 120 dB image rejection. The
   first images (`fa ± f`) lie on the stereo subband, and a stereo
   receiver demodulates them into the audio band at `|fa − 38 kHz ± f|`
   (13 kHz → 1 kHz at `fa` = 50 kHz), where the de-emphasis does not take
   back the up to 13.7 dB the pre-emphasis gave the original tone; with
   90 dB these products reached −80 dB. The branches are evaluated
   together with independent accumulators. With the
   limiter, `x` and `d` are interpolated separately (four channels in
   stereo).
3. **Limiter**, per audio-rate frame on the interpolated samples, so it
   bounds the true peak of `M + S·sin 2φ`, which lies between L and R
   (`max(|L|, |R|) = |M| + |S|`). Two gains: the HF gain `h` scales only the
   boost, the peak gain `g` everything: output `g·(x + h·d)`. `h` is the
   largest gain with `|x + h·d| ≤ max(T, |x|)` for every sample of L and R:
   a high-frequency overload is resolved by reducing the pre-emphasis for
   a moment, which a receiver hears as less treble rather than the whole
   programme ducking. After that every sample is within `max(T, |x|)`, so
   `g` follows from the flat peak alone and only acts when the flat
   programme is over full scale. Each gain = box-average over the
   look-ahead (1.5 ms) of the sliding minimum of the required gain, with a
   release (HF 50 ms, peak 80 ms); interpolated linearly across each frame.
   The construction guarantees the threshold for every sample.
4. **Composite**: `a·(M + S·sin 2φ) + p·sin φ + r·rds·sin 3φ`, where φ is a
   single 19 kHz phase accumulator (64-bit, 40 fractional bits). sin 2φ and
   sin 3φ are computed from sin φ and cos φ, so all subcarriers are exactly
   phase locked; the RDS bit clock is the same accumulator divided by 16.
5. **Output equaliser**: a linear-phase FIR (9 or 11 taps), least-squares
   fit to `1 / (sinc(f/fs) · measured response)` up to 62 kHz.
6. **Safety clip** at the maximum deviation, peak metering, and the MPX
   power (BS.412) over the last 60 s, in one-second buckets.

RDS: the data are differentially encoded and each bit produces a biphase
symbol filtered by `H_T(f) = cos(π f td / 4)`, whose impulse response is used
in closed form. The symbol is truncated to ±6 bits; the waveform of one bit
depends on 12 symbols, which are tabulated in three groups of four (16 sign
patterns each). Once per bit the waveform of that bit is assembled from
three table rows, and each output sample is one interpolated look-up. The
level is normalised to the all-zero reference sine (EN 50067 1.3).

## Signal processing (narrow-band FM)

Voice in a 12.5 or 25 kHz channel, with a CTCSS tone or a DCS code for
the receivers' squelch (`txengine-nfm`), in blocks of up to 512 output
samples like the other modes. The samples are the deviation in Hz, as in
WFM.

1. **Audio rate** `fa = fs / floor(fs / 16 kHz)` (16.7 kHz at 250 and
   166.7 kS/s, 17.3 kHz at 190 kS/s): mono mix, gain, high-pass,
   pre-emphasis, low-pass.
   * The high-pass is an 8th-order elliptic filter (four second-order
     sections in `f64`, `txengine-dsp::iir`): the voice band from 300 Hz
     within 0.1 dB, 60 dB down from 254 Hz, the highest standard CTCSS
     tone, to DC. Its analogue prototype is tabulated (the tests check its
     response), made a high-pass at the pre-warped edge and digital by the
     bilinear transform for whatever `fa` the output gives.
   * The pre-emphasis is the WFM mode's first-order shelf (750 µs:
     6 dB/octave from 212 Hz), normalised to 1 at 1 kHz: a full-scale
     1 kHz tone deviates the full voice deviation.
   * The low-pass at the audio bandwidth (3 kHz) is linear phase, 60 dB
     from 10 % above it.
2. **Interpolation** to `fs` (polyphase, 80 dB): the images of the audio
   would widen the signal beyond its channel.
3. **Limiter** (`PeakLimiter`, as in AM): the true peaks of the voice
   within ±1; look-ahead 1.5 ms, release 80 ms. It neither clips nor
   splatters.
4. **Deviation**: voice × `Dv` + sub-audio × `Ds`, `Dv = D − Ds` (`Ds`
   15 % of `D` by default), so the sum never exceeds `D`. The CTCSS tone
   is a phase accumulator; DCS sends its 23-bit Golay word (polynomial
   `0xC75`, data `100` and the code's 9 bits, lowest bit first) at
   134.4 bit/s with raised-cosine transitions one bit long, 52 dB less
   power above 300 Hz than below. Both follow `RenderContext::sample_rate`,
   so the crystal correction keeps the tone and the bit rate true.

The output's sample-and-hold and the PLL responses the sinks report do not
matter below 3 kHz (under 0.01 dB), so nothing is equalised.

## Signal processing (AM)

Double sideband with full carrier (A3E), as on long, medium and short wave
(`txengine-am`). The envelope generator works like the multiplex: blocks of
up to 512 output samples, one stage at a time.

1. **Audio rate** `fa = fs / floor(fs / 24 kHz)` (25 kHz at 100 kS/s): mono
   mix, DC blocker (10 Hz), gain, low-pass at the audio bandwidth (4.5 kHz
   for the 9 kHz channels of ITU regions 1 and 3, up to 10 kHz; 60 dB from
   10 % above it).
2. **Interpolation** to `fs` (polyphase, 80 dB).
3. **Limiter** (`PeakLimiter`): the multiplex's construction for one
   channel, on the interpolated samples, so the true peaks stay within
   ±1; look-ahead 1.5 ms, release 80 ms.
4. **Envelope** `c·(1 + m·x)` with the carrier `c = 1 / (1 + m)`: the
   positive peaks reach the transmitter's full output, the negative ones
   never cut the carrier off.

The sink turns the envelope into the transmitter's waveform; for a PWM
output, `txengine-hal/src/am.rs`: one pulse per carrier period, centred on
the same instant of every period, so that the fundamental's amplitude is
`sin(π w / P)` (width `w`, period `P`) and its phase does not depend on the
width. Every carrier cycle gets its own width: the envelope is interpolated
linearly across the cycles of each sample (response `sinc²(f / fs)`,
reported in `response_db`), mapped through an arcsine table and quantised
by a second-order error-feedback loop, which moves the quantisation error
from the carrier towards ±fc/2 (in a simulation of 999 kHz from 400 MHz,
89 dB below the carrier within ±5 kHz). The loop runs in fixed point: on
an in-order Cortex-A53 its float version took ~60 ns per carrier cycle
(of 1 µs at 1 MHz), mostly waiting for its own results, the integer one
~29 ns.

## Hardware backends

`--device BACKEND[:ARGS]` picks one (`txengine/src/hw/`): every backend
implements `hw::Backend` (its name, the devices of its kind present now,
a check of the options that apply to it, the name of its instance lock,
`open` returning an `RfSink`, and `probe`) and is listed in `hw::BACKENDS`.
Without `--device`, txengine uses the only device any backend finds.
Options that only some backends use stay flags and are rejected by the
others (`--pll` on a Pico, for example).

The Raspberry Pi backends share the same structure: plan the clocks,
snapshot every register they will touch, start a **guardian** (a process
forked before any thread exists; when the transmitter's pipe closes, for
whatever reason including `kill -9`, it stops the DMA and restores the
snapshot), then run a DMA ring, paced by hardware, that writes one
frequency word per sample into a PLL or clock divider register (FM) or a
pulse width per carrier cycle into a PWM (AM). A clock generator that is
already running belongs to something else (the Pi 2's Ethernet and USB
chip on GPCLK1, the Zero 2 W's WiFi on GPCLK2): FM refuses it before
touching anything, unless `--force-gpclk` takes it over until the
transmitter stops.

Every planner that modulates a PLL (RP1, BCM, the Pico's PLL method)
prefers, above its other preferences, divisions that keep the carrier clear
of the points where the PLL makes a spur next to it: the VCO at a half
multiple of the frequency its phase detector compares at, and on the Pico
also the carrier at some fractions of it (`boundary::Spurs`; the module
has the measurements). The clearance it aims for is an eighth of the
largest offset planned (the quiet passages) plus 30 ppm (the crystal
correction); a plan that falls short of the first part logs a warning (and
`txengine wfm probe` shows it).

Each frequency-modulating backend makes carriers up to a limit of its
clock outputs (`MAX_CARRIER_HZ`: 400 MHz on the BCM boards, 1 GHz on the
RP1 and the Pico). Beyond, `--harmonic N` opens the backend at `f / N`
with `1 / N` of the mode's largest offset, and wraps its sink in
`hw::harmonic::HarmonicSink`, which reports the carrier, offsets and
resolution `N` times larger and divides every sample by `N` on its way to
the hardware: the `N`-th harmonic of the pin's square wave then carries
the modulation as planned. The crystal correction passes through
unchanged (it is relative). AM refuses it: a pulse's harmonics do not
follow its width as the fundamental does.

### Raspberry Pi 5 (RP1)

* The carrier is `pll_audio` (or `pll_video`) → `clk_gp0..2` → GPIO. The PLL's
  24-bit fractional feedback divider is the modulation input; post dividers
  and the clock generator divide down to the carrier (up to 1 GHz the PLL
  drives the pin undivided). The plan is verified with RP1's frequency
  counter. The fraction only moves within its whole number, so a carrier
  whose VCO lands near a whole multiple of the 50 MHz reference for every
  division (near multiples of 50 MHz) cannot be modulated, and the planner
  says so. The VCO is planned within 1.0–3.0 GHz, down to 0.8 GHz if
  needed (`clocks::VCO_WINDOWS`): `pll_audio` locked from 0.8 GHz (the
  feedback divider's minimum) to 4.2 GHz and modulated equally cleanly
  from 1.46 to 2.91 GHz, while at 4.07 GHz its phase noise 1 kHz out was
  9 dB higher. Within them the planner prefers no clock generator division
  and the fraction far from whole numbers.
* Pacing: an RP1 DMA "tick" (xosc / N, e.g. 50 MHz / 200 = 250 kS/s) drives
  the handshake of one channel of the Synopsys AXI DMAC. Samples live in
  host memory pinned through io_uring (anonymous pages, long-term pinned,
  cache maintenance done explicitly), reached by the DMAC at
  `phys + 0x10_0000_0000` over PCIe.
* The ring is a circular linked list of blocks (1024 samples each). A block
  is handed to the hardware by setting the VALID bit in its LLI; the DMAC
  clears it when it writes the LLI back after the block, which is also how
  txengine tracks progress. If the DMAC reaches an LLI that is not valid it
  stops and never re-fetches it. txengine detects any standstill of more
  than 2 ms (a running engine moves every 4 µs), counts an underrun, parks
  the carrier at the centre frequency, and re-arms the channel at the first
  block whose LLI is still valid.
* The kernel runtime-suspends the DMAC when its own clients are idle; the
  health check re-enables it. Channel interrupts are kept disabled so the
  kernel's handler never sees our channel.
* PCIe ASPM is disabled on the RP1 link while transmitting: waking the link
  from L1 at a block boundary can take long enough to lose a tick.

### Raspberry Pi 5 (RP1), AM

* The carrier is a PWM0 channel in phase-correct mode (GPIO 12, 13, 18 or
  19) from `pll_audio` → `pll_audio_pri_ph` → `clk_pwm0`, at up to 400 MHz
  (RP1's datasheet gives 150 MHz; the counter and the pin follow at 400).
  The channel counts up and down over `2n` clocks, one carrier period, and
  its output is high while the count is below the duty value: one pulse of
  `2·duty` clocks centred on the count's turning point. The planner picks
  the largest `n` whose clock `2n · fc` the PLL reaches with room for the
  crystal correction in its fraction (999 kHz: `n` = 200, 399.6 MHz).
* The duty values come from the channel's 16-word FIFO, one per period,
  which an AXI DMA channel refills on the PWM's request: the carrier paces
  the samples. The ring and its bookkeeping are the FM backend's (LLIs in
  pinned memory, VALID bits, stall detection and re-arming), with one word
  per carrier cycle; the blocks are pages or fractions of a page so that
  the buffer comes close to the length asked for. Channels 0 and 1 (8-beat
  FIFO, bursts of 8 words) sustain 6.25 M words/s, the others 3.8 (both
  measured), which limits the carrier to 6.2 MHz. A sample lasts the
  whole number of carrier cycles that comes closest to 10 µs.
* The crystal correction scales the PLL's feedback divider, which moves the
  carrier and the sample clock together (`paced_by_carrier`).
* The guardian restores the PLL, `clk_pwm0`, the PWM block, the pin and
  the link power states, as in the FM backend.

### Raspberry Pi 1-4 (BCM283x, BCM2711)

* The carrier is a PLL (A, C or D) → PER divider → GPCLK → GPIO, or a GPCLK
  fractional divider from PLLD (`clkdiv`). Clock generators divide by at
  least 2 (`cprman::MIN_DIVI`): set to 1, GPCLK0 and the PWM clock divide
  by 4096, from any source (measured). GPCLK2 has no MASH control and no
  frequency counter input (`cm::has_mash`, `cm::tcnt_gp`): it divides by
  whole numbers only (no `clkdiv`), and its carrier is neither verified at
  start nor watched. The planner scans every clock
  generator to find out who else uses each PLL (`Pll::users`) and refuses
  changes of more than 3 % (or any change of a PER divider that others
  use): PLLC clocks the VPU (and through it the SD card host, SPI, I2C and
  the mini UART) and EMMC (the WiFi's SDIO host), PLLD the PL011 UART and
  HDMI; the UARTs tolerate about ±3 % of baud rate error, and the SD and
  SDIO buses run at their 50 MHz maximum. A PLL whose only users are the
  GPU's video and 3D blocks (`h264`, `isp`, `v3d` on its CORE channel:
  normally PLLA) may move down by up to 25 %
  (`cprman::GPU_PLL_MAX_DECREASE`): from 2250 MHz −12 % reach every carrier
  in 87.5–108 MHz, where ±3 % reaches 76 % of the band (PLLC at ±3 %:
  50 %), and −25 % the narrow-band FM channels (all of CB, the amateur
  bands and marine VHF up to 225 MHz; in 148–174 MHz all but three spots
  where every division puts the fraction at a whole number). The planner
  takes the least change that keeps the carrier clear of the PLL's spurs
  (above): in the FM band −12 % at most, but for 21 of 2051 carriers (down
  to −22 %). The
  carrier stays at or below 400 MHz, above which the generators' output
  fades (`fm::MAX_CARRIER_HZ`). Idle PLLs are preferred and powered up with
  the kernel's loop settings; otherwise, on the BCM2836/7, PLLA, then PLLC.
* The BCM2711 (Pi 4) differs. Its firmware runs the VCOs at 2.6–3.25 GHz
  without a feedback pre-divider (ANA1 bits 13–14 select a VCO range
  instead, and differ between the PLLs), so a running VCO only moves
  within the limits above. On a Pi 4 the VPU runs from PLLA and PLLC
  clocks nothing: PLLC comes first and may move down by 25 % (it locked,
  divided exactly and modulated equally cleanly from −21 % to +1 %), which
  reaches every FM carrier but those within 140 kHz of 108 MHz (twice the
  54 MHz crystal). Its GPCLKs take at most 600 MHz
  (`cprman::max_gpclk_input_hz`): fed faster, from 675 MHz, they divide
  wrongly, even divisors first (from 864 MHz most give 0.22 MHz), so the
  carrier stays at or below 300 MHz, and `clkdiv` from PLLD_PER (750 MHz)
  is refused; the PWM and PCM clocks take 1.3 GHz. Its PLLs made no
  integer-boundary spurs. It keeps the firmware's loop (KI 2, KP 3),
  whose modulation response is flat within 0.04 dB to 45 kHz.
* Loop gain of PLLA and PLLC: the firmware and Linux run every PLL with the loop
  filter gains KI 2, KP 8, KA 2 (`A2W_PLLx_ANA1/3`), which give PLLA a
  loop bandwidth of about 125 kHz (natural frequency fitted to the
  measured modulation response, damping 1.6). Inside the multiplex band
  that loop neither suppresses the VCO's phase noise well (a hump at
  10–30 kHz offset; on a Pi 2 −91 dBc/Hz at 10 kHz, which limited the
  stereo SNR to 44 dB) nor follows the modulation flatly (+0.4 dB at
  38 kHz, +0.57 dB and 6° of lag at 67 kHz). While modulating PLLA or PLLC
  of the BCM2836/7, txengine raises the integral gain to KI 5 (`fm::
  MODULATION_KI`): about 600 kHz of bandwidth, −101 dBc/Hz at 10 kHz,
  and a response within 0.12 dB and 0.25° up to 67 kHz. The wider loop
  moves its noise peak out to 300–600 kHz offset (+1 dB in the 200 kHz
  channels at ±400 kHz); KI 6 gains 1 dB of SNR for 3–5 dB more there,
  and KI 7 also raises the noise close to the carrier. The lock behaviour
  (400 retunes of ±150 MHz, 400 deviation-sized steps) is the same for
  KI 2–7. The firmware's gain is restored with the rest of the PLL. On
  the Zero 2 W the noise has another source (it does not change with any
  loop gain), but the flatter response is the same. PLLA and PLLC are the
  same design: at the same VCO frequency their responses agree within
  0.01 dB at KI 2 and at KI 5, and at KI 5 PLLA's response is the same
  (±0.02 dB) from 1927 to 2365 MHz, so one table per SoC compensates both
  PLLs at any VCO frequency.
* Pacing: the PWM (or PCM) FIFO's DMA request. Each sample is two control
  blocks: write the frequency word, then write a dummy word to the FIFO.
  Writes to the PWM registers must pass a slow clock-domain synchroniser,
  so the set-up confirms each write by reading it back or waiting for the
  status it causes, checks `STA.BERR`, and the pacer clock is kept between
  5 and 25 MHz; after starting, the DMA position must advance or the pacer
  is configured again. The pacer clock divides PLLD_PER by an integer when
  that hits the rate within 1 ppm, otherwise by a fractional divider
  (MASH 1, at most 2 ns of jitter), which hits any rate within a fraction of
  a hertz.
* The DMA engine's write rate to the PLL registers is limited (≈ 231 k
  writes/s on the Pi 2 and the Zero 2 W, 646 k on the Pi 4), so the sample
  rate is chosen from a measurement at start-up: the highest even multiple
  of 19 kHz within 85 % of it and at most 250 kS/s (`fm::RATE_STEP`; the
  Pi 4: 228 kS/s), which puts the images of the hold onto
  harmonics of 19 kHz, where a switching stereo decoder makes no tones of
  them. The achieved rate is checked continuously.
* The PLL registers sit behind the slow A2W bus, which the firmware also
  uses to change the ARM clock (PLLB). If a firmware write meets one of the
  DMA engine's, it lands in the modulated PLL's registers instead (PLLB's
  registers are PLLA's plus 0xe0; on a Zero 2 W, PLLA's control register
  read 0 and its channel dividers were overwritten). While a PLL is
  modulated, the cpufreq governor is therefore set to `performance` (and
  restored afterwards, also by the guardian). If the PLL is changed anyway
  (e.g. by a firmware thermal throttle) it is repaired, including the
  channels other clocks use. It is watched through the clock manager
  only, every 20 ms: its lock bit, and the carrier, which the frequency
  counter measures in the background (gates of 5 and 10 ms alternate, so
  that a stopped clock's stale count does not fit). A count more than
  0.5 % off (one feedback divider step is at least 0.8 %) is checked
  against the PLL's registers before anything is repaired, since another
  user of the counter could also cause it. The A2W registers are not
  polled: each CPU read of one delays the DMA engine's next fraction write
  by about 3 µs, and the samples after it catch up through the pacer's
  FIFO. On a Pi 2 the stereo pilot alone turned one read into a click of
  about 450 Hz of deviation; reading the PLL's nine registers once a
  second (as txengine did before) delayed the sample timeline by 27 µs
  each time, an audible click every second. Forced changes of NDIV or of
  the PER divider, which leave the PLL locked, were repaired after
  19-36 ms.
* Every register and DMA-memory access is a single 32-bit `ldr`/`str` from
  a general-purpose register (`mmio::load32`/`store32`). Plain volatile
  accesses may be compiled to floating-point register loads when the value
  is converted to a float next, and on the BCM2837 those return the word 4
  bytes below for addresses ≡ 4 (mod 8); merged copies would make unaligned
  stores, which fault on AArch64 Device memory.
* The frequency counter that verifies the carrier is read with two gate
  times: its count register cannot be cleared while the measured clock is
  stopped, so a dead clock would otherwise read back the previous result.
* Underruns: samples are handed to the engine in blocks of 16. The newest
  block's last sample leads, through the block's exit control block,
  which writes the block's sequence number to a progress word, to an idle
  loop (two control blocks that keep the carrier at the centre
  frequency). The exits are off the samples' path: between underruns the
  engine runs the samples' control blocks alone, as before. A parked
  engine is sent on through a jump control block, as in AM (below), to
  the first unplayed sample, so an underrun neither replays nor skips
  samples. Before, the CPU closed the idle loop again only once it saw the
  engine back in the ring, and took everything written as played when it
  found the engine parked: on a Zero 2 W each underrun lost about 60 ms of
  programme, and a stall right after the refill played it twice.
* Memory comes from the VideoCore mailbox (uncached `DIRECT` alias), freed
  by the guardian if the process dies.

### Raspberry Pi 1-4 (BCM283x, BCM2711), AM

* This PWM (PWM0 on GPIO 12 or 18, PWM1 on 13 or 19) has no phase-correct
  mode. In mark-space mode each FIFO word is one range period, high for
  the word's number of clocks from the period's start. With the range at
  half the carrier period (`R` clocks, even) and two words per cycle,
  `R/2 − u` and `R/2 + u`, the cycle's two pulses have a fundamental of
  `sin(π u / R)` with a constant phase: `u` takes the place of the width
  in `hal::am` (a period of `R` steps, `R/2` steps to the full square
  wave). At `u = 0` the pulses are equal and half a period apart, all
  second harmonic, which at amplitude `a` is `(1 − a²) / a` of the
  carrier: +2.8 dB at the carrier of 90 % depth. A word missing on the way
  swaps the pulses, which shifts the carrier by half a period.
* The PWM clock divides a PLL's PER channel by an integer (a fractional
  divider would jitter by a whole source cycle), at most 250 MHz (500 MHz
  fails from a range of 4, on a Zero 2 W). The planner (`am::plan`) takes
  the finest steps (`R = 2k`, the largest `k` with `4k · fc` ≤ 250 MHz)
  that the PLL can make within the limits its users allow
  (`cprman::PllRoom`, as for FM) with room for the crystal correction in
  its fraction, and for those the least change of its VCO; the PLLs are
  tried in FM's order (`cprman::pll_preference`). 999 kHz: PLLA from 2250
  to 2229.8 MHz (−0.9 %), PER /3, PWM clock /3 = 247.75 MHz, `R` = 124.
  The crystal correction writes the PLL's fraction, only when it has moved
  by most of a unit (each write is a slow A2W transaction). The PWM clock
  is verified with the frequency counter and watched in the background
  like FM's carrier; a changed PLL is repaired.
* DMA: blocks of up to 512 cycles (1024 words, a page) from VideoCore
  memory into the FIFO, in bursts of four words requested while the FIFO
  holds fewer than 8, with panic priority. On a Zero 2 W a lite channel
  kept the PWM supplied up to 10 M words/s (5 MHz) and missed 1.7 % at
  12; 8.4 (4.2 MHz) is allowed. The BCM2711's engine is slower: so fed,
  a Pi 4's PWM missed cycles from 1.8 MHz on (2 % at 3.5 MHz), from a
  lite channel or a full one. Bursts that do not wait for their writes'
  responses, requested from 12 words on, keep it supplied up to 2.2 MHz,
  the Pi 4's limit (`am::BCM2711_MAX_CARRIER`). After each block a
  marker control block writes the block's sequence number to a progress
  word: exact bookkeeping, as the RP1's VALID bits give, and within a
  block the engine's source address. The newest block links to an idle
  loop of 16 cycles at the mean amplitude of the samples (an unmodulated
  carrier: an underrun is silent). A parked engine is sent on through a
  jump control block, which first closes the idle loop again (a DMA
  write to the idle block's `next`) and then continues at the first
  unplayed block, so the idle loop never points into the ring when the
  engine comes back to it. The jump is armed only when the idle loop is
  found closed and the engine then seen in it: an armed jump stays armed
  until the engine takes it, and arming it again before the engine has
  been seen back in the ring would send it into a block already played.
* The CPU clock is held (`cpufreq`) here too: the firmware's ARM clock
  changes stall the DMA engine for longer than the words in the FIFO last
  (8 to 11 of them: 4 µs at 1 MHz), and a PWM without a word holds its
  output low for the range period. With `schedutil` on a Zero 2 W, 0.4 % (500 kHz) to 1 % (3 MHz)
  of the carrier's cycles went missing, and the phase shifts spread the
  carrier into a cluster of lines; with the clock held, none. The PWM's gap
  flag (`STA.GAPO1`) never showed them in mark-space mode, so the
  transmitter counts the cycles taken against the time (5 s windows) and
  warns from 0.2 % missing.
* The producer writes each cycle's two words with one 64-bit store
  (`mmio::store64`, from general-purpose registers, 8-byte aligned): an
  uncached store costs ~10 ns on a Cortex-A53, 32 or 64 bits alike.
* The guardian restores the PLL, the PWM clock and registers, the pin and
  its pads and the cpufreq governor, and frees the memory.

### Raspberry Pi Pico and Pico 2 (USB)

The Pico is a front end: txengine renders the samples on the computer, the
firmware turns them into a modulated carrier. One firmware does all: FM by
either method, and AM.

* **Link** (`txengine-pico-protocol`): one vendor interface with a bulk OUT
  endpoint for the samples (FM: `i16` in units of the plan's resolution,
  about 3 Hz; AM: `u16` amplitudes) and control requests for the rest:
  `Info` (whose protocol version must match txengine's; also what AM costs
  the firmware), `Version` (the firmware's `git describe`), `Configure`,
  `ConfigurePio` or `ConfigureAm` (the whole clock plan of a method,
  computed by txengine: `txengine-hal/src/pico/plan.rs`, `pico/am.rs`),
  `Start`, `Stop`, `Status` (the count of samples whose emission has
  started, underruns, faults), `Retune` (crystal correction, also before
  the start) and `Bootsel`.
  A second interface is the Pico SDK's reset interface, for `picotool reboot
  -f`. On the host, `usbfs.rs` talks to `/dev/bus/usb` directly
  (asynchronous URBs for the samples, synchronous control transfers).
  The firmware wraps embassy-rp's USB driver (`usb_driver.rs`, until a
  release has the fixes): a control transfer the host gives up (after a
  bus error, or cancelled) no longer blocks the device until it is
  unplugged, and the RP2350's controller keeps the hardware fixes its
  `Driver::new` switched off. txengine repeats a status read or a retune
  once after an error on the bus.
* **Carrier (PLL method)**: `PLL_USB` (the system and USB clocks both run
  from `PLL_SYS` at 144 MHz, USB at 144 / 3), its post dividers and
  `clk_gpout0` onto GP21. The PLL compares at the crystal / 2 (6 MHz: a
  divider step moves the carrier half as far as at 12 MHz); in narrow-band
  FM at the crystal's 12 MHz where only that is clear of the PLL's spurs
  (`plan::reference_dividers`, `plan_clearest`). The divider from
  the VCO is the largest the VCO range allows (the smallest steps) among
  those clear of the PLL's spurs (above), 9–18 in the FM band, 2 or 3 at
  UHF. `clk_gpout0` divides only up to 160 MHz of input (it is specified to
  150 MHz); passing the post dividers on undivided it puts out up to 1 GHz
  (measured).
* **Modulation (PLL method)**: the feedback divider is integer only, so a
  MASH 1-1 (`sigma_delta::Mash11`, input in 30-bit fixed point over 0–2
  steps) dithers it over `base … base + 4` at 3 MHz, interpolating linearly
  between the samples (the samples last 18 updates, 166.7 kS/s). The sink
  reports the extra `sinc` of the interpolation and the PLL loop's measured
  lift of the multiplex band (as f², per chip and reference divider) in
  `SinkInfo::response_db`. An odd input keeps a steady carrier from
  repeating a short pattern. Core 1 runs the modulator alone, in assembly
  from RAM (15 cycles per update on the Cortex-M0+, 11 on the M33, checked
  against the Rust version at boot), and writes 5-bit codes, six to a word,
  into a 4 KB ring. DMA feeds them to a PIO state machine, which prepends
  `FBDIV[11:5]` and pushes the 12-bit value; a second DMA channel writes it
  to `FBDIV_INT`. The state machine paces the writes (one every four crystal
  cycles, from `clk_sys`, which is locked to the same crystal), and once at
  start it waits for an edge of the crystal clock (put on GPIO 25 for that
  moment) so that every write lands at the same point of the reference
  cycle. Both rules, the four cycles and the write phase, are measured
  requirements of the PLL (README). On the RP2040 the DMA channels are
  ping-pong pairs (transfer counts run out); on the RP2350 they run in
  `ENDLESS` mode, and ACCESSCTRL must let the DMA at `PLL_USB`.
* **PIO method** (`--synthesis pio`, `PioConfig`, firmware `direct.rs`): the
  system clock switches to `PLL_USB` at the bit rate B (240–264 MHz, chosen
  per carrier by `plan::pio_plan` to keep the bit stream's aliases out of
  the FM band), with the core voltage at 1.20 V (the RP2350's regulator
  ignores a new setting, which still reads back, until its software control
  is unlocked; at the default 1.10 V the cores crash at these clocks) and
  the flash interface's divider scaled up to keep the flash clock where
  boot put it (on the RP2040, whose SSI must be disabled for that, core 0
  makes the change from RAM while core 1 waits; `clock.rs`, shared with
  AM). Core 1 looks up one 32-bit word per 32 bits in a table of 4096
  square-wave patterns indexed by the phase at the word's start, advances
  the phase by the sample's increment (interpolated per word), and DMA
  feeds a 32 KB ring of words to a state machine that runs `out pins, 1`
  every cycle. The M0+ word loop is assembly (11 of the 32 cycles a word
  lasts; the compiled one took about 25, too many with the per-sample
  work on top). A starved state machine is `Fault::Stall`, a ring
  overtaken by the DMA `Fault::Late`. Stopping restores the clock,
  divider and voltage.
* **AM** (`AmConfig`, firmware `am.rs`, the quantiser in
  `txengine-pico-protocol::am`): the system clock is raised as for the PIO
  method (to 264 MHz, the most cycles per carrier cycle), and a state
  machine puts out one pulse per carrier cycle from words of two counts,
  low then high (side-set, so the counts are exact to the cycle and a
  pulse may have no width). The carrier is `clock · k / 2³²`; samples last
  a whole number of carrier cycles, so `Retune` (a new `k`) corrects the
  carrier and the sample clock together (`paced_by_carrier`). For every
  pulse the quantiser chooses the width and centre near `v = a − (2E₁ −
  E₂)`, `E` the complex errors of the last pulses' phasors, which it
  computes from their integer edges (a 4096-entry sine table corrected to
  first order): second-order noise shaping of the carrier's complex
  envelope, so that the edges can be rounded to clock cycles although the
  period is not a whole number of them. Rare paths keep the loop from
  winding up (v pointing backwards: a pulse a quarter period off; beyond
  the full output, which sits 10·(π/P)² below the square wave for the
  noise shaping's room). On the RP2040 the quantiser is assembly (Rust
  spilled to the stack, 280 cycles a pulse): core 1's interpolators turn a
  phase into its table entry's address and remainder, and their spare
  registers hold the errors, the amplitude step and constants, at one
  cycle per access on the I/O port; checked against the Rust version at
  boot. It takes about 120 cycles per pulse and 280 per sample, which
  `Info` reports (with a margin) and txengine plans with: carriers up to
  1.76 MHz at 100 kS/s. The RP2350 runs the Rust version: about 131 and
  157 cycles on its Cortex-M33, 1.71 MHz. A starved state machine is
  `Fault::Stall`; an underrun holds the samples' recent mean.
* **Buffering**: core 0 (embassy, USB) puts the samples into a 32768-sample
  ring; core 1 takes one per sample period and counts underruns (a missing
  sample holds an FM carrier at its centre, an AM one at its recent mean).
  Without samples for one second the firmware assumes the host is gone and
  switches the carrier off.
* **Faults**: core 0 feeds a one-second watchdog, so a hung firmware
  reboots (and the carrier stops; on the RP2350 the firmware sets what the
  watchdog resets itself, as embassy-rp's choice would leave the cores
  running); the firmware leaves breadcrumbs in a watchdog scratch
  register, reported after such a reset in `Info` (shown by `txengine
  probe`). Panics and hard faults, on either core, reboot into the USB
  bootloader: the system clock first leaves `PLL_USB` (the raised clock)
  before that PLL is switched off with the carrier, and core 0 stops
  feeding the watchdog, on which the boot ROM counts the reboot down.
* **Timing**: `poll` reads the status with one control transfer (0.1–0.15
  ms), which `ClockDiscipline` brackets like a register read, so the crystal
  error is measured and corrected as on the Raspberry Pi
  (`set_clock_error_ppm` sends a new centre with `Retune`).

### Adding a hardware target

Implement `RfSink` in `txengine-hal` (plan the clocks, provide DMA memory,
build a paced ring, implement `poll`/`write`, fill `SinkInfo` with the exact
rate and the measured response) and a `Backend` in `txengine/src/hw/`
(discovery, the options it accepts, opening the sink), and list it in
`hw::BACKENDS`. Measure the modulation response with
`txengine wfm tone` and an SDR, and record it in `SinkInfo::response_db`
if it deviates from a pure sample-and-hold. Plan the clocks from the
nominal crystal frequency, leave room for a correction of
±`MAX_CLOCK_ERROR_PPM`, and implement `set_clock_error_ppm`.

## Crystal error (`txengine/src/clock.rs`)

On every supported board the carrier synthesiser and the timer that paces
the samples derive from one crystal, so a crystal that runs `e` fast has
two effects with the same `e`: the carrier is too high, and the samples
leave too fast, which raises the pilot, the subcarriers and the playback
speed. One measured error therefore drives both corrections, which is why
there is a single `--ppm` (and not one for the carrier and one for the
sample rate):

* **Carrier**: `RfSink::set_clock_error_ppm(e)`. The backends plan from the
  nominal crystal frequency and scale the whole feedback divider (integer
  and fraction) of the modulated PLL by `1 / (1 + e)`, or the clock
  divider's source frequency by `1 + e`; the frequency word of each sample
  is computed with the correction from then on.
* **Timeline**: the engine passes `RenderContext::sample_rate = fs · (1 +
  e)` to the modulator. The WFM modulator retunes its 19 kHz phase
  accumulator (phase continuously; the subcarriers and the RDS bit clock
  follow) and asks the audio input to resample by `1 + e` more, so the
  programme plays at its true speed.

The hardware sample clock is never changed: the output buffer is refilled
from the hardware position as before, and a live input's buffer is held
half full by its drift loop, which absorbs whatever difference remains
between the source's clock and true time. (Changing the hardware rate
instead, as PiFmAdv's `--ppm` does, makes a live input over- or underrun
unless the input follows.)

Except where the carrier paces the samples (`SinkInfo::paced_by_carrier`,
AM): there the carrier correction corrects the sample clock as well, the
samples leave at the nominal rate in true time, and the engine passes the
nominal rate to the modulator. The estimator must not see those
corrections, or it would measure an error that the correction already
removed and undo it: every poll's position is mapped to the position the
sink would have reached uncorrected (each sample emitted under a
correction `c` counts as `1 + c` samples), and the fit runs on that.

The error is measured by `ClockDiscipline`, which wraps `RfSink::poll`:
each poll reads the hardware position bracketed by two readings of
`CLOCK_MONOTONIC_RAW`. A `TimeReference` turns the readings into
observations "sample n was emitted at time t"; the `RateEstimator` keeps
the most precise observation of every second and fits the time error
`n / fs - t` with a least-squares line weighted by `exp(-age / 120 s)`
(window 12 minutes); its slope is `e`. A fit is used once it spans 20 s
with a standard error below 0.2 ppm, which takes about 25 s. Underruns
interrupt the timeline, so the fit starts again after each one (the
correction in effect is kept until the new fit qualifies); single outliers
are dropped. Because the corrections do not act on the measured clock, this
is an open-loop estimate and cannot oscillate. A crystal drifting at a
constant rate is followed with a lag of about 3.5 minutes (1.78 time
constants), e.g. 0.5 ppm behind at 0.15 ppm per minute.

The `SystemClock` reference is the raw monotonic clock (the CPU's timer)
running at the frequency the NTP daemon has given the system clock (adjtimex
`freq` and `tick`, read once a second), with the bracket's middle as the
time and readings that took over 0.5 ms dropped. It does not use
`CLOCK_MONOTONIC`: systemd-timesyncd and ntpd leave the removal of time
offsets to the kernel's PLL, which slews the clock by up to tens of ppm for
many minutes after each NTP poll (a 15 ms offset removed over 512 s starts
at 29 ppm); measured against it, a Pi 5's estimate moved by 20 ppm within
half an hour. chrony slews through the frequency, which this follows; its
slews are small. On the BCM boards the CPU timer and the DMA pacing share
the crystal, so the estimate is exactly the NTP daemon's frequency
correction (negated), smoothed; on the Pi 5 the RP1 has its own crystal and
the estimate combines both. The accuracy is therefore that of the NTP
daemon's frequency estimate. Measured on air with a USRP B210, all boards
with chrony polling a LAN server (16-64 s); the receiver's own error
(3.5 ppm) is taken from the Pi 2 and the Zero 2 W, whose corrected carriers
agree within 3 Hz:

| Board | Crystal error on air | Estimate | Error left (carrier; 1 kHz tone) |
|---|---|---|---|
| Pi 2 | +7.28 ppm | +7.18…7.37 ppm | (reference); +0.1 ppm |
| Zero 2 W | +4.67 ppm | +4.64…4.66 ppm | (reference) |
| Pi 5 | −53.9…−54.4 ppm | −53.70…−53.93 ppm | −0.44 ppm; −0.3 ppm |

The Pi 5's RP1 has its own 50 MHz crystal, which on our board runs 54 ppm
slow; that is a static offset of that crystal (it moved by less than 0.5 ppm
over 18 °C of CPU and 5 °C of RP1 warming). It does drift by about −0.5 ppm
in the first minute of a transmission, as the RP1 warms up, which the
estimate follows with its lag. The pilot follows the tone: 18998.966 Hz
uncorrected on the Pi 5, 18999.994 Hz corrected. With systemd-timesyncd
instead of chrony the Pi 5's estimate was 2.7 ppm off (−57.31 ppm), the
error of timesyncd's frequency.

Observations are only used while the reference is synchronised: for the
system clock, while the kernel's NTP state says so, or the kernel's maximum
error is below 0.5 s (chrony keeps the kernel's unsynchronised flag set
unless `rtcsync` is configured, but every NTP daemon updates the error
bounds). With `--ppm <value>` the estimate still runs and is reported, but
the given value is applied; `--ppm off` neither measures nor corrects. The
`PPM` control command switches between the three at run time.

## Single frequency networks (design)

An SFN needs every transmitter to emit the same sample at the same instant
(within a few microseconds) at the same carrier frequency. The current code
already provides the pieces this rests on:

* **Deterministic signal.** Given the same programme audio, the same start
  sample index and the same parameters, the modulator produces bit-identical
  output: the pilot phase accumulator and the RDS bit clock are functions of
  the sample index alone, and RDS groups depend only on the emission time.
* **Sample index ↔ time.** Sinks report the hardware read position as an
  absolute sample index, and modulators receive the emission time of every
  block. The Pi 5 backend reads the exact DMA source address, so the
  position is known to one sample.
* **Hardware timing without slips.** Both backends pace the DMA from a
  crystal-derived hardware clock, with no software in the timing path, and
  the Pi 5's link power saving that caused one-sample slips is disabled.

The planned extension:

1. **Server**: decodes the programme once and sends it to the transmitters
   as timestamped audio frames (or as a pre-rendered multiplex) together
   with RDS state, over the network, with enough delay (e.g. 500 ms) to
   cover jitter.
2. **Client time base**: a GPS receiver's PPS signal on a GPIO pin is
   timestamped (the kernel's `pps-gpio` driver, or a GPIO edge captured by
   DMA/PIO on the Pi 5). The simplest form already works today: chrony
   disciplines the system clock to the PPS, and the `SystemClock` reference
   measures the crystal against it. For the timeline alignment below, a
   `PpsReference` implements `TimeReference` directly: it interpolates the
   sample index emitted at each PPS edge between the position readings
   around it, which gives observations on the GPS time scale with the
   interrupt latency as the only noise.
3. **Start alignment**: the client chooses the PPS edge at which sample `n0`
   must be emitted and starts the DMA there (pre-filled buffer, tick
   started from the PPS-timestamped moment), then verifies the alignment by
   sampling the DMA position at subsequent PPS edges.
4. **Rate discipline**: the measured crystal error is applied as it is now,
   to the carrier (`RfSink::set_clock_error_ppm`; resolution < 0.1 Hz on
   the Pi 5) and to the sample timeline (`RenderContext::sample_rate`). An
   SFN additionally steers the timeline's phase: the estimator's line also
   gives the time offset of the sample clock, and a small extra term in the
   rate given to the modulator (or single samples inserted/dropped in a
   pre-rendered multiplex) pulls the emitted sample index onto GPS time.
   Carrier and timeline then get different corrections: the carrier the
   crystal error alone, the timeline the error plus the phase term.
5. **Monitoring**: each client reports its sample-index-at-PPS and carrier
   offset; the server checks the network is coherent.

None of this requires changes to the modulator or sink interfaces beyond
what exists: the time mapping is already explicit, and the hardware paths
are already free of software timing.
