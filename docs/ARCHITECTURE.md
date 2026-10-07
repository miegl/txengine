# txengine architecture

txengine is split into layers that meet only at two small interfaces in
`txengine-core`: a **modulator** produces RF control samples, an **RF sink**
emits them. The engine in the binary connects one to the other. Adding a
transmission mode or a hardware platform means implementing one side.

```text
 audio file / stdin                                         GPIO pin
        │                                                      ▲
 ┌──────▼───────┐  ring   ┌──────────────┐  f32 Hz   ┌────────┴───────┐
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
| `txengine-dsp` | Allocation-free DSP blocks: Kaiser FIR design, linear-phase block FIRs, polyphase interpolator, pre-emphasis, frame-based look-ahead limiter, phase accumulator and sine table, small optimiser. |
| `txengine-rds` | EN 50067: group coding and CRC, character set, scheduler/encoder, the physical layer (`RdsModulator`), and an independent decoder used by tests and `analyze`. |
| `txengine-fm` | The wide-band FM mode: `MpxGenerator` (stereo multiplex) and `WfmModulator` (the `Modulator` implementation, control commands). |
| `txengine-audio` | Input decoding (symphonia), resampling (rubato), the lock-free ring to the transmit thread, clock-drift steering for live input. |
| `txengine-hal` | Hardware: platform detection, MMIO, DMA memory providers, the BCM283x/BCM2711 backend, the RP1 (Pi 5) backend, the shutdown guardian. |
| `txengine` | The command-line program: engine loop, control FIFO/socket, programme set-up, file sinks, `analyze`, `probe`, `tone`. |

## The two interfaces

### `RfSink`

A sink accepts a stream of `f32` samples, each the instantaneous carrier
frequency offset in Hz (`SampleFormat::FrequencyOffset`), held for one
sample period. It describes itself once in `SinkInfo`:

* `sample_rate`: the exact rate the hardware runs at (derived from its
  dividers, e.g. 50 MHz / 200 on the Pi 5). Modulators render at this rate.
* `buffer_len`, `realtime`.
* `zero_order_hold` and `response_db`: the output stage's frequency response
  (a hold, plus any measured correction such as the PLL loop of the Pi 2 and
  Zero 2 W).
  Modulators may equalise it.
* `max_offset_hz`, `resolution_hz`, `carrier_hz`.

`poll()` reports how many samples can be written, how many the hardware has
consumed (`consumed`, an absolute sample index) and the underrun count;
`write()` appends; `start()`/`stop()`. `set_carrier_correction_ppm()` is the
hook for disciplining the carrier (see SFN below).

### `Modulator`

`configure(&SinkInfo)` adapts to the sink, `render(&RenderContext, &mut
[f32])` produces samples, `command()` handles control lines. The
`RenderContext` carries the absolute index of the first sample and, for
real-time sinks, the wall-clock time at which it will leave the antenna
(`emit_time_unix`). Everything time-related builds on that mapping: the RDS
clock-time group is scheduled so that it *ends* at the minute edge of the
emission time, not the render time.

A new transmission mode (e.g. narrow-band FM, AM via pad drive strength, a
digital mode) implements `Modulator`. A mode that needs a different kind of
sample (amplitude, I/Q) adds a `SampleFormat` variant, and sinks that can
produce it advertise it.

## Signal processing (wide-band FM)

All per-sample work happens in blocks of up to 512 output samples, one stage
at a time; this keeps the inner loops short, which matters on the in-order
Cortex-A7/A53 cores. On a Pi 2 throttled to 600 MHz, transmitting stereo +
RDS at 195 kS/s takes about 24 % of one core for the transmit thread and
16 % for decoding and resampling the audio.

1. **Audio rate** `fa = fs / round(fs / 48 kHz)`: gain, mono mix (if not
   stereo), pre-emphasis (first-order shelf fitted minimax to the analogue
   curve, ≤ 0.06 dB), 15 kHz low-pass (≥ 80 dB from 18.5 kHz), M/S.
2. **Interpolation** to `fs`: polyphase FIR, ≥ 90 dB image rejection; the
   branches are evaluated together with independent accumulators.
3. **Limiter**: peak of `|M| + |S|` per audio-rate frame, measured on the
   interpolated samples (so it bounds the true peak of `M + S·sin 2φ`, which
   lies between L and R). Gain = box-average over the look-ahead of the
   sliding minimum of the required gain, with a slow release; interpolated
   linearly across each frame. The construction guarantees the threshold
   for every sample.
4. **Composite**: `a·(M + S·sin 2φ) + p·sin φ + r·rds·sin 3φ`, where φ is a
   single 19 kHz phase accumulator (64-bit, 40 fractional bits). sin 2φ and
   sin 3φ are computed from sin φ and cos φ, so all subcarriers are exactly
   phase locked; the RDS bit clock is the same accumulator divided by 16.
5. **Output equaliser**: a linear-phase FIR (9 or 11 taps), least-squares
   fit to `1 / (sinc(f/fs) · measured response)` up to 62 kHz.
6. **Safety clip** at the maximum deviation and peak metering.

RDS: the data are differentially encoded and each bit produces a biphase
symbol filtered by `H_T(f) = cos(π f td / 4)`, whose impulse response is used
in closed form. The symbol is truncated to ±6 bits; the waveform of one bit
depends on 12 symbols, which are tabulated in three groups of four (16 sign
patterns each). Once per bit the waveform of that bit is assembled from
three table rows, and each output sample is one interpolated look-up. The
level is normalised to the all-zero reference sine (EN 50067 1.3).

## Hardware backends

Both backends share the same structure: plan the clocks, snapshot every
register they will touch, start a **guardian** (a process forked before any
thread exists; when the transmitter's pipe closes, for whatever reason
including `kill -9`, it stops the DMA and restores the snapshot), then run a
DMA ring that writes one frequency word per sample into a PLL or clock
divider register, paced by hardware.

### Raspberry Pi 5 (RP1)

* The carrier is `pll_audio` (or `pll_video`) → `clk_gp0..2` → GPIO. The PLL's
  24-bit fractional feedback divider is the modulation input; post dividers
  and the clock generator divide down to the carrier. The plan is verified
  with RP1's frequency counter.
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

### Raspberry Pi 1-4 (BCM283x, BCM2711)

* The carrier is a PLL (A, C or D) → PER divider → GPCLK → GPIO, or a GPCLK
  fractional divider from PLLD (`clkdiv`). The planner scans every clock
  generator to find out who else uses each PLL and refuses changes of more
  than 3 % (or any change of a PER divider that others use); idle PLLs are
  preferred and powered up with the kernel's loop settings.
* Pacing: the PWM (or PCM) FIFO's DMA request. Each sample is two control
  blocks: write the frequency word, then write a dummy word to the FIFO.
  Writes to the PWM registers must pass a slow clock-domain synchroniser,
  so the set-up confirms each write by reading it back or waiting for the
  status it causes, checks `STA.BERR`, and the pacer clock is kept between
  5 and 25 MHz; after starting, the DMA position must advance or the pacer
  is configured again.
* The DMA engine's write rate to the PLL registers is limited (≈ 231 k
  writes/s on the Pi 2), so the sample rate is chosen from a measurement at
  start-up, and the achieved rate is checked continuously.
* The PLL registers sit behind the slow A2W bus, which the firmware also
  uses to change the ARM clock (PLLB). If a firmware write meets one of the
  DMA engine's, it lands in the modulated PLL's registers instead (PLLB's
  registers are PLLA's plus 0xe0; on a Zero 2 W, PLLA's control register
  read 0 and its channel dividers were overwritten). While a PLL is
  modulated, the cpufreq governor is therefore set to `performance` (and
  restored afterwards, also by the guardian). If the PLL is changed anyway
  (e.g. by a firmware thermal throttle) it is repaired, including the
  channels other clocks use: its lock bit is checked every 20 ms, its full
  configuration once a second. Reading the A2W registers themselves
  disturbs the DMA engine's writes (on a Pi 2, reading them every 20 ms
  cost 0.08 dB of stereo balance), so they are not polled more often.
* Every register and DMA-memory access is a single 32-bit `ldr`/`str` from
  a general-purpose register (`mmio::load32`/`store32`). Plain volatile
  accesses may be compiled to floating-point register loads when the value
  is converted to a float next, and on the BCM2837 those return the word 4
  bytes below for addresses ≡ 4 (mod 8); merged copies would make unaligned
  stores, which fault on AArch64 Device memory.
* The frequency counter that verifies the carrier is read with two gate
  times: its count register cannot be cleared while the measured clock is
  stopped, so a dead clock would otherwise read back the previous result.
* Underruns: the ring ends in an idle loop (two control blocks that keep
  the carrier at the centre frequency). New data re-links the tail; a
  parked engine is redirected into the ring.
* Memory comes from the VideoCore mailbox (uncached `DIRECT` alias), freed
  by the guardian if the process dies.

### Adding a hardware target

Implement `RfSink` in `txengine-hal` (plan the clocks, provide DMA memory,
build a paced ring, implement `poll`/`write`, fill `SinkInfo` with the exact
rate and the measured response), add detection in `platform.rs` and a
branch in `txengine/src/hw.rs`. Measure the modulation response with
`txengine tone` and an SDR, and record it in `SinkInfo::response_db` if it
deviates from a pure sample-and-hold.

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
   DMA/PIO on the Pi 5). This gives the offset and frequency error of the
   local crystal.
3. **Start alignment**: the client chooses the PPS edge at which sample `n0`
   must be emitted and starts the DMA there (pre-filled buffer, tick
   started from the PPS-timestamped moment), then verifies the alignment by
   sampling the DMA position at subsequent PPS edges.
4. **Rate discipline**: the measured crystal error is applied in two places:
   the carrier (`RfSink::set_carrier_correction_ppm`, which adjusts the PLL
   fraction; resolution < 0.1 Hz on the Pi 5) and the sample timeline (the
   modulator renders with a resampling ratio slightly different from 1, or
   single samples are inserted/dropped in the pre-rendered multiplex, so the
   emitted sample index tracks GPS time).
5. **Monitoring**: each client reports its sample-index-at-PPS and carrier
   offset; the server checks the network is coherent.

None of this requires changes to the modulator or sink interfaces beyond
what exists: the time mapping is already explicit, and the hardware paths
are already free of software timing.
