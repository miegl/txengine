//! AM transmitter backend for BCM2835/6/7 and BCM2711 (Pi 2, 3, 4, Zero 2 W).
//!
//! Signal path:
//!
//! ```text
//!  crystal ─► PLL (A, C or D) ─► PER ─► PWM clock ─► PWM channel (M/S) ─► GPIO pad
//!                                                     ▲ two widths per carrier cycle
//!  VideoCore memory ─► DMA channel ◄── PWM FIFO request ┘
//! ```
//!
//! The PWM channel runs in mark-space mode: each word from its FIFO is one
//! range period of `R` clocks, high for the first `m` clocks (the word) and
//! low for the rest. With the range at half the carrier period and two
//! words per period, `R/2 − u` and `R/2 + u` (`R` even), the period carries
//! the pulses `[0, R/2 − u)` and `[R, 3R/2 + u)`, whose fundamental is
//! `sin(π u / R)` of the full square wave's and whose phase does not depend
//! on `u`: the law of one centred pulse of width `2u` in a period of `2R`
//! (see [`crate::am`]), from nothing at `u = 0` (two equal pulses half a
//! period apart: twice the carrier frequency) to the full square wave at
//! `u = R/2` (`R/2` steps: 62 at 999 kHz). This PWM has no phase-correct
//! mode that would centre one pulse per period, and a pulse at the start of
//! every period would move the carrier's phase with its width. The price is
//! the second harmonic, the stronger the narrower the pulses: at amplitude
//! `a` it is `(1 − a²) / a` of the carrier, +2.8 dB at the carrier of 90 %
//! depth (a single centred pulse: −1.4 dB).
//!
//! The PWM clock is a PER channel of a PLL divided by an integer (a
//! fractional divider would jitter by a whole source cycle), at up to
//! [`MAX_PWM_CLOCK`]; the planner retunes the PLL, within the limits its
//! other users allow, so that the carrier period is a whole number `2R` of
//! its clocks. The crystal correction scales the PLL's fractional feedback
//! divider, which moves the carrier and the sample clock together
//! (`paced_by_carrier`): each sample is a whole number of carrier cycles.
//! The CPU clock is held while transmitting: the firmware's changes of it
//! stall the DMA engine for longer than the FIFO lasts (see
//! [`crate::cpufreq`]).
//!
//! The DMA engine feeds the FIFO on its request, from a ring of blocks of
//! words in VideoCore memory. Each block's control block is followed by a
//! marker control block that writes the block's sequence number to a
//! progress word, so the transmitter knows exactly which blocks have been
//! played. The last block handed over links to an idle loop that holds the
//! carrier at its mean amplitude: an underrun leaves the carrier
//! unmodulated (silence) instead of replaying old samples. A parked engine
//! is sent on through a jump control block, which first lets the DMA engine
//! close the idle loop again, then continues at the first unplayed block.

use std::time::{Duration, Instant};

use log::{debug, info, warn};
use txengine_core::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll};

use super::cprman::{ClockWatch, Cprman, Pll, PllConfig, PllId, PllRoom, PllState, PllUsers, pll_preference};
use super::dma::{ControlBlock, DmaChannel};
use super::regs::{self, cm, dma as dmar, pwm};
use super::{Bcm, PinState, pwm_for_pin, pwm_write_confirmed, set_pin_function};
use crate::MAX_CLOCK_ERROR_PPM;
use crate::am::{PulseWidths, interpolation_response_db};
use crate::cpufreq::{CpuClockPin, CpuPinGuard};
use crate::error::{HalError, Result};
use crate::guardian::Guardian;
use crate::mem::mailbox::VcMemory;
use crate::mem::{DmaMemory, DmaMemoryExt};
use crate::mmio::{Mmio, barrier, store64};
use crate::sys::wait_until;

/// Highest PWM clock (Hz): measured on a Zero 2 W (BCM2837), the PWM runs
/// at 250 MHz (PLLD_PER / 2), but not at 500 MHz.
pub const MAX_PWM_CLOCK: f64 = 250e6;

/// Lowest carrier (Hz).
pub const MIN_CARRIER: f64 = 100e3;

/// Words per second the DMA engine moves into the PWM FIFO, with a margin:
/// on a Zero 2 W (core clock 400 MHz) a lite channel, in bursts of four
/// words, gave the PWM every word up to 10 M words/s (a 5 MHz carrier) and
/// missed 1.7 % at 12 M words/s. Two words per carrier cycle: the highest
/// carrier, which takes in the 75 m band. (Boards with a slower core clock
/// may not keep up; [`BcmAmTransmitter`] warns when the PWM misses cycles.)
const MAX_WORD_RATE: f64 = 8.4e6;

/// Highest carrier (Hz).
pub const MAX_CARRIER: f64 = MAX_WORD_RATE / 2.0;

/// The DMA engine moves bursts (BURST_LENGTH 3) while the FIFO holds fewer
/// than this many words, with its panic priority: a burst lands with room
/// for another one in flight, and the PWM has 8 words in hand while the
/// engine waits.
const DREQ_LEVEL: u32 = 8;

/// Carrier cycles in the idle loop's block.
const IDLE_CYCLES: usize = 16;

/// Time constant of the mean amplitude the idle loop holds (s).
const MEAN_TIME: f64 = 0.05;

/// Byte offset of the `next` field in a control block.
const CB_NEXT: usize = 20;

const FRAC_ONE: f64 = (1u64 << cm::A2W_FRAC_BITS) as f64;

/// Configuration of the BCM AM transmitter.
#[derive(Debug, Clone)]
pub struct BcmAmConfig {
    /// Carrier frequency in Hz.
    pub carrier_hz: f64,
    /// Output pin: 12, 13, 18 or 19.
    pub gpio: u32,
    /// The PLL that clocks the PWM; `None`: choose one (see
    /// [`pll_preference`]).
    pub pll: Option<PllId>,
    /// Pad drive strength 0..=7 (2..16 mA).
    pub drive: u8,
    /// Desired sample rate; each sample lasts the whole number of carrier
    /// cycles that comes closest.
    pub sample_rate: f64,
    /// Buffer length (transmit latency).
    pub buffer: Duration,
    pub dma_channel: Option<usize>,
    /// Largest relative VCO change accepted for a PLL with other users (see
    /// [`BcmFmConfig::max_vco_change`](super::fm::BcmFmConfig::max_vco_change)).
    pub max_vco_change: f64,
}

/// The clock plan: `carrier = VCO / (per_div · clk_div · 2 · range)`.
#[derive(Debug, Clone)]
pub struct BcmAmPlan {
    pub pll: PllId,
    /// Power up an idle PLL; change its PER divider.
    pub power_up: bool,
    pub set_per: bool,
    pub ndiv: u32,
    /// The whole feedback divider NDIV.FRAC in units of 2^-20.
    pub center: f64,
    pub per_div: u32,
    /// PWM clock divider (integer).
    pub clk_div: u32,
    pub vco_old: f64,
    pub vco_new: f64,
    pub clk_hz: f64,
    /// PWM range: clocks per half carrier period (even); the widths run
    /// over `range / 2` steps to the full output.
    pub range: u32,
    pub carrier_hz: f64,
    pub cycles_per_sample: u32,
    pub sample_rate: f64,
    /// Who else uses the PLL.
    pub users: PllUsers,
}

impl BcmAmPlan {
    /// The fraction (FRAC, relative to NDIV) when the crystal runs `ppm`
    /// fast: the whole feedback divider shrinks by `1 + ppm · 10⁻⁶`.
    pub fn corrected_frac(&self, ppm: f64) -> f64 {
        self.center / (1.0 + ppm * 1e-6) - self.ndiv as f64 * FRAC_ONE
    }

    /// Width steps from no carrier to the full output.
    pub fn steps(&self) -> u32 {
        self.range / 2
    }
}

fn check_carrier(carrier_hz: f64) -> Result<()> {
    if !(MIN_CARRIER..=MAX_CARRIER).contains(&carrier_hz) {
        return Err(HalError::Config(format!(
            "the Raspberry Pi 2-4 and Zero 2 W make AM carriers of {:.0} kHz to {:.1} MHz",
            MIN_CARRIER / 1e3,
            MAX_CARRIER / 1e6
        )));
    }
    Ok(())
}

/// Plan the clocks for `carrier_hz` from `pll`, with a crystal of nominally
/// `osc` Hz: the finest width steps (the fastest PWM clock up to
/// [`MAX_PWM_CLOCK`] at which half the carrier period, the range, is an
/// even number of its cycles), with room in the PLL's fraction for
/// correcting the crystal; for those, the least change of a PLL that
/// others use (`max_change` at most, see [`PllRoom`]).
pub fn plan(pll: &Pll, carrier_hz: f64, osc: f64, sample_rate: f64, max_change: f64) -> Result<BcmAmPlan> {
    plan_in(&PllRoom::of(pll, cm::PWM, max_change), pll.id, carrier_hz, osc, sample_rate)
}

/// [`plan`] within `room` on the PLL `id`.
fn plan_in(room: &PllRoom, id: PllId, carrier_hz: f64, osc: f64, sample_rate: f64) -> Result<BcmAmPlan> {
    check_carrier(carrier_hz)?;
    let unit_hz = osc * room.fb / FRAC_ONE;
    // `range = 2k`: k steps. Down to half the finest steps (6 dB more
    // quantisation noise) for a plan the PLL can make.
    let k_max = (MAX_PWM_CLOCK / (4.0 * carrier_hz)).floor() as u32;
    for k in ((k_max / 2).max(2)..=k_max).rev() {
        let clk = 4.0 * k as f64 * carrier_hz;
        let mut best: Option<(f64, BcmAmPlan)> = None;
        for &per in &room.per_choices {
            for clk_div in 1..=4095u32 {
                let vco = clk * (per * clk_div) as f64;
                if vco > room.vco_max {
                    break;
                }
                if !room.allows(vco, per) {
                    continue;
                }
                let fb = vco / unit_hz / FRAC_ONE;
                let ndiv = fb.floor();
                let frac = fb - ndiv;
                let reserve = fb * MAX_CLOCK_ERROR_PPM * 1e-6 + 0.005;
                if frac < reserve || frac > 1.0 - reserve || !(1.0..=1023.0).contains(&ndiv) {
                    continue;
                }
                let score = room.change(vco) + if room.running { 0.0 } else { (vco - 1.5e9).abs() / 1e9 };
                if best.as_ref().is_none_or(|(s, _)| score < *s) {
                    let center = (fb * FRAC_ONE).round();
                    let vco = center * unit_hz;
                    let carrier = vco / (per * clk_div * 4 * k) as f64;
                    let cycles = (carrier / sample_rate).round().max(1.0) as u32;
                    best = Some((
                        score,
                        BcmAmPlan {
                            pll: id,
                            power_up: !room.running,
                            set_per: room.per_now != Some(per) || !room.running,
                            ndiv: ndiv as u32,
                            center,
                            per_div: per,
                            clk_div,
                            vco_old: room.vco_old,
                            vco_new: vco,
                            clk_hz: vco / (per * clk_div) as f64,
                            range: 2 * k,
                            carrier_hz: carrier,
                            cycles_per_sample: cycles,
                            sample_rate: carrier / cycles as f64,
                            users: room.users.clone(),
                        },
                    ));
                }
            }
        }
        if let Some((_, p)) = best {
            return Ok(p);
        }
    }
    Err(room.exhausted(id, &format!("an AM carrier of {:.3} kHz", carrier_hz / 1e3)))
}

/// Plan on `cfg.pll`, or on the first PLL of [`pll_preference`] that can
/// make the carrier.
pub fn choose_plan(cfg: &BcmAmConfig, bcm: &Bcm) -> Result<BcmAmPlan> {
    check_carrier(cfg.carrier_hz)?;
    let cpr = Cprman::new(bcm.cm.clone());
    let plan_on = |id| {
        plan(
            &Pll::new(&cpr, id, bcm.osc_hz, bcm.bcm2711),
            cfg.carrier_hz,
            bcm.osc_hz,
            cfg.sample_rate,
            cfg.max_vco_change,
        )
    };
    match cfg.pll {
        Some(id) => plan_on(id),
        None => {
            let mut last = None;
            for id in pll_preference(&cpr, bcm.osc_hz, bcm.bcm2711, cm::PWM) {
                match plan_on(id) {
                    Ok(p) => return Ok(p),
                    Err(e) => last = Some(e),
                }
            }
            Err(last.expect("a planning error"))
        }
    }
}

/// Everything needed to put the hardware back (also run by the guardian).
#[derive(Debug, Clone)]
struct Restorer {
    cm: Cprman,
    gpio: Mmio,
    pads: Mmio,
    pwm: Mmio,
    dma: DmaChannel,
    pin: PinState,
    clk_saved: (u32, u32),
    /// CTL, DMAC and both ranges.
    pwm_saved: [u32; 4],
    pll: Pll,
    pll_saved: PllState,
    cpu: CpuClockPin,
}

impl Restorer {
    fn restore(&self) {
        // Carrier off: the pin back to its previous function.
        self.pin.restore(&self.gpio, &self.pads);
        if let Err(e) = self.dma.stop() {
            warn!("DMA channel {} did not stop cleanly: {e}", self.dma.ch);
        }
        // The PWM registers while its clock still runs (their writes pass a
        // synchroniser in its domain), then the clocks.
        self.pwm.write(pwm::CTL, 0);
        self.pwm.write(pwm::DMAC, self.pwm_saved[1]);
        self.pwm.write(pwm::rng(0), self.pwm_saved[2]);
        self.pwm.write(pwm::rng(1), self.pwm_saved[3]);
        self.pwm.write(pwm::CTL, self.pwm_saved[0]);
        self.cm.restore_clock(cm::PWM, self.clk_saved);
        if let Err(e) = self.pll.restore(&self.pll_saved) {
            warn!("{}: {e}", self.pll.id.name());
        }
        self.cpu.restore();
    }
}

/// Byte offsets of the parts of the DMA memory.
#[derive(Debug, Clone, Copy)]
struct Layout {
    /// Per block: its control block, then its marker control block.
    cbs: usize,
    idle_cb: usize,
    jump_cb: usize,
    /// Per block: the sequence number its marker writes.
    numbers: usize,
    /// Where the markers write.
    progress: usize,
    /// The idle loop's control block address (the jump's source).
    idle_addr: usize,
    idle_data: usize,
    data: usize,
    len: usize,
}

impl Layout {
    fn new(blocks: usize, block_cycles: usize) -> Layout {
        let cbs = 0;
        let idle_cb = cbs + 2 * blocks * dmar::CB_SIZE;
        let jump_cb = idle_cb + dmar::CB_SIZE;
        let numbers = jump_cb + dmar::CB_SIZE;
        let progress = numbers + 4 * blocks;
        let idle_addr = progress + 4;
        let idle_data = (idle_addr + 4).next_multiple_of(dmar::CB_SIZE);
        let data = idle_data + 8 * IDLE_CYCLES;
        let len = data + 8 * blocks * block_cycles;
        Layout { cbs, idle_cb, jump_cb, numbers, progress, idle_addr, idle_data, data, len }
    }
}

/// Where the DMA engine is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Place {
    /// In the ring: cycles of the first unplayed block taken so far.
    Ring(usize),
    /// In the idle loop.
    Idle,
    /// On the way from the idle loop back into the ring.
    Jump,
    /// Stopped (the channel was reset).
    Stopped,
}

/// The BCM AM transmitter.
pub struct BcmAmTransmitter {
    info: SinkInfo,
    plan: BcmAmPlan,
    cm: Cprman,
    ch: DmaChannel,
    pwm: Mmio,
    gpio: Mmio,
    pads: Mmio,
    pll: Pll,
    /// The PLL as configured for transmission (checked for interference).
    pll_good: Option<PllConfig>,
    clock_watch: ClockWatch,
    /// (GPIO, PWM channel, ALT function).
    pin: (u32, usize, u32),
    drive: u8,
    restorer: Restorer,
    mem: VcMemory,
    guardian: Option<Guardian>,
    widths: PulseWidths,
    /// The widths of one sample's carrier cycles.
    sample_widths: Vec<u32>,
    /// Half the range: the words of a cycle are `half ∓ width`.
    half: u32,
    /// The FRAC value in the PLL.
    frac: u32,
    osc_hz: f64,

    // Ring geometry.
    blocks: usize,
    block_cycles: usize,
    layout: Layout,
    bus_base: u32,

    // Producer / consumer bookkeeping (monotonic block counts; `fill`
    // cycles of the next block are written).
    validated: u64,
    completed: u64,
    fill: usize,
    /// Cycles consumed (monotonic).
    consumed_cycles: u64,
    written: u64,
    underruns: u64,
    parked: bool,
    /// Start of the current rate measurement window, and `consumed_cycles`
    /// and `underruns` then.
    rate_window: (Instant, u64, u64),
    /// When a shortfall was last reported.
    rate_warned: Option<Instant>,
    /// Mean amplitude of the samples written, and the width the idle loop
    /// holds.
    mean: f32,
    mean_weight: f32,
    idle_width: u32,
    running: bool,
    stopped: bool,
    last_health: Instant,
}

impl std::fmt::Debug for BcmAmTransmitter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.info.description)
    }
}

impl BcmAmTransmitter {
    pub fn new(cfg: &BcmAmConfig, bcm: &Bcm) -> Result<BcmAmTransmitter> {
        let (chan, alt) = pwm_for_pin(cfg.gpio)
            .ok_or_else(|| HalError::Config(format!("GPIO{} has no PWM output (use 12, 13, 18 or 19)", cfg.gpio)))?;
        if cfg.drive > 7 {
            return Err(HalError::Config("drive strength must be 0-7".into()));
        }
        let ctl = bcm.pwm.read(pwm::CTL);
        if ctl & (pwm::CTL_PWEN1 | pwm::CTL_PWEN1 << pwm::CTL_CHAN2_SHIFT) != 0 {
            return Err(HalError::Hardware(format!(
                "the PWM is in use (CTL {ctl:#x}): analogue audio, or another program"
            )));
        }
        let plan = choose_plan(cfg, bcm)?;
        // The firmware's ARM clock changes stall the DMA engine for longer
        // than the PWM's FIFO lasts (see `cpufreq`).
        let mut cpu = CpuPinGuard::new(Some(CpuClockPin::pin()));
        let cpr = Cprman::new(bcm.cm.clone());
        let pll = Pll::new(&cpr, plan.pll, bcm.osc_hz, bcm.bcm2711);
        let ch = DmaChannel::claim(&bcm.dma, cfg.dma_channel, bcm.bcm2711)?;

        // Ring geometry: blocks of up to a page (512 cycles), at least 16
        // of them in the buffer, which comes close to the length asked for.
        let want = cfg.buffer.as_secs_f64() * plan.carrier_hz;
        let mut block_cycles = 512;
        while block_cycles > 64 && want / (block_cycles as f64) < 16.0 {
            block_cycles /= 2;
        }
        let mut blocks = ((want / block_cycles as f64).ceil() as usize).clamp(8, 4096);
        let (mem, layout) = loop {
            let layout = Layout::new(blocks, block_cycles);
            match VcMemory::alloc(layout.len) {
                Ok(m) => break (m, layout),
                Err(e) if blocks > 16 => {
                    debug!("{e}; retrying with a smaller buffer");
                    blocks /= 2;
                }
                Err(e) => return Err(e),
            }
        };

        let restorer = Restorer {
            cm: cpr.clone(),
            gpio: bcm.gpio.clone(),
            pads: bcm.pads.clone(),
            pwm: bcm.pwm.clone(),
            dma: ch.clone(),
            pin: PinState::save(bcm, cfg.gpio),
            clk_saved: (cpr.read(cm::PWM), cpr.read(cm::div(cm::PWM))),
            pwm_saved: [ctl, bcm.pwm.read(pwm::DMAC), bcm.pwm.read(pwm::rng(0)), bcm.pwm.read(pwm::rng(1))],
            pll: pll.clone(),
            pll_saved: pll.snapshot(),
            cpu: cpu.take(),
        };

        let fs = plan.sample_rate;
        let change = if plan.vco_old > 0.0 {
            format!(
                "VCO {:.3} -> {:.3} MHz ({:+.2} %)",
                plan.vco_old / 1e6,
                plan.vco_new / 1e6,
                (plan.vco_new / plan.vco_old - 1.0) * 100.0
            )
        } else {
            format!("VCO {:.3} MHz (was idle)", plan.vco_new / 1e6)
        };
        let description = format!(
            "BCM{} {}_PER /{} -> PWM clock /{} = {:.3} MHz -> PWM{chan} on GPIO{}: carrier {:.4} kHz ({change}, \
             {} width steps to full output), {:.3} kS/s ({} cycles per sample), DMA ch{}, {} x {} cycles \
             ({:.1} ms), drive {} mA",
            if bcm.bcm2711 { "2711" } else { "283x" },
            plan.pll.name(),
            plan.per_div,
            plan.clk_div,
            plan.clk_hz / 1e6,
            cfg.gpio,
            plan.carrier_hz / 1e3,
            plan.steps(),
            fs / 1e3,
            plan.cycles_per_sample,
            ch.ch,
            blocks,
            block_cycles,
            (blocks * block_cycles) as f64 / plan.carrier_hz * 1e3,
            2 * (cfg.drive as u32 + 1),
        );
        if !plan.users.is_empty() {
            warn!(
                "{} is shared with: {}; they will run {:+.2} % off their nominal frequency while transmitting",
                plan.pll.name(),
                plan.users.names().join(", "),
                (plan.vco_new / plan.vco_old - 1.0) * 100.0
            );
        }
        let info = SinkInfo {
            description,
            sample_rate: fs,
            carrier_hz: plan.carrier_hz,
            max_offset_hz: 0.0,
            resolution_hz: 0.0,
            max_clock_error_ppm: MAX_CLOCK_ERROR_PPM,
            paced_by_carrier: true,
            buffer_len: (blocks - 1) * block_cycles / plan.cycles_per_sample as usize,
            format: SampleFormat::Amplitude,
            realtime: true,
            zero_order_hold: false,
            response_db: interpolation_response_db(fs, fs / 4.0),
        };

        let cycles = plan.cycles_per_sample as usize;
        let steps = plan.steps();
        let mut tx = BcmAmTransmitter {
            info,
            widths: PulseWidths::new(cycles, plan.range as f64, steps),
            sample_widths: vec![0; cycles],
            half: steps,
            frac: plan.corrected_frac(0.0).round() as u32,
            osc_hz: bcm.osc_hz,
            clock_watch: ClockWatch::new(cm::TCNT_PWM, bcm.osc_hz, plan.clk_hz),
            mean_weight: (1.0 / (fs * MEAN_TIME)) as f32,
            plan,
            cm: cpr,
            ch,
            pwm: bcm.pwm.clone(),
            gpio: bcm.gpio.clone(),
            pads: bcm.pads.clone(),
            pll,
            pll_good: None,
            pin: (cfg.gpio, chan, alt),
            drive: cfg.drive,
            restorer,
            bus_base: mem.dma_addr_at(0) as u32,
            mem,
            guardian: None,
            blocks,
            block_cycles,
            layout,
            validated: 0,
            completed: 0,
            fill: 0,
            consumed_cycles: 0,
            written: 0,
            underruns: 0,
            parked: false,
            rate_window: (Instant::now(), 0, 0),
            rate_warned: None,
            mean: 0.0,
            idle_width: 0,
            running: false,
            stopped: false,
            last_health: Instant::now(),
        };
        tx.init_ring();
        tx.spawn_guardian()?;
        if let Err(e) = tx.setup() {
            tx.shutdown();
            return Err(e);
        }
        info!("{}", tx.info.description);
        Ok(tx)
    }

    pub fn plan(&self) -> &BcmAmPlan {
        &self.plan
    }

    fn bus(&self, off: usize) -> u32 {
        self.bus_base + off as u32
    }

    fn block_cb(&self, k: usize) -> usize {
        self.layout.cbs + 2 * k * dmar::CB_SIZE
    }

    fn marker_cb(&self, k: usize) -> usize {
        self.block_cb(k) + dmar::CB_SIZE
    }

    fn block_data(&self, k: usize) -> usize {
        self.layout.data + 8 * k * self.block_cycles
    }

    fn init_ring(&mut self) {
        let fifo = regs::BUS_PERIPH_BASE + (regs::PWM + pwm::FIF1) as u32;
        // Data: bursts paced by the PWM's requests. Bookkeeping: plain writes.
        let paced = dmar::TI_SRC_INC
            | dmar::TI_DEST_DREQ
            | (pwm::DREQ << dmar::TI_PERMAP_SHIFT)
            | (3 << dmar::TI_BURST_LENGTH_SHIFT)
            | dmar::TI_WAIT_RESP
            | dmar::TI_NO_WIDE_BURSTS;
        let plain = dmar::TI_WAIT_RESP | dmar::TI_NO_WIDE_BURSTS;
        let l = self.layout;
        let idle = self.bus(l.idle_cb);
        for k in 0..self.blocks {
            let block = ControlBlock {
                ti: paced,
                source: self.bus(self.block_data(k)),
                dest: fifo,
                length: (8 * self.block_cycles) as u32,
                stride: 0,
                next: self.bus(self.marker_cb(k)),
            };
            let marker = ControlBlock {
                ti: plain,
                source: self.bus(l.numbers + 4 * k),
                dest: self.bus(l.progress),
                length: 4,
                stride: 0,
                next: idle,
            };
            self.mem.write_words(self.block_cb(k), &block.words());
            self.mem.write_words(self.marker_cb(k), &marker.words());
        }
        let idle_cb = ControlBlock {
            ti: paced,
            source: self.bus(l.idle_data),
            dest: fifo,
            length: (8 * IDLE_CYCLES) as u32,
            stride: 0,
            next: idle,
        };
        // The jump closes the idle loop again, then continues in the ring
        // (its `next` is set when it is used).
        let jump_cb = ControlBlock {
            ti: plain,
            source: self.bus(l.idle_addr),
            dest: self.bus(l.idle_cb + CB_NEXT),
            length: 4,
            stride: 0,
            next: idle,
        };
        self.mem.write_words(l.idle_cb, &idle_cb.words());
        self.mem.write_words(l.jump_cb, &jump_cb.words());
        self.mem.write_u32(l.idle_addr, idle);
        self.mem.write_u32(l.progress, 0);
        self.write_idle(0);
        self.mem.flush(0, self.mem.len());
    }

    /// Fill the idle loop's block with the width `u`.
    fn write_idle(&mut self, u: u32) {
        for c in 0..IDLE_CYCLES {
            self.mem.write_u32(self.layout.idle_data + 8 * c, self.half - u);
            self.mem.write_u32(self.layout.idle_data + 8 * c + 4, self.half + u);
        }
        self.idle_width = u;
    }

    fn spawn_guardian(&mut self) -> Result<()> {
        let restorer = self.restorer.clone();
        let token = self.mem.release_token()?;
        let keep = [token.fd()];
        self.guardian = Some(Guardian::spawn(&keep, move || {
            restorer.restore();
            token.release();
        })?);
        Ok(())
    }

    /// The PLL and the PWM clock, checked with the frequency counter; the
    /// PWM channel waits (disabled) for its FIFO.
    fn setup(&mut self) -> Result<()> {
        let p = self.plan.clone();
        self.cm.clock_stop(cm::PWM)?;
        let frac = p.corrected_frac(0.0).round() as u32;
        if p.power_up {
            self.pll.power_up(p.ndiv, frac)?;
        } else {
            self.pll.retune(p.ndiv, frac)?;
        }
        if p.set_per {
            self.pll.set_per(p.per_div)?;
        }
        self.frac = frac;
        self.pll_good = Some(self.pll.config());
        self.cm.clock_start(cm::PWM, p.pll.gpclk_source(), p.clk_div, 0, 0)?;
        let hz = self.cm.measure(cm::TCNT_PWM, self.osc_hz, 0.01)?;
        if (hz - p.clk_hz).abs() > 5_000.0 + p.clk_hz * 2e-4 {
            return Err(HalError::Hardware(format!(
                "the PWM clock runs at {:.4} MHz instead of {:.4} MHz",
                hz / 1e6,
                p.clk_hz / 1e6
            )));
        }
        debug!("PWM clock measured {:.4} MHz", hz / 1e6);

        let chan = self.pin.1;
        pwm_write_confirmed(&self.pwm, pwm::CTL, 0)?;
        pwm_write_confirmed(&self.pwm, pwm::DMAC, 0)?;
        pwm_write_confirmed(&self.pwm, pwm::rng(chan), p.range)?;
        self.pwm.write(pwm::CTL, pwm::CTL_CLRF1);
        wait_until(Duration::from_millis(5), "PWM FIFO clear", || self.pwm.read(pwm::STA) & pwm::STA_EMPT1 != 0)?;
        self.pwm.write(pwm::STA, pwm::STA_ERRORS);
        pwm_write_confirmed(&self.pwm, pwm::DMAC, pwm::DMAC_ENAB | (DREQ_LEVEL << pwm::DMAC_PANIC_SHIFT) | DREQ_LEVEL)?;
        Ok(())
    }

    /// Append the samples to the ring. Not inlined, so that the code of the
    /// loop does not depend on the rest of `write`.
    #[inline(never)]
    fn push_samples(&mut self, samples: &[f32]) {
        for &s in samples {
            self.mean += (s.clamp(0.0, 1.0) - self.mean) * self.mean_weight;
            self.widths.sample(s, &mut self.sample_widths);
            self.push_sample();
        }
    }

    /// Append the widths of one sample to the ring, two words per carrier
    /// cycle, and hand each block to the hardware once it is full.
    fn push_sample(&mut self) {
        let mut i = 0;
        while i < self.sample_widths.len() {
            let k = (self.validated % self.blocks as u64) as usize;
            let n = (self.sample_widths.len() - i).min(self.block_cycles - self.fill);
            let off = self.block_data(k) + 8 * self.fill;
            assert!(off + 8 * n <= self.mem.len());
            // SAFETY: within the block area (checked above); the blocks are
            // 8-byte aligned.
            let dst = unsafe { self.mem.as_ptr().add(off).cast::<u64>() };
            for (j, &u) in self.sample_widths[i..i + n].iter().enumerate() {
                // SAFETY: j < n, within the range checked above.
                unsafe { store64(dst.add(j), self.half - u, self.half + u) };
            }
            self.fill += n;
            i += n;
            if self.fill == self.block_cycles {
                self.link_block();
            }
        }
    }

    /// Hand the next block to the hardware: it becomes the tail, which
    /// leads to the idle loop, and the previous tail leads to it.
    fn link_block(&mut self) {
        let v = self.validated;
        let k = (v % self.blocks as u64) as usize;
        self.mem.write_u32(self.layout.numbers + 4 * k, (v + 1) as u32);
        self.mem.write_u32(self.marker_cb(k) + CB_NEXT, self.bus(self.layout.idle_cb));
        if v > 0 {
            let prev = ((v - 1) % self.blocks as u64) as usize;
            self.mem.write_u32(self.marker_cb(prev) + CB_NEXT, self.bus(self.block_cb(k)));
        }
        self.mem.flush(0, 0);
        self.validated += 1;
        self.fill = 0;
    }

    /// Hold the mean amplitude in the idle loop.
    fn update_idle(&mut self) {
        let u = self.widths.steady(self.mean);
        if u != self.idle_width {
            self.write_idle(u);
        }
    }

    /// Read where the DMA engine is, and count the blocks it has finished
    /// (from the progress word, read before and after the engine's
    /// registers so that the three readings belong together). In the idle
    /// loop, the engine stays put and the marker that led there has written
    /// its number: the progress word is read after the registers.
    fn place(&mut self) -> Result<Place> {
        let idle = self.bus(self.layout.idle_cb);
        let (progress, cb, src) = loop {
            let p = self.mem.read_u32(self.layout.progress);
            let (cb, src) = (self.ch.current_cb(), self.ch.source());
            if cb == idle {
                barrier::dsb();
                break (self.mem.read_u32(self.layout.progress), cb, src);
            }
            if self.mem.read_u32(self.layout.progress) == p {
                break (p, cb, src);
            }
        };
        let done = progress.wrapping_sub(self.completed as u32) as u64;
        if self.completed + done > self.validated {
            return Err(HalError::Hardware(format!(
                "DMA progress {progress} is past the {} blocks handed over",
                self.validated
            )));
        }
        self.completed += done;
        let off = cb.wrapping_sub(self.bus_base) as usize;
        let k = (self.completed % self.blocks as u64) as usize;
        Ok(if cb == 0 {
            Place::Stopped
        } else if off == self.layout.idle_cb {
            Place::Idle
        } else if off == self.layout.jump_cb {
            Place::Jump
        } else if off == self.block_cb(k) && self.completed < self.validated {
            let taken = src.wrapping_sub(self.bus(self.block_data(k))) as usize / 8;
            Place::Ring(taken.min(self.block_cycles))
        } else if off == self.marker_cb(k) && self.completed < self.validated {
            Place::Ring(self.block_cycles)
        } else if off < self.layout.idle_cb && off.is_multiple_of(dmar::CB_SIZE) {
            // Between the reads: the marker of the block before.
            Place::Ring(0)
        } else {
            return Err(HalError::Hardware(format!("DMA control block address {cb:#010x} outside the ring")));
        })
    }

    /// Send a parked engine on to the first unplayed block, through the
    /// jump. Only from the closed idle loop: an armed jump stays armed until
    /// the engine takes it, and then the engine is in the ring until it runs
    /// out again (arming it again before the engine is seen in the ring
    /// would replay). The loop is found closed before the engine is seen in
    /// it, so no jump can be pending: the engine is parked, and the progress
    /// word tells exactly how far it got.
    fn resume_if_parked(&mut self) -> Result<()> {
        let idle_next = self.layout.idle_cb + CB_NEXT;
        let idle = self.bus(self.layout.idle_cb);
        if !self.parked || self.mem.read_u32(idle_next) != idle {
            return Ok(());
        }
        barrier::dsb();
        if self.place()? == Place::Idle && self.validated > self.completed {
            let k = (self.completed % self.blocks as u64) as usize;
            self.mem.write_u32(self.layout.jump_cb + CB_NEXT, self.bus(self.block_cb(k)));
            self.mem.flush(0, 0);
            self.mem.write_u32(idle_next, self.bus(self.layout.jump_cb));
            self.mem.flush(0, 0);
        }
        Ok(())
    }

    /// Start the DMA engine at the first unplayed block, or in the idle
    /// loop if there is none.
    fn restart_dma(&mut self) -> Result<()> {
        self.mem.write_u32(self.layout.idle_cb + CB_NEXT, self.bus(self.layout.idle_cb));
        let start = if self.validated > self.completed {
            self.block_cb((self.completed % self.blocks as u64) as usize)
        } else {
            self.layout.idle_cb
        };
        self.ch.start(self.bus(start))
    }

    /// Detect and repair interference from other users of the hardware.
    fn check_health(&mut self) -> Result<()> {
        if let Some(e) = self.ch.errors() {
            warn!("DMA channel {} error ({e}); restarting it", self.ch.ch);
            self.restart_dma()?;
        } else if !self.ch.is_active() {
            warn!("DMA channel {} was stopped externally; restarting it", self.ch.ch);
            self.restart_dma()?;
        }
        let chan = self.pin.1;
        let sta = self.pwm.read(pwm::STA) & pwm::STA_ERRORS;
        if sta != 0 {
            debug!("PWM status errors {sta:#x}");
            self.pwm.write(pwm::STA, sta);
        }
        if self.cm.read(cm::PWM) & cm::ENAB == 0 {
            return Err(HalError::Hardware("the PWM clock was disabled externally".into()));
        }
        if self.pwm.read(pwm::CTL) & (pwm::CTL_PWEN1 << (pwm::CTL_CHAN2_SHIFT * chan as u32)) == 0 {
            return Err(HalError::Hardware(format!("PWM{chan} was disabled externally")));
        }
        // The PLL is watched through the clock manager: its lock bit, and
        // the PWM clock, counted in the background. Its own registers are
        // only read once the clock is wrong (another user of the counter
        // can cause that too).
        let wrong_hz = self.clock_watch.check(&self.cm);
        if let Some(good) = &self.pll_good {
            if !self.pll.is_locked() || (wrong_hz.is_some() && !self.pll.matches(good)) {
                let now = self.pll.config();
                warn!(
                    "{} was changed behind txengine's back (CTRL {:#x}, PER {:#x}, {}{}); restoring it",
                    self.pll.id.name(),
                    now.ctrl,
                    now.per,
                    if self.pll.is_locked() { "locked" } else { "not locked" },
                    wrong_hz.map(|hz| format!(", PWM clock {:.3} MHz", hz / 1e6)).unwrap_or_default()
                );
                self.pll.repair(good)?;
                self.cm.write(self.pll.frac_offset(), self.frac);
            } else if let Some(hz) = wrong_hz {
                let frac = self.cm.read(self.pll.frac_offset()) & cm::A2W_FRAC_MASK;
                if frac != self.frac {
                    warn!("{}'s fraction was changed behind txengine's back; restoring it", self.pll.id.name());
                    self.cm.write(self.pll.frac_offset(), self.frac);
                } else {
                    debug!("PWM clock counted at {:.3} MHz, but the PLL is as configured", hz / 1e6);
                }
            }
        }
        Ok(())
    }

    /// The PWM must take a word pair every carrier cycle: a DMA engine
    /// that stalls for longer than the FIFO lasts leaves it without words
    /// for whole cycles (it holds the output low), which also shifts the
    /// carrier's phase. The PWM's gap flag stays clear in mark-space mode,
    /// so the cycles taken are counted against the time, over 5 s windows
    /// without underruns (the crystal error is at most 0.02 %).
    fn check_rate(&mut self) {
        let (t0, c0, u0) = self.rate_window;
        let dt = t0.elapsed().as_secs_f64();
        if dt < 5.0 {
            return;
        }
        self.rate_window = (Instant::now(), self.consumed_cycles, self.underruns);
        if self.underruns != u0 {
            return;
        }
        let missed = 1.0 - (self.consumed_cycles - c0) as f64 / (dt * self.plan.carrier_hz);
        if missed > 0.002 {
            let now = Instant::now();
            if self.rate_warned.is_none_or(|t| now.duration_since(t) > Duration::from_secs(60)) {
                warn!(
                    "the PWM missed {:.2} % of the carrier cycles: the DMA engine stalls (CPU clock changes?), \
                     which shifts the carrier's phase",
                    missed * 100.0
                );
                self.rate_warned = Some(now);
            } else {
                debug!("the PWM missed {:.2} % of the carrier cycles", missed * 100.0);
            }
        }
    }

    fn shutdown(&mut self) {
        if self.stopped {
            return;
        }
        self.stopped = true;
        self.running = false;
        self.restorer.restore();
        if let Some(mut g) = self.guardian.take() {
            g.dismiss();
        }
    }

    /// Free space in carrier cycles.
    fn free_cycles(&self) -> usize {
        let in_flight = (self.validated - self.completed) as usize;
        ((self.blocks - in_flight) * self.block_cycles).saturating_sub(self.fill)
    }
}

impl RfSink for BcmAmTransmitter {
    fn info(&self) -> &SinkInfo {
        &self.info
    }

    fn start(&mut self) -> Result<(), SinkError> {
        if self.running {
            return Ok(());
        }
        if self.validated == 0 {
            return Err(SinkError::config("pre-fill the buffer before starting"));
        }
        self.ch.start(self.bus(self.block_cb(0)))?;
        // Enable the channel once the DMA engine has filled its FIFO (to
        // the request level).
        wait_until(
            Duration::from_millis(20),
            "PWM FIFO fill",
            || matches!(self.place(), Ok(Place::Ring(c)) if 2 * c >= DREQ_LEVEL as usize),
        )?;
        let (gpio, chan, alt) = self.pin;
        let bits = pwm::CTL_USEF1 | pwm::CTL_MSEN1 | pwm::CTL_PWEN1;
        pwm_write_confirmed(&self.pwm, pwm::CTL, bits << (pwm::CTL_CHAN2_SHIFT * chan as u32))?;
        // Carrier on.
        set_pin_function(&self.gpio, &self.pads, gpio, alt, self.drive);
        // The PWM must take words: the engine moves past what fits into the FIFO.
        let moving =
            |t: &mut Self| matches!(t.place(), Ok(Place::Ring(c)) if c > pwm::FIFO_LEN as usize) || t.completed > 0;
        if wait_until(Duration::from_millis(20), "PWM pacing", || moving(self)).is_err() {
            return Err(SinkError::hw(format!("the PWM takes no words from DMA channel {}", self.ch.ch)));
        }
        self.running = true;
        let now = Instant::now();
        self.last_health = now;
        self.rate_window = (now, self.consumed_cycles, self.underruns);
        Ok(())
    }

    fn poll(&mut self) -> Result<SinkPoll, SinkError> {
        let cycles = self.plan.cycles_per_sample as u64;
        if self.running {
            if self.last_health.elapsed() > Duration::from_millis(100) {
                self.last_health = Instant::now();
                self.check_health()?;
            }
            let intra = match self.place()? {
                Place::Ring(c) => {
                    self.parked = false;
                    c
                }
                Place::Idle => {
                    // The engine ran out of samples (an underrun) and holds
                    // the carrier at its mean amplitude.
                    if !self.parked {
                        self.parked = true;
                        self.underruns += 1;
                    }
                    self.resume_if_parked()?;
                    0
                }
                Place::Jump => 0,
                Place::Stopped => {
                    self.check_health()?;
                    0
                }
            };
            let at = self.completed * self.block_cycles as u64 + intra as u64;
            self.consumed_cycles = self.consumed_cycles.max(at);
            self.check_rate();
        }
        let writable = self.free_cycles() / cycles as usize;
        Ok(SinkPoll {
            writable,
            consumed: self.consumed_cycles / cycles,
            written: self.written,
            underruns: self.underruns,
        })
    }

    fn write(&mut self, samples: &[f32]) -> Result<(), SinkError> {
        if samples.len() * self.widths.cycles() > self.free_cycles() {
            return Err(SinkError::config("write exceeds the writable space"));
        }
        self.push_samples(samples);
        self.written += samples.len() as u64;
        self.update_idle();
        if self.running {
            self.resume_if_parked()?;
        }
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SinkError> {
        self.shutdown();
        Ok(())
    }

    fn set_clock_error_ppm(&mut self, ppm: f64) -> Result<(), SinkError> {
        if !(-MAX_CLOCK_ERROR_PPM..=MAX_CLOCK_ERROR_PPM).contains(&ppm) {
            return Err(SinkError::config(format!("crystal error {ppm} ppm is outside ±{MAX_CLOCK_ERROR_PPM} ppm")));
        }
        // Every write is a slow transaction on the A2W bus (~4 µs), so the
        // fraction is only written when it has moved by most of a unit
        // (0.016 ppm at 2.2 GHz).
        let frac = self.plan.corrected_frac(ppm).clamp(0.0, cm::A2W_FRAC_MASK as f64);
        if (frac - self.frac as f64).abs() >= 0.75 {
            self.frac = frac.round() as u32;
            self.cm.write(self.pll.frac_offset(), self.frac);
        }
        Ok(())
    }
}

impl Drop for BcmAmTransmitter {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The PLLs of a Zero 2 W as found: PLLA clocks only the GPU's 3D
    /// block, PLLC_PER the EMMC (WiFi), and an idle PLL.
    fn rooms() -> [(PllRoom, &'static str); 3] {
        let running = |vco: f64, users: PllUsers, per_choices: Vec<u32>, down: f64| PllRoom {
            running: true,
            users,
            vco_old: vco,
            fb: 2.0,
            per_now: (per_choices.len() == 1).then(|| per_choices[0]),
            per_choices,
            vco_min: vco * (1.0 - down),
            vco_max: vco * 1.03,
            max_decrease: down,
            max_increase: 0.03,
        };
        let gpu = PllUsers { per: vec![], other: vec!["PLLA_CORE (v3d)".into()], gpu_only: true };
        let emmc = PllUsers { per: vec!["emmc".into()], other: vec!["PLLC_CORE0 (vpu)".into()], gpu_only: false };
        let idle = PllRoom {
            running: false,
            users: PllUsers::default(),
            vco_old: 0.0,
            fb: 1.0,
            per_choices: (1..=8).collect(),
            per_now: None,
            vco_min: 1.0e9,
            vco_max: 1.75e9,
            max_decrease: 0.03,
            max_increase: 0.03,
        };
        [
            (running(2250e6, gpu, (1..=8).collect(), 0.15), "PLLA"),
            (running(2000e6, emmc, vec![2], 0.03), "PLLC"),
            (idle, "idle"),
        ]
    }

    #[test]
    fn plans_for_the_broadcast_bands() {
        let osc = 19.2e6;
        for (room, name) in rooms() {
            for khz in [153.0, 198.0, 279.0, 531.0, 999.0, 1_602.0, 1_710.0, 2_300.0, 3_255.0, 3_950.0, 4_200.0] {
                let p = plan_in(&room, PllId::A, khz * 1e3, osc, 100e3).unwrap();
                let what = format!("{name}, {khz} kHz: {p:?}");
                // Within half a step of the 20-bit fraction.
                let step_hz = osc * room.fb / FRAC_ONE * p.carrier_hz / p.vco_new;
                assert!((p.carrier_hz - khz * 1e3).abs() <= 0.5 * step_hz, "{what}");
                let k_max = (MAX_PWM_CLOCK / (4e3 * khz)).floor() as u32;
                // PLLC, held within ±3 % with its PER divider fixed, cannot
                // always make the finest steps at the highest carriers.
                let k_min = if name == "PLLC" { k_max / 2 } else { k_max * 9 / 10 };
                assert!(p.range.is_multiple_of(2) && (k_min..=k_max).contains(&p.steps()), "{what}");
                assert!(p.clk_hz <= MAX_PWM_CLOCK, "{what}");
                assert!((p.clk_hz / (2 * p.range) as f64 - p.carrier_hz).abs() < 1e-6, "{what}");
                assert!((p.clk_hz * (p.per_div * p.clk_div) as f64 - p.vco_new).abs() < 1e-3, "{what}");
                assert!(room.allows(p.vco_new, p.per_div) && room.per_choices.contains(&p.per_div), "{what}");
                assert_eq!(p.power_up, !room.running);
                assert!((p.sample_rate - 100e3).abs() <= p.sample_rate / p.cycles_per_sample as f64, "{what}");
                // The crystal can be corrected either way within the fraction.
                for ppm in [-MAX_CLOCK_ERROR_PPM, MAX_CLOCK_ERROR_PPM] {
                    let f = p.corrected_frac(ppm);
                    assert!((0.0..FRAC_ONE).contains(&f), "{what}, {ppm} ppm: {f}");
                    let vco = osc * (1.0 + ppm * 1e-6) * room.fb * (p.ndiv as f64 + f / FRAC_ONE);
                    let carrier = vco / (p.per_div * p.clk_div * 2 * p.range) as f64;
                    assert!((carrier - p.carrier_hz).abs() < 1e-3, "{what}, {ppm} ppm: {carrier}");
                }
            }
        }
        // PLLA, retuned least for the finest steps: 999 kHz at 2229.8 MHz (-0.9 %).
        let p = plan_in(&rooms()[0].0, PllId::A, 999e3, osc, 100e3).unwrap();
        assert_eq!((p.steps(), p.per_div * p.clk_div), (62, 9), "{p:?}");
        assert!(plan_in(&rooms()[0].0, PllId::A, 50e3, osc, 100e3).is_err());
        assert!(plan_in(&rooms()[0].0, PllId::A, 4.3e6, osc, 100e3).is_err());
    }

    #[test]
    fn layout_keeps_control_blocks_aligned() {
        for (blocks, cycles) in [(8, 64), (17, 512), (4096, 512)] {
            let l = Layout::new(blocks, cycles);
            for off in [l.cbs, l.idle_cb, l.jump_cb, l.idle_data, l.data] {
                assert!(off.is_multiple_of(dmar::CB_SIZE), "{l:?}");
            }
            assert!(l.numbers >= l.jump_cb + dmar::CB_SIZE && l.progress == l.numbers + 4 * blocks);
            assert!(l.idle_data >= l.idle_addr + 4 && l.len == l.data + 8 * blocks * cycles);
        }
    }
}
