//! FM transmitter backend for the Raspberry Pi 5 (RP1).
//!
//! Signal path:
//!
//! ```text
//!  xosc 50 MHz ─► PLL (fractional-N) ─► post dividers ─► clk_gpN ─► GPIO pad
//!                   ▲ FBDIV_FRAC
//!                   │ one 24-bit write per sample
//!  pinned memory ─► AXI DMA channel ◄── DMA tick (xosc / N) handshake
//! ```
//!
//! Every sample is the PLL's 24-bit feedback fraction. The DMA engine copies
//! one sample per tick into `FBDIV_FRAC`, so the carrier frequency follows
//! the sample stream with crystal accurate timing; the CPU only refills the
//! buffer.
//!
//! The buffer is a ring of `K` linked-list items (LLIs), each describing one
//! block of samples inside a single physical page. The hardware clears an
//! LLI's VALID bit when it completes the block; the producer sets it again
//! after refilling the block. If the producer falls behind (or dies), the
//! DMA engine stops at the first stale block instead of replaying old data.

use std::os::fd::RawFd;
use std::time::{Duration, Instant};

use log::{debug, info, warn};
use txengine_core::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll};

use super::Rp1;
use super::clocks::{ClockGen, ClockGenRegs, FcSource, Pll, PllId, PllRegs, measure};
use super::dma::{DmaChannel, Dmac};
use super::gpio::{Drive, Gpio, PinRegs, gpclk_for_pin};
use super::regs::{self, dmac::*, lli};
use super::ticks::DmaTick;
use crate::MAX_CLOCK_ERROR_PPM;
use crate::boundary;
use crate::error::{HalError, Result};
use crate::guardian::Guardian;
use crate::mem::pinned::PinnedMemory;
use crate::mem::{DmaMemory, DmaMemoryExt};
use crate::sys::{page_size, round_up};

/// PLL used to synthesise the carrier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rp1Pll {
    /// `pll_audio` – idle unless I2S audio is in use. Default.
    Audio,
    /// `pll_video` – idle unless a DSI/DPI/composite display is driven by RP1
    /// (HDMI is not). Only reaches GPCLK2 (GPIO6).
    Video,
}

impl Rp1Pll {
    fn id(self) -> PllId {
        match self {
            Rp1Pll::Audio => PllId::Audio,
            Rp1Pll::Video => PllId::Video,
        }
    }
}

/// Configuration of the RP1 FM transmitter.
#[derive(Debug, Clone)]
pub struct Rp1FmConfig {
    /// Carrier frequency in Hz.
    pub carrier_hz: f64,
    /// Output pins (all must belong to the same GPCLK generator).
    pub gpios: Vec<u32>,
    pub pll: Rp1Pll,
    pub drive: Drive,
    /// Desired sample rate; the closest rate the tick divider allows is used.
    pub sample_rate: f64,
    /// Buffer length (transmit latency).
    pub buffer: Duration,
    /// Largest frequency offset the plan must accommodate (Hz).
    pub max_offset_hz: f64,
    /// Force a DMA channel (0-7) instead of picking a free one.
    pub dma_channel: Option<usize>,
    /// DMA tick generator to pace with (0 or 1).
    pub tick: usize,
    /// Take the clock generator over even if it is already running: then
    /// something else uses it.
    pub force_gpclk: bool,
}

impl Default for Rp1FmConfig {
    fn default() -> Self {
        Rp1FmConfig {
            carrier_hz: 87.6e6,
            gpios: vec![4],
            pll: Rp1Pll::Audio,
            drive: Drive::Ma12,
            sample_rate: 250_000.0,
            buffer: Duration::from_millis(60),
            max_offset_hz: 100_000.0,
            dma_channel: None,
            tick: 0,
            force_gpclk: false,
        }
    }
}

/// The frequency synthesis plan.
#[derive(Debug, Clone)]
pub struct ClockPlan {
    pub pll: PllId,
    pub gpclk: usize,
    pub auxsrc: u32,
    /// Fixed divider between the PLL primary output and the clock generator
    /// input (2 for `pll_*_pri_ph`).
    pub phase_div: u32,
    pub div1: u32,
    pub div2: u32,
    pub gp_div: u32,
    /// Total division from VCO to carrier.
    pub n: u32,
    pub ref_hz: f64,
    pub vco_hz: f64,
    pub fbdiv_int: u32,
    /// Feedback fraction at the carrier centre, in units of 2^-24.
    pub frac_center: f64,
    /// Fraction units per Hz of carrier offset.
    pub frac_per_hz: f64,
    /// Achieved carrier frequency (after rounding of the fraction).
    pub carrier_hz: f64,
    /// Largest offset that keeps the fraction inside [0, 2^24) under any
    /// crystal error correction up to ±[`MAX_CLOCK_ERROR_PPM`].
    pub max_offset_hz: f64,
    /// How far the carrier is from where the PLL puts a spur next to it
    /// ([`boundary`]).
    pub clearance_hz: f64,
}

impl ClockPlan {
    /// A warning if the carrier is too close to a spur of the PLL for
    /// quiet passages of a modulation up to ±`max_offset_hz`.
    pub fn spur_warning(&self, max_offset_hz: f64) -> Option<String> {
        boundary::warning(self.clearance_hz, self.carrier_hz, max_offset_hz, self.pll.name())
    }

    /// Feedback fraction at the carrier centre and fraction units per Hz
    /// when the crystal runs `ppm` fast: the whole feedback divider shrinks
    /// by `1 + ppm · 10⁻⁶`, so that the VCO keeps its nominal frequency.
    pub fn corrected(&self, ppm: f64) -> (f64, f64) {
        let k = 1.0 / (1.0 + ppm * 1e-6);
        let int = self.fbdiv_int as f64 * FRAC_ONE;
        ((int + self.frac_center) * k - int, self.frac_per_hz * k)
    }
}

const FRAC_ONE: f64 = (1u64 << regs::clk::PLL_FRAC_BITS) as f64;

/// Highest carrier planned (Hz): measured up to 1 GHz, the PLL itself
/// divided by 1. Beyond, the frequency counter that verifies the PLL
/// output no longer counts it (1.3 GHz read as 1.8 MHz).
pub const MAX_CARRIER_HZ: f64 = 1e9;

/// Whether clock generator `gpclk` is already running: then something else
/// uses it.
pub fn gpclk_running(gpclk: usize) -> Result<bool> {
    let rp1 = Rp1::open()?;
    let clocks = rp1.clocks();
    let gp = ClockGen::new(&clocks, &rp1.block_set(regs::CLOCKS_MAIN), &rp1.block_clr(regs::CLOCKS_MAIN), gpclk);
    Ok(gp.is_enabled())
}

/// Choose dividers for `carrier_hz`, from a reference of nominally `ref_hz`.
pub fn plan_clocks(pll: Rp1Pll, gpclk: usize, carrier_hz: f64, ref_hz: f64, max_offset_hz: f64) -> Result<ClockPlan> {
    if carrier_hz > MAX_CARRIER_HZ {
        return Err(crate::beyond_fundamental(carrier_hz, MAX_CARRIER_HZ, "the clock generators"));
    }
    let (auxsrc, phase_div) = match (pll, gpclk) {
        (Rp1Pll::Audio, 0) => (7, 1), // pll_audio
        (Rp1Pll::Audio, 1) => (7, 2), // pll_audio_pri_ph
        (Rp1Pll::Video, 2) => (8, 1), // pll_video
        (Rp1Pll::Audio, _) => {
            return Err(HalError::Config(
                "the audio PLL reaches GPCLK0 (GPIO4/20) and GPCLK1 (GPIO5/18/21); use one of those pins".into(),
            ));
        }
        (Rp1Pll::Video, _) => {
            return Err(HalError::Config("the video PLL only reaches GPCLK2 (GPIO6)".into()));
        }
    };
    // Room for correcting the crystal error at run time.
    let reserve_hz = carrier_hz * MAX_CLOCK_ERROR_PPM * 1e-6;
    let target = boundary::target_hz(carrier_hz, max_offset_hz);
    // The kernel runs pll_audio_core at 1536 MHz; stay in that region
    // unless only a VCO outside keeps the carrier clear of the PLL's spurs.
    const PREFERRED_VCO: f64 = 1536e6;
    let windows = [(1000e6, 1600e6), (800e6, 2400e6)];
    // Some division reached the VCO range, but the fraction sat too close
    // to a whole number for the modulation.
    let mut near_whole = false;
    let mut best: Option<((f64, bool, f64), ClockPlan)> = None;
    for (window, (vmin, vmax)) in windows.into_iter().enumerate() {
        for gp_div in 1..=16u32 {
            for div1 in 1..=7u32 {
                for div2 in 1..=div1 {
                    let n = div1 * div2 * phase_div * gp_div;
                    let vco = carrier_hz * n as f64;
                    if !(vmin..=vmax).contains(&vco) {
                        continue;
                    }
                    let fb = vco / ref_hz;
                    let int = fb.floor();
                    let frac = fb - int;
                    if !(16.0..=320.0).contains(&int) {
                        continue;
                    }
                    let dev = (max_offset_hz + reserve_hz) * n as f64 / ref_hz;
                    let margin = (frac - dev).min(1.0 - (frac + dev));
                    if margin < 0.01 {
                        near_whole |= dev < 0.49;
                        continue;
                    }
                    // Prefer: clear of the PLL's spurs, then a VCO in the
                    // first window, no GPCLK division, a VCO near the
                    // known-good rate, the fraction far from whole numbers.
                    let score =
                        (gp_div - 1) as f64 * 10.0 + ((vco - PREFERRED_VCO) / PREFERRED_VCO).abs() * 5.0 - margin;
                    // Its spurs: the VCO at a half multiple of the reference.
                    let spurs = boundary::Spurs { vco: &[ref_hz / 2.0], carrier: &[] };
                    let clearance_hz = spurs.clearance_hz(carrier_hz, n as f64);
                    let rank = (boundary::rank(clearance_hz, target), window == 0, -score);
                    if best.as_ref().is_none_or(|(r, _)| rank > *r) {
                        let frac_center = (frac * FRAC_ONE).round();
                        let frac_per_hz = n as f64 / ref_hz * FRAC_ONE;
                        let achieved = (int + frac_center / FRAC_ONE) * ref_hz / n as f64;
                        let max_off = (frac_center.min(FRAC_ONE - 1.0 - frac_center)) / frac_per_hz - reserve_hz;
                        best = Some((
                            rank,
                            ClockPlan {
                                pll: pll.id(),
                                gpclk,
                                auxsrc,
                                phase_div,
                                div1,
                                div2,
                                gp_div,
                                n,
                                ref_hz,
                                vco_hz: vco,
                                fbdiv_int: int as u32,
                                frac_center,
                                frac_per_hz,
                                carrier_hz: achieved,
                                max_offset_hz: max_off,
                                clearance_hz,
                            },
                        ));
                    }
                }
            }
        }
    }
    if let Some((_, plan)) = best {
        return Ok(plan);
    }
    if near_whole {
        // The VCO is a multiple of the carrier: near a multiple of the
        // reference, every division puts it near a whole feedback divider.
        return Err(HalError::Config(format!(
            "{:.4} MHz cannot be modulated: it is too close to a multiple of {:.0} MHz, where the PLL's \
             feedback divider would have to cross a whole number",
            carrier_hz / 1e6,
            ref_hz / 1e6
        )));
    }
    Err(HalError::Config(format!("no PLL configuration can produce {:.4} MHz", carrier_hz / 1e6)))
}

/// Hardware state saved before we touched it.
#[derive(Debug, Clone)]
struct Saved {
    pll: PllRegs,
    gp: ClockGenRegs,
    pins: Vec<PinRegs>,
}

/// Everything needed to put the hardware back (also used by the guardian).
#[derive(Debug, Clone)]
struct Restorer {
    pll: Pll,
    gp: ClockGen,
    gpio: Gpio,
    ch: DmaChannel,
    tick_ticks: crate::mmio::Mmio,
    tick_dma: crate::mmio::Mmio,
    tick: usize,
    saved: Saved,
    aspm: super::link::AspmGuard,
}

impl Restorer {
    /// Stop the DMA, silence the output and restore every register.
    fn restore(&self) {
        if let Err(e) = self.ch.stop() {
            warn!("RP1 DMA channel {} did not stop cleanly: {e}", self.ch.ch);
        }
        // DmaTick::new snapshots the *current* state; build the restore
        // values from the saved copy instead.
        let t = DmaTick::new(self.tick_ticks.clone(), self.tick_dma.clone(), self.tick);
        t.stop();
        for p in &self.saved.pins {
            self.gpio.restore(p);
        }
        self.gp.restore(&self.saved.gp);
        if let Err(e) = self.pll.restore(&self.saved.pll) {
            warn!("{}: {e}", self.pll.id.name());
        }
        self.aspm.restore();
    }
}

/// The RP1 FM transmitter.
pub struct Rp1FmTransmitter {
    info: SinkInfo,
    plan: ClockPlan,
    /// Fraction at the carrier centre and per Hz of offset, corrected for
    /// the crystal error.
    frac_center: f64,
    frac_per_hz: f64,
    rp1: Rp1,
    dmac: Dmac,
    ch: DmaChannel,
    tick: DmaTick,
    tick_cycles: u32,
    gp: ClockGen,
    pll: Pll,
    gpio: Gpio,
    pins: Vec<(u32, u32)>,
    drive: Drive,
    restorer: Restorer,
    mem: PinnedMemory,
    guardian: Option<Guardian>,

    // Ring geometry.
    blocks: usize,
    block_len: usize,
    lli_off: Vec<usize>,
    lli_addr: Vec<u64>,
    block_off: Vec<usize>,
    block_addr: Vec<u64>,
    ctl: u64,
    cfg: u64,

    // Producer / consumer bookkeeping (monotonic block counters).
    validated: u64,
    completed: u64,
    fill: usize,
    written: u64,
    underruns: u64,
    /// The engine stopped (underrun or a stale LLI fetch) and has not
    /// completed a block since.
    stalled: bool,
    /// Last (LLP, SAR) seen and when they last changed.
    last_regs: (u64, u64),
    last_progress: Instant,
    /// When the channel was last re-armed (restarts are rate limited).
    last_restart: Option<Instant>,
    running: bool,
    stopped: bool,
    last_health: Instant,
}

impl std::fmt::Debug for Rp1FmTransmitter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.info.description)
    }
}

impl Rp1FmTransmitter {
    pub fn new(cfg: &Rp1FmConfig, xosc_hz: f64) -> Result<Rp1FmTransmitter> {
        if cfg.gpios.is_empty() {
            return Err(HalError::Config("no output GPIO given".into()));
        }
        let mut gpclk = None;
        let mut pins = Vec::new();
        for &g in &cfg.gpios {
            let (n, func) = gpclk_for_pin(g).ok_or_else(|| {
                HalError::Config(format!(
                    "GPIO{g} cannot output a clock on the Raspberry Pi 5 (use 4, 5, 6, 18, 20 or 21)"
                ))
            })?;
            if gpclk.is_some_and(|c| c != n) {
                return Err(HalError::Config("all output GPIOs must use the same GPCLK generator".into()));
            }
            gpclk = Some(n);
            pins.push((g, func));
        }
        let gpclk = gpclk.expect("at least one pin");

        let xosc = xosc_hz;
        let rp1 = Rp1::open()?;
        let clocks = rp1.clocks();
        let clocks_set = rp1.block_set(regs::CLOCKS_MAIN);
        let clocks_clr = rp1.block_clr(regs::CLOCKS_MAIN);
        let pll = Pll::new(&clocks, cfg.pll.id(), xosc);
        let plan = plan_clocks(cfg.pll, gpclk, cfg.carrier_hz, pll.ref_hz(), cfg.max_offset_hz)?;
        if let Some(w) = plan.spur_warning(cfg.max_offset_hz) {
            warn!("{w}");
        }
        let gp = ClockGen::new(&clocks, &clocks_set, &clocks_clr, gpclk);
        // A running clock generator is someone else's.
        if gp.is_enabled() {
            if !cfg.force_gpclk {
                return Err(HalError::Config(format!(
                    "clk_gp{gpclk} is already running: something else uses it; transmit on another clock \
                     generator's pin (GPCLK0: GPIO 4, 20; GPCLK1: 5, 18, 21; GPCLK2: 6, from the video PLL), or \
                     take this one over with --force-gpclk"
                )));
            }
            warn!("clk_gp{gpclk} is already running: taking it over until txengine stops (--force-gpclk)");
        }
        let gpio = Gpio::new(rp1.block(regs::IO_BANK0), rp1.block(regs::PADS_BANK0));
        let dmac = Dmac::new(rp1.block(regs::DMA));

        let tick_cycles = (xosc / cfg.sample_rate).round().clamp(2.0, regs::ticks::CYCLES_MAX as f64) as u32;
        let sample_rate = xosc / tick_cycles as f64;
        let tick = DmaTick::new(rp1.block(regs::TICKS), rp1.block(regs::DMA_TICK), cfg.tick);
        if tick.in_use() {
            debug!("DMA tick {} was configured before (will be restored on exit)", cfg.tick);
        }

        // Ring geometry: blocks of up to 1024 samples, never crossing a page.
        let page = page_size();
        let block_len = (page / 4).min(1024);
        let blocks = ((cfg.buffer.as_secs_f64() * sample_rate / block_len as f64).ceil() as usize).clamp(8, 1024);
        let lli_bytes = round_up(blocks * lli::SIZE, page);
        let mem = PinnedMemory::alloc(lli_bytes + blocks * block_len * 4, regs::HOST_MEM_DMA_OFFSET)?;
        let lli_off: Vec<usize> = (0..blocks).map(|k| k * lli::SIZE).collect();
        let block_off: Vec<usize> = (0..blocks).map(|k| lli_bytes + k * block_len * 4).collect();
        let lli_addr: Vec<u64> = lli_off.iter().map(|&o| mem.dma_addr_at(o)).collect();
        let block_addr: Vec<u64> = block_off.iter().map(|&o| mem.dma_addr_at(o)).collect();

        let ch = dmac.claim(cfg.dma_channel)?;
        let saved = Saved {
            pll: pll.snapshot(),
            gp: gp.snapshot(),
            pins: pins.iter().map(|&(g, _)| gpio.snapshot(g)).collect::<Result<_>>()?,
        };
        if cfg.pll == Rp1Pll::Audio && saved.pll.fbdiv_int != 0 {
            let consumers = ["clk_i2s", "clk_audio_out"];
            debug!("pll_audio was at {:.3} MHz; its users ({consumers:?}) will be affected", pll.vco_hz() / 1e6);
        }

        let ctl = (WIDTH_32 << CTL_SRC_WIDTH_SHIFT)
            | (WIDTH_32 << CTL_DST_WIDTH_SHIFT)
            | (MSIZE_4 << CTL_SRC_MSIZE_SHIFT)
            | (MSIZE_1 << CTL_DST_MSIZE_SHIFT)
            | CTL_DINC_FIXED
            | CTL_ARLEN_EN
            | (3u64 << CTL_ARLEN_SHIFT);
        let ch_cfg = (MULTBLK_LL << CFG_SRC_MULTBLK_SHIFT)
            | (MULTBLK_LL << CFG_DST_MULTBLK_SHIFT)
            | ((tick.dreq() as u64) << CFG_DST_PER_SHIFT)
            | (TT_FC_MEM_TO_PER_DMAC << CFG_TT_FC_SHIFT)
            | (7u64 << CFG_PRIORITY_SHIFT);

        let restorer = Restorer {
            pll: pll.clone(),
            gp: gp.clone(),
            gpio: gpio.clone(),
            ch: ch.clone(),
            tick_ticks: rp1.block(regs::TICKS),
            tick_dma: rp1.block(regs::DMA_TICK),
            tick: cfg.tick,
            saved,
            aspm: super::link::AspmGuard::disable(&rp1.pci_path),
        };

        let description = format!(
            "RP1 {} -> clk_gp{gpclk} on GPIO{:?}: carrier {:.6} MHz (VCO {:.3} MHz = {:.4} MHz x {} [/{}{}/{}]), \
             {:.3} kS/s (xosc/{tick_cycles}), DMA ch{} tick{}, {} x {} samples ({:.1} ms), drive {} mA",
            plan.pll.name(),
            cfg.gpios,
            plan.carrier_hz / 1e6,
            plan.vco_hz / 1e6,
            plan.carrier_hz / 1e6,
            plan.n,
            plan.div1 * plan.div2,
            if plan.phase_div > 1 { "/2" } else { "" },
            plan.gp_div,
            sample_rate / 1e3,
            ch.ch,
            cfg.tick,
            blocks,
            block_len,
            (blocks * block_len) as f64 / sample_rate * 1e3,
            cfg.drive.milliamps(),
        );
        let info = SinkInfo {
            description,
            sample_rate,
            carrier_hz: plan.carrier_hz,
            max_offset_hz: plan.max_offset_hz,
            resolution_hz: 1.0 / plan.frac_per_hz,
            max_clock_error_ppm: MAX_CLOCK_ERROR_PPM,
            paced_by_carrier: false,
            buffer_len: (blocks - 1) * block_len,
            format: SampleFormat::FrequencyOffset,
            realtime: true,
            zero_order_hold: true,
            response_db: Vec::new(),
        };

        let mut tx = Rp1FmTransmitter {
            info,
            frac_center: plan.frac_center,
            frac_per_hz: plan.frac_per_hz,
            plan,
            rp1,
            dmac,
            ch,
            tick,
            tick_cycles,
            gp,
            pll,
            gpio,
            pins,
            drive: cfg.drive,
            restorer,
            mem,
            guardian: None,
            blocks,
            block_len,
            lli_off,
            lli_addr,
            block_off,
            block_addr,
            ctl,
            cfg: ch_cfg,
            validated: 0,
            completed: 0,
            fill: 0,
            written: 0,
            underruns: 0,
            stalled: false,
            last_regs: (0, 0),
            last_progress: Instant::now(),
            last_restart: None,
            running: false,
            stopped: false,
            last_health: Instant::now(),
        };
        tx.init_ring();
        tx.spawn_guardian()?;
        if let Err(e) = tx.setup_clocks() {
            tx.shutdown();
            return Err(e);
        }
        info!("{}", tx.info.description);
        Ok(tx)
    }

    pub fn plan(&self) -> &ClockPlan {
        &self.plan
    }

    fn frac_word(&self, offset_hz: f32) -> u32 {
        let f = self.frac_center + offset_hz as f64 * self.frac_per_hz;
        f.round().clamp(0.0, FRAC_ONE - 1.0) as u32
    }

    fn init_ring(&mut self) {
        let dar = Rp1::dma_addr_of(regs::CLOCKS_MAIN + self.pll.frac_reg_offset());
        let center = self.frac_word(0.0);
        for k in 0..self.blocks {
            let l = self.lli_off[k];
            self.mem.zero(l, lli::SIZE);
            self.mem.write_u64(l + lli::SAR, self.block_addr[k]);
            self.mem.write_u64(l + lli::DAR, dar);
            self.mem.write_u32(l + lli::BLOCK_TS, self.block_len as u32 - 1);
            self.mem.write_u64(l + lli::LLP, self.lli_addr[(k + 1) % self.blocks]);
            // Not valid until filled.
            self.mem.write_u64(l + lli::CTL, self.ctl);
            for i in 0..self.block_len {
                self.mem.write_u32(self.block_off[k] + i * 4, center);
            }
        }
        self.mem.flush(0, self.mem.len());
    }

    fn spawn_guardian(&mut self) -> Result<()> {
        let restorer = self.restorer.clone();
        let keep: Vec<RawFd> = self.mem.pin_fd().into_iter().collect();
        self.guardian = Some(Guardian::spawn(&keep, move || restorer.restore())?);
        Ok(())
    }

    fn setup_clocks(&mut self) -> Result<()> {
        let p = &self.plan;
        self.pll.configure(p.fbdiv_int, p.frac_center as u32, p.div1, p.div2)?;
        if p.phase_div == 2 {
            // Enable the primary phase output (pll_*_pri_ph) at phase 0.
            let off = match p.pll {
                PllId::Audio => regs::clk::PLL_AUDIO,
                PllId::Video => regs::clk::PLL_VIDEO,
                PllId::Sys => regs::clk::PLL_SYS,
            } + regs::clk::PLL_PRIM;
            let clocks = self.rp1.clocks();
            clocks.modify(off, 0x1f, 1 << 4);
        }
        // Verify the PLL output feeding the clock generator with the
        // hardware frequency counter (resolution 31.25 Hz).
        let (fc, expect) = match (p.pll, p.phase_div) {
            (PllId::Audio, 1) => (FcSource::PLL_AUDIO, p.vco_hz / (p.div1 * p.div2) as f64),
            (PllId::Audio, _) => (FcSource::PLL_AUDIO_PRI_PH, p.vco_hz / (p.div1 * p.div2 * 2) as f64),
            _ => (FcSource::PLL_VIDEO, p.vco_hz / (p.div1 * p.div2) as f64),
        };
        let fc_ref = self.pll.ref_hz() * self.pll.refdiv() as f64;
        let hz = measure(&self.rp1.clocks(), fc, fc_ref)?;
        debug!("{} output measured {:.4} MHz (planned {:.4} MHz)", p.pll.name(), hz / 1e6, expect / 1e6);
        if (hz - expect).abs() > 1000.0 + expect * 1e-4 {
            return Err(HalError::Hardware(format!(
                "{} output runs at {:.4} MHz instead of {:.4} MHz",
                p.pll.name(),
                hz / 1e6,
                expect / 1e6
            )));
        }
        self.gp.configure(p.auxsrc, p.gp_div, 0)?;
        Ok(())
    }

    /// Map a channel LLP value to an LLI index.
    fn lli_index(&self, llp: u64) -> Option<usize> {
        let a = llp & !0x3f;
        self.lli_addr.iter().position(|&x| x == a)
    }

    /// Position inside block `k` from the source address register.
    fn intra_block(&self, k: usize, sar: u64) -> usize {
        let start = self.block_addr[k];
        match sar.checked_sub(start) {
            Some(d) if d <= (self.block_len * 4) as u64 => d as usize / 4,
            _ => 0,
        }
    }

    fn validate_block(&mut self, k: usize) {
        let off = self.block_off[k];
        self.mem.flush(off, self.block_len * 4);
        let l = self.lli_off[k];
        self.mem.write_u64(l + lli::CTL, self.ctl | CTL_LLI_VALID);
        self.mem.flush(l, lli::SIZE);
        self.validated += 1;
    }

    /// (Re)start the channel at block number `block` (monotonic count).
    fn restart_channel_at(&mut self, block: u64) -> Result<()> {
        let k = (block % self.blocks as u64) as usize;
        self.ch.stop()?;
        self.ch.program_linked_list(self.lli_addr[k], self.cfg)?;
        let now = Instant::now();
        self.last_progress = now;
        self.last_restart = Some(now);
        self.ch.enable();
        Ok(())
    }

    /// Whether the LLI of ring block `k` is still marked valid in memory.
    /// The engine clears VALID when it writes an LLI back after finishing
    /// its block (verified on hardware), so this is the play position.
    fn lli_valid(&self, k: usize) -> bool {
        let l = self.lli_off[k];
        self.mem.invalidate(l + lli::CTL, 8);
        self.mem.read_u64(l + lli::CTL) & CTL_LLI_VALID != 0
    }

    /// Registers and ring bookkeeping, for diagnosing stalls.
    fn debug_state(&self) -> String {
        let sar = self.ch.sar();
        let sar_at = (0..self.blocks)
            .find_map(|b| {
                let d = sar.checked_sub(self.block_addr[b])?;
                (d <= (self.block_len * 4) as u64).then(|| format!("block {b} + {}", d / 4))
            })
            .unwrap_or_else(|| "outside the ring".into());
        let llp = self.ch.llp();
        let next = self.lli_index(llp).map_or_else(|| format!("{llp:#x}"), |i| i.to_string());
        let valid: String = (0..self.blocks).map(|b| if self.lli_valid(b) { 'V' } else { '.' }).collect();
        format!(
            "LLP -> LLI {next}, SAR at {sar_at}, CTL {:#x}, STATUS {:#x}, INTSTATUS {:#x}, enabled {}, tick {}, \
             completed {} validated {} fill {}, LLI valid [{valid}]",
            self.ch.ctl(),
            self.ch.status(),
            self.ch.intstatus(),
            self.ch.is_enabled(),
            self.tick.is_running(),
            self.completed,
            self.validated,
            self.fill,
        )
    }

    /// Re-arm a stopped engine at the first unplayed block once there is
    /// one. Rate limited: a restart that has not taken effect yet is given
    /// 20 ms before the next attempt.
    fn resume_if_possible(&mut self) -> Result<()> {
        let recent = self.last_restart.is_some_and(|t| t.elapsed() < Duration::from_millis(20));
        if self.validated > self.completed && !recent {
            debug!("RP1 DMA: re-arming at block {} ({})", self.completed, self.debug_state());
            self.restart_channel_at(self.completed)?;
        }
        Ok(())
    }

    /// Detect and repair interference from other users of the controller
    /// (the kernel suspends the DMAC when its own clients go idle).
    fn check_health(&mut self) -> Result<()> {
        let clocks = self.rp1.clocks();
        let set = self.rp1.block_set(regs::CLOCKS_MAIN);
        if self.dmac.ensure_running(&clocks, &set) {
            warn!("RP1 DMA controller had been suspended by another driver; re-enabled");
        }
        if !self.tick.is_running() {
            warn!("DMA tick generator was stopped externally; restarting");
            self.tick.start(self.tick_cycles)?;
        }
        if !self.gp.is_enabled() {
            return Err(HalError::Hardware(format!("clk_gp{} was disabled externally", self.plan.gpclk)));
        }
        if !self.ch.is_enabled() {
            warn!("RP1 DMA channel {} was disabled externally; restarting", self.ch.ch);
            self.restart_channel_at(self.completed)?;
        }
        Ok(())
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
}

impl RfSink for Rp1FmTransmitter {
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
        let clocks = self.rp1.clocks();
        let set = self.rp1.block_set(regs::CLOCKS_MAIN);
        if self.dmac.ensure_running(&clocks, &set) {
            debug!("RP1 DMA controller was runtime-suspended; enabled it");
        }
        self.tick.start(self.tick_cycles)?;
        self.restart_channel_at(0)?;
        // Carrier on: route the clock to the pins.
        self.gp.enable_output();
        for &(g, func) in &self.pins {
            self.gpio.set_function_output(g, func, self.drive)?;
        }
        self.running = true;
        self.last_health = Instant::now();
        Ok(())
    }

    fn poll(&mut self) -> Result<SinkPoll, SinkError> {
        let k = self.blocks as u64;
        if !self.running {
            let writable =
                ((k - (self.validated - self.completed)) as usize * self.block_len).saturating_sub(self.fill);
            return Ok(SinkPoll { writable, consumed: 0, written: self.written, underruns: self.underruns });
        }
        if self.last_health.elapsed() > Duration::from_millis(100) {
            self.last_health = Instant::now();
            self.check_health()?;
        }

        // Finished blocks: their LLIs have been written back with VALID
        // cleared.
        let k_blocks = self.blocks as u64;
        while self.completed < self.validated && !self.lli_valid((self.completed % k_blocks) as usize) {
            self.completed += 1;
            self.stalled = false;
            self.last_restart = None;
        }
        let llp = self.ch.llp();
        let sar = self.ch.sar();
        let now = Instant::now();
        if (llp, sar) != self.last_regs {
            self.last_regs = (llp, sar);
            self.last_progress = now;
        }
        let intra = if self.completed < self.validated {
            self.intra_block((self.completed % k_blocks) as usize, sar).min(self.block_len - 1)
        } else {
            0
        };
        // A running engine moves every tick (4 µs at 250 kS/s), and an LLI
        // fetch takes microseconds. Standing still means it has stopped:
        // the ring ran dry (an underrun), or it fetched an LLI that was
        // invalid at that moment; it never re-fetches on its own (two
        // register signatures were observed: SAR one past the previous
        // block, or SAR already loaded from the invalid LLI).
        if now.duration_since(self.last_progress) > Duration::from_millis(2) {
            if !self.stalled {
                self.stalled = true;
                self.underruns += 1;
                // Park the carrier at the centre frequency while starved.
                self.rp1.clocks().write(self.pll.frac_reg_offset(), self.frac_word(0.0));
            }
            self.resume_if_possible()?;
        }
        let consumed = self.completed * self.block_len as u64 + intra as u64;
        let in_flight = self.validated - self.completed;
        let writable = ((k - in_flight) as usize * self.block_len).saturating_sub(self.fill);
        Ok(SinkPoll { writable, consumed, written: self.written, underruns: self.underruns })
    }

    fn write(&mut self, samples: &[f32]) -> Result<(), SinkError> {
        let mut i = 0;
        while i < samples.len() {
            let k = (self.validated % self.blocks as u64) as usize;
            if self.validated - self.completed >= self.blocks as u64 {
                return Err(SinkError::config("write exceeds the writable space"));
            }
            let n = (self.block_len - self.fill).min(samples.len() - i);
            let base = self.block_off[k] + self.fill * 4;
            for (j, &s) in samples[i..i + n].iter().enumerate() {
                self.mem.write_u32(base + j * 4, self.frac_word(s));
            }
            self.fill += n;
            i += n;
            if self.fill == self.block_len {
                self.validate_block(k);
                self.fill = 0;
            }
        }
        self.written += samples.len() as u64;
        if self.running && self.stalled {
            self.resume_if_possible()?;
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
        (self.frac_center, self.frac_per_hz) = self.plan.corrected(ppm);
        Ok(())
    }
}

impl Drop for Rp1FmTransmitter {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every FM carrier that can be modulated (not within about 150 kHz of
    /// 100 MHz, a multiple of the reference) is clear of the PLL's spurs.
    #[test]
    fn fm_carriers_clear_of_the_spurs() {
        for k in 8750..=10800 {
            let f = k as f64 * 10e3;
            match plan_clocks(Rp1Pll::Audio, 0, f, 50e6, 100e3) {
                Ok(p) => assert!(p.clearance_hz >= 12.5e3 && p.spur_warning(100e3).is_none(), "{f}: {p:?}"),
                Err(e) => assert!((f - 100e6).abs() < 150e3, "{f}: {e}"),
            }
        }
        // Division 10 would put the VCO at 29.5 x 50 MHz.
        let p = plan_clocks(Rp1Pll::Audio, 0, 147.5e6, 50e6, 10e3).unwrap();
        assert!(p.n != 10 && p.clearance_hz > 100e3, "{p:?}");
    }

    /// Narrow-band FM channels from short wave to UHF, except within
    /// 300 kHz of a multiple of 50 MHz.
    #[test]
    fn narrow_fm_bands() {
        let bands: [(f64, f64, f64); _] = [
            (26.965e6, 27.405e6, 10e3),     // CB
            (29.51e6, 29.69e6, 10e3),       // 10 m FM
            (51.0e6, 54.0e6, 20e3),         // 6 m FM
            (144.0e6, 148.0e6, 12.5e3),     // 2 m
            (156.0e6, 162.05e6, 25e3),      // marine VHF
            (430.0e6, 440.0e6, 12.5e3),     // 70 cm
            (446.00625e6, 446.2e6, 12.5e3), // PMR446
            (450.0e6, 470.0e6, 12.5e3),     // UHF land mobile
            (868.0e6, 870.0e6, 25e3),       // SRD
        ];
        for (lo, hi, step) in bands {
            let n = ((hi - lo) / step).round() as u32;
            for k in 0..=n {
                let f = lo + k as f64 * step;
                let near_50 = (f / 50e6 - (f / 50e6).round()).abs() * 50e6 < 300e3;
                match plan_clocks(Rp1Pll::Audio, 0, f, 50e6, 10e3) {
                    Ok(p) => {
                        let half_step = 0.5 / p.frac_per_hz;
                        assert!((p.carrier_hz - f).abs() <= half_step && p.max_offset_hz >= 10e3, "{f}: {p:?}");
                    }
                    Err(e) if near_50 => assert!(e.to_string().contains("multiple of 50 MHz"), "{f}: {e}"),
                    Err(e) => panic!("{f}: {e}"),
                }
            }
        }
        assert!(plan_clocks(Rp1Pll::Audio, 0, 1.2e9, 50e6, 10e3).unwrap_err().to_string().contains("--harmonic 3"));
    }

    #[test]
    fn plans_leave_room_to_correct_the_crystal() {
        for (pll, gpclk) in [(Rp1Pll::Audio, 0), (Rp1Pll::Audio, 1), (Rp1Pll::Video, 2)] {
            for mhz in [65.8, 76.0, 87.6, 88.1, 98.5, 107.9] {
                let plan = plan_clocks(pll, gpclk, mhz * 1e6, 50e6, 100e3).unwrap();
                assert!(plan.max_offset_hz >= 100e3, "{mhz} MHz: {plan:?}");
                for ppm in [-MAX_CLOCK_ERROR_PPM, -12.5, 0.0, 30.0, MAX_CLOCK_ERROR_PPM] {
                    let (center, per_hz) = plan.corrected(ppm);
                    for offset in [-100e3, 0.0, 100e3] {
                        let frac = center + offset * per_hz;
                        assert!((0.0..FRAC_ONE).contains(&frac), "{mhz} MHz, {ppm} ppm, {offset} Hz");
                        // With the reference `ppm` off, the output is where
                        // it would be with a perfect reference.
                        let out = 50e6 * (1.0 + ppm * 1e-6) * (plan.fbdiv_int as f64 + frac / FRAC_ONE) / plan.n as f64;
                        assert!((out - plan.carrier_hz - offset).abs() < 1e-3, "{mhz} MHz, {ppm} ppm: {out}");
                    }
                }
            }
        }
    }
}
