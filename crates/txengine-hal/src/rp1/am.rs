//! AM transmitter backend for the Raspberry Pi 5 (RP1).
//!
//! Signal path:
//!
//! ```text
//!  xosc 50 MHz ─► pll_audio ─► /2 (pri_ph) ─► clk_pwm0 ─► PWM0 channel ─► GPIO pad
//!                                                          ▲ one width per carrier cycle
//!  pinned memory ─► AXI DMA channel ◄── PWM0 FIFO request ─┘
//! ```
//!
//! The PWM channel runs in phase-correct mode: it counts up and down over
//! `2n` clocks, one carrier period, and its output is high while the count
//! is below the duty value, so every period carries one pulse of `2·duty`
//! clocks centred on the same instant: the carrier's amplitude follows the
//! duty as `sin(π·duty / n)` (see [`crate::am`]), from nothing at duty 0 to
//! the full square wave at `n / 2` (100 steps at 999 kHz, 16 at 6.1 MHz).
//! The duty values come from the channel's FIFO, one per period, which the
//! DMA engine refills from a ring in memory. The carrier thus paces the
//! samples: each sample is a whole number of carrier cycles, and correcting
//! the crystal error (through the PLL's fractional feedback divider)
//! corrects the sample clock too. The ring and its bookkeeping work as in
//! the FM backend ([`super::fm`]).
//!
//! `clk_pwm0` runs at up to 400 MHz ([`MAX_PWM_CLOCK`]), beyond the 150 MHz
//! of RP1's datasheet: the finer widths lower the quantisation noise
//! (measured on a Pi 5: the counter and the pin follow).

use std::os::fd::RawFd;
use std::time::{Duration, Instant};

use log::{debug, info, warn};
use txengine_core::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll};

use super::Rp1;
use super::clocks::{ClockGen, ClockGenRegs, FcSource, Pll, PllId, PllRegs, measure};
use super::dma::{DmaChannel, Dmac};
use super::gpio::{Drive, Gpio, PinRegs, pwm0_for_pin};
use super::regs::{self, dmac::*, lli, pwm};
use crate::MAX_CLOCK_ERROR_PPM;
use crate::am::{PulseWidths, interpolation_response_db};
use crate::error::{HalError, Result};
use crate::guardian::Guardian;
use crate::mem::pinned::PinnedMemory;
use crate::mem::{DmaMemory, DmaMemoryExt};
use crate::mmio::Mmio;
use crate::sys::{page_size, round_up, wait_until};

/// Highest `clk_pwm0` frequency used (Hz).
pub const MAX_PWM_CLOCK: f64 = 400e6;

/// Lowest carrier (Hz).
pub const MIN_CARRIER: f64 = 100e3;

/// Words per second the DMA engine sustains into the PWM FIFO (measured on
/// a Pi 5, with a margin): channels 0 and 1 have an 8-beat FIFO and move
/// bursts of 8 words (6.25 M words/s measured), the others bursts of 4
/// (3.8 M words/s). One word per carrier cycle: the highest carrier.
const MAX_WORD_RATE_8: f64 = 6.2e6;
const MAX_WORD_RATE_4: f64 = 3.6e6;

/// The kernel runs pll_audio at 1536 MHz; prefer plans near it.
const PREFERRED_VCO: f64 = 1536e6;

/// Configuration of the RP1 AM transmitter.
#[derive(Debug, Clone)]
pub struct Rp1AmConfig {
    /// Carrier frequency in Hz.
    pub carrier_hz: f64,
    /// Output pin: 12, 13, 18 or 19 (PWM0 channels 0-3).
    pub gpio: u32,
    pub drive: Drive,
    /// Desired sample rate; each sample lasts the whole number of carrier
    /// cycles that comes closest.
    pub sample_rate: f64,
    /// Buffer length (transmit latency).
    pub buffer: Duration,
    /// Force a DMA channel (0-7) instead of picking one.
    pub dma_channel: Option<usize>,
}

/// The clock plan: `carrier = VCO / (div1 · div2 · 2 · clk_div · 2n)`.
#[derive(Debug, Clone)]
pub struct AmPlan {
    pub ref_hz: f64,
    pub vco_hz: f64,
    pub fbdiv_int: u32,
    /// Feedback fraction in units of 2^-24.
    pub frac: f64,
    pub div1: u32,
    pub div2: u32,
    /// `clk_pwm0` divider from `pll_audio_pri_ph`.
    pub clk_div: u32,
    pub clk_hz: f64,
    /// Clocks per half period: COMMON_RANGE + 1; the duty runs 0..=n.
    pub n: u32,
    /// Achieved carrier (after rounding of the fraction).
    pub carrier_hz: f64,
    pub cycles_per_sample: u32,
    pub sample_rate: f64,
}

const FRAC_ONE: f64 = (1u64 << regs::clk::PLL_FRAC_BITS) as f64;

impl AmPlan {
    /// The feedback fraction when the crystal runs `ppm` fast: the whole
    /// feedback divider shrinks by `1 + ppm · 10⁻⁶`.
    pub fn corrected_frac(&self, ppm: f64) -> f64 {
        let int = self.fbdiv_int as f64 * FRAC_ONE;
        (int + self.frac) / (1.0 + ppm * 1e-6) - int
    }
}

/// Plan the clocks for `carrier_hz` from a reference of nominally `ref_hz`:
/// the fastest PWM clock up to [`MAX_PWM_CLOCK`] whose period is a whole
/// number `2n` of its cycles, from a PLL frequency that leaves room for
/// correcting the crystal.
pub fn plan(carrier_hz: f64, ref_hz: f64, sample_rate: f64) -> Result<AmPlan> {
    if !(MIN_CARRIER..=MAX_WORD_RATE_8).contains(&carrier_hz) {
        return Err(HalError::Config(format!(
            "the Raspberry Pi 5 makes AM carriers of {:.0} kHz to {:.1} MHz",
            MIN_CARRIER / 1e3,
            MAX_WORD_RATE_8 / 1e6
        )));
    }
    let n_max = (MAX_PWM_CLOCK / (2.0 * carrier_hz)).floor() as u32;
    for n in (n_max.saturating_sub(64).max(8)..=n_max).rev() {
        let clk = 2.0 * n as f64 * carrier_hz;
        let mut best: Option<(f64, AmPlan)> = None;
        for (vmin, vmax) in [(1000e6, 1600e6), (800e6, 2400e6)] {
            for clk_div in 1..=64u32 {
                for div1 in 1..=7u32 {
                    for div2 in 1..=div1 {
                        let vco = clk * (2 * div1 * div2 * clk_div) as f64;
                        if !(vmin..=vmax).contains(&vco) {
                            continue;
                        }
                        let fb = vco / ref_hz;
                        let int = fb.floor();
                        let frac = fb - int;
                        // Room for the crystal correction.
                        let reserve = fb * MAX_CLOCK_ERROR_PPM * 1e-6 + 1e-4;
                        if !(16.0..=320.0).contains(&int) || frac < reserve || frac > 1.0 - reserve {
                            continue;
                        }
                        let score = ((vco - PREFERRED_VCO) / PREFERRED_VCO).abs();
                        if best.as_ref().is_none_or(|(s, _)| score < *s) {
                            let frac = (frac * FRAC_ONE).round();
                            let vco = (int + frac / FRAC_ONE) * ref_hz;
                            let carrier = vco / (2 * div1 * div2 * clk_div) as f64 / (2 * n) as f64;
                            let cycles = (carrier / sample_rate).round().max(1.0) as u32;
                            best = Some((
                                score,
                                AmPlan {
                                    ref_hz,
                                    vco_hz: vco,
                                    fbdiv_int: int as u32,
                                    frac,
                                    div1,
                                    div2,
                                    clk_div,
                                    clk_hz: carrier * (2 * n) as f64,
                                    n,
                                    carrier_hz: carrier,
                                    cycles_per_sample: cycles,
                                    sample_rate: carrier / cycles as f64,
                                },
                            ));
                        }
                    }
                }
            }
            if let Some((_, p)) = best {
                return Ok(p);
            }
        }
    }
    Err(HalError::Config(format!("no clock plan for an AM carrier of {:.3} kHz", carrier_hz / 1e3)))
}

/// Hardware state saved before we touched it.
#[derive(Debug, Clone)]
struct Saved {
    pll: PllRegs,
    clk: ClockGenRegs,
    pwm: Vec<u32>,
    pin: PinRegs,
}

/// Everything needed to put the hardware back (also used by the guardian).
#[derive(Debug, Clone)]
struct Restorer {
    pll: Pll,
    clk: ClockGen,
    pwm: Mmio,
    gpio: Gpio,
    ch: DmaChannel,
    saved: Saved,
    aspm: super::link::AspmGuard,
}

impl Restorer {
    /// Silence the pin, stop the DMA and restore every register.
    fn restore(&self) {
        self.gpio.restore(&self.saved.pin);
        self.pwm.write(pwm::GLOBAL_CTRL, pwm::GLOBAL_SET_UPDATE);
        if let Err(e) = self.ch.stop() {
            warn!("RP1 DMA channel {} did not stop cleanly: {e}", self.ch.ch);
        }
        self.pwm.write(pwm::FIFO_CTRL, pwm::FIFO_FLUSH);
        std::thread::sleep(Duration::from_millis(1));
        for (i, &v) in self.saved.pwm.iter().enumerate() {
            let off = 4 * i;
            if ![pwm::GLOBAL_CTRL, pwm::FIFO_CTRL, pwm::DUTY_FIFO].contains(&off) {
                self.pwm.write(off, v);
            }
        }
        self.pwm.write(pwm::FIFO_CTRL, self.saved.pwm[pwm::FIFO_CTRL / 4] & !pwm::FIFO_FLUSH);
        self.pwm.write(pwm::GLOBAL_CTRL, self.saved.pwm[pwm::GLOBAL_CTRL / 4] | pwm::GLOBAL_SET_UPDATE);
        self.clk.restore(&self.saved.clk);
        if let Err(e) = self.pll.restore(&self.saved.pll) {
            warn!("{}: {e}", self.pll.id.name());
        }
        self.aspm.restore();
    }
}

/// The RP1 AM transmitter.
pub struct Rp1AmTransmitter {
    info: SinkInfo,
    plan: AmPlan,
    rp1: Rp1,
    dmac: Dmac,
    ch: DmaChannel,
    clk: ClockGen,
    pll: Pll,
    pwm: Mmio,
    gpio: Gpio,
    /// (GPIO, PWM channel, function select).
    pin: (u32, usize, u32),
    drive: Drive,
    restorer: Restorer,
    mem: PinnedMemory,
    guardian: Option<Guardian>,
    widths: PulseWidths,
    /// The widths of one sample's carrier cycles.
    sample_words: Vec<u32>,

    // Ring geometry (one word per carrier cycle).
    blocks: usize,
    block_len: usize,
    lli_off: Vec<usize>,
    lli_addr: Vec<u64>,
    block_off: Vec<usize>,
    block_addr: Vec<u64>,
    ctl: u64,
    cfg: u64,

    // Producer / consumer bookkeeping (monotonic block counters; `fill`
    // words of the next block are written).
    validated: u64,
    completed: u64,
    fill: usize,
    written: u64,
    underruns: u64,
    stalled: bool,
    last_regs: (u64, u64),
    last_progress: Instant,
    last_restart: Option<Instant>,
    running: bool,
    stopped: bool,
    last_health: Instant,
}

impl std::fmt::Debug for Rp1AmTransmitter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.info.description)
    }
}

impl Rp1AmTransmitter {
    pub fn new(cfg: &Rp1AmConfig, xosc_hz: f64) -> Result<Rp1AmTransmitter> {
        let (chan, funcsel) = pwm0_for_pin(cfg.gpio).ok_or_else(|| {
            HalError::Config(format!("GPIO{} has no PWM output on the Raspberry Pi 5 (use 12, 13, 18 or 19)", cfg.gpio))
        })?;
        let rp1 = Rp1::open()?;
        let clocks = rp1.clocks();
        let clocks_set = rp1.block_set(regs::CLOCKS_MAIN);
        let clocks_clr = rp1.block_clr(regs::CLOCKS_MAIN);
        let pll = Pll::new(&clocks, PllId::Audio, xosc_hz);
        if pll.refdiv() != 1 {
            return Err(HalError::Hardware(format!("pll_audio's reference divider is {}, not 1", pll.refdiv())));
        }
        let plan = plan(cfg.carrier_hz, pll.ref_hz(), cfg.sample_rate)?;
        let clk = ClockGen::pwm0(&clocks, &clocks_set, &clocks_clr);
        let pwm = rp1.block(regs::PWM0);
        let gpio = Gpio::new(rp1.block(regs::IO_BANK0), rp1.block(regs::PADS_BANK0));
        if pwm.read(pwm::GLOBAL_CTRL) & 0xf != 0 {
            return Err(HalError::Hardware("PWM0 is in use (a channel is enabled)".into()));
        }
        let dmac = Dmac::new(rp1.block(regs::DMA));
        // Channel 1 has the 8-beat FIFO the fast carriers need.
        let ch = match cfg.dma_channel {
            Some(c) => dmac.claim(Some(c))?,
            None => dmac.claim(Some(1)).or_else(|_| dmac.claim(None))?,
        };
        let wide = ch.ch <= 1;
        let max_rate = if wide { MAX_WORD_RATE_8 } else { MAX_WORD_RATE_4 };
        if plan.carrier_hz > max_rate {
            return Err(HalError::Config(format!(
                "DMA channel {} moves at most {:.1} M words/s: carriers up to {:.1} MHz (channels 0 and 1: {:.1} MHz)",
                ch.ch,
                max_rate / 1e6,
                max_rate / 1e6,
                MAX_WORD_RATE_8 / 1e6
            )));
        }

        // Ring geometry: blocks within a page (whole pages, or halves,
        // quarters, ... of one), small enough that the buffer is made of at
        // least 16 of them and comes close to the length asked for.
        let page = page_size();
        let want = cfg.buffer.as_secs_f64() * plan.carrier_hz;
        let mut block_len = page / 4;
        while block_len > 256 && want / (block_len as f64) < 16.0 {
            block_len /= 2;
        }
        let blocks = ((want / block_len as f64).ceil() as usize).clamp(8, 4096);
        let lli_bytes = round_up(blocks * lli::SIZE, page);
        let mem = PinnedMemory::alloc(lli_bytes + blocks * block_len * 4, regs::HOST_MEM_DMA_OFFSET)?;
        let lli_off: Vec<usize> = (0..blocks).map(|k| k * lli::SIZE).collect();
        let block_off: Vec<usize> = (0..blocks).map(|k| lli_bytes + k * block_len * 4).collect();
        let lli_addr: Vec<u64> = lli_off.iter().map(|&o| mem.dma_addr_at(o)).collect();
        let block_addr: Vec<u64> = block_off.iter().map(|&o| mem.dma_addr_at(o)).collect();

        let saved = Saved {
            pll: pll.snapshot(),
            clk: clk.snapshot(),
            pwm: (0..pwm::END).step_by(4).map(|r| pwm.read(r)).collect(),
            pin: gpio.snapshot(cfg.gpio)?,
        };
        if saved.pll.fbdiv_int != 0 && saved.pll.pwr & regs::clk::PLL_PWR_PD == 0 {
            debug!("pll_audio was at {:.3} MHz; its users (I2S audio) will be affected", pll.vco_hz() / 1e6);
        }
        // Bursts of 8 (or 4) words, requested while the FIFO has room for one.
        let (msize, threshold) = if wide { (MSIZE_8, pwm::FIFO_LEN - 8) } else { (MSIZE_4, pwm::FIFO_LEN - 4) };
        let ctl = (WIDTH_32 << CTL_SRC_WIDTH_SHIFT)
            | (WIDTH_32 << CTL_DST_WIDTH_SHIFT)
            | (msize << CTL_SRC_MSIZE_SHIFT)
            | (msize << CTL_DST_MSIZE_SHIFT)
            | CTL_DINC_FIXED
            | CTL_ARLEN_EN
            | (7u64 << CTL_ARLEN_SHIFT);
        let ch_cfg = (MULTBLK_LL << CFG_SRC_MULTBLK_SHIFT)
            | (MULTBLK_LL << CFG_DST_MULTBLK_SHIFT)
            | ((regs::dreq::PWM0 as u64) << CFG_DST_PER_SHIFT)
            | (TT_FC_MEM_TO_PER_DMAC << CFG_TT_FC_SHIFT)
            | (7u64 << CFG_PRIORITY_SHIFT);

        let restorer = Restorer {
            pll: pll.clone(),
            clk: clk.clone(),
            pwm: pwm.clone(),
            gpio: gpio.clone(),
            ch: ch.clone(),
            saved,
            aspm: super::link::AspmGuard::disable(&rp1.pci_path),
        };

        let fs = plan.sample_rate;
        let description = format!(
            "RP1 pll_audio -> clk_pwm0 -> PWM0 channel {chan} on GPIO{}: carrier {:.4} kHz (VCO {:.3} MHz \
             [/{}/2/{}], PWM clock {:.3} MHz, {} width steps to full output), {:.3} kS/s ({} cycles per sample), \
             DMA ch{}, {} x {} cycles ({:.1} ms), drive {} mA",
            cfg.gpio,
            plan.carrier_hz / 1e3,
            plan.vco_hz / 1e6,
            plan.div1 * plan.div2,
            plan.clk_div,
            plan.clk_hz / 1e6,
            plan.n / 2,
            fs / 1e3,
            plan.cycles_per_sample,
            ch.ch,
            blocks,
            block_len,
            (blocks * block_len) as f64 / plan.carrier_hz * 1e3,
            cfg.drive.milliamps(),
        );
        let info = SinkInfo {
            description,
            sample_rate: fs,
            carrier_hz: plan.carrier_hz,
            max_offset_hz: 0.0,
            resolution_hz: 0.0,
            max_clock_error_ppm: MAX_CLOCK_ERROR_PPM,
            paced_by_carrier: true,
            buffer_len: (blocks - 1) * block_len / plan.cycles_per_sample as usize,
            format: SampleFormat::Amplitude,
            realtime: true,
            zero_order_hold: false,
            response_db: interpolation_response_db(fs, fs / 4.0),
        };

        let cycles = plan.cycles_per_sample as usize;
        let mut tx = Rp1AmTransmitter {
            info,
            widths: PulseWidths::new(cycles, plan.n as f64, plan.n),
            sample_words: vec![0; cycles],
            plan,
            rp1,
            dmac,
            ch,
            clk,
            pll,
            pwm,
            gpio,
            pin: (cfg.gpio, chan, funcsel),
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
        if let Err(e) = tx.setup(threshold) {
            tx.shutdown();
            return Err(e);
        }
        info!("{}", tx.info.description);
        Ok(tx)
    }

    pub fn plan(&self) -> &AmPlan {
        &self.plan
    }

    fn init_ring(&mut self) {
        let dar = Rp1::dma_addr_of(regs::PWM0 + pwm::DUTY_FIFO);
        for k in 0..self.blocks {
            let l = self.lli_off[k];
            self.mem.zero(l, lli::SIZE);
            self.mem.write_u64(l + lli::SAR, self.block_addr[k]);
            self.mem.write_u64(l + lli::DAR, dar);
            self.mem.write_u32(l + lli::BLOCK_TS, self.block_len as u32 - 1);
            self.mem.write_u64(l + lli::LLP, self.lli_addr[(k + 1) % self.blocks]);
            // Not valid until filled.
            self.mem.write_u64(l + lli::CTL, self.ctl);
        }
        self.mem.flush(0, self.mem.len());
    }

    fn spawn_guardian(&mut self) -> Result<()> {
        let restorer = self.restorer.clone();
        let keep: Vec<RawFd> = self.mem.pin_fd().into_iter().collect();
        self.guardian = Some(Guardian::spawn(&keep, move || restorer.restore())?);
        Ok(())
    }

    /// Clocks and the PWM channel, which waits (disabled) for its FIFO.
    fn setup(&mut self, threshold: u32) -> Result<()> {
        let p = &self.plan;
        self.pll.configure(p.fbdiv_int, p.frac as u32, p.div1, p.div2)?;
        // Enable the primary phase output (pll_audio_pri_ph) at phase 0.
        self.rp1.clocks().modify(regs::clk::PLL_AUDIO + regs::clk::PLL_PRIM, 0x1f, 1 << 4);
        // clk_pwm0's source 0: pll_audio_pri_ph.
        self.clk.configure(0, p.clk_div, 0)?;
        let fc_ref = self.pll.ref_hz() * self.pll.refdiv() as f64;
        let hz = measure(&self.rp1.clocks(), FcSource::PWM0, fc_ref)?;
        debug!("clk_pwm0 measured {:.4} MHz (planned {:.4} MHz)", hz / 1e6, p.clk_hz / 1e6);
        if (hz - p.clk_hz).abs() > 1000.0 + p.clk_hz * 1e-4 {
            return Err(HalError::Hardware(format!(
                "clk_pwm0 runs at {:.4} MHz instead of {:.4} MHz",
                hz / 1e6,
                p.clk_hz / 1e6
            )));
        }
        let chan = self.pin.1;
        self.pwm.write(pwm::GLOBAL_CTRL, pwm::GLOBAL_SET_UPDATE);
        self.pwm.write(pwm::COMMON_RANGE, p.n - 1);
        self.pwm.write(
            pwm::chan_ctrl(chan),
            pwm::CHAN_FIFO_POP_MASK | pwm::CHAN_USEFIFO | pwm::CHAN_BIND | pwm::CHAN_MODE_PHASE_CORRECT,
        );
        self.pwm.write(pwm::GLOBAL_CTRL, pwm::GLOBAL_SET_UPDATE);
        self.pwm.write(pwm::FIFO_CTRL, pwm::FIFO_FLUSH);
        std::thread::sleep(Duration::from_millis(1));
        // The dwell time is the measured prototype's (sustained 6.1 M words/s).
        self.pwm.write(
            pwm::FIFO_CTRL,
            pwm::FIFO_DREQ_EN | (threshold << pwm::FIFO_THRESHOLD_SHIFT) | (2 << pwm::FIFO_DWELL_SHIFT),
        );
        Ok(())
    }

    /// Map a channel LLP value to an LLI index.
    fn lli_index(&self, llp: u64) -> Option<usize> {
        let a = llp & !0x3f;
        self.lli_addr.iter().position(|&x| x == a)
    }

    /// Position inside block `k` from the source address register.
    fn intra_block(&self, k: usize, sar: u64) -> usize {
        match sar.checked_sub(self.block_addr[k]) {
            Some(d) if d <= (self.block_len * 4) as u64 => d as usize / 4,
            _ => 0,
        }
    }

    /// Append the widths of one sample to the ring, and hand each block to
    /// the hardware once it is full.
    fn push_sample(&mut self) {
        let mut i = 0;
        while i < self.sample_words.len() {
            let k = (self.validated % self.blocks as u64) as usize;
            let n = (self.sample_words.len() - i).min(self.block_len - self.fill);
            self.mem.write_words(self.block_off[k] + self.fill * 4, &self.sample_words[i..i + n]);
            self.fill += n;
            i += n;
            if self.fill == self.block_len {
                self.validate_block(k);
            }
        }
    }

    fn validate_block(&mut self, k: usize) {
        self.mem.flush(self.block_off[k], self.block_len * 4);
        let l = self.lli_off[k];
        self.mem.write_u64(l + lli::CTL, self.ctl | CTL_LLI_VALID);
        self.mem.flush(l, lli::SIZE);
        self.validated += 1;
        self.fill = 0;
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

    /// Whether the LLI of ring block `k` is still marked valid in memory
    /// (the engine clears VALID when it writes the LLI back after the block).
    fn lli_valid(&self, k: usize) -> bool {
        let l = self.lli_off[k];
        self.mem.invalidate(l + lli::CTL, 8);
        self.mem.read_u64(l + lli::CTL) & CTL_LLI_VALID != 0
    }

    fn debug_state(&self) -> String {
        let llp = self.ch.llp();
        let next = self.lli_index(llp).map_or_else(|| format!("{llp:#x}"), |i| i.to_string());
        format!(
            "LLP -> LLI {next}, SAR {:#x}, STATUS {:#x}, INTSTATUS {:#x}, enabled {}, FIFO level {}, completed {} \
             validated {} fill {}",
            self.ch.sar(),
            self.ch.status(),
            self.ch.intstatus(),
            self.ch.is_enabled(),
            self.pwm.read(pwm::FIFO_CTRL) & pwm::FIFO_LEVEL_MASK,
            self.completed,
            self.validated,
            self.fill,
        )
    }

    /// Re-arm a stopped engine at the first unplayed block once there is
    /// one (rate limited, as in the FM backend).
    fn resume_if_possible(&mut self) -> Result<()> {
        let recent = self.last_restart.is_some_and(|t| t.elapsed() < Duration::from_millis(20));
        if self.validated > self.completed && !recent {
            debug!("RP1 DMA: re-arming at block {} ({})", self.completed, self.debug_state());
            self.restart_channel_at(self.completed)?;
        }
        Ok(())
    }

    /// Detect and repair interference from other users of the controller.
    fn check_health(&mut self) -> Result<()> {
        let clocks = self.rp1.clocks();
        let set = self.rp1.block_set(regs::CLOCKS_MAIN);
        if self.dmac.ensure_running(&clocks, &set) {
            warn!("RP1 DMA controller had been suspended by another driver; re-enabled");
        }
        if !self.clk.is_enabled() {
            return Err(HalError::Hardware("clk_pwm0 was disabled externally".into()));
        }
        if self.pwm.read(pwm::GLOBAL_CTRL) & pwm::global_chan_en(self.pin.1) == 0 {
            return Err(HalError::Hardware(format!("PWM0 channel {} was disabled externally", self.pin.1)));
        }
        if !self.ch.is_enabled() && !self.stalled {
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

    /// Free space in carrier cycles.
    fn free_cycles(&self) -> usize {
        let in_flight = (self.validated - self.completed) as usize;
        ((self.blocks - in_flight) * self.block_len).saturating_sub(self.fill)
    }
}

impl RfSink for Rp1AmTransmitter {
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
        self.restart_channel_at(0)?;
        // Start the PWM once the DMA engine has filled its FIFO.
        wait_until(Duration::from_millis(20), "PWM0 FIFO fill", || {
            self.pwm.read(pwm::FIFO_CTRL) & pwm::FIFO_LEVEL_MASK >= pwm::FIFO_LEN - 4
        })?;
        let (gpio, chan, funcsel) = self.pin;
        self.pwm.write(pwm::GLOBAL_CTRL, pwm::GLOBAL_SET_UPDATE | pwm::global_chan_en(chan));
        // Carrier on: route the channel to the pin.
        self.gpio.set_function_output(gpio, funcsel, self.drive)?;
        self.running = true;
        self.last_health = Instant::now();
        Ok(())
    }

    fn poll(&mut self) -> Result<SinkPoll, SinkError> {
        let cycles = self.plan.cycles_per_sample as u64;
        if !self.running {
            let writable = self.free_cycles() / cycles as usize;
            return Ok(SinkPoll { writable, consumed: 0, written: self.written, underruns: self.underruns });
        }
        if self.last_health.elapsed() > Duration::from_millis(100) {
            self.last_health = Instant::now();
            self.check_health()?;
        }
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
        // A running engine moves every few microseconds; standing still for
        // 2 ms means it ran dry (an underrun) or fetched an LLI that was not
        // valid yet. The PWM repeats the last width meanwhile.
        if now.duration_since(self.last_progress) > Duration::from_millis(2) {
            if !self.stalled {
                self.stalled = true;
                self.underruns += 1;
            }
            self.resume_if_possible()?;
        }
        let consumed = (self.completed * self.block_len as u64 + intra as u64) / cycles;
        let writable = self.free_cycles() / cycles as usize;
        Ok(SinkPoll { writable, consumed, written: self.written, underruns: self.underruns })
    }

    fn write(&mut self, samples: &[f32]) -> Result<(), SinkError> {
        if samples.len() * self.widths.cycles() > self.free_cycles() {
            return Err(SinkError::config("write exceeds the writable space"));
        }
        for &s in samples {
            self.widths.sample(s, &mut self.sample_words);
            self.push_sample();
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
        let frac = self.plan.corrected_frac(ppm).round().clamp(0.0, FRAC_ONE - 1.0) as u32;
        self.rp1.clocks().write(self.pll.frac_reg_offset(), frac);
        Ok(())
    }
}

impl Drop for Rp1AmTransmitter {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plans_for_the_broadcast_bands() {
        for khz in [153.0, 198.0, 279.0, 531.0, 999.0, 1_602.0, 1_710.0, 2_300.0, 3_950.0, 5_960.0, 6_100.0] {
            let p = plan(khz * 1e3, 50e6, 100e3).unwrap();
            assert!((p.carrier_hz - khz * 1e3).abs() < 0.01, "{khz} kHz: {p:?}");
            assert!(p.clk_hz <= MAX_PWM_CLOCK && p.clk_hz > MAX_PWM_CLOCK - 2.0 * 64.0 * p.carrier_hz, "{p:?}");
            assert!((p.vco_hz / (p.clk_hz * (2 * p.div1 * p.div2 * p.clk_div) as f64) - 1.0).abs() < 1e-12);
            assert!((p.sample_rate - 100e3).abs() <= p.sample_rate / p.cycles_per_sample as f64, "{p:?}");
            // The crystal can be corrected either way within the fraction.
            for ppm in [-MAX_CLOCK_ERROR_PPM, MAX_CLOCK_ERROR_PPM] {
                let f = p.corrected_frac(ppm);
                assert!((0.0..FRAC_ONE).contains(&f), "{khz} kHz, {ppm} ppm: {f}");
                let carrier = 50e6 * (1.0 + ppm * 1e-6) * (p.fbdiv_int as f64 + f / FRAC_ONE) * p.carrier_hz / p.vco_hz;
                assert!((carrier - p.carrier_hz).abs() < 1e-3, "{khz} kHz, {ppm} ppm: {carrier}");
            }
        }
        assert!(plan(50e3, 50e6, 100e3).is_err());
        assert!(plan(7.1e6, 50e6, 100e3).is_err());
    }
}
