//! FM transmitter backend for BCM2835/6/7 and BCM2711 (Pi 2, 3, 4, Zero 2 W).
//!
//! The carrier is a general purpose clock (GPCLK) routed to a GPIO. It is
//! frequency modulated by one of two methods:
//!
//! * **PLL** (`a`, `c`, `d`): the GPCLK divides the PER channel of a PLL by
//!   an integer; the DMA engine writes the PLL's 20-bit fractional feedback
//!   divider every sample. Clean spectrum; but every other user of that PLL
//!   sees the (tiny) frequency changes, so the planner keeps a shared PLL's
//!   VCO within a few percent of where the firmware put it.
//! * **Clock divider** (`clkdiv`): the GPCLK's own 12.12 fractional divider
//!   (MASH noise shaping) is modulated. No other clock is touched, at the cost
//!   of a noisier spectrum and coarse frequency steps (dithered).
//!
//! Pacing: each sample is two DMA control blocks – write the sample to the
//! clock register, then write a dummy word to the PWM (or PCM) FIFO, which
//! the peripheral drains at exactly the sample rate (its clock is a division
//! of PLLD or the crystal: an integer one without jitter, or a fractional one
//! whose jitter is at most 2 ns, see [`plan_pacer`]).
//!
//! The control blocks form a ring whose last written sample links to an idle
//! loop that holds the carrier at the centre frequency. If the producer falls
//! behind (or dies), the DMA engine parks there instead of replaying stale
//! samples; new data is linked in front of the idle loop.

use std::time::{Duration, Instant};

use log::{debug, info, warn};
use txengine_core::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll};

use super::cprman::{Cprman, Pll, PllConfig, PllId, PllState, PllUsers};
use super::dma::{ControlBlock, DmaChannel};
use super::regs::{self, cm, dma as dmar, pads, pcm, pwm};
use super::{Bcm, gpclk_for_pin};
use crate::MAX_CLOCK_ERROR_PPM;
use crate::cpufreq::CpuClockPin;
use crate::error::{HalError, Result};
use crate::guardian::Guardian;
use crate::mem::mailbox::VcMemory;
use crate::mem::{DmaMemory, DmaMemoryExt};
use crate::mmio::Mmio;
use crate::platform::Soc;

/// How the carrier is frequency modulated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BcmMethod {
    Pll(PllId),
    ClkDiv,
}

/// Peripheral whose FIFO paces the DMA engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pacer {
    Pwm,
    Pcm,
}

#[derive(Debug, Clone)]
pub struct BcmFmConfig {
    pub carrier_hz: f64,
    pub gpios: Vec<u32>,
    /// `None`: choose automatically (an idle PLL, else PLLC).
    pub method: Option<BcmMethod>,
    pub pacer: Pacer,
    /// Pad drive strength 0..=7 (2..16 mA).
    pub drive: u8,
    /// Output sample rate; `None`: the highest rate the DMA engine
    /// sustains on this board (measured), at most 250 kS/s.
    pub sample_rate: Option<f64>,
    pub buffer: Duration,
    pub max_offset_hz: f64,
    pub dma_channel: Option<usize>,
    /// Largest relative VCO change accepted for a PLL with other users
    /// (below, see [`GPU_PLL_MAX_DECREASE`] for one that only clocks the
    /// GPU). PLLC clocks the VPU, from which the SD card host, SPI, I2C
    /// and the mini UART (the console on boards with WiFi) derive their
    /// clocks, and EMMC (the WiFi's SDIO host); PLLD the PL011 UART and
    /// HDMI. The UARTs tolerate a baud rate error of about ±3 %, and the
    /// SD and SDIO buses already run at their 50 MHz maximum.
    pub max_vco_change: f64,
}

impl Default for BcmFmConfig {
    fn default() -> Self {
        BcmFmConfig {
            carrier_hz: 87.6e6,
            gpios: vec![4],
            method: None,
            pacer: Pacer::Pwm,
            drive: 7,
            sample_rate: None,
            buffer: Duration::from_millis(60),
            max_offset_hz: 100_000.0,
            dma_channel: None,
            max_vco_change: 0.03,
        }
    }
}

/// Frequency plan.
#[derive(Debug, Clone)]
pub struct BcmPlan {
    pub method: BcmMethod,
    pub gpclk: usize,
    pub gpclk_src: u32,
    /// PLL method: power up an idle PLL / change its PER divider.
    pub power_up: bool,
    pub set_per: bool,
    pub ndiv: u32,
    /// Carrier centre in units of the DMA register (2^-20 NDIV, or 2^-12 GPCLK divider).
    pub center: f64,
    /// Register units per Hz of carrier offset (negative for the divider method).
    pub units_per_hz: f64,
    pub per_div: u32,
    pub gp_div: u32,
    pub vco_old: f64,
    pub vco_new: f64,
    pub carrier_hz: f64,
    /// Largest offset the register can represent under any crystal error
    /// correction up to ±[`MAX_CLOCK_ERROR_PPM`].
    pub max_offset_hz: f64,
    /// Who else uses the PLL.
    pub users: PllUsers,
    /// Hz per register unit at the carrier.
    pub step_hz: f64,
    /// Loop filter integral gain to set on the PLL (see [`modulation_ki`]);
    /// `None` keeps the PLL's current one.
    pub loop_ki: Option<u32>,
}

impl BcmPlan {
    /// PLL method: FRAC at the carrier centre and FRAC units per Hz of
    /// offset when the crystal runs `ppm` fast. The whole feedback divider
    /// (NDIV.FRAC) shrinks by `1 + ppm · 10⁻⁶`, so that the VCO keeps its
    /// nominal frequency.
    pub fn corrected_frac(&self, ppm: f64) -> (f64, f64) {
        let k = 1.0 / (1.0 + ppm * 1e-6);
        (self.center * k - self.ndiv as f64 * (1u64 << cm::A2W_FRAC_BITS) as f64, self.units_per_hz * k)
    }

    /// Clock-divider method: the frequency of the divided clock (PLLD_PER)
    /// in units of 2^-12 Hz when the crystal runs `ppm` fast.
    pub fn corrected_divider_source(&self, ppm: f64) -> f64 {
        self.carrier_hz * self.center * (1.0 + ppm * 1e-6)
    }
}

/// Loop filter integral gain (ANA1 KI) txengine sets on PLLA and PLLC of
/// the BCM2836/7 while modulating them, instead of the firmware's KI 2.
///
/// The gain sets the loop bandwidth: about 125 kHz at KI 2, 600 kHz at KI 5
/// (measured from the modulation response, each step ×1.5–1.9). At KI 2
/// the loop leaves the VCO's phase noise around 10–30 kHz offset, where it
/// lands in the audio and stereo bands, poorly suppressed: on a Pi 2 the
/// noise at 10 kHz drops from −91 to −101 dBc/Hz at KI 5, the stereo SNR
/// rises by 11 dB, and the modulation response becomes flat (see the
/// tables). The price is the noise peak of the wider loop, which moves out
/// to 300–600 kHz offset: +1 dB in the 200 kHz channels at ±400 kHz. KI 6
/// gains 1 dB more SNR but adds 3–5 dB there; KI 7 also raises the noise
/// close to the carrier. On the Zero 2 W, whose noise has another source
/// (its supply), the gain changes neither the noise nor anything else.
/// PLLA and PLLC are the same design: at the same VCO frequency their
/// responses agree within 0.01 dB at KI 2 and at KI 5, on both boards.
pub const MODULATION_KI: u32 = 5;

/// The loop filter integral gain for modulating `pll`, if it differs from
/// the PLL's current setting: PLLA and PLLC of the BCM2836/7. (PLLD, which
/// clocks the UART and HDMI, and the BCM2711 have not been measured.)
pub fn modulation_ki(pll: &Pll) -> Option<u32> {
    (matches!(pll.id, PllId::A | PllId::C) && !pll.is_bcm2711()).then_some(MODULATION_KI)
}

/// How far down the VCO of a PLL whose only users are the GPU's video and
/// 3D blocks (normally PLLA) may be moved on the BCM2836/7: those blocks
/// then run up to this much slower, and nothing else depends on them. From
/// PLLA's 2250 MHz this reaches every carrier in 87.5–108 MHz (at most
/// −12 % is needed; ±3 % reaches 76 % of the band). The loop's response
/// and noise measure the same from 1927 to 2365 MHz (−14 % to +5 %).
/// Upwards, which would overclock the blocks, the general limit applies.
pub const GPU_PLL_MAX_DECREASE: f64 = 0.15;

/// Plan PLL modulation for `pll`, from a crystal of nominally `osc` Hz.
pub fn plan_pll(
    pll: &Pll,
    gpclk: usize,
    carrier: f64,
    osc: f64,
    max_offset: f64,
    max_change: f64,
) -> Result<BcmPlan> {
    let gp_ctl = cm::gp(gpclk);
    let running = pll.is_powered() && pll.is_locked();
    let users = if running { pll.users(gp_ctl) } else { PllUsers::default() };
    let shared = !users.is_empty();
    let max_decrease =
        if users.gpu_only && !pll.is_bcm2711() { max_change.max(GPU_PLL_MAX_DECREASE) } else { max_change };
    let fb = if running { pll.fb_mult() } else { 1.0 };
    let unit_hz = osc * fb / (1u64 << cm::A2W_FRAC_BITS) as f64;
    let vco_old = if running { pll.vco_hz() } else { 0.0 };
    let cur_per = pll.per_divider();
    // A PER channel used by another clock must keep its divider.
    let per_choices: Vec<u32> = match (cur_per, users.per.is_empty()) {
        (Some(d), false) => vec![d],
        _ => (1..=8).collect(),
    };
    // Room for correcting the crystal error at run time.
    let reserve_hz = carrier * MAX_CLOCK_ERROR_PPM * 1e-6;
    let mut best: Option<(f64, BcmPlan)> = None;
    for &per in &per_choices {
        for gp in 1..=4095u32 {
            let n = per * gp;
            let vco = carrier * n as f64;
            if vco > 2.4e9 {
                break;
            }
            // VCO limits; BCM2835/6/7 switch the feedback pre-divider above
            // 1.75 GHz, which we never change on a running PLL.
            let (lo, hi) = if running && shared {
                (vco_old * (1.0 - max_decrease), vco_old * (1.0 + max_change))
            } else if running {
                (600e6, if fb > 1.0 { 2.4e9 } else { 1.75e9 })
            } else {
                (1.0e9, 1.75e9)
            };
            if vco < lo || vco > hi || vco < 600e6 {
                continue;
            }
            // The firmware never runs a PER channel faster than ~1 GHz
            // (PLLC_PER feeds EMMC at 1 GHz); stay in that region.
            if vco / per as f64 > 1.25e9 {
                continue;
            }
            let units = vco / unit_hz;
            let ndiv = (units / (1u64 << cm::A2W_FRAC_BITS) as f64).floor();
            let frac = units / (1u64 << cm::A2W_FRAC_BITS) as f64 - ndiv;
            let dev = (max_offset + reserve_hz) * n as f64 / unit_hz / (1u64 << cm::A2W_FRAC_BITS) as f64;
            let margin = (frac - dev).min(1.0 - frac - dev);
            if margin < 0.005 || !(1.0..=1023.0).contains(&ndiv) {
                continue;
            }
            let change = if running { (vco - vco_old).abs() / vco_old } else { 0.0 };
            // Prefer: least disturbance, then no GPCLK division beyond what
            // is needed, then a high VCO (lower relative jitter).
            let score = change * 100.0 + if running { 0.0 } else { (vco - 1.5e9).abs() / 1e9 } - margin * 0.01;
            if best.as_ref().is_none_or(|(s, _)| score < *s) {
                let center = units.round();
                let units_per_hz = n as f64 / unit_hz;
                best = Some((
                    score,
                    BcmPlan {
                        method: BcmMethod::Pll(pll.id),
                        gpclk,
                        gpclk_src: pll.id.gpclk_source(),
                        power_up: !running,
                        set_per: cur_per != Some(per) || !running,
                        ndiv: ndiv as u32,
                        center,
                        units_per_hz,
                        per_div: per,
                        gp_div: gp,
                        vco_old,
                        vco_new: center * unit_hz,
                        carrier_hz: center * unit_hz / n as f64,
                        max_offset_hz: ((frac.min(1.0 - frac)) * (1u64 << cm::A2W_FRAC_BITS) as f64) / units_per_hz
                            - reserve_hz,
                        users: users.clone(),
                        step_hz: 1.0 / units_per_hz,
                        loop_ki: modulation_ki(pll),
                    },
                ));
            }
        }
    }
    best.map(|b| b.1).ok_or_else(|| {
        HalError::Config(format!(
            "{} cannot produce {:.4} MHz with its VCO within -{:.0} % / +{:.0} % of {:.1} MHz \
             (it also drives: {}); choose another --pll",
            pll.id.name(),
            carrier / 1e6,
            max_decrease * 100.0,
            max_change * 100.0,
            vco_old / 1e6,
            users.names().join(", ")
        ))
    })
}

/// Plan clock-divider modulation from the PER channel of `pll` (unchanged).
pub fn plan_clkdiv(pll: &Pll, gpclk: usize, carrier: f64, max_offset: f64) -> Result<BcmPlan> {
    let per = pll.per_divider().filter(|_| pll.is_powered() && pll.is_locked()).ok_or_else(|| {
        HalError::Hardware(format!("{} is not running; cannot use it as the clock divider source", pll.id.name()))
    })?;
    let src = pll.vco_hz() / per as f64;
    let d = src / carrier;
    // MASH 1 needs an integer part of at least 2.
    if !(2.0..4095.0).contains(&d) {
        return Err(HalError::Config(format!(
            "{:.1} MHz from {}_PER ({:.1} MHz) needs divider {d:.3}, outside 2-4095",
            carrier / 1e6,
            pll.id.name(),
            src / 1e6
        )));
    }
    let center = (d * 4096.0).round();
    let units_per_hz = -src * 4096.0 / (carrier * carrier);
    Ok(BcmPlan {
        method: BcmMethod::ClkDiv,
        gpclk,
        gpclk_src: pll.id.gpclk_source(),
        power_up: false,
        set_per: false,
        ndiv: 0,
        center,
        units_per_hz,
        per_div: per,
        gp_div: d as u32,
        vco_old: pll.vco_hz(),
        vco_new: pll.vco_hz(),
        carrier_hz: src * 4096.0 / center,
        max_offset_hz: max_offset,
        users: PllUsers::default(),
        step_hz: 1.0 / units_per_hz.abs(),
        loop_ki: None,
    })
}

/// Modulation response of PLLA and PLLC on the BCM2836 (Raspberry Pi 2
/// v1.1) beyond the zero-order hold, `(Hz, dB)`: retuned while running
/// (feedback pre-divider on), with txengine's loop gain [`MODULATION_KI`].
/// Measured over the air on PLLA at 2277.6 MHz (`txengine tone`, 13 probe
/// tones of 2.5 and 5 kHz, USRP B210; three runs agree within ±0.01 dB).
/// The loop lifts high modulation frequencies slightly, and delays them by
/// at most 0.25° at 67 kHz. (With the firmware's KI 2 the lift was 0.40 dB
/// at 38 kHz and 0.57 dB at 67 kHz, with 6° of phase lag.) PLLA from 1927
/// to 2365 MHz and PLLC at 1927 MHz measure the same within 0.02 dB in
/// one session; between sessions the whole response moved by up to
/// 0.15 dB at 67 kHz (0.05 dB at 38 kHz), on both boards alike.
const BCM2836_PLL_RESPONSE_DB: &[(f64, f64)] = &[
    (0.0, 0.0),
    (10e3, 0.005),
    (15e3, 0.005),
    (19e3, 0.01),
    (23e3, 0.02),
    (30e3, 0.025),
    (38e3, 0.04),
    (45e3, 0.055),
    (53e3, 0.075),
    (57e3, 0.085),
    (61e3, 0.10),
    (67e3, 0.12),
];

/// The same for PLLA and PLLC on the BCM2837, measured on a Raspberry Pi Zero 2 W
/// (RP3A0; two runs agree within ±0.04 dB). (KI 2: 0.33 dB at 38 kHz,
/// 0.48 dB and 5° at 67 kHz.)
const BCM2837_PLL_RESPONSE_DB: &[(f64, f64)] = &[
    (0.0, 0.0),
    (19e3, 0.0),
    (30e3, 0.01),
    (38e3, 0.015),
    (45e3, 0.035),
    (53e3, 0.045),
    (57e3, 0.045),
    (61e3, 0.06),
    (67e3, 0.075),
];

/// Samples per second the DMA engine can deliver when each sample is a
/// write of `word` to the register at `target_bus` plus a second control
/// block (as in the transmit ring). Runs an unpaced chain; `word` should be
/// the register's current content so nothing changes.
fn measure_dma_rate(ch: &DmaChannel, target_bus: u32, word: u32) -> Result<f64> {
    const PAIRS: usize = 512;
    let mem = VcMemory::alloc(2 * PAIRS * dmar::CB_SIZE + 64)?;
    let base = mem.dma_addr_at(0) as u32;
    let word_off = 2 * PAIRS * dmar::CB_SIZE;
    let dummy_off = word_off + 4;
    mem.write_u32(word_off, word);
    let ti = dmar::TI_NO_WIDE_BURSTS | dmar::TI_WAIT_RESP;
    let at = |off: usize| base + off as u32;
    for k in 0..PAIRS {
        let (a, b) = (2 * k * dmar::CB_SIZE, (2 * k + 1) * dmar::CB_SIZE);
        let next = if k + 1 == PAIRS { 0 } else { at(b + dmar::CB_SIZE) };
        let cb_a = ControlBlock { ti, source: at(word_off), dest: target_bus, length: 4, stride: 0, next: at(b) };
        let cb_b = ControlBlock { ti, source: at(word_off), dest: at(dummy_off), length: 4, stride: 0, next };
        mem.write_words(a, &cb_a.words());
        mem.write_words(b, &cb_b.words());
    }
    mem.flush(0, mem.len());
    let mut worst = Duration::ZERO;
    for _ in 0..3 {
        let t = Instant::now();
        ch.start(base)?;
        crate::sys::wait_until(Duration::from_millis(200), "DMA rate measurement", || !ch.is_active())?;
        worst = worst.max(t.elapsed());
        if let Some(e) = ch.errors() {
            return Err(HalError::Hardware(format!("DMA error during the rate measurement: {e}")));
        }
    }
    ch.stop()?;
    Ok(PAIRS as f64 / worst.as_secs_f64())
}

/// Pacer clock: the source divided by `divi + divf / 4096` (a fractional
/// divider runs MASH 1), and one FIFO word per `rng` clocks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PacerClock {
    pub divi: u32,
    pub divf: u32,
    pub rng: u32,
    /// Samples per second (exact, from the nominal source frequency).
    pub fs: f64,
}

impl PacerClock {
    /// Divider `units / 4096`.
    fn new(src_hz: f64, units: u32, rng: u32) -> PacerClock {
        PacerClock { divi: units >> 12, divf: units & 0xfff, rng, fs: src_hz * 4096.0 / (units as f64 * rng as f64) }
    }

    pub fn divider(&self) -> f64 {
        self.divi as f64 + self.divf as f64 / 4096.0
    }

    pub fn mash(&self) -> u32 {
        u32::from(self.divf != 0)
    }
}

/// Slowest source for a fractional pacer divider. MASH 1 makes each clock
/// period one source cycle longer or shorter, so a sample leaves up to one
/// source cycle early or late: 2 ns from PLLD_PER (500 MHz), no more than
/// the DMA engine's own timing resolution (one core clock cycle). On a
/// Pi 2 at 190 kS/s the multiplex noise measured the same (±0.2 dB) with
/// MASH 1 as with the nearest integer divider (189.97 kS/s). From the
/// 19.2 MHz crystal it would be 52 ns.
const MASH_MIN_SOURCE_HZ: f64 = 250e6;

/// Clock divider and frame length (clocks per sample, within
/// `min_rng..=max_rng`) for the pacer, closest to `target` samples/s.
///
/// The peripheral clock is kept between 5 MHz and `max_clk_hz`. An integer
/// divider has no jitter at all and is used if one comes within 1 ppm of
/// the target; otherwise, from a fast source, a fractional one (which hits
/// any rate within a fraction of a hertz). Among settings that are equally
/// close the fastest clock wins: the PWM and PCM register synchronisers run
/// on it, and with a slow clock successive register writes get lost
/// (BCM2835 peripherals, PWM `STA.BERR`).
pub fn plan_pacer(src_hz: f64, target: f64, min_rng: u32, max_rng: u32, max_clk_hz: f64) -> Option<PacerClock> {
    let mut candidates = Vec::new();
    for divi in 2..=4095u32 {
        let clk = src_hz / divi as f64;
        if clk > max_clk_hz || clk < 5e6 {
            continue;
        }
        let rng = (clk / target).round() as u32;
        if (min_rng..=max_rng).contains(&rng) {
            candidates.push(PacerClock::new(src_hz, divi << 12, rng));
        }
    }
    if src_hz >= MASH_MIN_SOURCE_HZ {
        let rngs = ((5e6 / target).ceil() as u32).max(min_rng)..=((max_clk_hz / target).floor() as u32).min(max_rng);
        for rng in rngs {
            let c = PacerClock::new(src_hz, (src_hz * 4096.0 / (target * rng as f64)).round() as u32, rng);
            // MASH 1 alternates between DIVI and DIVI + 1 source cycles.
            if c.divf != 0 && (2..=4095).contains(&c.divi) && src_hz / c.divi as f64 <= max_clk_hz && src_hz / c.divider() >= 5e6 {
                candidates.push(c);
            }
        }
    }
    let clk = |c: &PacerClock| src_hz / c.divider();
    let error_uhz = |c: &PacerClock| ((c.fs - target).abs() * 1e6).round() as u64;
    let close = candidates.iter().filter(|c| (c.fs - target).abs() <= 1e-6 * target);
    if let Some(c) = close.min_by(|a, b| a.mash().cmp(&b.mash()).then(clk(b).total_cmp(&clk(a)))) {
        return Some(*c);
    }
    candidates.into_iter().min_by(|a, b| error_uhz(a).cmp(&error_uhz(b)).then(clk(b).total_cmp(&clk(a))))
}

#[derive(Debug, Clone, Copy)]
struct PinState {
    gpio: u32,
    fsel: u32,
    pads_reg: usize,
    pads: u32,
}

/// Everything needed to restore the hardware (also run by the guardian).
#[derive(Debug, Clone)]
struct Restorer {
    cm: Cprman,
    gpio: Mmio,
    pads: Mmio,
    pwm: Mmio,
    pcm: Mmio,
    dma: DmaChannel,
    pacer: Pacer,
    gp_ctl: usize,
    gp_saved: (u32, u32),
    pacer_clk_saved: (u32, u32),
    pwm_saved: [u32; 3],
    pcm_saved: [u32; 4],
    pins: Vec<PinState>,
    pll: Option<(Pll, PllState)>,
    cpu: CpuClockPin,
}

impl Restorer {
    fn restore(&self) {
        if let Err(e) = self.dma.stop() {
            warn!("DMA channel {} did not stop cleanly: {e}", self.dma.ch);
        }
        // Pacer off.
        match self.pacer {
            Pacer::Pwm => {
                self.pwm.write(pwm::CTL, 0);
                self.pwm.write(pwm::DMAC, self.pwm_saved[1]);
                self.pwm.write(pwm::RNG1, self.pwm_saved[2]);
                self.pwm.write(pwm::CTL, self.pwm_saved[0]);
            }
            Pacer::Pcm => {
                self.pcm.write(pcm::CS, 0);
                self.pcm.write(pcm::MODE, self.pcm_saved[1]);
                self.pcm.write(pcm::TXC, self.pcm_saved[2]);
                self.pcm.write(pcm::DREQ_A, self.pcm_saved[3]);
                self.pcm.write(pcm::CS, self.pcm_saved[0]);
            }
        }
        let pclk = if self.pacer == Pacer::Pwm { cm::PWM } else { cm::PCM };
        restore_clock(&self.cm, pclk, self.pacer_clk_saved);
        // Output off: pins back to their previous function, then the clock.
        for p in &self.pins {
            let reg = regs::gpio::fsel(p.gpio);
            let sh = regs::gpio::fsel_shift(p.gpio);
            self.gpio.modify(reg, 7 << sh, p.fsel << sh);
            self.pads.write(p.pads_reg, pads::PASSWORD | (p.pads & 0xffffff));
        }
        restore_clock(&self.cm, self.gp_ctl, self.gp_saved);
        if let Some((pll, s)) = &self.pll
            && let Err(e) = pll.restore(s) {
                warn!("{}: {e}", pll.id.name());
            }
        self.cpu.restore();
    }
}

/// Restores the CPU clock if construction fails before the [`Restorer`]
/// has taken it over.
struct CpuPinGuard(Option<CpuClockPin>);

impl Drop for CpuPinGuard {
    fn drop(&mut self) {
        if let Some(p) = self.0.take() {
            p.restore();
        }
    }
}

fn restore_clock(cm: &Cprman, ctl: usize, saved: (u32, u32)) {
    let _ = cm.clock_stop(ctl);
    cm.write(cm::div(ctl), saved.1);
    cm.write(ctl, saved.0 & !cm::ENAB);
    if saved.0 & cm::ENAB != 0 {
        cm.write(ctl, saved.0);
    }
}

/// The BCM FM transmitter.
pub struct BcmFmTransmitter {
    info: SinkInfo,
    plan: BcmPlan,
    cm: Cprman,
    ch: DmaChannel,
    pwm: Mmio,
    pcm: Mmio,
    gpio: Mmio,
    pads_regs: Mmio,
    pacer: Pacer,
    /// Clock source and setting of the pacer.
    pacer_clk: (u32, PacerClock),
    pins: Vec<(u32, u32)>,
    drive: u8,
    restorer: Restorer,
    mem: VcMemory,
    guardian: Option<Guardian>,
    pll: Option<Pll>,
    /// The PLL as configured for transmission (checked for interference).
    pll_good: Option<PllConfig>,
    /// Gate (crystal cycles) of the carrier frequency count in progress.
    carrier_count: Option<u32>,
    // Layout (byte offsets in `mem`).
    n: usize,
    cb_base: u32,
    samples_off: usize,
    idle_off: usize,
    idle_word_off: usize,
    dummy_off: usize,
    target_bus: u32,
    fifo_bus: u32,
    permap: u32,
    // State.
    written: u64,
    consumed: u64,
    last_pos: usize,
    tail_linked: bool,
    parked: bool,
    redirect: bool,
    underruns: u64,
    running: bool,
    stopped: bool,
    dither: f64,
    /// PLL method: FRAC register value for 0 Hz and FRAC units per Hz,
    /// corrected for the crystal error.
    frac_base: f32,
    frac_scale: f32,
    /// Clock-divider method: the divided clock in units of 2^-12 Hz,
    /// corrected for the crystal error.
    div_source: f64,
    last_health: Instant,
    /// Last time the DMA position moved (stall watchdog).
    last_progress: Instant,
    /// Start of the current rate measurement window and `consumed`, `underruns` then.
    rate_window: (Instant, u64, u64),
    osc_hz: f64,
}

impl std::fmt::Debug for BcmFmTransmitter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.info.description)
    }
}

impl BcmFmTransmitter {
    pub fn new(cfg: &BcmFmConfig, bcm: &Bcm) -> Result<BcmFmTransmitter> {
        if cfg.gpios.is_empty() {
            return Err(HalError::Config("no output GPIO given".into()));
        }
        let mut gpclk = None;
        let mut pins = Vec::new();
        for &g in &cfg.gpios {
            let (n, alt) = gpclk_for_pin(g).ok_or_else(|| {
                HalError::Config(format!("GPIO{g} cannot output a clock (use 4, 5, 6, 20, 21, 32, 34, 42, 43 or 44)"))
            })?;
            if gpclk.is_some_and(|c| c != n) {
                return Err(HalError::Config("all output GPIOs must use the same GPCLK generator".into()));
            }
            gpclk = Some(n);
            pins.push((g, alt));
        }
        let gpclk = gpclk.expect("at least one pin");
        if cfg.drive > 7 {
            return Err(HalError::Config("drive strength must be 0-7".into()));
        }
        let osc = bcm.osc_hz;
        let cpr = Cprman::new(bcm.cm.clone());
        let gp_ctl = cm::gp(gpclk);
        let pll_of = |id| Pll::new(&cpr, id, osc, bcm.bcm2711);

        // Choose the modulation method.
        let plan = match cfg.method {
            Some(BcmMethod::Pll(id)) => plan_pll(&pll_of(id), gpclk, cfg.carrier_hz, osc, cfg.max_offset_hz, cfg.max_vco_change)?,
            Some(BcmMethod::ClkDiv) => plan_clkdiv(&pll_of(PllId::D), gpclk, cfg.carrier_hz, cfg.max_offset_hz)?,
            None => {
                // An idle PLL is ideal. Otherwise, on the BCM2836/7, PLLA,
                // which normally clocks only the GPU's blocks and covers the
                // whole band (see `GPU_PLL_MAX_DECREASE`), then PLLC, which
                // clocks the VPU, the SD card and the WiFi. The BCM2711 has
                // not been measured and keeps PLLC first.
                let candidates = [PllId::A, PllId::C, PllId::D];
                let idle = candidates.into_iter().find(|&id| {
                    let p = pll_of(id);
                    !(p.is_powered() && p.is_locked()) || p.users(gp_ctl).is_empty()
                });
                let shared = if bcm.bcm2711 { [PllId::C, PllId::A] } else { [PllId::A, PllId::C] };
                let order: Vec<PllId> = idle.into_iter().chain(shared).collect();
                let mut last = None;
                let mut found = None;
                for id in order {
                    match plan_pll(&pll_of(id), gpclk, cfg.carrier_hz, osc, cfg.max_offset_hz, cfg.max_vco_change) {
                        Ok(p) => {
                            found = Some(p);
                            break;
                        }
                        Err(e) => last = Some(e),
                    }
                }
                found.ok_or_else(|| last.expect("a planning error"))?
            }
        };
        let pll = match plan.method {
            BcmMethod::Pll(id) => Some(pll_of(id)),
            BcmMethod::ClkDiv => None,
        };
        // From here on the DMA engine writes the PLL over the A2W bus, which
        // must not meet the firmware's CPU clock changes (see `cpufreq`).
        let mut cpu = CpuPinGuard(pll.is_some().then(CpuClockPin::pin));

        // Pacer clock: PLLD_PER unless we modulate PLLD, else the crystal.
        let plld = pll_of(PllId::D);
        let (pclk_src, pclk_hz) = match (plan.method, plld.per_divider()) {
            (BcmMethod::Pll(PllId::D), _) | (_, None) => (cm::SRC_OSC, osc),
            (_, Some(d)) => (cm::SRC_PLLD_PER, plld.vco_hz() / d as f64),
        };
        // PCM: frame length 8..=1024 bit clocks; PWM: any range (the
        // serialiser output is not used).
        let (min_rng, max_rng, max_clk) = match cfg.pacer {
            Pacer::Pcm => (8, 1024, 25e6),
            Pacer::Pwm => (2, 1 << 16, 25e6),
        };
        let ch = DmaChannel::claim(&bcm.dma, cfg.dma_channel, bcm.bcm2711)?;

        // Every sample is one DMA write to the frequency register, and writes
        // to the PLL registers (A2W) are slow: measure what the engine
        // delivers on this board and choose the sample rate accordingly.
        let (target_bus, step_desc) = match plan.method {
            BcmMethod::Pll(_) => (Cprman::bus_addr(pll.as_ref().expect("pll").frac_offset()), "PLL fraction"),
            BcmMethod::ClkDiv => (Cprman::bus_addr(cm::div(gp_ctl)), "GPCLK divider"),
        };
        let current = match plan.method {
            BcmMethod::Pll(_) => cpr.read(pll.as_ref().expect("pll").frac_offset()) & cm::A2W_FRAC_MASK,
            BcmMethod::ClkDiv => cpr.read(cm::div(gp_ctl)) & 0x00ff_ffff,
        };
        let capacity = measure_dma_rate(&ch, target_bus, cm::PASSWORD | current)?;
        debug!("DMA delivers up to {:.1} kS/s to the {step_desc} register", capacity / 1e3);
        let target = match cfg.sample_rate {
            Some(fs) if fs > 0.97 * capacity => {
                return Err(HalError::Config(format!(
                    "{:.1} kS/s is more than the DMA engine sustains on this board ({:.1} kS/s); use --sample-rate {:.0} or less",
                    fs / 1e3,
                    capacity / 1e3,
                    (0.9 * capacity / 1e3).floor() * 1e3
                )));
            }
            Some(fs) => fs,
            None => {
                let fs = ((0.85 * capacity / 1e3).floor() * 1e3).min(250_000.0);
                if fs < 150_000.0 {
                    return Err(HalError::Hardware(format!(
                        "the DMA engine sustains only {:.1} kS/s on this board; an FM multiplex needs at least 150 kS/s",
                        capacity / 1e3
                    )));
                }
                fs
            }
        };
        let pclock = plan_pacer(pclk_hz, target, min_rng, max_rng, max_clk)
            .ok_or_else(|| HalError::Config(format!("no pacer setting for {target} S/s")))?;
        let fs = pclock.fs;

        // Ring size.
        let n = ((cfg.buffer.as_secs_f64() * fs) as usize).clamp(4096, 65536);
        let mut alloc_n = n;
        let mem = loop {
            let bytes = 2 * alloc_n * dmar::CB_SIZE + 2 * dmar::CB_SIZE + 4 * alloc_n + 64;
            match VcMemory::alloc(bytes) {
                Ok(m) => break m,
                Err(e) if alloc_n > 4096 => {
                    debug!("{e}; retrying with a smaller buffer");
                    alloc_n /= 2;
                }
                Err(e) => return Err(e),
            }
        };
        let n = alloc_n;
        let ring_bytes = 2 * n * dmar::CB_SIZE;
        let idle_off = ring_bytes;
        let samples_off = idle_off + 2 * dmar::CB_SIZE;
        let idle_word_off = samples_off + 4 * n;
        let dummy_off = idle_word_off + 4;

        let (fifo_bus, permap) = match cfg.pacer {
            Pacer::Pwm => (regs::BUS_PERIPH_BASE + (regs::PWM + pwm::FIF1) as u32, pwm::DREQ),
            Pacer::Pcm => (regs::BUS_PERIPH_BASE + (regs::PCM + pcm::FIFO) as u32, pcm::DREQ),
        };

        // Snapshot everything we will touch.
        let pins_state: Vec<PinState> = pins
            .iter()
            .map(|&(g, _)| {
                let pads_reg = if g < 28 { pads::GPIO_0_27 } else { pads::GPIO_28_45 };
                PinState {
                    gpio: g,
                    fsel: (bcm.gpio.read(regs::gpio::fsel(g)) >> regs::gpio::fsel_shift(g)) & 7,
                    pads_reg,
                    pads: bcm.pads.read(pads_reg),
                }
            })
            .collect();
        let pacer_ctl = if cfg.pacer == Pacer::Pwm { cm::PWM } else { cm::PCM };
        if cpr.read(gp_ctl) & cm::ENAB != 0 {
            warn!("GPCLK{gpclk} was already running (another transmitter?); taking it over");
        }
        let restorer = Restorer {
            cm: cpr.clone(),
            gpio: bcm.gpio.clone(),
            pads: bcm.pads.clone(),
            pwm: bcm.pwm.clone(),
            pcm: bcm.pcm.clone(),
            dma: ch.clone(),
            pacer: cfg.pacer,
            gp_ctl,
            gp_saved: (cpr.read(gp_ctl), cpr.read(cm::div(gp_ctl))),
            pacer_clk_saved: (cpr.read(pacer_ctl), cpr.read(cm::div(pacer_ctl))),
            pwm_saved: [bcm.pwm.read(pwm::CTL), bcm.pwm.read(pwm::DMAC), bcm.pwm.read(pwm::RNG1)],
            pcm_saved: [bcm.pcm.read(pcm::CS), bcm.pcm.read(pcm::MODE), bcm.pcm.read(pcm::TXC), bcm.pcm.read(pcm::DREQ_A)],
            pins: pins_state,
            pll: pll.as_ref().map(|p| (p.clone(), p.snapshot())),
            cpu: cpu.0.take().unwrap_or_default(),
        };

        let users = plan.users.names();
        let method = match plan.method {
            BcmMethod::Pll(id) => {
                let change = if plan.vco_old > 0.0 {
                    format!(", VCO {:.3} -> {:.3} MHz ({:+.2} %)", plan.vco_old / 1e6, plan.vco_new / 1e6, (plan.vco_new / plan.vco_old - 1.0) * 100.0)
                } else {
                    format!(", VCO {:.3} MHz (was idle)", plan.vco_new / 1e6)
                };
                let ki = plan.loop_ki.map_or(String::new(), |k| format!(", loop KI {} -> {k}", pll.as_ref().map_or(0, |p| p.loop_ki())));
                format!("{}_PER /{} -> GPCLK{gpclk} /{}{change}{ki}", id.name(), plan.per_div, plan.gp_div)
            }
            BcmMethod::ClkDiv => format!(
                "GPCLK{gpclk} fractional divider (MASH 1) from PLLD_PER {:.1} MHz",
                plan.vco_old / plan.per_div as f64 / 1e6
            ),
        };
        let pdiv = if pclock.divf == 0 { pclock.divi.to_string() } else { format!("{:.4} (MASH 1)", pclock.divider()) };
        let description = format!(
            "BCM{} {method}: carrier {:.6} MHz on GPIO{:?}, {:.3} kS/s ({} clock {:.1} MHz /{pdiv} /{}), \
             {step_desc} step {:.2} Hz, DMA ch{}, {n} samples ({:.1} ms), drive {} mA",
            if bcm.bcm2711 { "2711" } else { "283x" },
            plan.carrier_hz / 1e6,
            cfg.gpios,
            fs / 1e3,
            if cfg.pacer == Pacer::Pwm { "PWM" } else { "PCM" },
            pclk_hz / 1e6,
            pclock.rng,
            plan.step_hz,
            ch.ch,
            n as f64 / fs * 1e3,
            2 * (cfg.drive as u32 + 1),
        );
        if !users.is_empty() {
            warn!(
                "{} is shared with: {}; they will run {:+.2} % off their nominal frequency while transmitting",
                pll.as_ref().map_or("the PLL", |p| p.id.name()),
                users.join(", "),
                (plan.vco_new / plan.vco_old - 1.0) * 100.0
            );
        }

        let info = SinkInfo {
            description,
            sample_rate: fs,
            carrier_hz: plan.carrier_hz,
            max_offset_hz: plan.max_offset_hz,
            resolution_hz: plan.step_hz,
            max_clock_error_ppm: MAX_CLOCK_ERROR_PPM,
            buffer_len: n - 128,
            format: SampleFormat::FrequencyOffset,
            realtime: true,
            zero_order_hold: true,
            response_db: match plan.method {
                // Only the configurations that were measured: PLLA or PLLC
                // retuned while running, with txengine's loop gain.
                BcmMethod::Pll(PllId::A | PllId::C) if !plan.power_up && plan.loop_ki == Some(MODULATION_KI) => match bcm.soc {
                    Soc::Bcm2836 => BCM2836_PLL_RESPONSE_DB.to_vec(),
                    Soc::Bcm2837 => BCM2837_PLL_RESPONSE_DB.to_vec(),
                    _ => Vec::new(),
                },
                _ => Vec::new(),
            },
        };
        let mut tx = BcmFmTransmitter {
            info,
            cm: cpr,
            ch,
            pwm: bcm.pwm.clone(),
            pcm: bcm.pcm.clone(),
            gpio: bcm.gpio.clone(),
            pads_regs: bcm.pads.clone(),
            pacer: cfg.pacer,
            pacer_clk: (pclk_src, pclock),
            pins,
            drive: cfg.drive,
            restorer,
            cb_base: mem.dma_addr_at(0) as u32,
            mem,
            guardian: None,
            pll,
            pll_good: None,
            carrier_count: None,
            n,
            samples_off,
            idle_off,
            idle_word_off,
            dummy_off,
            target_bus,
            fifo_bus,
            permap,
            written: 0,
            consumed: 0,
            last_pos: 0,
            tail_linked: false,
            parked: false,
            redirect: false,
            underruns: 0,
            running: false,
            stopped: false,
            dither: 0.0,
            frac_base: plan.corrected_frac(0.0).0 as f32,
            frac_scale: plan.corrected_frac(0.0).1 as f32,
            div_source: plan.corrected_divider_source(0.0),
            last_health: Instant::now(),
            last_progress: Instant::now(),
            rate_window: (Instant::now(), 0, 0),
            osc_hz: bcm.osc_hz,
            plan,
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

    pub fn plan(&self) -> &BcmPlan {
        &self.plan
    }

    fn cb_off(&self, k: usize) -> usize {
        k * dmar::CB_SIZE
    }

    fn bus(&self, off: usize) -> u32 {
        self.cb_base + off as u32
    }

    fn write_cb(&self, off: usize, cb: ControlBlock) {
        self.mem.write_words(off, &cb.words());
    }

    /// Word written to the DMA target for a given offset.
    #[inline]
    fn word(&mut self, offset_hz: f32) -> u32 {
        match self.plan.method {
            BcmMethod::Pll(_) => {
                // FRAC = centre + offset · units/Hz, relative to NDIV. f32 is
                // exact to < 0.1 unit for 20-bit values; rounding by +0.5 and
                // a saturating conversion avoids `round()`, a library call on
                // 32-bit ARM.
                let v = self.frac_base + offset_hz * self.frac_scale;
                ((v + 0.5) as i32).clamp(0, cm::A2W_FRAC_MASK as i32) as u32 | cm::PASSWORD
            }
            BcmMethod::ClkDiv => {
                // Exact divider for the requested frequency, with first order
                // error feedback so the coarse 12-bit steps average out.
                let v = self.div_source / (self.plan.carrier_hz + offset_hz as f64) + self.dither;
                let q = v.round().clamp((2u32 << 12) as f64, (4095u32 << 12 | 0xfff) as f64);
                self.dither = v - q;
                cm::PASSWORD | q as u32
            }
        }
    }

    /// Word for the carrier centre, without dither (the idle loop's).
    fn center_word(&mut self) -> u32 {
        let dither = std::mem::replace(&mut self.dither, 0.0);
        let w = self.word(0.0);
        self.dither = dither;
        w
    }

    fn init_ring(&mut self) {
        let center = self.center_word();
        let a_ti = dmar::TI_NO_WIDE_BURSTS | dmar::TI_WAIT_RESP;
        let b_ti = dmar::TI_NO_WIDE_BURSTS | dmar::TI_WAIT_RESP | dmar::TI_DEST_DREQ | (self.permap << dmar::TI_PERMAP_SHIFT);
        for k in 0..self.n {
            let a = self.cb_off(2 * k);
            let b = self.cb_off(2 * k + 1);
            self.write_cb(
                a,
                ControlBlock { ti: a_ti, source: self.bus(self.samples_off + 4 * k), dest: self.target_bus, length: 4, stride: 0, next: self.bus(b) },
            );
            self.write_cb(
                b,
                ControlBlock {
                    ti: b_ti,
                    source: self.bus(self.dummy_off),
                    dest: self.fifo_bus,
                    length: 4,
                    stride: 0,
                    next: self.bus(self.cb_off(2 * ((k + 1) % self.n))),
                },
            );
            self.mem.write_u32(self.samples_off + 4 * k, center);
        }
        // Idle loop: hold the carrier at the centre frequency.
        let ia = self.idle_off;
        let ib = self.idle_off + dmar::CB_SIZE;
        self.write_cb(ia, ControlBlock { ti: a_ti, source: self.bus(self.idle_word_off), dest: self.target_bus, length: 4, stride: 0, next: self.bus(ib) });
        self.write_cb(ib, ControlBlock { ti: b_ti, source: self.bus(self.dummy_off), dest: self.fifo_bus, length: 4, stride: 0, next: self.bus(ia) });
        self.mem.write_u32(self.idle_word_off, center);
        self.mem.write_u32(self.dummy_off, 0);
        self.mem.flush(0, self.mem.len());
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

    fn setup_clocks(&mut self) -> Result<()> {
        let p = self.plan.clone();
        let gp_ctl = cm::gp(p.gpclk);
        self.cm.clock_stop(gp_ctl)?;
        match p.method {
            BcmMethod::Pll(_) => {
                let pll = self.pll.as_ref().expect("pll");
                let frac = (p.center - p.ndiv as f64 * (1u64 << cm::A2W_FRAC_BITS) as f64) as u32;
                if p.power_up {
                    pll.power_up(p.ndiv, frac)?;
                } else {
                    pll.retune(p.ndiv, frac)?;
                }
                if p.set_per {
                    pll.set_per(p.per_div)?;
                }
                if let Some(ki) = p.loop_ki {
                    pll.set_loop_ki(ki)?;
                }
                self.pll_good = Some(pll.config());
                self.cm.clock_start(gp_ctl, p.gpclk_src, p.gp_div, 0, 0)?;
            }
            BcmMethod::ClkDiv => {
                let c = p.center as u32;
                self.cm.clock_start(gp_ctl, p.gpclk_src, c >> 12, c & 0xfff, 1)?;
            }
        }
        // Verify the generator output with the frequency counter (±1 kHz:
        // 10 ms gate time, plus the modulation-free carrier).
        match self.cm.measure(cm::TCNT_GP0 + p.gpclk as u32, self.osc_hz, 0.01) {
            Ok(hz) if (hz - p.carrier_hz).abs() < 5_000.0 + p.carrier_hz * 2e-4 => {
                debug!("GPCLK{} measured {:.4} MHz", p.gpclk, hz / 1e6)
            }
            Ok(hz) => {
                return Err(HalError::Hardware(format!(
                    "GPCLK{} runs at {:.4} MHz instead of {:.4} MHz",
                    p.gpclk,
                    hz / 1e6,
                    p.carrier_hz / 1e6
                )));
            }
            Err(e) => warn!("could not verify the carrier frequency: {e}"),
        }
        Ok(())
    }

    fn start_pacer(&mut self) -> Result<()> {
        let (src, PacerClock { divi, divf, rng, .. }) = self.pacer_clk;
        let mash = self.pacer_clk.1.mash();
        match self.pacer {
            Pacer::Pwm => {
                // Every write must pass the PWM clock-domain synchroniser
                // before the next one; each step is confirmed by reading back
                // the register or the status it causes, and STA.BERR reports
                // a write that was lost anyway.
                let mut attempt = 0;
                loop {
                    attempt += 1;
                    self.pwm.write(pwm::CTL, 0);
                    self.pwm.write(pwm::DMAC, 0);
                    self.cm.clock_start(cm::PWM, src, divi, divf, mash)?;
                    self.pwm_write_confirmed(pwm::CTL, 0)?;
                    self.pwm_write_confirmed(pwm::DMAC, 0)?;
                    self.pwm_write_confirmed(pwm::RNG1, rng)?;
                    self.pwm.write(pwm::CTL, pwm::CTL_CLRF1);
                    crate::sys::wait_until(Duration::from_millis(5), "PWM FIFO clear", || {
                        self.pwm.read(pwm::STA) & pwm::STA_EMPT1 != 0
                    })?;
                    self.pwm.write(pwm::STA, pwm::STA_ERRORS);
                    // DREQ while fewer than 8 words are queued; panic at 15.
                    self.pwm_write_confirmed(pwm::DMAC, pwm::DMAC_ENAB | (15 << 8) | 8)?;
                    let ctl = pwm::CTL_USEF1 | pwm::CTL_MODE1 | pwm::CTL_PWEN1;
                    self.pwm_write_confirmed(pwm::CTL, ctl)?;
                    let sta = self.pwm.read(pwm::STA);
                    if sta & pwm::STA_BERR == 0 {
                        break;
                    }
                    if attempt == 3 {
                        return Err(HalError::Hardware(format!("PWM register writes keep failing (STA {sta:#x})")));
                    }
                    debug!("PWM bus error during set-up (STA {sta:#x}); repeating");
                }
            }
            Pacer::Pcm => {
                self.pcm.write(pcm::CS, pcm::CS_EN);
                self.cm.clock_start(cm::PCM, src, divi, divf, mash)?;
                // One 8-bit channel per frame of `rng` clocks.
                self.pcm.write(pcm::TXC, pcm::TXC_CH1EN);
                self.pcm.write(pcm::MODE, ((rng - 1) << pcm::MODE_FLEN_SHIFT) | (1 << pcm::MODE_FSLEN_SHIFT));
                self.pcm.write(pcm::CS, pcm::CS_EN | pcm::CS_TXCLR);
                // TXCLR takes two PCM clocks; SYNC echoes after the same delay.
                self.pcm.write(pcm::CS, pcm::CS_EN | pcm::CS_SYNC);
                crate::sys::wait_until(Duration::from_millis(10), "PCM sync", || self.pcm.read(pcm::CS) & pcm::CS_SYNC != 0)?;
                self.pcm.write(pcm::DREQ_A, (15 << 24) | (15 << 8));
                self.pcm.write(pcm::CS, pcm::CS_EN | pcm::CS_DMAEN | pcm::CS_TXERR);
            }
        }
        Ok(())
    }

    /// The engine must sustain the configured rate; otherwise the multiplex
    /// is stretched (pilot and audio pitch off). Measured over 2 s windows
    /// without underruns.
    fn check_rate(&mut self) -> Result<()> {
        let (t0, c0, u0) = self.rate_window;
        let dt = t0.elapsed().as_secs_f64();
        if dt < 2.0 {
            return Ok(());
        }
        self.rate_window = (Instant::now(), self.consumed, self.underruns);
        if self.underruns != u0 {
            return Ok(());
        }
        let rate = (self.consumed - c0) as f64 / dt;
        let fs = self.info.sample_rate;
        if rate < 0.97 * fs {
            return Err(HalError::Hardware(format!(
                "the DMA engine sustains only {:.1} kS/s of the configured {:.1} kS/s on this board; \
                 use a lower --sample-rate (e.g. {:.0})",
                rate / 1e3,
                fs / 1e3,
                (rate * 0.9 / 1e3).floor() * 1e3
            )));
        }
        Ok(())
    }

    /// Write a PWM register and wait until it reads back.
    fn pwm_write_confirmed(&self, reg: usize, v: u32) -> Result<()> {
        self.pwm.write(reg, v);
        crate::sys::wait_until(Duration::from_millis(5), "PWM register write", || self.pwm.read(reg) == v)?;
        Ok(())
    }

    /// Ring position of the DMA engine: Some(samples consumed modulo n), or
    /// None when parked in the idle loop.
    fn dma_position(&self) -> Result<Option<usize>> {
        let cb = self.ch.current_cb();
        let off = cb.wrapping_sub(self.cb_base) as usize;
        if off < 2 * self.n * dmar::CB_SIZE {
            let idx = off / dmar::CB_SIZE;
            // At A_k sample k is not yet written; at B_k it has been.
            Ok(Some((idx / 2 + idx % 2) % self.n))
        } else if off < self.samples_off {
            Ok(None)
        } else {
            Err(HalError::Hardware(format!("DMA control block address {cb:#010x} outside the ring")))
        }
    }

    fn check_health(&mut self) -> Result<()> {
        if let Some(e) = self.ch.errors() {
            warn!("DMA channel {} error ({e}); restarting it", self.ch.ch);
            let k = (self.consumed % self.n as u64) as usize;
            self.ch.start(self.bus(self.cb_off(2 * k)))?;
        } else if !self.ch.is_active() {
            warn!("DMA channel {} was stopped externally; restarting it", self.ch.ch);
            let k = (self.consumed % self.n as u64) as usize;
            self.ch.start(self.bus(self.cb_off(2 * k)))?;
        }
        if self.pacer == Pacer::Pwm {
            let sta = self.pwm.read(pwm::STA) & pwm::STA_ERRORS;
            if sta != 0 {
                debug!("PWM status errors {sta:#x}");
                self.pwm.write(pwm::STA, sta);
            }
        }
        if self.cm.read(cm::gp(self.plan.gpclk)) & cm::ENAB == 0 {
            return Err(HalError::Hardware(format!("GPCLK{} was disabled externally", self.plan.gpclk)));
        }
        // The PLL is watched through the clock manager only: its lock bit,
        // and the carrier frequency. Its own registers sit on the A2W bus,
        // which the DMA engine keeps busy writing the fraction; every read
        // of one delays the next write by about 3 µs, and the samples after
        // it catch up through the pacer's FIFO. That timing error is a click
        // (on a Pi 2 about 450 Hz of deviation per register read, with the
        // stereo pilot alone), so the registers are only read once the
        // frequency is wrong: another user of the counter can cause that too.
        if self.pll_good.is_some() {
            let wrong_hz = self.count_carrier();
            if let (Some(p), Some(good)) = (&self.pll, &self.pll_good)
                && (!p.is_locked() || (wrong_hz.is_some() && !p.matches(good)))
            {
                let now = p.config();
                warn!(
                    "{} was changed behind txengine's back (CTRL {:#x}, PER {:#x}, {}{}); restoring it",
                    p.id.name(),
                    now.ctrl,
                    now.per,
                    if p.is_locked() { "locked" } else { "not locked" },
                    wrong_hz.map(|hz| format!(", carrier {:.3} MHz", hz / 1e6)).unwrap_or_default()
                );
                p.repair(good)?;
            } else if let Some(hz) = wrong_hz {
                debug!("carrier counted at {:.3} MHz, but the PLL is as configured", hz / 1e6);
            }
        }
        Ok(())
    }

    /// Count the carrier with the frequency counter in the background, one
    /// gate per health check. Returns the frequency of a finished count if
    /// it is off by more than the modulation and the crystal correction can
    /// move the average (0.5 %; a feedback divider step is at least 0.8 %).
    /// The gate alternates between 5 and 10 ms: a stopped clock leaves the
    /// previous count in the register, which then does not fit the gate.
    fn count_carrier(&mut self) -> Option<f64> {
        let short = (self.osc_hz * 0.005) as u32;
        let mut wrong = None;
        let gate = match self.carrier_count {
            Some(gate) => {
                let n = self.cm.count_result()?;
                let hz = n as f64 * self.osc_hz / gate as f64;
                if (hz - self.plan.carrier_hz).abs() > 0.005 * self.plan.carrier_hz {
                    wrong = Some(hz);
                }
                if gate == short { 2 * short } else { short }
            }
            None => short,
        };
        self.cm.count_start(cm::TCNT_GP0 + self.plan.gpclk as u32, gate);
        self.carrier_count = Some(gate);
        wrong
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

    pub fn drive(&self) -> u8 {
        self.drive
    }
}

impl RfSink for BcmFmTransmitter {
    fn info(&self) -> &SinkInfo {
        &self.info
    }

    fn start(&mut self) -> Result<(), SinkError> {
        if self.running {
            return Ok(());
        }
        if self.written == 0 {
            return Err(SinkError::config("pre-fill the buffer before starting"));
        }
        self.start_pacer()?;
        self.ch.start(self.bus(self.cb_off(0)))?;
        if self.pacer == Pacer::Pcm {
            self.pcm.write(pcm::CS, self.pcm.read(pcm::CS) | pcm::CS_TXON);
        }
        // Carrier on: pads, then the pin function.
        for &(g, alt) in &self.pins {
            let preg = if g < 28 { pads::GPIO_0_27 } else { pads::GPIO_28_45 };
            self.pads_regs.write(preg, pads::PASSWORD | pads::SLEW_UNLIMITED | pads::HYST | self.drive as u32);
            let reg = regs::gpio::fsel(g);
            let sh = regs::gpio::fsel_shift(g);
            self.gpio.modify(reg, 7 << sh, alt << sh);
        }
        // Confirm that the pacer requests data: the ring position must move
        // past what fits into the FIFO.
        let moving = |t: &Self| matches!(t.dma_position(), Ok(Some(p)) if p >= 20);
        if crate::sys::wait_until(Duration::from_millis(20), "DMA pacing", || moving(self)).is_err() {
            warn!("the {:?} pacer did not start; configuring it again", self.pacer);
            self.start_pacer()?;
            crate::sys::wait_until(Duration::from_millis(20), "DMA pacing", || moving(self)).map_err(|_| {
                HalError::Hardware(format!("DMA channel {} is not being paced by the {:?} (no DREQ)", self.ch.ch, self.pacer))
            })?;
        }
        self.running = true;
        let now = Instant::now();
        self.last_health = now;
        self.last_progress = now;
        self.rate_window = (now, 0, 0);
        Ok(())
    }

    fn poll(&mut self) -> Result<SinkPoll, SinkError> {
        if self.running {
            // Frequent enough that interference with the PLL is repaired
            // within a few tens of milliseconds.
            if self.last_health.elapsed() > Duration::from_millis(20) {
                self.last_health = Instant::now();
                self.check_health()?;
            }
            let now = Instant::now();
            match self.dma_position()? {
                Some(pos) => {
                    if pos != self.last_pos || self.parked {
                        self.last_progress = now;
                    } else if self.written > self.consumed && now.duration_since(self.last_progress) > Duration::from_millis(100) {
                        // Data is queued but the engine has not moved: the pacer
                        // stopped requesting (or was reconfigured by someone else).
                        warn!(
                            "DMA stalled at sample {} (PWM STA {:#x}, DMA CS {:#x}); restarting the pacer",
                            self.consumed,
                            self.pwm.read(pwm::STA),
                            self.ch.cs()
                        );
                        self.underruns += 1;
                        self.start_pacer()?;
                        self.last_progress = now;
                    }
                    let delta = (pos + self.n - self.last_pos) % self.n;
                    self.consumed = (self.consumed + delta as u64).min(self.written);
                    self.last_pos = pos;
                    if self.parked {
                        self.parked = false;
                    }
                    if self.redirect {
                        // Back in the ring: re-close the idle loop.
                        let ib = self.idle_off + dmar::CB_SIZE;
                        self.mem.write_u32(ib + 20, self.bus(self.idle_off));
                        self.mem.flush(ib, dmar::CB_SIZE);
                        self.redirect = false;
                    }
                }
                None => {
                    if !self.parked {
                        self.parked = true;
                        self.underruns += 1;
                    }
                    self.consumed = self.written;
                    self.last_pos = (self.written % self.n as u64) as usize;
                }
            }
        }
        if self.running {
            self.check_rate()?;
        }
        let in_flight = (self.written - self.consumed) as usize;
        let writable = self.info.buffer_len.saturating_sub(in_flight);
        Ok(SinkPoll { writable, consumed: self.consumed, written: self.written, underruns: self.underruns })
    }

    fn write(&mut self, samples: &[f32]) -> Result<(), SinkError> {
        if samples.is_empty() {
            return Ok(());
        }
        if (self.written - self.consumed) as usize + samples.len() > self.info.buffer_len {
            return Err(SinkError::config("write exceeds the writable space"));
        }
        let first = (self.written % self.n as u64) as usize;
        // Direct 32-bit stores (the memory is uncached; no per-sample
        // virtual calls or bounds checks).
        assert!(samples.len() <= self.n && self.samples_off + 4 * self.n <= self.mem.len());
        // SAFETY: the sample area lies within the buffer (checked above).
        let base = unsafe { self.mem.as_ptr().add(self.samples_off).cast::<u32>() };
        let mut k = first;
        for &s in samples {
            let w = self.word(s);
            // SAFETY: k < n, within the sample area.
            unsafe { crate::mmio::store32(base.add(k), w) };
            k += 1;
            if k == self.n {
                k = 0;
            }
        }
        let last = (first + samples.len() - 1) % self.n;
        // New tail → idle loop, then open the old tail into the new data.
        self.mem.write_u32(self.cb_off(2 * last + 1) + 20, self.bus(self.idle_off));
        self.mem.flush(0, 0);
        if self.tail_linked {
            let prev = (first + self.n - 1) % self.n;
            self.mem.write_u32(self.cb_off(2 * prev + 1) + 20, self.bus(self.cb_off(2 * first)));
        }
        self.tail_linked = true;
        if self.running && self.parked && !self.redirect {
            // The engine is parked: send it into the new data.
            let ib = self.idle_off + dmar::CB_SIZE;
            self.mem.write_u32(ib + 20, self.bus(self.cb_off(2 * first)));
            self.redirect = true;
        }
        self.mem.flush(0, 0);
        self.written += samples.len() as u64;
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
        let (base, scale) = self.plan.corrected_frac(ppm);
        (self.frac_base, self.frac_scale) = (base as f32, scale as f32);
        self.div_source = self.plan.corrected_divider_source(ppm);
        // The idle loop holds the carrier centre.
        let w = self.center_word();
        self.mem.write_u32(self.idle_word_off, w);
        Ok(())
    }
}

impl Drop for BcmFmTransmitter {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pacer_is_exact_with_a_fast_clock() {
        // PLLD_PER at 500 MHz: 250 kS/s exactly, with the fastest clock
        // within 5..25 MHz.
        let c = plan_pacer(500e6, 250_000.0, 2, 1 << 16, 25e6).unwrap();
        assert_eq!(c.fs, 250_000.0);
        assert_eq!((c.divi, c.divf, c.rng), (20, 0, 100), "fastest exact clock is 25 MHz, integer");
    }

    #[test]
    fn pacer_is_fractional_for_a_multiple_of_the_pilot() {
        // 190 kS/s = 10 x 19 kHz: integer dividers of 500 MHz miss it by
        // 30 Hz (/47 /56); MASH 1 comes within 1 ppm.
        for (min, max) in [(2, 1 << 16), (8, 1024)] {
            let c = plan_pacer(500e6, 190_000.0, min, max, 25e6).unwrap();
            assert!((c.fs - 190_000.0).abs() <= 0.19, "{c:?}");
            assert_eq!(c.mash(), 1);
            let clk = 500e6 / c.divider();
            assert!((5e6..=25e6).contains(&clk) && 500e6 / c.divi as f64 <= 25e6, "{c:?}");
            assert!((c.fs - 500e6 / (c.divider() * c.rng as f64)).abs() < 1e-6);
        }
        // Not from the crystal: one of its cycles is 52 ns of jitter.
        let c = plan_pacer(19.2e6, 190_000.0, 2, 1 << 16, 25e6).unwrap();
        assert_eq!(c.mash(), 0);
    }

    fn plan(method: BcmMethod, center: f64, units_per_hz: f64, ndiv: u32, carrier_hz: f64) -> BcmPlan {
        BcmPlan {
            method,
            gpclk: 0,
            gpclk_src: 0,
            power_up: false,
            set_per: false,
            ndiv,
            center,
            units_per_hz,
            per_div: 2,
            gp_div: 8,
            vco_old: 0.0,
            vco_new: 0.0,
            carrier_hz,
            max_offset_hz: 100e3,
            users: PllUsers::default(),
            step_hz: 1.0 / units_per_hz.abs(),
            loop_ki: None,
        }
    }

    #[test]
    fn crystal_correction_keeps_the_carrier() {
        // PLLA from 19.2 MHz: VCO 1401.6 MHz = 87.6 MHz x 16.
        let osc = 19.2e6;
        let unit_hz = osc / (1u64 << cm::A2W_FRAC_BITS) as f64;
        let center = (87.6e6 * 16.0 / unit_hz).round();
        let p = plan(BcmMethod::Pll(PllId::A), center, 16.0 / unit_hz, (center / (1u64 << cm::A2W_FRAC_BITS) as f64) as u32, center * unit_hz / 16.0);
        for ppm in [-MAX_CLOCK_ERROR_PPM, -7.0, 0.0, 6.44, MAX_CLOCK_ERROR_PPM] {
            let (frac, per_hz) = p.corrected_frac(ppm);
            for offset in [-75e3, 0.0, 75e3] {
                let units = p.ndiv as f64 * (1u64 << cm::A2W_FRAC_BITS) as f64 + frac + offset * per_hz;
                let out = units * unit_hz * (1.0 + ppm * 1e-6) / 16.0;
                assert!((out - p.carrier_hz - offset).abs() < 1e-3, "{ppm} ppm, {offset} Hz: {out}");
            }
        }
        // The clock divider from PLLD_PER (500 MHz): the divided source
        // runs `ppm` fast, and the divider for the carrier grows with it.
        let center = (500e6_f64 / 87.6e6 * 4096.0).round();
        let p = plan(BcmMethod::ClkDiv, center, -500e6 * 4096.0 / (87.6e6 * 87.6e6), 0, 500e6 * 4096.0 / center);
        for ppm in [-30.0, 0.0, 30.0] {
            let divider = p.corrected_divider_source(ppm) / p.carrier_hz;
            assert!((500e6 * (1.0 + ppm * 1e-6) * 4096.0 / divider - p.carrier_hz).abs() < 1e-3);
        }
    }

    #[test]
    fn pacer_respects_limits() {
        for (src, target, min, max) in [(500e6, 194_000.0, 2, 1 << 16), (19.2e6, 228_000.0, 2, 1 << 16), (500e6, 195_000.0, 8, 1024)] {
            let c = plan_pacer(src, target, min, max, 25e6).unwrap();
            let clk = src / c.divider();
            assert!((5e6..=25e6).contains(&clk), "{src} {target}: clock {clk}");
            assert!((min..=max).contains(&c.rng));
            // The rate is always exact (the DSP uses it); with the 19.2 MHz
            // crystal as source only a few integer dividers qualify, so the
            // request is met within 0.5 %.
            assert!((c.fs - target).abs() / target < 5e-3, "{src} {target}: {}", c.fs);
        }
        // Nothing fits: a clock source below 5 MHz.
        assert!(plan_pacer(4e6, 100_000.0, 2, 1 << 16, 25e6).is_none());
    }
}
