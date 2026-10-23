//! Frequency modulation through a Pico: the [`RfSink`].
//!
//! Samples become `i16` in units of the plan's resolution (corrected for
//! the crystal error) and go out as bulk transfers; the firmware buffers
//! them (about 175 ms) and emits them at its own crystal-locked pace. Each
//! poll reads the firmware's count of emitted samples with one control
//! transfer, which is also what the clock discipline times.

use std::time::{Duration, Instant};

use log::{debug, info, warn};
use txengine_core::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll};
use txengine_pico_protocol::{Chip, Config, Retune, State};

use super::plan::{self, PioPlan, Plan, Timing};
use super::{Pico, find};
use crate::MAX_CLOCK_ERROR_PPM;

/// Lowest sample rate a modulator may get (the FM multiplex needs 124 kS/s).
const MIN_SAMPLE_RATE: f64 = 124e3;
/// Default sample rate: near 190 kS/s like the Pi 2/3 (166.7 kS/s).
const DEFAULT_SAMPLE_RATE: f64 = 190e3;

#[derive(Debug, Clone)]
pub struct PicoFmConfig {
    pub carrier_hz: f64,
    /// Serial number or USB port of the Pico; `None` for the only one.
    pub device: Option<String>,
    pub gpio: u8,
    /// Pad drive in mA (2, 4, 8 or 12).
    pub drive_ma: u8,
    pub sample_rate: Option<f64>,
    pub buffer: Duration,
    pub max_offset_hz: f64,
    /// Interpolate linearly between samples (else hold each).
    pub interpolate: bool,
    /// When the feedback divider is written within the reference cycle
    /// (system clock cycles after its rising edge) [default: Config::PHASE].
    pub phase: Option<u8>,
    /// Sigma-delta update rate [default: the fastest the firmware sustains].
    pub update_hz: Option<f64>,
    /// Reference divider (the PLL compares at the crystal / `refdiv`)
    /// [default: [`plan::reference_dividers`]].
    pub refdiv: Option<u8>,
    pub synthesis: Synthesis,
}

/// How the Pico makes the carrier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Synthesis {
    /// `PLL_USB`'s feedback divider dithered by a sigma-delta modulator.
    Pll,
    /// A PIO bit stream at the (raised) system clock.
    Pio,
}

/// The plan of either method.
#[derive(Debug, Clone)]
pub enum Method {
    Pll(Plan),
    Pio(PioPlan),
}

/// The modulation response beyond the zero-order hold (dB), tabulated for
/// the modulator's equaliser: linear interpolation adds one more `sinc`,
/// and the PLL method's loop lifts the multiplex band by `loop_db_38k` at
/// 38 kHz, growing as the square of the frequency.
fn response(fs: f64, interpolate: bool, loop_db_38k: f64) -> Vec<(f64, f64)> {
    (0..=60)
        .map(|i| {
            let f = i as f64 * 2_000.0;
            let x = std::f64::consts::PI * f / fs;
            let sinc = if i == 0 || !interpolate { 1.0 } else { x.sin() / x };
            // Measured up to 67 kHz; held beyond.
            (f, 20.0 * sinc.log10() + loop_db_38k * (f.min(70e3) / 38e3).powi(2))
        })
        .collect()
}

/// How much the PLL method's loop lifts 38 kHz relative to low modulation
/// frequencies (dB): the carrier follows the feedback divider through
/// `PLL_USB`'s loop, which peaks far above the multiplex band (its noise
/// hump is near 300 kHz). Measured over the air (`txengine tone`, 13 probe
/// tones of 5 kHz up to 67 kHz, and the stereo separation of a left-only
/// 1 kHz tone, USRP B210): the lift grows as f² within ±0.04 dB, and in
/// proportion to the reference divider (the loop is slower with a lower
/// comparison frequency): RP2040 +0.062 dB at 38 kHz with a 12 MHz
/// reference, +0.113 to +0.122 dB with 6 MHz from 87.6 to 107.7 MHz;
/// RP2350 +0.066 and +0.146 to +0.150 dB. Without it the separation is
/// 41-44 dB (the PIO method, which has no loop, measures 73 dB).
fn pll_loop_db_38k(chip: Chip, refdiv: u8) -> f64 {
    let per_refdiv = match chip {
        Chip::Rp2040 => 0.060,
        Chip::Rp2350 => 0.072,
    };
    per_refdiv * refdiv as f64
}

/// Plan the PIO method for `cfg` on a Pico with this description.
pub fn pio_plan_for(cfg: &PicoFmConfig, info: &txengine_pico_protocol::Info) -> Result<PioPlan, SinkError> {
    Ok(plan::pio_plan(
        cfg.carrier_hz,
        info.xosc_hz as f64,
        cfg.max_offset_hz,
        cfg.sample_rate.unwrap_or(DEFAULT_SAMPLE_RATE),
        MIN_SAMPLE_RATE,
        (info.pio_min_hz as f64, info.pio_max_hz as f64),
    )?)
}

/// Plan for `cfg` on a Pico with these properties, without opening it.
pub fn plan_for(cfg: &PicoFmConfig, xosc_hz: f64, max_update_hz: f64) -> Result<Plan, SinkError> {
    let max_update_hz = cfg.update_hz.map_or(max_update_hz, |u| u.min(max_update_hz));
    let rate = cfg.sample_rate.unwrap_or(DEFAULT_SAMPLE_RATE);
    let given = cfg.refdiv.map(|r| [r]);
    let refdivs = given.as_ref().map_or(plan::reference_dividers(cfg.max_offset_hz), |r| &r[..]);
    Ok(plan::plan_clearest(cfg.carrier_hz, xosc_hz, cfg.max_offset_hz, refdivs, |fref| {
        Timing::choose(fref, max_update_hz, rate, MIN_SAMPLE_RATE)
    })?)
}

pub struct PicoFmTransmitter {
    pico: Pico,
    method: Method,
    info: SinkInfo,
    /// Hz per sample unit, corrected for the crystal error.
    resolution_hz: f64,
    ppm: f64,
    written: u64,
    running: bool,
    underruns: u64,
}

impl std::fmt::Debug for PicoFmTransmitter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PicoFmTransmitter({})", self.info.description)
    }
}

impl PicoFmTransmitter {
    pub fn new(cfg: &PicoFmConfig) -> Result<PicoFmTransmitter, SinkError> {
        let usb = find(cfg.device.as_deref())?;
        let pico = Pico::open(&usb)?;
        // Whatever it was doing, start from idle.
        pico.stop()?;
        let (method, description, carrier_hz, resolution_hz, fs, loop_db_38k) = match cfg.synthesis {
            Synthesis::Pll => {
                if pico.info.gpio_mask & (1 << cfg.gpio) == 0 {
                    let pins: Vec<String> =
                        (0..32).filter(|g| pico.info.gpio_mask & (1 << g) != 0).map(|g| g.to_string()).collect();
                    return Err(SinkError::config(format!(
                        "GPIO{} has no clock output on the {}; use {} (or the PIO method)",
                        cfg.gpio,
                        pico.info.chip.name(),
                        pins.join(", ")
                    )));
                }
                let plan = plan_for(cfg, pico.info.xosc_hz as f64, pico.info.max_update_hz as f64)?;
                if let Some(w) = plan.spur_warning(cfg.max_offset_hz) {
                    warn!("{w}");
                }
                let phase = cfg.phase.unwrap_or(Config::PHASE);
                pico.configure(&plan.config(cfg.gpio, cfg.drive_ma, cfg.interpolate, phase, 0.0))?;
                let fs = plan.sample_rate();
                let description = format!(
                    "{}: GPIO{} from PLL_USB, VCO {:.3} MHz = {:.0} MHz x {:.6} (FBDIV {}..{} dithered at {:.0} MHz), \
                     division {} ({}x{}, gpout /{}), {:.0} S/s{}",
                    pico.describe(),
                    cfg.gpio,
                    plan.fref_hz * plan.n / 1e6,
                    plan.fref_hz / 1e6,
                    plan.n,
                    plan.base,
                    plan.base + 4,
                    plan.update_hz() / 1e6,
                    plan.division(),
                    plan.post_div1,
                    plan.post_div2,
                    plan.gpout_div,
                    fs,
                    if cfg.interpolate { " interpolated" } else { "" }
                );
                let loop_db_38k = pll_loop_db_38k(pico.info.chip, plan.refdiv);
                (Method::Pll(plan.clone()), description, plan.carrier_hz, plan.resolution_hz, fs, loop_db_38k)
            }
            Synthesis::Pio => {
                let plan = pio_plan_for(cfg, &pico.info)?;
                pico.configure_pio(&plan.config(cfg.gpio, cfg.drive_ma, cfg.interpolate, 0.0))?;
                let fs = plan.sample_rate();
                let description = format!(
                    "{}: GPIO{} from PIO, {:.3} Mbit/s (system clock from PLL_USB), {:.0} S/s{}; {} \
                     (ideal bit stream)",
                    pico.describe(),
                    cfg.gpio,
                    plan.bit_rate / 1e6,
                    fs,
                    if cfg.interpolate { " interpolated" } else { "" },
                    plan.describe_alias()
                );
                (Method::Pio(plan.clone()), description, plan.carrier_hz, plan.resolution_hz, fs, 0.0)
            }
        };
        info!("{description}");
        let buffer_len = ((cfg.buffer.as_secs_f64() * fs) as usize).clamp(1024, pico.info.buffer_samples as usize / 2);
        let info = SinkInfo {
            description,
            sample_rate: fs,
            carrier_hz,
            max_offset_hz: i16::MAX as f64 * resolution_hz,
            resolution_hz,
            max_clock_error_ppm: MAX_CLOCK_ERROR_PPM,
            paced_by_carrier: false,
            buffer_len,
            format: SampleFormat::FrequencyOffset,
            realtime: true,
            zero_order_hold: true,
            response_db: response(fs, cfg.interpolate, loop_db_38k),
        };
        Ok(PicoFmTransmitter { resolution_hz, pico, method, info, ppm: 0.0, written: 0, running: false, underruns: 0 })
    }

    pub fn method(&self) -> &Method {
        &self.method
    }
}

impl RfSink for PicoFmTransmitter {
    fn info(&self) -> &SinkInfo {
        &self.info
    }

    fn start(&mut self) -> Result<(), SinkError> {
        if self.running {
            return Ok(());
        }
        // The control request could overtake the samples still on their
        // way: wait until the pre-fill has arrived.
        let t0 = Instant::now();
        loop {
            self.pico.reap()?;
            let s = self.pico.status()?;
            if s.received >= self.written {
                break;
            }
            if t0.elapsed() > Duration::from_secs(2) {
                return Err(SinkError::hw(format!(
                    "the Pico received {} of {} samples before starting",
                    s.received, self.written
                )));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        self.pico.start()?;
        self.running = true;
        debug!("Pico started with {} samples buffered", self.written);
        Ok(())
    }

    fn poll(&mut self) -> Result<SinkPoll, SinkError> {
        self.pico.reap()?;
        let s = self.pico.status()?;
        match s.state {
            State::Fault => return Err(SinkError::hw(format!("the Pico stopped: {}", s.fault.describe()))),
            State::Running | State::Configured | State::Idle => {}
        }
        if s.min_lead != u32::MAX && s.min_lead < 1024 {
            debug!("Pico modulator lead down to {} updates", s.min_lead);
        }
        self.underruns = s.underruns as u64;
        let consumed = if self.running { s.consumed } else { 0 };
        let queued = self.written.saturating_sub(consumed) as usize;
        Ok(SinkPoll {
            writable: self.info.buffer_len.saturating_sub(queued),
            consumed,
            written: self.written,
            underruns: self.underruns,
        })
    }

    fn write(&mut self, samples: &[f32]) -> Result<(), SinkError> {
        if samples.is_empty() {
            return Ok(());
        }
        let k = 1.0 / self.resolution_hz as f32;
        let mut bytes = Vec::with_capacity(samples.len() * 2);
        for &s in samples {
            let v = (s * k).round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        self.pico.send(bytes)?;
        self.written += samples.len() as u64;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SinkError> {
        self.pico.discard();
        self.pico.stop()?;
        self.running = false;
        Ok(())
    }

    fn set_clock_error_ppm(&mut self, ppm: f64) -> Result<(), SinkError> {
        if ppm.abs() > MAX_CLOCK_ERROR_PPM {
            return Err(SinkError::config(format!("crystal error {ppm} ppm beyond ±{MAX_CLOCK_ERROR_PPM}")));
        }
        let (retune, resolution_hz) = match &self.method {
            Method::Pll(p) => (Retune { center: p.center(ppm), scale: p.scale }, p.resolution_hz),
            Method::Pio(p) => (Retune { center: p.center(ppm), scale: p.scale }, p.resolution_hz),
        };
        self.pico.retune(&retune)?;
        self.ppm = ppm;
        // Everything scales with the crystal: one sample unit is that much
        // larger in true frequency.
        self.resolution_hz = resolution_hz * (1.0 + ppm * 1e-6);
        Ok(())
    }
}

impl Drop for PicoFmTransmitter {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
