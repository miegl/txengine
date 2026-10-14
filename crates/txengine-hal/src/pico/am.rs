//! Amplitude modulation through a Pico: the [`RfSink`].
//!
//! The firmware puts out one pulse per carrier cycle from a PIO state
//! machine at the raised system clock and places its edges itself
//! (`txengine_pico_protocol::am`); txengine plans the clock and sends the
//! amplitudes, `u16`, each lasting a whole number of carrier cycles. The
//! carrier is an exact fraction of the system clock (`k / 2³²`), so a
//! crystal correction retunes it, and the samples with it
//! (`paced_by_carrier`).

use std::time::{Duration, Instant};

use log::{debug, info};
use txengine_core::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll};
use txengine_pico_protocol::am::{MAX_PERIOD, MIN_PERIOD};
use txengine_pico_protocol::{AmConfig, Info, Retune, State, SysClock};

use super::{Pico, find};
use crate::am::interpolation_response_db;
use crate::{HalError, MAX_CLOCK_ERROR_PPM, Result};

#[derive(Debug, Clone)]
pub struct PicoAmConfig {
    pub carrier_hz: f64,
    /// Serial number or USB port of the Pico; `None` for the only one.
    pub device: Option<String>,
    pub gpio: u8,
    /// Pad drive in mA (2, 4, 8 or 12).
    pub drive_ma: u8,
    /// Desired sample rate; each sample lasts the whole number of carrier
    /// cycles that comes closest.
    pub sample_rate: f64,
    pub buffer: Duration,
}

/// The plan: the system clock and the carrier as a fraction of it.
#[derive(Debug, Clone, PartialEq)]
pub struct AmPlan {
    pub clock: SysClock,
    pub clock_hz: f64,
    /// Carrier phase per clock cycle (2³² = one period).
    pub k: u32,
    pub carrier_hz: f64,
    /// Carrier cycles per sample.
    pub cycles: u16,
}

impl AmPlan {
    /// The carrier period in clock cycles.
    pub fn period(&self) -> f64 {
        self.clock_hz / self.carrier_hz
    }

    pub fn sample_rate(&self) -> f64 {
        self.carrier_hz / self.cycles as f64
    }

    /// The phase step for a crystal `ppm` fast: the clock is that much
    /// faster, the step that much smaller.
    pub fn k(&self, ppm: f64) -> u32 {
        (self.k as f64 / (1.0 + ppm * 1e-6)).round() as u32
    }

    /// The firmware's configuration.
    pub fn config(&self, gpio: u8, drive_ma: u8, ppm: f64) -> AmConfig {
        AmConfig {
            refdiv: self.clock.refdiv,
            fbdiv: self.clock.fbdiv,
            post_div1: self.clock.post_div1,
            post_div2: self.clock.post_div2,
            gpio,
            drive_ma,
            k: self.k(ppm),
            cycles: self.cycles,
        }
    }
}

/// The fastest system clock `PLL_USB` makes within the firmware's range
/// (the quantiser gets the most cycles per pulse, the pulses the finest
/// edges), from the fastest VCO (the least jitter).
fn fastest_clock(xosc_hz: f64, (min_hz, max_hz): (f64, f64)) -> Option<(SysClock, f64)> {
    let mut best: Option<(SysClock, f64, f64)> = None;
    for fbdiv in 16..=320u16 {
        let vco = xosc_hz * fbdiv as f64;
        if !(750e6..=1600e6).contains(&vco) {
            continue;
        }
        for post_div1 in 1..=7u8 {
            for post_div2 in 1..=post_div1 {
                let hz = vco / (post_div1 as f64 * post_div2 as f64);
                if (min_hz..=max_hz).contains(&hz) && best.as_ref().is_none_or(|&(_, b, v)| (hz, vco) > (b, v)) {
                    best = Some((SysClock { refdiv: 1, fbdiv, post_div1, post_div2 }, hz, vco));
                }
            }
        }
    }
    best.map(|(c, hz, _)| (c, hz))
}

/// The carriers a Pico with this description makes (Hz) at about
/// `sample_rate`: the firmware spends [`Info::am_pulse_cycles`] per carrier
/// cycle and [`Info::am_sample_cycles`] per sample, so a clock `f` keeps up
/// to `(f - sample_cycles · fs) / pulse_cycles`, and the pulses need room
/// ([`MIN_PERIOD`]).
pub fn carrier_range(info: &Info, sample_rate: f64) -> Option<(f64, f64)> {
    let (_, clk) = fastest_clock(info.xosc_hz as f64, (info.pio_min_hz as f64, info.pio_max_hz as f64))?;
    let (pulse, sample) = (info.am_pulse_cycles as f64, info.am_sample_cycles as f64);
    let hi = ((clk - sample * sample_rate) / pulse).min(clk / MIN_PERIOD as f64);
    (pulse > 0.0 && hi > 0.0).then_some((clk / MAX_PERIOD as f64, hi))
}

/// Plan `carrier_hz` on a Pico with this description.
pub fn plan(carrier_hz: f64, sample_rate: f64, info: &Info) -> Result<AmPlan> {
    let (clock, clock_hz) = fastest_clock(info.xosc_hz as f64, (info.pio_min_hz as f64, info.pio_max_hz as f64))
        .ok_or_else(|| HalError::Config("no system clock for AM on this Pico".into()))?;
    let k = (carrier_hz / clock_hz * 2f64.powi(32)).round() as u32;
    let actual_hz = clock_hz * k as f64 / 2f64.powi(32);
    let cycles = (actual_hz / sample_rate).round().clamp(1.0, 1024.0) as u16;
    let plan = AmPlan { clock, clock_hz, k, carrier_hz: actual_hz, cycles };
    // Within reach with the crystal corrected either way.
    let margin = 1.0 + MAX_CLOCK_ERROR_PPM * 1e-6;
    let reach = |k: u32| {
        let c = plan.config(0, 2, 0.0);
        let p = 2f64.powi(32) / k as f64;
        (MIN_PERIOD as f64..=MAX_PERIOD as f64).contains(&p)
            && AmConfig { k, ..c }.sustainable(info.am_pulse_cycles, info.am_sample_cycles)
    };
    if !(reach((k as f64 * margin) as u32) && reach((k as f64 / margin) as u32)) {
        let (lo, hi) = carrier_range(info, sample_rate).unwrap_or((f64::INFINITY, 0.0));
        return Err(HalError::Config(format!(
            "the {} makes AM carriers of {:.0} kHz to {:.2} MHz (at {:.0} kS/s)",
            info.chip.board(),
            lo * margin / 1e3,
            hi / margin / 1e6,
            sample_rate / 1e3
        )));
    }
    Ok(plan)
}

pub struct PicoAmTransmitter {
    pico: Pico,
    plan: AmPlan,
    info: SinkInfo,
    written: u64,
    running: bool,
}

impl std::fmt::Debug for PicoAmTransmitter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PicoAmTransmitter({})", self.info.description)
    }
}

impl PicoAmTransmitter {
    pub fn new(cfg: &PicoAmConfig) -> std::result::Result<PicoAmTransmitter, SinkError> {
        let usb = find(cfg.device.as_deref())?;
        let pico = Pico::open(&usb)?;
        // Whatever it was doing, start from idle.
        pico.stop()?;
        let plan = plan(cfg.carrier_hz, cfg.sample_rate, &pico.info)?;
        pico.configure_am(&plan.config(cfg.gpio, cfg.drive_ma, 0.0))?;
        let fs = plan.sample_rate();
        let description = format!(
            "{}: GPIO{} from PIO, one pulse per carrier cycle, carrier {:.4} kHz = {:.3} MHz system clock (PLL_USB) \
             / {:.3}, {:.3} kS/s ({} cycles per sample)",
            pico.describe(),
            cfg.gpio,
            plan.carrier_hz / 1e3,
            plan.clock_hz / 1e6,
            plan.period(),
            fs / 1e3,
            plan.cycles
        );
        info!("{description}");
        let buffer_len = ((cfg.buffer.as_secs_f64() * fs) as usize).clamp(1024, pico.info.buffer_samples as usize / 2);
        let info = SinkInfo {
            description,
            sample_rate: fs,
            carrier_hz: plan.carrier_hz,
            max_offset_hz: 0.0,
            resolution_hz: 0.0,
            max_clock_error_ppm: MAX_CLOCK_ERROR_PPM,
            paced_by_carrier: true,
            buffer_len,
            format: SampleFormat::Amplitude,
            realtime: true,
            zero_order_hold: false,
            response_db: interpolation_response_db(fs, fs / 4.0),
        };
        Ok(PicoAmTransmitter { pico, plan, info, written: 0, running: false })
    }

    pub fn plan(&self) -> &AmPlan {
        &self.plan
    }
}

impl RfSink for PicoAmTransmitter {
    fn info(&self) -> &SinkInfo {
        &self.info
    }

    fn start(&mut self) -> std::result::Result<(), SinkError> {
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

    fn poll(&mut self) -> std::result::Result<SinkPoll, SinkError> {
        self.pico.reap()?;
        let s = self.pico.status()?;
        if s.state == State::Fault {
            return Err(SinkError::hw(format!("the Pico stopped: {}", s.fault.describe())));
        }
        if s.min_lead != u32::MAX && (s.min_lead as usize) < self.plan.cycles as usize * 4 {
            debug!("Pico pulse ring down to {} carrier cycles", s.min_lead);
        }
        let consumed = if self.running { s.consumed } else { 0 };
        let queued = self.written.saturating_sub(consumed) as usize;
        Ok(SinkPoll {
            writable: self.info.buffer_len.saturating_sub(queued),
            consumed,
            written: self.written,
            underruns: s.underruns as u64,
        })
    }

    fn write(&mut self, samples: &[f32]) -> std::result::Result<(), SinkError> {
        if samples.is_empty() {
            return Ok(());
        }
        let mut bytes = Vec::with_capacity(samples.len() * 2);
        for &a in samples {
            // NaN (as 0) and beyond the full output are clamped.
            let v = (a.clamp(0.0, 1.0) * 65535.0).round() as u16;
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        self.pico.send(bytes)?;
        self.written += samples.len() as u64;
        Ok(())
    }

    fn stop(&mut self) -> std::result::Result<(), SinkError> {
        self.pico.discard();
        self.pico.stop()?;
        self.running = false;
        Ok(())
    }

    fn set_clock_error_ppm(&mut self, ppm: f64) -> std::result::Result<(), SinkError> {
        if ppm.abs() > MAX_CLOCK_ERROR_PPM {
            return Err(SinkError::config(format!("crystal error {ppm} ppm beyond ±{MAX_CLOCK_ERROR_PPM}")));
        }
        self.pico.retune(&Retune { center: self.plan.k(ppm), scale: 0 })?;
        Ok(())
    }
}

impl Drop for PicoAmTransmitter {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    use txengine_pico_protocol::Chip;

    use super::*;

    fn info() -> Info {
        Info {
            protocol: txengine_pico_protocol::PROTOCOL_VERSION,
            chip: Chip::Rp2040,
            xosc_hz: 12_000_000,
            sys_hz: 144_000_000,
            buffer_samples: 32768,
            max_update_hz: 3_000_000,
            gpio_mask: 1 << 21,
            pio_min_hz: 240_000_000,
            pio_max_hz: 264_000_000,
            watchdog_note: 0,
            am_pulse_cycles: 132,
            am_sample_cycles: 310,
        }
    }

    #[test]
    fn plans_the_broadcast_bands() {
        let i = info();
        for khz in [153.0, 279.0, 531.0, 999.0, 1_602.0] {
            let p = plan(khz * 1e3, 100e3, &i).unwrap();
            assert_eq!(p.clock_hz, 264e6, "{p:?}");
            assert_eq!(p.clock.fbdiv, 132, "{p:?}");
            assert!((p.carrier_hz - khz * 1e3).abs() < 0.07, "{khz}: {p:?}");
            assert!((p.sample_rate() - 100e3).abs() <= p.sample_rate() / p.cycles as f64, "{p:?}");
            // The crystal correction moves the carrier back.
            for ppm in [-MAX_CLOCK_ERROR_PPM, 11.0, MAX_CLOCK_ERROR_PPM] {
                let true_clock = p.clock_hz * (1.0 + ppm * 1e-6);
                let f = true_clock * p.k(ppm) as f64 / 2f64.powi(32);
                assert!((f - p.carrier_hz).abs() < 0.07, "{khz} kHz, {ppm} ppm: {f}");
            }
        }
        // The whole medium wave band; beyond what the firmware keeps up with
        // (more samples, more work).
        assert!(plan(1.71e6, 100e3, &i).is_ok());
        assert!(plan(1.9e6, 100e3, &i).is_err());
        assert!(plan(1.71e6, 200e3, &i).is_err());
        assert!(plan(50e3, 100e3, &i).is_err());
        let (lo, hi) = carrier_range(&i, 100e3).unwrap();
        assert!((lo - 264e6 / 4095.0).abs() < 1.0, "{lo}");
        assert!((hi - (264e6 - 310.0 * 100e3) / 132.0).abs() < 1.0, "{hi}");
    }
}
