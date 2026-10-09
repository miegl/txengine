//! Frequency plan for the Pico: which divisions of `PLL_USB` reach the
//! carrier, and how samples map onto the sigma-delta modulator.
//!
//! The carrier is `fref · N / D` with `fref` the crystal divided by
//! `refdiv`, `N` the average feedback divider and `D` the product of the
//! two post dividers and the clock generator's divider. One step of the
//! feedback divider moves the carrier by `S = fref / D` (0.67–1.7 MHz in
//! the FM band), far more than a deviation: the sigma-delta modulator
//! dithers `FBDIV` between `base … base + 4` so that its average is
//! `N = base + 1 + x`, with `x` (0–2) set by each sample.

use txengine_pico_protocol::{Config, FRAC_BITS, PioConfig};

use crate::{HalError, MAX_CLOCK_ERROR_PPM, Result};

const VCO_MIN: f64 = 750e6;
const VCO_MAX: f64 = 1600e6;
/// Highest output of the post dividers we use (the clock generators are
/// specified up to 150 MHz).
const POSTDIV_OUT_MAX: f64 = 160e6;
const ONE: f64 = (1u64 << FRAC_BITS) as f64;

/// A frequency plan.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub fref_hz: f64,
    pub refdiv: u8,
    pub post_div1: u8,
    pub post_div2: u8,
    pub gpout_div: u16,
    /// `FBDIV` for code 0.
    pub base: u16,
    /// Average feedback divider at the nominal carrier.
    pub n: f64,
    /// Carrier the plan produces (nominal crystal).
    pub carrier_hz: f64,
    /// Carrier movement per step of the feedback divider (Hz).
    pub step_hz: f64,
    /// Modulator input per sample unit.
    pub scale: u32,
    /// Carrier offset per sample unit (Hz, nominal crystal).
    pub resolution_hz: f64,
    /// Reference cycles per sigma-delta update, and updates per sample.
    pub update_div: u8,
    pub sample_updates: u8,
}

impl Plan {
    /// Total division from the VCO to the carrier.
    pub fn division(&self) -> u32 {
        self.post_div1 as u32 * self.post_div2 as u32 * self.gpout_div as u32
    }

    pub fn update_hz(&self) -> f64 {
        self.fref_hz / self.update_div as f64
    }

    pub fn sample_rate(&self) -> f64 {
        self.update_hz() / self.sample_updates as f64
    }

    /// Largest offset a sample can carry (Hz).
    pub fn max_offset_hz(&self) -> f64 {
        i16::MAX as f64 * self.resolution_hz
    }

    /// Modulator input at the carrier for a crystal `ppm` fast: the
    /// feedback divider must be lower by `1 / (1 + ppm)`.
    pub fn center(&self, ppm: f64) -> u32 {
        let n = self.n / (1.0 + ppm * 1e-6);
        ((n - self.base as f64 - 1.0) * ONE).round().clamp(0.0, Config::X_MAX as f64) as u32
    }

    /// The firmware's configuration.
    pub fn config(&self, gpio: u8, drive_ma: u8, interpolate: bool, phase: u8, ppm: f64) -> Config {
        Config {
            refdiv: self.refdiv,
            post_div1: self.post_div1,
            post_div2: self.post_div2,
            gpout_div: self.gpout_div,
            gpio,
            drive_ma,
            base: self.base,
            update_div: self.update_div,
            sample_updates: self.sample_updates,
            interpolate,
            center: self.center(ppm),
            scale: self.scale,
            phase,
        }
    }
}

/// Sample timing: `update_div` reference cycles per update,
/// `sample_updates` updates per sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    pub update_div: u8,
    pub sample_updates: u8,
}

impl Timing {
    /// The fastest updates within `max_update_hz`, and the sample rate
    /// (at least `min_rate`) closest to `rate` with them.
    pub fn choose(fref_hz: f64, max_update_hz: f64, rate: f64, min_rate: f64) -> Result<Timing> {
        let mut best: Option<(Timing, f64)> = None;
        for update_div in 1..=11u8 {
            let fu = fref_hz / update_div as f64;
            if fu > max_update_hz {
                continue;
            }
            for words in 1..=40u8 {
                let sample_updates = words * Config::CODES_PER_WORD;
                let fs = fu / sample_updates as f64;
                if fs < min_rate {
                    continue;
                }
                // The fastest updates first (the modulator's noise falls
                // steeply with their rate), then the nearest sample rate.
                let cost = update_div as f64 * 1e3 + (fs / rate).ln().abs();
                if best.is_none_or(|(_, c)| cost < c) {
                    best = Some((Timing { update_div, sample_updates }, cost));
                }
            }
        }
        best.map(|(t, _)| t).ok_or_else(|| HalError::Config("no sample rate fits the Pico".into()))
    }
}

/// Plan `carrier_hz` from a `xosc_hz` crystal, able to move by
/// ±`max_offset_hz` and to correct ±[`MAX_CLOCK_ERROR_PPM`].
pub fn plan(carrier_hz: f64, xosc_hz: f64, max_offset_hz: f64, timing: Timing, refdiv: u8) -> Result<Plan> {
    let fref = xosc_hz / refdiv as f64;
    let reserve = max_offset_hz + carrier_hz * MAX_CLOCK_ERROR_PPM * 1e-6;
    // The modulator takes 0-2 steps; keep x away from the ends by a
    // sample's worth of interpolation steps.
    let guard = timing.sample_updates as f64 / ONE;
    let (x_lo, x_hi) = (guard, 2.0 - guard);
    let mut best: Option<Plan> = None;
    for post_div1 in 1..=7u8 {
        for post_div2 in 1..=post_div1 {
            for gpout_div in 1..=64u16 {
                let d = post_div1 as f64 * post_div2 as f64 * gpout_div as f64;
                let vco = carrier_hz * d;
                if vco / (post_div1 as f64 * post_div2 as f64) > POSTDIV_OUT_MAX || vco > VCO_MAX {
                    continue;
                }
                let n = vco / fref;
                // x = n - base - 1 in the middle of 0..2.
                let base = n.round() - 2.0;
                if base < 16.0 || fref * base < VCO_MIN || fref * (base + 4.0) > VCO_MAX || base + 4.0 > 320.0 {
                    continue;
                }
                let base = base as u16;
                // The firmware adds the codes 0-4 to the low 5 bits of base.
                if !Config::codes_fit(base) {
                    continue;
                }
                let step = fref / d;
                let x = n - base as f64 - 1.0;
                let m = reserve / step;
                if x - m < x_lo || x + m > x_hi {
                    continue;
                }
                // Largest offset over i16: the finest resolution that reaches it.
                let scale = (max_offset_hz / i16::MAX as f64 / step * ONE).ceil();
                if scale < 1.0 || scale > Config::SCALE_MAX as f64 {
                    continue;
                }
                let p = Plan {
                    fref_hz: fref,
                    refdiv,
                    post_div1,
                    post_div2,
                    gpout_div,
                    base,
                    n,
                    carrier_hz: fref * n / d,
                    step_hz: step,
                    scale: scale as u32,
                    resolution_hz: scale * step / ONE,
                    update_div: timing.update_div,
                    sample_updates: timing.sample_updates,
                };
                // The highest VCO (smallest steps, least jitter), then the
                // fewest dividers.
                let better = match &best {
                    None => true,
                    Some(b) => {
                        (d, std::cmp::Reverse(gpout_div)) > (b.division() as f64, std::cmp::Reverse(b.gpout_div))
                    }
                };
                if better {
                    best = Some(p);
                }
            }
        }
    }
    best.ok_or_else(|| HalError::Config(format!("the Pico cannot synthesise {:.3} MHz", carrier_hz / 1e6)))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 6 MHz / 2 = 3 MHz updates, 18 per sample.
    const T: Timing = Timing { update_div: 2, sample_updates: 18 };

    #[test]
    fn whole_fm_band() {
        let mut worst_step = 0f64;
        for k in 8750..=10800 {
            let f = k as f64 * 10e3;
            let p = plan(f, 12e6, 100e3, T, 2).unwrap_or_else(|e| panic!("{f}: {e}"));
            assert!((p.carrier_hz - f).abs() < 1e-3, "{f}");
            let x = p.center(0.0) as f64 / ONE;
            assert!((x - (p.n - p.base as f64 - 1.0)).abs() < 1e-8);
            assert!(p.max_offset_hz() >= 100e3);
            assert!(p.resolution_hz < 3.2, "{f}: {}", p.resolution_hz);
            worst_step = worst_step.max(p.step_hz);
        }
        assert!(worst_step < 0.45e6, "{worst_step}");
    }

    #[test]
    fn the_crystal_correction_fits() {
        let p = plan(87.9e6, 12e6, 100e3, T, 2).unwrap();
        // The largest division: the smallest steps (333 kHz).
        assert_eq!((p.post_div1, p.post_div2, p.gpout_div), (6, 3, 1), "{p:?}");
        for ppm in [-MAX_CLOCK_ERROR_PPM, 0.0, MAX_CLOCK_ERROR_PPM] {
            let c = p.center(ppm) as f64;
            let reach = 100e3 / p.resolution_hz * p.scale as f64;
            assert!(c - reach > 0.0 && c + reach < Config::X_MAX as f64, "{ppm}");
            // The carrier lands where it should with the crystal off.
            let fref = 6e6 * (1.0 + ppm * 1e-6);
            let f = fref * (p.base as f64 + 1.0 + c / ONE) / p.division() as f64;
            assert!((f - 87.9e6).abs() < 0.01, "{ppm}: {f}");
        }
    }

    #[test]
    fn timing() {
        // 6 MHz reference, 3 MHz updates at most.
        let t = Timing::choose(6e6, 3e6, 190e3, 124e3).unwrap();
        assert_eq!(t, Timing { update_div: 2, sample_updates: 18 });
        let t = Timing::choose(6e6, 3e6, 250e3, 124e3).unwrap();
        assert_eq!((6e6 / t.update_div as f64) / t.sample_updates as f64, 250e3);
    }
}

/// The PIO method's plan: `PLL_USB` runs the system clock (and so the
/// bit stream) at `bit_rate`; words of 32 bits carry a square wave at the
/// carrier.
#[derive(Debug, Clone, PartialEq)]
pub struct PioPlan {
    pub refdiv: u8,
    pub fbdiv: u16,
    pub post_div1: u8,
    pub post_div2: u8,
    pub bit_rate: f64,
    pub carrier_hz: f64,
    /// Carrier / bit rate (2³² = 1).
    pub ratio: u32,
    /// Phase advance per word and sample unit.
    pub scale: u32,
    /// Carrier offset per sample unit (Hz, nominal crystal).
    pub resolution_hz: f64,
    pub sample_words: u8,
    /// The strongest alias of the square wave's harmonics that lands in the
    /// FM band (dB relative to the carrier), and where.
    pub alias_dbc: f64,
    pub alias_hz: f64,
}

/// The FM broadcast band, where aliases matter.
const BAND: (f64, f64) = (87.5e6, 108.0e6);

/// The strongest product of the bit stream inside the FM band other than
/// the carrier itself: the square wave's odd harmonics `k·f` appear at
/// `|k·f − m·B|` with `1/k` of the fundamental's amplitude, shaped by the
/// pin's hold (`sinc`). Returns (dBc, frequency).
///
/// This is the ideal bit stream (an exact simulation of the firmware's
/// stream agrees within a few dB). The pin's output measures 6-17 dB
/// stronger on many of these lines, up to about −15 dBc (RP2040 and
/// RP2350 alike, at 4 and 12 mA, at 240 and 249.6 Mbit/s), for reasons not
/// yet understood. The bit rate is still chosen by this model.
pub fn pio_alias(f: f64, b: f64) -> (f64, f64) {
    let hold = |x: f64| {
        let a = std::f64::consts::PI * x / b;
        if a == 0.0 { 1.0 } else { (a.sin() / a).abs() }
    };
    let mut worst = (-200.0, 0.0);
    for k in (1..=25).step_by(2) {
        for m in 0..=k {
            let a = (k as f64 * f - m as f64 * b).abs();
            if a < BAND.0 || a > BAND.1 || (k == 1 && m == 0) || (a - f).abs() < 300e3 {
                continue;
            }
            let db = 20.0 * (hold(a) / hold(f) / k as f64).log10();
            if db > worst.0 {
                worst = (db, a);
            }
        }
    }
    worst
}

/// Plan the PIO method for `carrier_hz`: of the bit rates `PLL_USB` can
/// make within `min_hz ..= max_hz`, the one whose strongest in-band alias
/// is weakest.
pub fn pio_plan(
    carrier_hz: f64,
    xosc_hz: f64,
    max_offset_hz: f64,
    sample_rate: f64,
    min_rate: f64,
    (min_hz, max_hz): (f64, f64),
) -> Result<PioPlan> {
    let mut best: Option<(f64, f64, u16, u8, u8)> = None;
    for fbdiv in 16..=320u16 {
        let vco = xosc_hz * fbdiv as f64;
        if !(VCO_MIN..=VCO_MAX).contains(&vco) {
            continue;
        }
        for post_div1 in 1..=7u8 {
            for post_div2 in 1..=post_div1 {
                let b = vco / (post_div1 as f64 * post_div2 as f64);
                if b < min_hz || b > max_hz || carrier_hz >= b / 2.0 {
                    continue;
                }
                let (db, at) = pio_alias(carrier_hz, b);
                if best.is_none_or(|(d, ..)| db < d - 0.01) {
                    best = Some((db, at, fbdiv, post_div1, post_div2));
                }
            }
        }
    }
    let (alias_dbc, alias_hz, fbdiv, post_div1, post_div2) =
        best.ok_or_else(|| HalError::Config(format!("no bit rate for {:.3} MHz", carrier_hz / 1e6)))?;
    let b = xosc_hz * fbdiv as f64 / (post_div1 as f64 * post_div2 as f64);
    let words_per_s = b / 32.0;
    let sample_words =
        (words_per_s / sample_rate).round().clamp(1.0, (words_per_s / min_rate).floor().min(255.0)) as u8;
    let two32 = 2f64.powi(32);
    let ratio = (carrier_hz / b * two32).round() as u32;
    let scale = (max_offset_hz / i16::MAX as f64 * 32.0 / b * two32).ceil();
    if scale > PioConfig::SCALE_MAX as f64 {
        return Err(HalError::Config("the PIO method cannot reach that deviation".into()));
    }
    Ok(PioPlan {
        refdiv: 1,
        fbdiv,
        post_div1,
        post_div2,
        bit_rate: b,
        carrier_hz: ratio as f64 / two32 * b,
        ratio,
        scale: scale as u32,
        resolution_hz: scale * b / 32.0 / two32,
        sample_words,
        alias_dbc,
        alias_hz,
    })
}

impl PioPlan {
    pub fn sample_rate(&self) -> f64 {
        self.bit_rate / 32.0 / self.sample_words as f64
    }

    pub fn max_offset_hz(&self) -> f64 {
        i16::MAX as f64 * self.resolution_hz
    }

    /// Phase advance per word at the carrier for a crystal `ppm` fast (the
    /// bit rate is that much higher, so the advance that much smaller).
    pub fn center(&self, ppm: f64) -> u32 {
        let v = 32.0 * self.carrier_hz / (self.bit_rate * (1.0 + ppm * 1e-6));
        ((v - v.floor()) * 2f64.powi(32)).round() as u64 as u32
    }

    pub fn config(&self, gpio: u8, drive_ma: u8, interpolate: bool, ppm: f64) -> PioConfig {
        PioConfig {
            refdiv: self.refdiv,
            fbdiv: self.fbdiv,
            post_div1: self.post_div1,
            post_div2: self.post_div2,
            gpio,
            drive_ma,
            ratio: self.ratio,
            center: self.center(ppm),
            scale: self.scale,
            sample_words: self.sample_words,
            interpolate,
        }
    }
}

#[cfg(test)]
mod pio_tests {
    use super::*;

    #[test]
    fn plans_the_band() {
        let mut worst: f64 = -200.0;
        for k in (8750..=10800).step_by(5) {
            let f = k as f64 * 10e3;
            let p = pio_plan(f, 12e6, 100e3, 190e3, 124e3, (240e6, 264e6)).unwrap();
            assert!((p.carrier_hz - f).abs() < 0.1, "{f}");
            assert!(p.sample_rate() >= 124e3 && p.max_offset_hz() >= 100e3);
            worst = worst.max(p.alias_dbc);
        }
        // Choosing the bit rate keeps every alias at least this far down.
        assert!(worst < -24.0, "{worst}");
    }

    #[test]
    fn the_center_advances_the_carrier() {
        let p = pio_plan(97.3e6, 12e6, 100e3, 190e3, 124e3, (240e6, 264e6)).unwrap();
        // The advance per word is 32 bits of the table's carrier.
        let per_word = 32.0 * p.carrier_hz / p.bit_rate;
        let frac = (per_word - per_word.floor()) * 2f64.powi(32);
        assert!((p.center(0.0) as f64 - frac).abs() < 2.0);
        assert_eq!(p.center(0.0), p.ratio.wrapping_mul(32));
    }
}
