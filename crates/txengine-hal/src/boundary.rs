//! Where a fractional-N PLL puts a spur next to its carrier.
//!
//! When the VCO, or on the Pico the carrier itself, is close to certain
//! multiples of the frequency the PLL's phase detector compares at, the
//! PLL makes a spur offset from the carrier by that distance (an
//! "integer-boundary spur"). The modulation moves the carrier, and with it
//! the spur: while the spur lies in the audio band it distorts quiet
//! passages, by some 10–30 Hz of deviation. Measured with a 1 kHz tone at
//! 1.25 kHz deviation near 145 MHz, the worst of the tone's harmonics:
//!
//! * Raspberry Pi 2, 3, Zero 2 W (19.2 MHz): −42 dB with the VCO at a half
//!   multiple, −50 dB at a quarter. The Raspberry Pi 4 (54 MHz) showed
//!   none: below −76 dB at a half, a quarter, and 1.4 MHz from a whole
//!   multiple.
//! * Raspberry Pi 5 (50 MHz): −51 dB at a half, −60 dB at a quarter.
//! * Pico, Pico 2 with the 6 MHz reference (the crystal / 2): −33 to
//!   −42 dB with the VCO at a multiple of 3 MHz (a half), or the carrier at
//!   one of 2 MHz or 1.5 MHz (a third or a quarter: whatever the
//!   division); −53 to −59 dB at other thirds, quarters, fifths and sixths.
//!   With the 12 MHz reference: −38 to −43 dB with the VCO at a multiple of
//!   3 MHz or 2 MHz. At 145.5 MHz, on a multiple of 1.5 MHz, it reads
//!   −59 dB (the 6 MHz reference −41 dB), but not everywhere: at 148.5 MHz
//!   −46 dB with either. The Pico's PLL has more, weaker points than these
//!   rules know.
//!
//! One 12.5 kHz channel away every one of them stayed below −66 dB. In
//! wide-band FM the spur's few hertz are about at the noise floor (at
//! 90.0 MHz, on a multiple of 3 MHz for every division, a Pico 2 put the
//! second harmonic of a tone 30 dB below full scale at −39 dB, −69 dB re
//! full scale), but narrow-band FM deviates thirty times less. Dithering
//! the Pico's modulator does not help: the spur comes from the PLL itself.
//!
//! The planners therefore prefer divisions (and the Pico a reference
//! divider) that keep the carrier clear of these points.

/// Where a PLL makes a spur next to its carrier: the VCO, or the carrier,
/// at a multiple of one of these spacings (Hz).
#[derive(Debug, Clone, Copy)]
pub struct Spurs<'a> {
    pub vco: &'a [f64],
    pub carrier: &'a [f64],
}

impl Spurs<'_> {
    /// How far `carrier_hz` is from these points, for a VCO `division`
    /// times the carrier: the carrier offset (Hz) at which the nearest is
    /// reached.
    pub fn clearance_hz(&self, carrier_hz: f64, division: f64) -> f64 {
        let to = |f: f64, g: f64| {
            let t = f / g;
            (t - t.round()).abs() * g
        };
        let vco = self.vco.iter().map(|&g| to(carrier_hz * division, g) / division);
        let carrier = self.carrier.iter().map(|&g| to(carrier_hz, g));
        vco.chain(carrier).fold(f64::INFINITY, f64::min)
    }
}

/// The clearance the planners aim for, for a carrier that moves by up to
/// ±`max_offset_hz`: room for quiet passages (an eighth of the largest
/// offset: wide-band FM's pilot and RDS, narrow-band FM's sub-audio
/// signalling) and for the crystal correction, which moves the VCO
/// relative to the reference (30 ppm, a crystal's usual tolerance).
pub fn target_hz(carrier_hz: f64, max_offset_hz: f64) -> f64 {
    max_offset_hz / 8.0 + carrier_hz * 30e-6
}

/// How planners rank a plan by its clearance, higher first: up to
/// `target_hz`, rounded to whole hertz (equal clearances rank equal).
pub fn rank(clearance_hz: f64, target_hz: f64) -> f64 {
    clearance_hz.min(target_hz).round()
}

/// A warning when the quiet passages of a modulation up to
/// ±`max_offset_hz` reach a spur of the PLL `pll`: the planner found no
/// plan clear of one.
pub fn warning(clearance_hz: f64, carrier_hz: f64, max_offset_hz: f64, pll: &str) -> Option<String> {
    (clearance_hz < max_offset_hz / 8.0).then(|| {
        format!(
            "at {:.4} MHz the carrier is {:.0} Hz from a point where {pll} makes a spur next to it: the spur \
             distorts quiet passages (by 10-30 Hz of deviation, measured)",
            carrier_hz / 1e6,
            clearance_hz
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distances_at_the_carrier() {
        let vco = Spurs { vco: &[3e6], carrier: &[] };
        // A VCO at 1455 MHz, 242.5 x 6 MHz, ten times 145.5 MHz.
        assert!(vco.clearance_hz(145.5e6, 10.0).abs() < 1e-6);
        // 12.5 kHz up: the VCO 125 kHz from 1455 MHz.
        assert!((vco.clearance_hz(145.5125e6, 10.0) - 12.5e3).abs() < 1e-3);
        // Division 9: 1309.5 MHz, 1.5 MHz from 1308 and 1311 MHz.
        assert!((vco.clearance_hz(145.5e6, 9.0) - 1.5e6 / 9.0).abs() < 1e-3);
        // ... but the carrier is a multiple of 1.5 MHz.
        let both = Spurs { vco: &[3e6], carrier: &[1.5e6] };
        assert!(both.clearance_hz(145.5e6, 9.0).abs() < 1e-3);
        assert!((both.clearance_hz(145.5125e6, 9.0) - 12.5e3).abs() < 1e-3);
    }

    #[test]
    fn target_and_warning() {
        assert!((target_hz(145.5e6, 10e3) - (1250.0 + 4365.0)).abs() < 1e-6);
        assert_eq!(rank(12_345.4, 20e3), 12_345.0);
        assert_eq!(rank(1e6, 20e3), 20e3);
        let w = warning(0.0, 145.5e6, 10e3, "PLL_USB").unwrap();
        assert!(w.contains("at 145.5000 MHz the carrier is 0 Hz"), "{w}");
        assert!(warning(1250.0, 145.5e6, 10e3, "PLL_USB").is_none());
    }
}
