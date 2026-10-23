//! Raspberry Pi hardware backends for txengine.

pub mod am;
pub mod bcm;
pub mod boundary;
pub mod cpufreq;
pub mod error;
pub mod guardian;
pub mod mem;
pub mod mmio;
pub mod pico;
pub mod platform;
pub mod rp1;
pub mod sys;
pub mod usbfs;

pub use error::{HalError, Result};

/// Largest crystal error (± ppm) the frequency plans leave room to correct
/// at run time (`RfSink::set_clock_error_ppm`). Raspberry Pi crystals are
/// specified to a few tens of ppm.
pub const MAX_CLOCK_ERROR_PPM: f64 = 200.0;

/// The error for an FM carrier above the highest one an output makes
/// (`what`, up to `max_hz`), with the harmonic that reaches it: the
/// lowest odd one (a square wave's even harmonics are weak) whose
/// fundamental the output makes.
pub(crate) fn beyond_fundamental(carrier_hz: f64, max_hz: f64, what: &str) -> HalError {
    let n = (carrier_hz / max_hz).ceil() as u32 | 1;
    HalError::Config(format!(
        "{what} put out at most {:.0} MHz; for {:.4} MHz transmit on a harmonic: --harmonic {n} makes {:.4} MHz \
         (filter the fundamental out)",
        max_hz / 1e6,
        carrier_hz / 1e6,
        carrier_hz / n as f64 / 1e6
    ))
}
