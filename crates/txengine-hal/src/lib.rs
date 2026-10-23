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
