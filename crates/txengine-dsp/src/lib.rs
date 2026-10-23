//! Signal processing building blocks used by txengine's transmission modes.
//!
//! Everything here is allocation free after construction and suitable for
//! the real-time transmit path.

pub mod fir;
pub mod iir;
pub mod limiter;
pub mod nco;
pub mod optimize;
pub mod preemphasis;
pub mod window;

pub use fir::{Fir, FirDesign, LinearPhaseFir, PolyphaseInterpolator};
pub use iir::Biquads;
pub use limiter::{LookaheadGain, PeakLimiter, PreemphasisLimiter};
pub use nco::{PhaseAccumulator, SineTable};
pub use preemphasis::Preemphasis;

/// `max` without NaN handling: `f32::max` is a library call on 32-bit ARM.
#[inline(always)]
pub fn fmax(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}

/// `min` without NaN handling (see [`fmax`]).
#[inline(always)]
pub fn fmin(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

/// Magnitude in dB of a linear amplitude ratio.
pub fn db(x: f64) -> f64 {
    20.0 * x.abs().log10()
}

/// Linear amplitude ratio of a value in dB.
pub fn from_db(db: f64) -> f64 {
    10f64.powf(db / 20.0)
}

/// Normalised sinc, `sin(pi x) / (pi x)`.
pub fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-12 {
        1.0
    } else {
        let px = std::f64::consts::PI * x;
        px.sin() / px
    }
}
