//! Radio Data System (EN 50067 / IEC 62106) for txengine.
//!
//! * [`encoder`] – station data and group generation (PS, RT, PTY, TP/TA,
//!   MS, DI, AF method A, CT, PTYN, ECC/LIC) with a weighted scheduler.
//! * [`modulator`] – differential coding, biphase symbols and the specified
//!   data-channel spectrum shaping, generated at any sample rate.
//! * [`decoder`] – an independent demodulator/decoder used to verify the
//!   transmitted signal (tests and the `txengine wfm analyze` tool).

pub mod charset;
pub mod decoder;
pub mod encoder;
pub mod group;
pub mod modulator;
pub mod pty;

pub use encoder::{DecoderInfo, RdsEncoder, RdsError, RdsParams};
pub use group::{Group, GroupType};
pub use modulator::RdsModulator;
