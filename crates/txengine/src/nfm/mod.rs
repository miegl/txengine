//! The narrow-band FM mode: `txengine nfm tx | render | probe | tone`.

mod args;

pub use args::Command;

use crate::hw::Output;
use crate::mode;

/// Carrier frequency offset the transmitter is planned for: the largest
/// `--deviation`.
pub const MAX_OFFSET_HZ: f64 = txengine_nfm::baseband::MAX_DEVIATION;

pub fn run(cmd: &Command) -> anyhow::Result<()> {
    match cmd {
        Command::Tx(a) => mode::transmit(&a.hw, &a.ctl, &a.mode),
        Command::Render(a) => mode::render(&a.out, &a.mode),
        Command::Probe(a) => mode::probe(a, Output::Frequency { max_offset_hz: MAX_OFFSET_HZ }),
        Command::Tone(a) => mode::tone(a, MAX_OFFSET_HZ),
    }
}
