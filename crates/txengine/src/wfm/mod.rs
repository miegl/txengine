//! The wide-band FM broadcast mode: `txengine wfm tx | render | analyze`.

mod analyze;
mod args;
mod programme;

pub use args::Command;

use crate::mode;

/// Carrier frequency offset the transmitter is planned for: the largest
/// `--deviation`.
pub const MAX_OFFSET_HZ: f64 = 100_000.0;

pub fn run(cmd: &Command) -> anyhow::Result<()> {
    match cmd {
        Command::Tx(a) => mode::transmit(&a.hw, &a.ctl, &a.mode),
        Command::Render(a) => mode::render(&a.out, &a.mode),
        Command::Analyze(a) => analyze::run(a),
    }
}
