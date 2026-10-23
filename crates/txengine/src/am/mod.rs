//! The AM broadcast mode: `txengine am tx | render | probe`.

mod args;

pub use args::Command;

use crate::hw::Output;
use crate::mode;

pub fn run(cmd: &Command) -> anyhow::Result<()> {
    match cmd {
        Command::Tx(a) => mode::transmit(&a.hw, &a.ctl, &a.mode),
        Command::Render(a) => mode::render(&a.out, &a.mode),
        Command::Probe(a) => mode::probe(a, Output::Amplitude),
    }
}
