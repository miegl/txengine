//! txengine – FM/RDS, NFM and AM transmitter for the Raspberry Pi.

mod am;
mod cli;
mod clock;
mod control;
mod engine;
mod hw;
mod logger;
mod mode;
mod nfm;
mod programme;
mod rt;
mod sinks;
mod wfm;

use clap::Parser;
use log::error;

use cli::{Cli, Command};

/// This build's version: `git describe` of its source tree (build.rs).
pub const VERSION: &str = env!("TXENGINE_VERSION");

fn main() {
    let cli = Cli::parse();
    logger::init(cli.verbose, cli.quiet);
    // Reporting commands end quietly when piped into e.g. `head`;
    // transmitting ones ignore SIGPIPE again (`rt::install_signal_handlers`).
    // SAFETY: resetting a signal disposition.
    unsafe { libc::signal(libc::SIGPIPE, libc::SIG_DFL) };
    let result = match &cli.command {
        Command::Wfm(c) => wfm::run(c),
        Command::Nfm(c) => nfm::run(c),
        Command::Am(c) => am::run(c),
        Command::Devices => hw::list_devices(),
    };
    if let Err(e) = result {
        error!("{e:#}");
        std::process::exit(1);
    }
}
