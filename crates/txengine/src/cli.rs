//! Command line interface: a command per transmission mode, each with its
//! own subcommands (`txengine wfm tx`), and tools that need no mode.
//!
//! Every mode transmits with [`Tx`] and renders to a file with [`Render`],
//! which add the options shared by all modes (hardware, control, file
//! output) to the mode's own.

use std::path::PathBuf;

use clap::{Args, Parser, Subcommand, ValueEnum};

use crate::wfm;

#[derive(Debug, Parser)]
#[command(
    name = "txengine",
    version,
    about = "FM/RDS transmitter for the Raspberry Pi (2, 3, 4, 5, Zero 2 W)",
    long_about = "FM/RDS transmitter for the Raspberry Pi (2, 3, 4, 5, Zero 2 W).\n\n\
        Transmitting without a licence is illegal in most countries. Connect the output to a \
        receiver with a shielded cable and attenuator, or use a dummy load, and always low-pass \
        filter the square-wave output before it reaches an antenna."
)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Command,
    /// More output (-v debug, -vv trace).
    #[arg(short, long, action = clap::ArgAction::Count, global = true)]
    pub verbose: u8,
    /// Only warnings and errors.
    #[arg(short, long, global = true)]
    pub quiet: bool,
}

#[derive(Debug, Subcommand)]
pub enum Command {
    /// Wide-band FM broadcast: stereo multiplex and RDS.
    #[command(subcommand)]
    Wfm(Box<wfm::Command>),
    /// Show the detected hardware and the frequency plan, without transmitting.
    Probe(ProbeArgs),
    /// Transmit test tones as raw frequency deviation (no audio processing,
    /// no compensation), e.g. to measure the modulation response.
    Tone(ToneArgs),
    /// List the transmitters found (for --device).
    Devices,
}

/// How the crystal error is corrected.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Ppm {
    /// Measured while transmitting.
    Auto,
    /// Given (ppm, positive when the crystal runs fast); still measured,
    /// for information.
    Fixed(f64),
    /// Neither corrected nor measured.
    Off,
}

pub(crate) fn parse_ppm(s: &str) -> Result<Ppm, String> {
    if s.eq_ignore_ascii_case("auto") {
        return Ok(Ppm::Auto);
    }
    if s.eq_ignore_ascii_case("off") {
        return Ok(Ppm::Off);
    }
    let v: f64 = s.parse().map_err(|_| format!("expected a number of ppm, 'auto' or 'off', got '{s}'"))?;
    let max = txengine_hal::MAX_CLOCK_ERROR_PPM;
    if !(-max..=max).contains(&v) {
        return Err(format!("the crystal error must be within ±{max} ppm"));
    }
    Ok(Ppm::Fixed(v))
}

#[derive(Debug, Clone, Args)]
pub struct HardwareArgs {
    /// Carrier frequency in MHz.
    #[arg(short, long, value_name = "MHZ")]
    pub freq: f64,
    /// Transmitter: "pi" (this Raspberry Pi's GPIOs) or "auto": the only
    /// one found. `txengine devices` lists them.
    #[arg(short, long, value_name = "DEVICE", help_heading = "Hardware")]
    pub device: Option<String>,
    /// Output GPIO (repeat for several pins on the same clock generator).
    #[arg(short, long = "gpio", value_name = "N", help_heading = "Hardware")]
    pub gpio: Vec<u32>,
    /// Carrier synthesis: PLL to modulate (Pi 1-4: a, c, d; Pi 5: audio, video)
    /// or "clkdiv" to modulate the clock divider instead of a PLL.
    #[arg(long, help_heading = "Raspberry Pi")]
    pub pll: Option<String>,
    /// Output drive strength 0-7 (2-16 mA; the Pi 5 has 4 steps up to 12 mA).
    #[arg(long, default_value_t = 7, value_name = "0-7", help_heading = "Hardware")]
    pub power: u8,
    /// Crystal frequency error to correct: "auto" measures it continuously
    /// against the system clock, which must be synchronised (NTP, or GPS
    /// through chrony); a number gives it in ppm (positive when the crystal
    /// runs fast); "off" neither measures nor corrects. The correction
    /// applies to the carrier frequency, the subcarriers and the playback
    /// speed.
    #[arg(long, default_value = "auto", value_parser = parse_ppm, allow_hyphen_values = true, value_name = "auto|PPM|off", help_heading = "Hardware")]
    pub ppm: Ppm,
    /// DMA channel to use [default: a free one].
    #[arg(long, help_heading = "Raspberry Pi")]
    pub dma_channel: Option<usize>,
    /// Pi 1-4: peripheral pacing the DMA (pwm or pcm).
    #[arg(long, help_heading = "Raspberry Pi")]
    pub pacer: Option<String>,
    /// Target sample rate of the output in Hz (the closest exact rate is
    /// used). Default: 250000 on the Pi 5; on the Pi 1-4 the highest multiple
    /// of 38000 the DMA engine sustains on the board (measured at start-up),
    /// at most 250000: 190000 on the Pi 2, 3 and Zero 2 W.
    #[arg(long, value_name = "HZ", help_heading = "Hardware")]
    pub sample_rate: Option<f64>,
    /// Output buffer (transmit latency) in milliseconds.
    #[arg(long, default_value_t = 60, value_name = "MS", help_heading = "Hardware")]
    pub buffer_ms: u64,
}

/// `<mode> tx`: transmit with the mode's options `M`.
#[derive(Debug, Clone, Args)]
pub struct Tx<M: Args> {
    #[command(flatten)]
    pub hw: HardwareArgs,
    #[command(flatten)]
    pub mode: M,
    #[command(flatten)]
    pub ctl: ControlArgs,
}

#[derive(Debug, Clone, Args)]
pub struct ControlArgs {
    /// Control FIFO: one command per line (the mode's settings, PPM, QUIT,
    /// ...); created if missing.
    #[arg(short = 'C', long, value_name = "PATH", help_heading = "Control")]
    pub ctl: Option<PathBuf>,
    /// Control socket (Unix stream): commands get a reply line.
    #[arg(long, value_name = "PATH", help_heading = "Control")]
    pub ctl_socket: Option<PathBuf>,
    /// Stop after this many seconds.
    #[arg(long, value_name = "S", help_heading = "Control")]
    pub duration: Option<f64>,
    /// Seconds between status lines (0 = never).
    #[arg(long, default_value_t = 10.0, value_name = "S", help_heading = "Control")]
    pub status_interval: f64,
    /// SCHED_FIFO priority of the transmit thread (0 = normal scheduling).
    #[arg(long, default_value_t = 30, value_name = "N", help_heading = "Control")]
    pub rt_priority: i32,
}

/// `<mode> render`: render the mode's signal (options `M`) to a file.
#[derive(Debug, Clone, Args)]
pub struct Render<M: Args> {
    #[command(flatten)]
    pub mode: M,
    #[command(flatten)]
    pub out: OutputArgs,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum RenderFormat {
    /// Instantaneous frequency deviation / 100 kHz as 32-bit float WAV (in
    /// WFM, the multiplex).
    #[value(alias = "mpx")]
    Deviation,
    /// Complex baseband signal, interleaved 32-bit float I/Q (.cf32).
    Iq,
    /// Discard (benchmark).
    Null,
}

#[derive(Debug, Clone, Args)]
pub struct OutputArgs {
    /// Output file.
    #[arg(short, long)]
    pub output: Option<PathBuf>,
    #[arg(long, value_enum, default_value = "deviation")]
    pub format: RenderFormat,
    /// Sample rate in Hz.
    #[arg(long, default_value_t = 250_000.0)]
    pub sample_rate: f64,
    /// Length in seconds.
    #[arg(long, default_value_t = 10.0)]
    pub duration: f64,
    /// Model a zero-order-hold output (as the hardware does).
    #[arg(long)]
    pub hold: bool,
    /// Unix time of the first rendered sample [default: now].
    #[arg(long, value_name = "UNIX")]
    pub start_time: Option<f64>,
}

#[derive(Debug, Clone, Args)]
pub struct ProbeArgs {
    #[command(flatten)]
    pub hw: HardwareArgs,
}

#[derive(Debug, Clone, Args)]
pub struct ToneArgs {
    #[command(flatten)]
    pub hw: HardwareArgs,
    /// Tone frequencies in Hz (comma separated), each at --deviation.
    #[arg(long, value_delimiter = ',', default_value = "1000", value_name = "HZ")]
    pub tones: Vec<f64>,
    /// Peak deviation of each tone in Hz.
    #[arg(long, default_value_t = 5_000.0, value_name = "HZ")]
    pub deviation: f64,
    /// Seconds to transmit.
    #[arg(long, default_value_t = 10.0, value_name = "S")]
    pub duration: f64,
}

#[cfg(test)]
mod tests {
    use clap::CommandFactory;

    use super::*;

    /// Every command combines shared options with a mode's own; clap
    /// checks them for clashes only when a command is used.
    #[test]
    fn commands_are_consistent() {
        Cli::command().debug_assert();
    }
}
