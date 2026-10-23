//! Command line interface: a command per transmission mode, each with its
//! own subcommands (`txengine wfm tx`), and tools that need no mode.
//!
//! Every mode transmits with [`Tx`] and renders to a file with [`Render`],
//! which add the options shared by all modes (hardware, control, file
//! output) to the mode's own.

use std::path::PathBuf;

use clap::{Args, Parser, Subcommand, ValueEnum};

use crate::{am, nfm, wfm};

#[derive(Debug, Parser)]
#[command(
    name = "txengine",
    version = crate::VERSION,
    about = "FM/RDS, NFM and AM transmitter for the Raspberry Pi (2, 3, 4, 5, Zero 2 W) and the Pico on USB",
    long_about = "FM/RDS, NFM and AM transmitter for the Raspberry Pi (2, 3, 4, 5, Zero 2 W) and the Pico on USB.\n\n\
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
    /// Narrow-band FM: voice with CTCSS or DCS, from short wave and CB to
    /// UHF.
    #[command(subcommand)]
    Nfm(Box<nfm::Command>),
    /// AM broadcast (double sideband, full carrier) on long, medium and
    /// short wave.
    #[command(subcommand)]
    Am(Box<am::Command>),
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

/// A frequency in MHz, or with a unit: `87.6`, `87.6M`, `999k`, `999kHz`,
/// `6100000Hz`. Returns Hz.
pub(crate) fn parse_frequency(s: &str) -> Result<f64, String> {
    let t = s.trim();
    let lower = t.to_ascii_lowercase();
    let (num, scale) = if let Some(n) = lower.strip_suffix("mhz").or_else(|| lower.strip_suffix('m')) {
        (n, 1e6)
    } else if let Some(n) = lower.strip_suffix("khz").or_else(|| lower.strip_suffix('k')) {
        (n, 1e3)
    } else if let Some(n) = lower.strip_suffix("hz") {
        (n, 1.0)
    } else {
        (lower.as_str(), 1e6)
    };
    let v: f64 = num
        .trim()
        .parse()
        .map_err(|_| format!("expected a frequency in MHz or with a unit (87.6, 999k, 6.1M), got '{s}'"))?;
    if !(v.is_finite() && v > 0.0) {
        return Err(format!("the frequency must be positive, got '{s}'"));
    }
    Ok(v * scale)
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
    /// Carrier frequency: MHz, or with a unit (87.6, 999k, 6.1M,
    /// 446.00625).
    #[arg(short = 'f', long = "freq", value_name = "FREQ", value_parser = parse_frequency)]
    pub freq_hz: f64,
    /// Transmitter: "pi" (this Raspberry Pi's GPIOs), "pico" (a Raspberry
    /// Pi Pico or Pico 2 on USB with the txengine firmware; pico:SERIAL or
    /// pico:PORT picks one of several), or "auto": the only one found.
    /// `txengine devices` lists them.
    #[arg(short, long, value_name = "DEVICE", help_heading = "Hardware")]
    pub device: Option<String>,
    /// Output GPIO (repeat for several pins on the same clock generator)
    /// [default: 4; AM: 18; Pico: 21].
    #[arg(short, long = "gpio", value_name = "N", help_heading = "Hardware")]
    pub gpio: Vec<u32>,
    /// Transmit on a clock generator that is already running, taking it over
    /// from whatever uses it until txengine stops (FM and NFM): on the Pi 2
    /// GPCLK1 (GPIO 5, 21) clocks the Ethernet and USB chip, on the Zero 2 W
    /// GPCLK2 (GPIO 6) the WiFi chip. Without it txengine refuses such a pin.
    #[arg(long, help_heading = "Raspberry Pi")]
    pub force_gpclk: bool,
    /// PLL that makes the carrier: in WFM and NFM the one to modulate (Pi
    /// 1-4: a, c, d, or "clkdiv" to modulate the clock divider instead; Pi
    /// 5: audio, video); in AM on the Pi 1-4 the one that clocks the PWM (a,
    /// c, d).
    #[arg(long, help_heading = "Raspberry Pi")]
    pub pll: Option<String>,
    /// Transmit on the Nth harmonic of the carrier the hardware makes (FM
    /// and NFM): it makes FREQ / N, and the modulation is scaled to it. For
    /// carriers above what the board makes directly (400 MHz on the Pi 2, 3
    /// and Zero 2 W, 300 MHz on the Pi 4, 1 GHz on the Pi 5 and the Pico).
    /// The fundamental is far stronger than the wanted harmonic: band-pass
    /// filter the output at FREQ.
    #[arg(long, default_value_t = 1, value_name = "N", help_heading = "Hardware")]
    pub harmonic: u32,
    /// Output drive strength 0-7 (2-16 mA; the Pi 5 and the Pico have 4
    /// steps up to 12 mA).
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
    /// Pico, WFM and NFM: how the carrier is made. "pll": PLL_USB's
    /// feedback divider, dithered (any of the clock-output GPIOs). "pio": a
    /// PIO bit stream at 240-264 Mbit/s (the Pico's system clock is raised
    /// for it, and its core voltage to 1.20 V; any GPIO; carriers below half
    /// the bit rate): less noise next to the carrier, but the bit stream
    /// puts lines around it, in the FM band measured up to about -15 dBc
    /// (txengine picks the bit rate that keeps them lowest). AM is always
    /// made by the PIO, one pulse per carrier cycle at the raised clock.
    #[arg(long, value_name = "pll|pio", help_heading = "Raspberry Pi Pico")]
    pub synthesis: Option<String>,
    /// DMA channel to use [default: a free one].
    #[arg(long, help_heading = "Raspberry Pi")]
    pub dma_channel: Option<usize>,
    /// Pi 1-4: peripheral pacing the DMA (pwm or pcm).
    #[arg(long, help_heading = "Raspberry Pi")]
    pub pacer: Option<String>,
    /// Target sample rate of the output in Hz (the closest exact rate is
    /// used). Default: 250000 on the Pi 5; on the Pi 1-4 the highest multiple
    /// of 38000 the DMA engine sustains on the board (measured at start-up),
    /// at most 250000: 190000 on the Pi 2, 3 and Zero 2 W; 166667 on the
    /// Pico (about 190000 with --synthesis pio). AM: 100000 (each sample a
    /// whole number of carrier cycles).
    #[arg(long, value_name = "HZ", help_heading = "Hardware")]
    pub sample_rate: Option<f64>,
    /// Pico: when the feedback divider is written, in system clock cycles
    /// after the reference edge (for measurements).
    #[arg(long, hide = true)]
    pub pico_phase: Option<u8>,
    /// Pico: PLL reference divider (for measurements) [default: 2, in
    /// narrow-band FM 1 where only that keeps the carrier clear of the
    /// PLL's spurs].
    #[arg(long, hide = true)]
    pub pico_refdiv: Option<u8>,
    /// Pico: sigma-delta update rate in Hz (for measurements).
    #[arg(long, hide = true)]
    pub pico_update_rate: Option<f64>,
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
    /// The modulation as 32-bit float WAV: in WFM the frequency deviation /
    /// 100 kHz (the multiplex), in NFM / 10 kHz, in AM the envelope (1 =
    /// the transmitter's full output).
    Wav,
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
    #[arg(long, value_enum, default_value = "wav")]
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

/// A transmission mode, for commands that need no mode's options.
#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum ModeName {
    Wfm,
    Nfm,
    Am,
}

#[derive(Debug, Clone, Args)]
pub struct ProbeArgs {
    #[command(flatten)]
    pub hw: HardwareArgs,
    /// The transmission mode to plan for.
    #[arg(long, value_enum, default_value = "wfm")]
    pub mode: ModeName,
}

#[derive(Debug, Clone, Args)]
pub struct ToneArgs {
    #[command(flatten)]
    pub hw: HardwareArgs,
    /// The transmission mode to plan the transmitter for (wfm or nfm), so
    /// that its plan applies to the measurement.
    #[arg(long, value_enum, default_value = "wfm")]
    pub mode: ModeName,
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

    #[test]
    fn frequencies() {
        assert_eq!(parse_frequency("87.6"), Ok(87.6e6));
        assert_eq!(parse_frequency("87.6M"), Ok(87.6e6));
        assert_eq!(parse_frequency("999k"), Ok(999e3));
        assert_eq!(parse_frequency("999 kHz"), Ok(999e3));
        assert_eq!(parse_frequency("6.1MHz"), Ok(6.1e6));
        assert_eq!(parse_frequency("153000Hz"), Ok(153e3));
        assert!(parse_frequency("fast").is_err());
        assert!(parse_frequency("-1").is_err());
    }
}
