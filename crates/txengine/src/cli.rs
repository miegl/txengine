//! Command line interface.

use std::path::PathBuf;

use clap::{Args, Parser, Subcommand, ValueEnum};

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
    /// Transmit.
    Tx(TxArgs),
    /// Render the signal to a file instead of transmitting (for testing).
    Render(RenderArgs),
    /// Show the detected hardware and the frequency plan, without transmitting.
    Probe(ProbeArgs),
    /// Measure an FM multiplex recording (WAV): levels, RDS decode.
    Analyze(AnalyzeArgs),
    /// Transmit test tones as raw frequency deviation (no audio processing,
    /// no compensation), e.g. to measure the modulation response.
    Tone(ToneArgs),
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

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum OnEnd {
    /// Start the file again from the beginning.
    Repeat,
    /// Keep transmitting (silence, RDS) after the input ends.
    Wait,
    /// Stop transmitting when the input ends.
    Stop,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum StereoArg {
    Auto,
    Mono,
    Stereo,
}

/// Programme, RDS and modulation options shared by `tx` and `render`.
#[derive(Debug, Clone, Args)]
pub struct ProgrammeArgs {
    /// Audio to transmit: a file (WAV, FLAC, MP3, OGG, AAC, ...) or '-' for
    /// standard input. Without it only the carrier and RDS are transmitted.
    #[arg(short, long, value_name = "FILE")]
    pub audio: Option<String>,
    /// Standard input is headerless S16LE PCM at this sample rate.
    #[arg(long, value_name = "HZ", requires = "raw_channels")]
    pub raw_rate: Option<u32>,
    /// Channels of the headerless PCM on standard input.
    #[arg(long, value_name = "N", requires = "raw_rate")]
    pub raw_channels: Option<u16>,
    /// What to do when the audio ends [default: repeat for files, wait for stdin].
    #[arg(long, value_enum)]
    pub on_end: Option<OnEnd>,
    /// Test tone instead of audio input (Hz).
    #[arg(long, value_name = "HZ", conflicts_with = "audio")]
    pub tone: Option<f64>,

    // RDS ------------------------------------------------------------------
    /// RDS on (1) or off (0).
    #[arg(long, default_value_t = 1, value_name = "0|1", help_heading = "RDS")]
    pub rds: u8,
    /// Programme identification code (4 hex digits).
    #[arg(long, default_value = "1234", help_heading = "RDS")]
    pub pi: String,
    /// Programme service name (max. 8 characters).
    #[arg(long, default_value = "txengine", help_heading = "RDS")]
    pub ps: String,
    /// RadioText (max. 64 characters).
    #[arg(long, default_value = "txengine - FM/RDS transmitter for the Raspberry Pi", help_heading = "RDS")]
    pub rt: String,
    /// Programme type: 0-31 or a name such as "Pop music".
    #[arg(long, default_value = "0", help_heading = "RDS")]
    pub pty: String,
    /// Traffic programme flag.
    #[arg(long, default_value_t = 0, value_name = "0|1", help_heading = "RDS")]
    pub tp: u8,
    /// Traffic announcement flag.
    #[arg(long, default_value_t = 0, value_name = "0|1", help_heading = "RDS")]
    pub ta: u8,
    /// Music (1) / speech (0).
    #[arg(long, default_value_t = 1, value_name = "0|1", help_heading = "RDS")]
    pub ms: u8,
    /// Alternative frequency in MHz (repeat for several, max. 25).
    #[arg(long, value_name = "MHZ", help_heading = "RDS")]
    pub af: Vec<f64>,
    /// Programme type name (max. 8 characters).
    #[arg(long, help_heading = "RDS")]
    pub ptyn: Option<String>,
    /// Extended country code (hex), sent in group 1A.
    #[arg(long, value_name = "HEX", help_heading = "RDS")]
    pub ecc: Option<String>,
    /// Language identification code (hex), sent in group 1A.
    #[arg(long, value_name = "HEX", help_heading = "RDS")]
    pub lic: Option<String>,
    /// Clock time and date (group 4A), from the system clock.
    #[arg(long, default_value_t = 1, value_name = "0|1", help_heading = "RDS")]
    pub ct: u8,
    /// Decoder identification bits d1-d3 as hex (d0/stereo is automatic).
    #[arg(long, default_value = "0", value_name = "HEX", help_heading = "RDS")]
    pub di: String,

    // Modulation -----------------------------------------------------------
    /// Maximum frequency deviation in Hz (75000 or 50000).
    #[arg(long = "deviation", default_value_t = 75_000.0, value_name = "HZ", help_heading = "Modulation")]
    pub deviation: f64,
    /// Peak deviation of the audio [default: what pilot and RDS leave of --deviation].
    #[arg(long, value_name = "HZ", help_heading = "Modulation")]
    pub audio_deviation: Option<f64>,
    /// Pilot tone deviation in Hz (BS.450: 8-10 % of 75 kHz).
    #[arg(long, default_value_t = 6_750.0, value_name = "HZ", help_heading = "Modulation")]
    pub pilot: f64,
    /// RDS injection in Hz (EN 50067: 1000-7500, 2000 recommended).
    #[arg(long, default_value_t = 2_000.0, value_name = "HZ", help_heading = "Modulation")]
    pub rds_level: f64,
    /// Pre-emphasis: eu (50 µs), us (75 µs), off, or a time constant in µs.
    #[arg(long, default_value = "eu", help_heading = "Modulation")]
    pub preemph: String,
    /// Audio low-pass cut-off in Hz.
    #[arg(long, default_value_t = 15_000.0, value_name = "HZ", help_heading = "Modulation")]
    pub cutoff: f64,
    /// Stereo coding.
    #[arg(long, value_enum, default_value = "auto", help_heading = "Modulation")]
    pub stereo: StereoArg,
    /// Same as --stereo mono.
    #[arg(long, help_heading = "Modulation")]
    pub mono: bool,
    /// Input gain in dB. With pre-emphasis, the level is already lowered by
    /// the rise of the multiplex power (ITU-R BS.412) the pre-emphasis
    /// causes for pink noise (4.3 dB at 50 µs, 6.8 dB at 75 µs), so a
    /// programme keeps about the MPX power it has without pre-emphasis.
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true, value_name = "DB", help_heading = "Modulation")]
    pub gain: f64,
    /// Disable the limiters (HF and peak); overload is then hard clipped.
    #[arg(long, help_heading = "Modulation")]
    pub no_limiter: bool,
    /// Do not pre-compensate the output stage's sample-and-hold roll-off.
    #[arg(long, help_heading = "Modulation")]
    pub no_hold_compensation: bool,
}

#[derive(Debug, Clone, Args)]
pub struct HardwareArgs {
    /// Carrier frequency in MHz.
    #[arg(short, long, default_value_t = 87.6, value_name = "MHZ")]
    pub freq: f64,
    /// Output GPIO (repeat for several pins on the same clock generator).
    #[arg(short, long = "gpio", value_name = "N", help_heading = "Hardware")]
    pub gpio: Vec<u32>,
    /// Carrier synthesis: PLL to modulate (Pi 1-4: a, c, d; Pi 5: audio, video)
    /// or "clkdiv" to modulate the clock divider instead of a PLL.
    #[arg(long, help_heading = "Hardware")]
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
    #[arg(long, help_heading = "Hardware")]
    pub dma_channel: Option<usize>,
    /// Pi 1-4: peripheral pacing the DMA (pwm or pcm).
    #[arg(long, help_heading = "Hardware")]
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

#[derive(Debug, Clone, Args)]
pub struct TxArgs {
    #[command(flatten)]
    pub hw: HardwareArgs,
    #[command(flatten)]
    pub prog: ProgrammeArgs,
    /// Control FIFO: one command per line (PS, RT, PTY, TA, ...); created if missing.
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

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum RenderFormat {
    /// Multiplex (instantaneous deviation / 100 kHz) as 32-bit float WAV.
    Mpx,
    /// Complex baseband FM signal, interleaved 32-bit float I/Q (.cf32).
    Iq,
    /// Discard (benchmark).
    Null,
}

#[derive(Debug, Clone, Args)]
pub struct RenderArgs {
    #[command(flatten)]
    pub prog: ProgrammeArgs,
    /// Output file.
    #[arg(short, long)]
    pub output: Option<PathBuf>,
    #[arg(long, value_enum, default_value = "mpx")]
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

#[derive(Debug, Clone, Args)]
pub struct AnalyzeArgs {
    /// WAV recording of an FM multiplex (sample rate >= 128 kHz).
    pub file: PathBuf,
    /// Deviation in Hz corresponding to a full-scale sample.
    #[arg(long, default_value_t = 100_000.0)]
    pub full_scale: f64,
    /// Unix time of the first sample (to check RDS clock-time accuracy).
    /// Read from `<file>.json` (`{"start_time": ...}`) if present.
    #[arg(long, value_name = "UNIX")]
    pub start_time: Option<f64>,
}
