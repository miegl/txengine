//! Command line options of the WFM mode.

use std::path::PathBuf;

use clap::{Args, Subcommand, ValueEnum};

use crate::cli::{Render, Tx};

// Parsed once at start-up: the size of the variants does not matter.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Subcommand)]
pub enum Command {
    /// Transmit.
    Tx(Tx<WfmArgs>),
    /// Render the signal to a file instead of transmitting (for testing).
    Render(Render<WfmArgs>),
    /// Measure an FM multiplex recording (WAV): levels, RDS decode.
    Analyze(AnalyzeArgs),
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

// Programme, RDS and modulation options of `wfm tx` and `wfm render`.
#[derive(Debug, Clone, Args)]
pub struct WfmArgs {
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
    /// What to do when the audio ends [default: repeat for files, wait for
    /// stdin; stop when rendering].
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
