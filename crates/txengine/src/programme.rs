//! The programme input of every mode: its command line options and opening
//! the input.

use std::path::PathBuf;
use std::time::Duration;

use clap::{Args, ValueEnum};
use txengine_audio::{AudioInput, AudioOptions, EndOfInput, InputSpec, Programme};

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum OnEnd {
    /// Start the file again from the beginning.
    Repeat,
    /// Keep transmitting (the carrier, in WFM also RDS) after the input ends.
    Wait,
    /// Stop transmitting when the input ends.
    Stop,
}

#[derive(Debug, Clone, Args)]
pub struct ProgrammeArgs {
    /// Audio to transmit: a file (WAV, FLAC, MP3, OGG, AAC, ...) or '-' for
    /// standard input. Without it only the carrier (in WFM also RDS) is
    /// transmitted.
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
    /// Test tone instead of audio input (Hz), at half full scale.
    #[arg(long, value_name = "HZ", conflicts_with = "audio")]
    pub tone: Option<f64>,
}

fn end_policy(p: &ProgrammeArgs, stdin: bool, realtime: bool) -> EndOfInput {
    if let Some(e) = p.on_end {
        return match e {
            OnEnd::Repeat => EndOfInput::Repeat,
            OnEnd::Wait => EndOfInput::Wait,
            OnEnd::Stop => EndOfInput::Stop,
        };
    }
    // A rendered file ends with the input.
    if !realtime {
        EndOfInput::Stop
    } else if stdin {
        EndOfInput::Wait
    } else {
        EndOfInput::Repeat
    }
}

/// Open the programme input at the modulator's audio rate. `realtime`:
/// for a transmitter, whose crystal error is corrected while running.
pub fn open(p: &ProgrammeArgs, audio_rate: f64, realtime: bool) -> anyhow::Result<Programme> {
    let (spec, stdin) = match (&p.audio, p.tone) {
        (_, Some(f)) => (InputSpec::Tone { freq: f, amplitude: 0.5, stereo: false }, false),
        (None, None) => return Ok(Programme::None),
        (Some(a), None) if a == "-" => match (p.raw_rate, p.raw_channels) {
            (Some(rate), Some(channels)) => (InputSpec::RawStdin { rate, channels }, true),
            _ => (InputSpec::Stdin, true),
        },
        (Some(a), None) => (InputSpec::File(PathBuf::from(a)), false),
    };
    let end = end_policy(p, stdin, realtime);
    let opts =
        AudioOptions { end, buffer: Duration::from_millis(500), adjustable_rate: realtime, ..Default::default() };
    let input = AudioInput::open(&spec, audio_rate, &opts)?;
    Ok(Programme::Audio(input, end))
}
