//! Audio sources: decoded files/streams, raw PCM, and generated signals.

use std::fs::File;
use std::io::Read;
use std::path::PathBuf;

use symphonia::core::codecs::audio::{AudioDecoder, AudioDecoderOptions};
use symphonia::core::errors::Error as SymError;
use symphonia::core::formats::probe::Hint;
use symphonia::core::formats::{FormatOptions, FormatReader, SeekMode, SeekTo, TrackType};
use symphonia::core::io::{MediaSourceStream, ReadOnlySource};
use symphonia::core::meta::MetadataOptions;
use symphonia::core::units::Time;

use crate::AudioError;

/// Which audio to transmit.
#[derive(Debug, Clone, PartialEq)]
pub enum InputSpec {
    /// An audio file in any format symphonia can decode (WAV, FLAC, MP3, OGG/Vorbis, AAC/M4A, ...).
    File(PathBuf),
    /// A container/encoded stream (e.g. WAV) on standard input.
    Stdin,
    /// Headerless signed 16-bit little-endian PCM on standard input.
    RawStdin { rate: u32, channels: u16 },
    /// No audio.
    Silence,
    /// Test tone (Hz, amplitude 0..1); `stereo` puts it on the left channel only.
    Tone { freq: f64, amplitude: f64, stereo: bool },
}

/// Properties of an opened source.
#[derive(Debug, Clone, PartialEq)]
pub struct SourceInfo {
    pub description: String,
    pub rate: u32,
    pub channels: usize,
    /// The source delivers data in real time (a live stream).
    pub live: bool,
    /// The source can be rewound.
    pub seekable: bool,
}

/// A decoded source producing interleaved `f32` frames.
pub(crate) trait Source: Send {
    fn info(&self) -> SourceInfo;
    /// Append decoded interleaved samples to `out`. Returns false at the end.
    fn read(&mut self, out: &mut Vec<f32>) -> Result<bool, AudioError>;
    /// Rewind to the start; false if not possible.
    fn rewind(&mut self) -> bool {
        false
    }
}

pub(crate) fn open(spec: &InputSpec) -> Result<Box<dyn Source>, AudioError> {
    Ok(match spec {
        InputSpec::File(path) => {
            let f = File::open(path).map_err(|e| AudioError::Io(path.display().to_string(), e))?;
            let mut hint = Hint::new();
            if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                hint.with_extension(ext);
            }
            let mss = MediaSourceStream::new(Box::new(f), Default::default());
            Box::new(Decoded::new(mss, hint, path.display().to_string(), false)?)
        }
        InputSpec::Stdin => {
            let src = ReadOnlySource::new(std::io::stdin());
            let mss = MediaSourceStream::new(Box::new(src), Default::default());
            Box::new(Decoded::new(mss, Hint::new(), "standard input".into(), true)?)
        }
        InputSpec::RawStdin { rate, channels } => {
            if *rate == 0 || *channels == 0 {
                return Err(AudioError::Unsupported("raw input needs a sample rate and channel count".into()));
            }
            Box::new(RawPcm { rate: *rate, channels: *channels as usize, buf: vec![0u8; 16384], carry: Vec::new() })
        }
        InputSpec::Silence => Box::new(Generated { kind: Gen::Silence, rate: 48_000, n: 0 }),
        InputSpec::Tone { freq, amplitude, stereo } => {
            Box::new(Generated { kind: Gen::Tone(*freq, *amplitude, *stereo), rate: 48_000, n: 0 })
        }
    })
}

/// Container formats and codecs via symphonia.
struct Decoded {
    format: Box<dyn FormatReader>,
    decoder: Box<dyn AudioDecoder>,
    track: u32,
    info: SourceInfo,
}

impl Decoded {
    fn new(mss: MediaSourceStream<'static>, hint: Hint, name: String, live: bool) -> Result<Decoded, AudioError> {
        let format = symphonia::default::get_probe()
            .probe(&hint, mss, FormatOptions::default(), MetadataOptions::default())
            .map_err(|e| AudioError::Unsupported(format!("{name}: {e}")))?;
        let track = format
            .default_track(TrackType::Audio)
            .ok_or_else(|| AudioError::Unsupported(format!("{name}: no audio track")))?;
        let params = track
            .codec_params
            .as_ref()
            .and_then(|p| p.audio())
            .ok_or_else(|| AudioError::Unsupported(format!("{name}: no audio codec parameters")))?
            .clone();
        let decoder = symphonia::default::get_codecs()
            .make_audio_decoder(&params, &AudioDecoderOptions::default())
            .map_err(|e| AudioError::Unsupported(format!("{name}: {e}")))?;
        let rate = params.sample_rate.ok_or_else(|| AudioError::Unsupported(format!("{name}: unknown sample rate")))?;
        let channels = params.channels.as_ref().map(|c| c.count()).unwrap_or(2);
        let track_id = track.id;
        Ok(Decoded {
            format,
            decoder,
            track: track_id,
            info: SourceInfo { description: name, rate, channels, live, seekable: !live },
        })
    }
}

impl Source for Decoded {
    fn info(&self) -> SourceInfo {
        self.info.clone()
    }

    fn read(&mut self, out: &mut Vec<f32>) -> Result<bool, AudioError> {
        loop {
            let packet = match self.format.next_packet() {
                Ok(Some(p)) => p,
                Ok(None) => return Ok(false),
                Err(SymError::IoError(e)) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(false),
                Err(e) => return Err(AudioError::Decode(e.to_string())),
            };
            if packet.track_id != self.track {
                continue;
            }
            match self.decoder.decode(&packet) {
                Ok(buf) => {
                    let ch = buf.spec().channels().count();
                    if ch != self.info.channels {
                        return Err(AudioError::Decode(format!(
                            "channel count changed from {} to {ch}",
                            self.info.channels
                        )));
                    }
                    let start = out.len();
                    out.resize(start + buf.samples_interleaved(), 0.0);
                    buf.copy_to_slice_interleaved(&mut out[start..]);
                    return Ok(true);
                }
                // Skip corrupt packets.
                Err(SymError::DecodeError(_)) | Err(SymError::IoError(_)) => continue,
                Err(e) => return Err(AudioError::Decode(e.to_string())),
            }
        }
    }

    fn rewind(&mut self) -> bool {
        if !self.info.seekable {
            return false;
        }
        let ok = self
            .format
            .seek(SeekMode::Accurate, SeekTo::Time { time: Time::default(), track_id: Some(self.track) })
            .is_ok();
        self.decoder.reset();
        ok
    }
}

/// Headerless S16LE on stdin (PiFmAdv's `--srate`/`--nochan`).
struct RawPcm {
    rate: u32,
    channels: usize,
    buf: Vec<u8>,
    carry: Vec<u8>,
}

impl Source for RawPcm {
    fn info(&self) -> SourceInfo {
        SourceInfo {
            description: "raw S16LE PCM on standard input".into(),
            rate: self.rate,
            channels: self.channels,
            live: true,
            seekable: false,
        }
    }

    fn read(&mut self, out: &mut Vec<f32>) -> Result<bool, AudioError> {
        let n = match std::io::stdin().lock().read(&mut self.buf) {
            Ok(0) => return Ok(false),
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => return Ok(true),
            Err(e) => return Err(AudioError::Io("stdin".into(), e)),
        };
        self.carry.extend_from_slice(&self.buf[..n]);
        let frame = 2 * self.channels;
        let usable = self.carry.len() / frame * frame;
        out.extend(self.carry[..usable].chunks_exact(2).map(|b| i16::from_le_bytes([b[0], b[1]]) as f32 / 32768.0));
        self.carry.drain(..usable);
        Ok(true)
    }
}

enum Gen {
    Silence,
    Tone(f64, f64, bool),
}

/// Generated test signals (not real time: paced by the buffer).
struct Generated {
    kind: Gen,
    rate: u32,
    n: u64,
}

impl Source for Generated {
    fn info(&self) -> SourceInfo {
        let (description, channels) = match self.kind {
            Gen::Silence => ("silence".to_string(), 1),
            Gen::Tone(f, a, st) => (format!("{f} Hz test tone at {a}{}", if st { " (left channel)" } else { "" }), if st { 2 } else { 1 }),
        };
        SourceInfo { description, rate: self.rate, channels, live: false, seekable: true }
    }

    fn read(&mut self, out: &mut Vec<f32>) -> Result<bool, AudioError> {
        const N: usize = 1024;
        match self.kind {
            Gen::Silence => out.extend(std::iter::repeat_n(0.0, N)),
            Gen::Tone(f, a, stereo) => {
                for i in 0..N as u64 {
                    let t = (self.n + i) as f64 / self.rate as f64;
                    let v = (a * (2.0 * std::f64::consts::PI * f * t).sin()) as f32;
                    out.push(v);
                    if stereo {
                        out.push(0.0);
                    }
                }
            }
        }
        self.n += N as u64;
        Ok(true)
    }
}
