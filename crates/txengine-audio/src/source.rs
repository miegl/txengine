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
            let mut stdin = std::io::stdin();
            let head = unbounded_wav_header(&mut stdin).map_err(|e| AudioError::Io("standard input".into(), e))?;
            let src = ReadOnlySource::new(std::io::Cursor::new(head).chain(stdin));
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

/// Read the header of a WAV stream from `r` and mark its RIFF and data
/// lengths as unknown (0xFFFFFFFF, what ffmpeg writes to a pipe), so that
/// the data is read until the stream ends. A live stream has no length:
/// servers declare some large number (or 0), and the reader would stop
/// there and leave the transmitter silent, hours into a broadcast. Returns
/// the bytes read, to be read again before the rest of `r`; anything that
/// is not WAV is returned unchanged.
fn unbounded_wav_header(r: &mut impl Read) -> std::io::Result<Vec<u8>> {
    // Up to `n` more bytes from `r` onto `buf`; false at the end of `r`.
    fn take(r: &mut impl Read, n: u64, buf: &mut Vec<u8>) -> std::io::Result<bool> {
        let start = buf.len();
        r.take(n).read_to_end(buf)?;
        Ok((buf.len() - start) as u64 == n)
    }
    let mut head = Vec::new();
    if !take(r, 12, &mut head)? || &head[0..4] != b"RIFF" || &head[8..12] != b"WAVE" {
        return Ok(head);
    }
    head[4..8].fill(0xff);
    // Chunks up to the data chunk (format, metadata).
    loop {
        let at = head.len();
        if !take(r, 8, &mut head)? {
            return Ok(head);
        }
        if &head[at..at + 4] == b"data" {
            head[at + 4..at + 8].fill(0xff);
            return Ok(head);
        }
        let len = u32::from_le_bytes(head[at + 4..at + 8].try_into().unwrap()) as u64;
        // A chunk this large is no header: leave it to the reader.
        if len > 1 << 20 || !take(r, len + (len & 1), &mut head)? {
            return Ok(head);
        }
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A 16-bit mono WAV stream with `frames` frames whose header declares
    /// `declared` of them, with an odd-sized chunk (padded) before the data.
    fn wav_stream(declared: u32, frames: u32) -> Vec<u8> {
        let mut w = b"RIFF".to_vec();
        w.extend((46 + 2 * declared).to_le_bytes());
        w.extend(b"WAVEfmt ");
        w.extend(16u32.to_le_bytes());
        for v in [1u16, 1] {
            w.extend(v.to_le_bytes()); // PCM, mono
        }
        w.extend(48_000u32.to_le_bytes());
        w.extend(96_000u32.to_le_bytes());
        for v in [2u16, 16] {
            w.extend(v.to_le_bytes()); // block align, bits
        }
        w.extend(b"junk");
        w.extend(1u32.to_le_bytes());
        w.extend([0, 0]);
        w.extend(b"data");
        w.extend((2 * declared).to_le_bytes());
        for i in 0..frames {
            w.extend((i as i16).to_le_bytes());
        }
        w
    }

    /// Frames decoded from `stream` read as standard input is.
    fn decode_stream(stream: Vec<u8>) -> usize {
        let mut r = std::io::Cursor::new(stream);
        let head = unbounded_wav_header(&mut r).unwrap();
        let src = ReadOnlySource::new(std::io::Cursor::new(head).chain(r));
        let mss = MediaSourceStream::new(Box::new(src), Default::default());
        let mut d = Decoded::new(mss, Hint::new(), "stream".into(), true).unwrap();
        let mut out = Vec::new();
        while d.read(&mut out).unwrap() {}
        assert!(out.iter().enumerate().all(|(i, &x)| x == i as f32 / 32768.0));
        out.len()
    }

    #[test]
    fn a_wav_stream_is_read_past_its_declared_length() {
        assert_eq!(decode_stream(wav_stream(100, 10_000)), 10_000);
        // Some servers declare no data at all.
        assert_eq!(decode_stream(wav_stream(0, 10_000)), 10_000);
    }

    #[test]
    fn other_streams_pass_unchanged() {
        for s in [&b"fLaC\0\0\0\x22rest of the stream"[..], b"RIFF\x10\0\0\0AVI LIST", b"RIF", b""] {
            let mut r = std::io::Cursor::new(s);
            let mut out = unbounded_wav_header(&mut r).unwrap();
            r.read_to_end(&mut out).unwrap();
            assert_eq!(out, s);
        }
    }
}
