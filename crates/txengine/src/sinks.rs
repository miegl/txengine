//! Software sinks: render to files for testing and verification.

use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::Path;

use txengine_core::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll};

enum Out {
    /// IEEE float WAV of deviation / `full_scale`, or of the envelope.
    Wav {
        w: BufWriter<File>,
        frames: u64,
        full_scale: f64,
    },
    /// Interleaved complex float baseband (FM modulated at unit amplitude,
    /// or the envelope).
    Iq {
        w: BufWriter<File>,
        phase: f64,
    },
    Null,
}

pub struct FileSink {
    info: SinkInfo,
    out: Out,
    written: u64,
    hold_model: bool,
}

const CHUNK: usize = 16384;

impl FileSink {
    /// The samples as WAV: a frequency offset / `full_scale` (Hz), an
    /// amplitude as it is.
    pub fn wav(path: &Path, fs: f64, format: SampleFormat, full_scale: f64, hold: bool) -> anyhow::Result<FileSink> {
        let mut w = BufWriter::new(File::create(path)?);
        write_wav_header(&mut w, fs as u32, 0)?;
        let what = match format {
            SampleFormat::FrequencyOffset => "deviation WAV file",
            SampleFormat::Amplitude => "envelope WAV file",
        };
        Ok(Self::new(Out::Wav { w, frames: 0, full_scale }, fs, format, what, hold))
    }

    pub fn iq(path: &Path, fs: f64, format: SampleFormat, hold: bool) -> anyhow::Result<FileSink> {
        let out = Out::Iq { w: BufWriter::new(File::create(path)?), phase: 0.0 };
        Ok(Self::new(out, fs, format, "I/Q file", hold))
    }

    pub fn null(fs: f64, format: SampleFormat) -> FileSink {
        Self::new(Out::Null, fs, format, "null output", false)
    }

    fn new(out: Out, fs: f64, format: SampleFormat, what: &str, hold: bool) -> FileSink {
        FileSink {
            info: SinkInfo {
                description: format!("{what} at {fs} S/s"),
                sample_rate: fs,
                carrier_hz: 0.0,
                max_offset_hz: f64::INFINITY,
                resolution_hz: 0.0,
                max_clock_error_ppm: 0.0,
                paced_by_carrier: false,
                buffer_len: CHUNK,
                format,
                realtime: false,
                zero_order_hold: hold,
                response_db: Vec::new(),
            },
            out,
            written: 0,
            hold_model: hold,
        }
    }
}

fn write_wav_header(w: &mut impl Write, rate: u32, frames: u64) -> std::io::Result<()> {
    let data = (frames * 4).min(u32::MAX as u64 - 64) as u32;
    w.write_all(b"RIFF")?;
    w.write_all(&(36 + 12 + data).to_le_bytes())?;
    w.write_all(b"WAVEfmt ")?;
    w.write_all(&16u32.to_le_bytes())?;
    w.write_all(&3u16.to_le_bytes())?; // IEEE float
    w.write_all(&1u16.to_le_bytes())?; // mono
    w.write_all(&rate.to_le_bytes())?;
    w.write_all(&(rate * 4).to_le_bytes())?;
    w.write_all(&4u16.to_le_bytes())?;
    w.write_all(&32u16.to_le_bytes())?;
    w.write_all(b"fact")?;
    w.write_all(&4u32.to_le_bytes())?;
    w.write_all(&(frames.min(u32::MAX as u64) as u32).to_le_bytes())?;
    w.write_all(b"data")?;
    w.write_all(&data.to_le_bytes())?;
    Ok(())
}

impl RfSink for FileSink {
    fn info(&self) -> &SinkInfo {
        &self.info
    }

    fn start(&mut self) -> Result<(), SinkError> {
        Ok(())
    }

    fn poll(&mut self) -> Result<SinkPoll, SinkError> {
        Ok(SinkPoll { writable: CHUNK, consumed: self.written, written: self.written, underruns: 0 })
    }

    fn write(&mut self, samples: &[f32]) -> Result<(), SinkError> {
        let fs = self.info.sample_rate;
        let am = self.info.format == SampleFormat::Amplitude;
        match &mut self.out {
            Out::Wav { w, frames, full_scale } => {
                for &s in samples {
                    let v = if am { s } else { (s as f64 / *full_scale) as f32 };
                    w.write_all(&v.to_le_bytes())?;
                }
                *frames += samples.len() as u64;
            }
            Out::Iq { w, .. } if am => {
                for &s in samples {
                    w.write_all(&s.to_le_bytes())?;
                    w.write_all(&0f32.to_le_bytes())?;
                }
            }
            Out::Iq { w, phase } => {
                let _ = self.hold_model;
                for &s in samples {
                    let (sn, cs) = phase.sin_cos();
                    w.write_all(&(cs as f32).to_le_bytes())?;
                    w.write_all(&(sn as f32).to_le_bytes())?;
                    *phase = (*phase + 2.0 * std::f64::consts::PI * s as f64 / fs) % (2.0 * std::f64::consts::PI);
                }
            }
            Out::Null => {}
        }
        self.written += samples.len() as u64;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SinkError> {
        match &mut self.out {
            Out::Wav { w, frames, .. } => {
                w.flush()?;
                let f = w.get_mut();
                f.seek(SeekFrom::Start(0))?;
                write_wav_header(f, self.info.sample_rate as u32, *frames)?;
                f.flush()?;
            }
            Out::Iq { w, .. } => w.flush()?,
            Out::Null => {}
        }
        Ok(())
    }
}
