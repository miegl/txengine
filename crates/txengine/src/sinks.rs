//! Software sinks: render to files for testing and verification.

use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::Path;

use txengine_core::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll};

/// Deviation corresponding to a full-scale sample in rendered WAV files.
pub const WAV_FULL_SCALE: f64 = 100_000.0;

enum Out {
    /// IEEE float WAV of deviation / 100 kHz.
    Wav {
        w: BufWriter<File>,
        frames: u64,
    },
    /// Interleaved complex float baseband (FM modulated, unit amplitude).
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
    pub fn deviation_wav(path: &Path, fs: f64, hold: bool) -> anyhow::Result<FileSink> {
        let mut w = BufWriter::new(File::create(path)?);
        write_wav_header(&mut w, fs as u32, 0)?;
        Ok(Self::new(Out::Wav { w, frames: 0 }, fs, "deviation WAV file", hold))
    }

    pub fn iq(path: &Path, fs: f64, hold: bool) -> anyhow::Result<FileSink> {
        Ok(Self::new(Out::Iq { w: BufWriter::new(File::create(path)?), phase: 0.0 }, fs, "I/Q file", hold))
    }

    pub fn null(fs: f64) -> FileSink {
        Self::new(Out::Null, fs, "null output", false)
    }

    fn new(out: Out, fs: f64, what: &str, hold: bool) -> FileSink {
        FileSink {
            info: SinkInfo {
                description: format!("{what} at {fs} S/s"),
                sample_rate: fs,
                carrier_hz: 0.0,
                max_offset_hz: f64::INFINITY,
                resolution_hz: 0.0,
                max_clock_error_ppm: 0.0,
                buffer_len: CHUNK,
                format: SampleFormat::FrequencyOffset,
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
        match &mut self.out {
            Out::Wav { w, frames } => {
                for &s in samples {
                    w.write_all(&((s as f64 / WAV_FULL_SCALE) as f32).to_le_bytes())?;
                }
                *frames += samples.len() as u64;
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
            Out::Wav { w, frames } => {
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
