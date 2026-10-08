//! Wide-band FM broadcast transmission mode: mono/stereo multiplex with RDS.

pub mod mpx;

use log::info;
use txengine_audio::{AudioInput, EndOfInput};
use txengine_core::{CommandError, CommandOutput, Modulator, ModulatorStatus, RenderContext, SampleFormat, SinkInfo};
use txengine_rds::encoder::{LocalOffsetFn, af_code, encode8, encode_rt};
use txengine_rds::{RdsEncoder, RdsParams};

pub use mpx::{MpxConfig, MpxGenerator, MpxStats, NoAudio, StereoMode};

/// Programme audio for the modulator.
pub enum Programme {
    Audio(AudioInput, EndOfInput),
    None,
}

/// The wide FM modulator.
pub struct WfmModulator {
    cfg: MpxConfig,
    programme: Programme,
    rds: RdsEncoder,
    mpx: Option<MpxGenerator>,
    /// The sink's nominal sample rate, and the actual one being rendered
    /// for (from [`RenderContext::sample_rate`]).
    fs_nominal: f64,
    fs_actual: f64,
    rds_enabled: bool,
    /// The end of the audio input has been reported.
    input_end_logged: bool,
}

impl std::fmt::Debug for WfmModulator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("WfmModulator").field("mpx", &self.mpx).finish_non_exhaustive()
    }
}

fn on_off(arg: &str) -> Result<bool, CommandError> {
    match arg.to_ascii_uppercase().as_str() {
        "ON" | "1" | "TRUE" | "YES" => Ok(true),
        "OFF" | "0" | "FALSE" | "NO" => Ok(false),
        _ => Err(CommandError::Invalid(format!("expected ON or OFF, got '{arg}'"))),
    }
}

fn invalid(e: impl std::fmt::Display) -> CommandError {
    CommandError::Invalid(e.to_string())
}

impl WfmModulator {
    pub fn new(cfg: MpxConfig, programme: Programme, rds: RdsParams, rds_enabled: bool) -> Result<WfmModulator, String> {
        let rds = RdsEncoder::new(rds).map_err(|e| e.to_string())?;
        Ok(WfmModulator {
            cfg,
            programme,
            rds,
            mpx: None,
            fs_nominal: 0.0,
            fs_actual: 0.0,
            rds_enabled,
            input_end_logged: false,
        })
    }

    /// Install the local time offset provider used for RDS clock time.
    pub fn set_local_offset_fn(&mut self, f: LocalOffsetFn) {
        self.rds.set_local_offset_fn(f);
    }

    pub fn rds_params(&self) -> &RdsParams {
        self.rds.params()
    }

    pub fn mpx(&self) -> Option<&MpxGenerator> {
        self.mpx.as_ref()
    }

    /// Audio processing rate the programme input should be resampled to,
    /// for a given output sample rate.
    pub fn audio_rate_for(fs: f64) -> f64 {
        fs / (fs / 48_000.0).round().max(3.0)
    }

    fn update_rds(&mut self, f: impl FnOnce(&mut RdsParams) -> Result<(), CommandError>) -> Result<(), CommandError> {
        let mut p = self.rds.params().clone();
        f(&mut p)?;
        self.rds.set_params(p).map_err(invalid)
    }

    fn status_line(&mut self) -> String {
        let st = self.status();
        st.fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ")
    }
}

impl Modulator for WfmModulator {
    fn name(&self) -> &'static str {
        "wfm"
    }

    fn configure(&mut self, sink: &SinkInfo) -> Result<(), String> {
        if sink.format != SampleFormat::FrequencyOffset {
            return Err("the WFM mode needs a frequency-modulating output".into());
        }
        if sink.max_offset_hz < self.cfg.max_deviation {
            return Err(format!(
                "the output can only deviate ±{:.0} Hz, less than the configured ±{:.0} Hz",
                sink.max_offset_hz, self.cfg.max_deviation
            ));
        }
        let stereo_src = match &self.programme {
            Programme::Audio(a, _) => a.info.channels > 1,
            Programme::None => false,
        };
        // A file is rendered faster than real time: wait for the decoder
        // rather than inserting silence.
        if let Programme::Audio(a, _) = &mut self.programme {
            a.set_blocking(!sink.realtime);
        }
        // Only pre-compensate a sample-and-hold if the output has one.
        self.cfg.hold_compensation &= sink.zero_order_hold;
        self.cfg.output_response_db = sink.response_db.clone();
        let mpx = MpxGenerator::new(self.cfg.clone(), sink.sample_rate, stereo_src)?;
        if let Programme::Audio(a, _) = &self.programme
            && (a.rate - mpx.audio_rate()).abs() > 1e-6 {
                return Err(format!("audio input runs at {} Hz, the multiplex needs {} Hz", a.rate, mpx.audio_rate()));
            }
        // DI d0 follows the actual stereo coding.
        let stereo = mpx.is_stereo();
        let mut p = self.rds.params().clone();
        p.di.stereo = stereo;
        self.rds.set_params(p).map_err(|e| e.to_string())?;
        info!(
            "WFM: {}, audio ±{:.1} kHz, pilot ±{:.2} kHz, RDS ±{:.2} kHz{}, max ±{:.1} kHz, pre-emphasis {}, audio rate {:.1} Hz",
            if stereo { "stereo" } else { "mono" },
            mpx.audio_deviation() / 1e3,
            if stereo { self.cfg.pilot_deviation / 1e3 } else { 0.0 },
            self.cfg.rds_deviation / 1e3,
            if self.rds_enabled { "" } else { " (off)" },
            self.cfg.max_deviation / 1e3,
            self.cfg.preemphasis.map_or("off".to_string(), |t| format!("{:.0} µs (drive {:+.1} dB to keep the MPX power)", t * 1e6, mpx.drive_db())),
            mpx.audio_rate()
        );
        let mut mpx = mpx;
        mpx.set_rds_enabled(self.rds_enabled);
        self.mpx = Some(mpx);
        self.fs_nominal = sink.sample_rate;
        self.fs_actual = sink.sample_rate;
        Ok(())
    }

    fn render(&mut self, ctx: &RenderContext, out: &mut [f32]) {
        let Some(mpx) = self.mpx.as_mut() else {
            out.fill(0.0);
            return;
        };
        if ctx.sample_rate != self.fs_actual {
            // The sink's crystal error changed: keep the subcarriers and the
            // programme's playback speed true.
            self.fs_actual = ctx.sample_rate;
            mpx.set_output_rate(ctx.sample_rate);
            if let Programme::Audio(a, _) = &self.programme {
                a.set_rate_correction_ppm((ctx.sample_rate / self.fs_nominal - 1.0) * 1e6);
            }
        }
        let t0 = ctx.emit_time_unix.unwrap_or(f64::NAN);
        // Without a real-time mapping, use the system clock for CT.
        let t0 = if t0.is_nan() {
            std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
        } else {
            t0
        };
        match &mut self.programme {
            Programme::Audio(a, end) => {
                mpx.render(out, a, &mut self.rds, t0);
                if *end == EndOfInput::Wait && !self.input_end_logged && a.finished() {
                    self.input_end_logged = true;
                    log::info!("audio input ended; transmitting the carrier and RDS without programme");
                }
            }
            Programme::None => mpx.render(out, &mut NoAudio, &mut self.rds, t0),
        }
    }

    fn command(&mut self, line: &str) -> Result<CommandOutput, CommandError> {
        let line = line.trim();
        let (cmd, arg) = match line.split_once(char::is_whitespace) {
            Some((c, a)) => (c, a),
            None => (line, ""),
        };
        // PS/RT keep the argument verbatim (leading spaces may be intended).
        let targ = arg.trim();
        match cmd.to_ascii_uppercase().as_str() {
            "PS" => {
                let ps = encode8(arg).map_err(invalid)?;
                self.update_rds(|p| {
                    p.ps = ps;
                    Ok(())
                })?;
                Ok(format!("PS set to \"{arg}\""))
            }
            "RT" => {
                let rt = encode_rt(arg).map_err(invalid)?;
                self.update_rds(|p| {
                    p.rt = rt;
                    Ok(())
                })?;
                Ok(format!("RT set to \"{arg}\""))
            }
            "PTY" => {
                let pty = txengine_rds::pty::parse(targ).ok_or_else(|| invalid("PTY must be 0-31 or a programme type name"))?;
                self.update_rds(|p| {
                    p.pty = pty;
                    Ok(())
                })?;
                Ok(format!("PTY set to {pty} ({})", txengine_rds::pty::RDS[pty as usize]))
            }
            "PI" => {
                let pi = u16::from_str_radix(targ.trim_start_matches("0x"), 16).map_err(|_| invalid("PI must be 4 hex digits"))?;
                self.update_rds(|p| {
                    p.pi = pi;
                    Ok(())
                })?;
                Ok(format!("PI set to {pi:04X}"))
            }
            "TA" | "TP" | "MS" | "CT" => {
                let v = on_off(targ)?;
                let c = cmd.to_ascii_uppercase();
                self.update_rds(|p| {
                    match c.as_str() {
                        "TA" => p.ta = v,
                        "TP" => p.tp = v,
                        "MS" => p.ms = v,
                        _ => p.ct = v,
                    }
                    Ok(())
                })?;
                Ok(format!("{c} {}", if v { "ON" } else { "OFF" }))
            }
            "AB" => {
                // Force receivers to clear their RadioText display.
                let mut p = self.rds.params().clone();
                let rt = std::mem::take(&mut p.rt);
                self.rds.set_params(p.clone()).map_err(invalid)?;
                p.rt = rt;
                self.rds.set_params(p).map_err(invalid)?;
                Ok("RadioText A/B flag toggled".into())
            }
            "PTYN" => {
                let v = if targ.eq_ignore_ascii_case("OFF") || targ.is_empty() { None } else { Some(encode8(arg).map_err(invalid)?) };
                self.update_rds(|p| {
                    p.ptyn = v;
                    Ok(())
                })?;
                Ok(if v.is_some() { format!("PTYN set to \"{arg}\"") } else { "PTYN off".into() })
            }
            "AF" => {
                let codes = if targ.eq_ignore_ascii_case("OFF") || targ.is_empty() {
                    Vec::new()
                } else {
                    targ.split([' ', ','])
                        .filter(|s| !s.is_empty())
                        .map(|s| s.parse::<f64>().map_err(invalid).and_then(|f| af_code(f).map_err(invalid)))
                        .collect::<Result<Vec<u8>, _>>()?
                };
                if codes.len() > 25 {
                    return Err(invalid("at most 25 alternative frequencies"));
                }
                let n = codes.len();
                self.update_rds(|p| {
                    p.af = codes;
                    Ok(())
                })?;
                Ok(format!("{n} alternative frequencies set"))
            }
            "DI" => {
                let v = u8::from_str_radix(targ.trim_start_matches("0x"), 16).map_err(|_| invalid("DI must be a hex value 0-F"))?;
                if v > 15 {
                    return Err(invalid("DI must be 0-F"));
                }
                self.update_rds(|p| {
                    // d0 (stereo) always follows the actual coding.
                    p.di.artificial_head = v & 2 != 0;
                    p.di.compressed = v & 4 != 0;
                    p.di.dynamic_pty = v & 8 != 0;
                    Ok(())
                })?;
                Ok(format!("DI set to {v:X}"))
            }
            "ECC" | "LIC" => {
                let v = if targ.eq_ignore_ascii_case("OFF") {
                    None
                } else {
                    Some(u16::from_str_radix(targ.trim_start_matches("0x"), 16).map_err(|_| invalid("expected a hex value or OFF"))?)
                };
                let c = cmd.to_ascii_uppercase();
                self.update_rds(|p| {
                    if c == "ECC" {
                        p.ecc = v.map(|x| x as u8);
                    } else {
                        p.lic = v;
                    }
                    Ok(())
                })?;
                Ok(format!("{c} {}", v.map_or("OFF".into(), |x| format!("{x:X}"))))
            }
            "RDS" => {
                let v = on_off(targ)?;
                self.rds_enabled = v;
                if let Some(m) = self.mpx.as_mut() {
                    m.set_rds_enabled(v);
                }
                Ok(format!("RDS {}", if v { "ON" } else { "OFF" }))
            }
            "RDSLEVEL" => {
                let hz: f64 = targ.parse().map_err(|_| invalid("expected a deviation in Hz"))?;
                if !(0.0..=7_500.0).contains(&hz) {
                    return Err(invalid("RDS level must be 0-7500 Hz (EN 50067 1.3)"));
                }
                self.cfg.rds_deviation = hz;
                if let Some(m) = self.mpx.as_mut() {
                    m.set_rds_deviation(hz);
                }
                Ok(format!("RDS level {hz} Hz"))
            }
            "GAIN" | "VOL" => {
                let db: f64 = targ.trim_end_matches("dB").parse().map_err(|_| invalid("expected a gain in dB"))?;
                if !(-60.0..=30.0).contains(&db) {
                    return Err(invalid("gain must be -60 to +30 dB"));
                }
                self.cfg.gain_db = db;
                if let Some(m) = self.mpx.as_mut() {
                    m.set_gain_db(db);
                }
                Ok(format!("gain {db} dB"))
            }
            "STATUS" => Ok(self.status_line()),
            _ => Err(CommandError::Unknown(cmd.to_string())),
        }
    }

    fn status(&self) -> ModulatorStatus {
        let mut f = Vec::new();
        let p = self.rds.params();
        let text = |b: &[u8]| -> String { b.iter().map(|&c| txengine_rds::charset::decode_char(c)).collect() };
        f.push(("ps".into(), format!("\"{}\"", text(&p.ps))));
        f.push(("pi".into(), format!("{:04X}", p.pi)));
        f.push(("pty".into(), p.pty.to_string()));
        f.push(("rt".into(), format!("\"{}\"", text(&p.rt))));
        f.push(("rds".into(), if self.rds_enabled { "on" } else { "off" }.into()));
        f.push(("rds_groups".into(), self.rds.groups_sent().to_string()));
        if let Some(m) = &self.mpx {
            f.push(("stereo".into(), m.is_stereo().to_string()));
        }
        if let Programme::Audio(a, _) = &self.programme {
            let s = a.stats();
            f.push(("audio_buffer".into(), format!("{}/{}", s.buffered, s.capacity)));
            if s.drift_ppm != 0.0 {
                f.push(("audio_drift_ppm".into(), format!("{:.1}", s.drift_ppm)));
            }
        }
        ModulatorStatus { fields: f }
    }

    fn take_meters(&mut self) -> String {
        let st = self.take_stats().unwrap_or_default();
        let mut s = format!(
            "peak deviation {:.1} kHz, MPX power {:+.1} dBr, HF limiter {:+.1} dB, limiter {:+.1} dB, clipped {}, audio underruns {}",
            st.peak_deviation / 1e3,
            st.mpx_power_dbr,
            -st.hf_limiter_db,
            20.0 * (st.limiter_gain.max(1e-6) as f64).log10(),
            st.clipped,
            st.audio_underruns,
        );
        if let Programme::Audio(a, _) = &self.programme {
            let a = a.stats();
            s.push_str(&format!(", audio buffer {}/{}", a.buffered, a.capacity));
            if a.drift_ppm != 0.0 {
                s.push_str(&format!(", input drift {:.1} ppm", a.drift_ppm));
            }
        }
        s
    }

    fn finished(&self) -> bool {
        match &self.programme {
            Programme::Audio(a, EndOfInput::Stop) => a.finished(),
            _ => false,
        }
    }
}

impl WfmModulator {
    /// Multiplex meters (peak deviation, limiter, clipping, underruns).
    pub fn take_stats(&mut self) -> Option<MpxStats> {
        self.mpx.as_mut().map(|m| m.stats(true))
    }
}
