//! Narrow-band FM transmission mode: voice in a 12.5 or 25 kHz channel,
//! with CTCSS or DCS squelch signalling, as on short wave (10 m), CB, VHF
//! and UHF (PMR446, land mobile, amateur radio).

pub mod baseband;
pub mod subaudio;

use log::{info, warn};
use txengine_audio::{EndOfInput, NoAudio, Programme};
use txengine_core::{CommandError, CommandOutput, Modulator, ModulatorStatus, RenderContext, SampleFormat, SinkInfo};

pub use baseband::{BasebandGenerator, NfmConfig, NfmStats};
pub use subaudio::{DcsCode, Subaudio};

/// The NFM modulator.
pub struct NfmModulator {
    cfg: NfmConfig,
    programme: Programme,
    baseband: Option<BasebandGenerator>,
    /// The sink's nominal sample rate, and the actual one being rendered
    /// for (from [`RenderContext::sample_rate`]).
    fs_nominal: f64,
    fs_actual: f64,
    /// The end of the audio input has been reported.
    input_end_logged: bool,
}

impl std::fmt::Debug for NfmModulator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NfmModulator").field("baseband", &self.baseband).finish_non_exhaustive()
    }
}

fn invalid(e: impl std::fmt::Display) -> CommandError {
    CommandError::Invalid(e.to_string())
}

/// A warning for a CTCSS tone that receivers do not offer, if it is one.
pub fn nonstandard_tone(what: Subaudio) -> Option<String> {
    match what {
        Subaudio::Ctcss(f) if !subaudio::is_standard_tone(f) => {
            Some(format!("{f} Hz is not a standard CTCSS tone (EIA/TIA-603); receivers may not offer it"))
        }
        _ => None,
    }
}

impl NfmModulator {
    pub fn new(cfg: NfmConfig, programme: Programme) -> Result<NfmModulator, String> {
        cfg.check()?;
        Ok(NfmModulator { cfg, programme, baseband: None, fs_nominal: 0.0, fs_actual: 0.0, input_end_logged: false })
    }

    /// Audio processing rate the programme input should be resampled to,
    /// for a given output sample rate.
    pub fn audio_rate_for(fs: f64) -> f64 {
        BasebandGenerator::audio_rate_for(fs)
    }

    /// Modulation meters (peak deviation, limiter, clipping, underruns).
    pub fn take_stats(&mut self) -> Option<NfmStats> {
        self.baseband.as_mut().map(|b| b.stats(true))
    }

    fn describe(&self) -> String {
        let c = &self.cfg;
        let signalling = match c.subaudio {
            Subaudio::Off => String::new(),
            s => format!(", {s} ±{:.0} Hz", c.subaudio_deviation),
        };
        format!(
            "deviation ±{:.2} kHz (voice ±{:.2} kHz{signalling}), audio {:.0}-{:.0} Hz, pre-emphasis {}",
            c.deviation / 1e3,
            c.voice_deviation() / 1e3,
            baseband::VOICE_LOW_HZ,
            c.bandwidth,
            c.preemphasis.map_or("off".to_string(), |t| format!("{:.0} µs", t * 1e6)),
        )
    }

    fn set_subaudio(&mut self, what: Subaudio) -> Result<CommandOutput, CommandError> {
        let cfg = NfmConfig { subaudio: what, ..self.cfg.clone() };
        cfg.check().map_err(invalid)?;
        if let Some(b) = self.baseband.as_mut() {
            b.set_subaudio(what, cfg.subaudio_deviation).map_err(invalid)?;
        }
        self.cfg = cfg;
        let mut reply = format!("sub-audio {what}, voice ±{:.2} kHz", self.cfg.voice_deviation() / 1e3);
        if let Some(w) = nonstandard_tone(what) {
            reply.push_str(&format!(" ({w})"));
        }
        Ok(reply)
    }

    fn status_line(&self) -> String {
        let st = self.status();
        st.fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ")
    }
}

impl Modulator for NfmModulator {
    fn name(&self) -> &'static str {
        "nfm"
    }

    fn configure(&mut self, sink: &SinkInfo) -> Result<(), String> {
        if sink.format != SampleFormat::FrequencyOffset {
            return Err("the NFM mode needs a frequency-modulating output".into());
        }
        if sink.max_offset_hz < self.cfg.deviation {
            return Err(format!(
                "the output can only deviate ±{:.0} Hz, less than the configured ±{:.0} Hz",
                sink.max_offset_hz, self.cfg.deviation
            ));
        }
        // A file is rendered faster than real time: wait for the decoder
        // rather than inserting silence.
        if let Programme::Audio(a, _) = &mut self.programme {
            a.set_blocking(!sink.realtime);
        }
        // The voice band ends at a few kHz, where neither the output's
        // sample-and-hold (under 0.01 dB at 3 kHz and 166 kS/s) nor any
        // measured response needs compensating.
        let bb = BasebandGenerator::new(self.cfg.clone(), sink.sample_rate)?;
        if let Programme::Audio(a, _) = &self.programme
            && (a.rate - bb.audio_rate()).abs() > 1e-6
        {
            return Err(format!("audio input runs at {} Hz, the modulator needs {} Hz", a.rate, bb.audio_rate()));
        }
        info!("NFM: {}, audio rate {:.1} Hz", self.describe(), bb.audio_rate());
        if let Some(w) = nonstandard_tone(self.cfg.subaudio) {
            warn!("{w}");
        }
        if sink.resolution_hz > 0.02 * self.cfg.deviation {
            warn!(
                "the output moves the carrier in steps of {:.1} Hz, coarse for ±{:.0} Hz of deviation",
                sink.resolution_hz, self.cfg.deviation
            );
        }
        self.baseband = Some(bb);
        self.fs_nominal = sink.sample_rate;
        self.fs_actual = sink.sample_rate;
        Ok(())
    }

    fn render(&mut self, ctx: &RenderContext, out: &mut [f32]) {
        let Some(bb) = self.baseband.as_mut() else {
            out.fill(0.0);
            return;
        };
        if ctx.sample_rate != self.fs_actual {
            // The samples leave at another rate (the crystal error): keep
            // the sub-audio tone, the DCS bit rate and the programme's
            // playback speed true.
            self.fs_actual = ctx.sample_rate;
            bb.set_output_rate(ctx.sample_rate);
            if let Programme::Audio(a, _) = &self.programme {
                a.set_rate_correction_ppm((ctx.sample_rate / self.fs_nominal - 1.0) * 1e6);
            }
        }
        match &mut self.programme {
            Programme::Audio(a, end) => {
                bb.render(out, a);
                if *end == EndOfInput::Wait && !self.input_end_logged && a.finished() {
                    self.input_end_logged = true;
                    info!("audio input ended; transmitting the carrier without programme");
                }
            }
            Programme::None => bb.render(out, &mut NoAudio),
        }
    }

    fn command(&mut self, line: &str) -> Result<CommandOutput, CommandError> {
        let line = line.trim();
        let (cmd, arg) = line.split_once(char::is_whitespace).unwrap_or((line, ""));
        let arg = arg.trim();
        let off = arg.eq_ignore_ascii_case("OFF");
        match cmd.to_ascii_uppercase().as_str() {
            "GAIN" | "VOL" => {
                let db: f64 = arg.trim_end_matches("dB").parse().map_err(|_| invalid("expected a gain in dB"))?;
                NfmConfig { gain_db: db, ..self.cfg.clone() }.check().map_err(invalid)?;
                self.cfg.gain_db = db;
                if let Some(b) = self.baseband.as_mut() {
                    b.set_gain_db(db);
                }
                Ok(format!("gain {db} dB"))
            }
            "CTCSS" => {
                let what = if off {
                    Subaudio::Off
                } else {
                    Subaudio::Ctcss(
                        arg.trim_end_matches("Hz")
                            .trim()
                            .parse()
                            .map_err(|_| invalid("expected a tone in Hz or OFF"))?,
                    )
                };
                self.set_subaudio(what)
            }
            "DCS" => {
                let what = if off { Subaudio::Off } else { Subaudio::Dcs(DcsCode::parse(arg).map_err(invalid)?) };
                self.set_subaudio(what)
            }
            "STATUS" => Ok(self.status_line()),
            _ => Err(CommandError::Unknown(cmd.to_string())),
        }
    }

    fn status(&self) -> ModulatorStatus {
        let c = &self.cfg;
        let mut f = vec![
            ("deviation".into(), format!("{:.0}", c.deviation)),
            ("voice_deviation".into(), format!("{:.0}", c.voice_deviation())),
            ("subaudio".into(), c.subaudio.to_string().replace(' ', "_")),
            ("gain".into(), format!("{}", c.gain_db)),
            ("bandwidth".into(), format!("{:.0}", c.bandwidth)),
        ];
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
        let Some(st) = self.take_stats() else { return String::new() };
        let mut s = format!(
            "peak deviation {:.2} kHz, limiter {:+.1} dB, clipped {}, audio underruns {}",
            st.peak_deviation / 1e3,
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

#[cfg(test)]
mod tests {
    use super::*;

    fn modulator() -> NfmModulator {
        NfmModulator::new(NfmConfig::default(), Programme::None).unwrap()
    }

    #[test]
    fn squelch_signalling_commands() {
        let mut m = modulator();
        assert!(m.command("CTCSS 88.5").unwrap().contains("CTCSS 88.5 Hz"));
        assert_eq!(m.cfg.subaudio, Subaudio::Ctcss(88.5));
        assert_eq!(m.cfg.voice_deviation(), 2_125.0);
        assert!(m.command("ctcss 88.4").unwrap().contains("not a standard CTCSS tone"));
        assert!(m.command("CTCSS 300").is_err());
        assert_eq!(m.cfg.subaudio, Subaudio::Ctcss(88.4));
        m.command("DCS 754I").unwrap();
        assert_eq!(m.cfg.subaudio, Subaudio::Dcs(DcsCode { code: 0o754, inverted: true }));
        assert!(m.command("DCS 089").is_err());
        m.command("DCS OFF").unwrap();
        assert_eq!((m.cfg.subaudio, m.cfg.voice_deviation()), (Subaudio::Off, 2_500.0));
        assert!(matches!(m.command("BURST"), Err(CommandError::Unknown(_))));
    }

    #[test]
    fn gain_command() {
        let mut m = modulator();
        assert_eq!(m.command("GAIN -6").unwrap(), "gain -6 dB");
        assert!(m.command("GAIN 40").is_err());
        assert_eq!(m.cfg.gain_db, -6.0);
    }
}
