//! AM broadcast transmission mode: double sideband with full carrier (A3E),
//! as on long, medium and short wave.

pub mod envelope;

use log::info;
use txengine_audio::{EndOfInput, NoAudio, Programme};
use txengine_core::{CommandError, CommandOutput, Modulator, ModulatorStatus, RenderContext, SampleFormat, SinkInfo};

pub use envelope::{AmConfig, AmStats, EnvelopeGenerator};

/// The AM modulator.
pub struct AmModulator {
    cfg: AmConfig,
    programme: Programme,
    envelope: Option<EnvelopeGenerator>,
    /// The sink's nominal sample rate, and the actual one being rendered
    /// for (from [`RenderContext::sample_rate`]).
    fs_nominal: f64,
    fs_actual: f64,
    /// The end of the audio input has been reported.
    input_end_logged: bool,
}

impl std::fmt::Debug for AmModulator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AmModulator").field("envelope", &self.envelope).finish_non_exhaustive()
    }
}

fn invalid(e: impl std::fmt::Display) -> CommandError {
    CommandError::Invalid(e.to_string())
}

impl AmModulator {
    pub fn new(cfg: AmConfig, programme: Programme) -> Result<AmModulator, String> {
        cfg.check()?;
        Ok(AmModulator { cfg, programme, envelope: None, fs_nominal: 0.0, fs_actual: 0.0, input_end_logged: false })
    }

    /// Audio processing rate the programme input should be resampled to,
    /// for a given output sample rate.
    pub fn audio_rate_for(fs: f64) -> f64 {
        EnvelopeGenerator::audio_rate_for(fs)
    }

    /// Modulation meters (peaks, limiter, clipping, underruns).
    pub fn take_stats(&mut self) -> Option<AmStats> {
        self.envelope.as_mut().map(|e| e.stats(true))
    }

    fn status_line(&self) -> String {
        let st = self.status();
        st.fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ")
    }
}

impl Modulator for AmModulator {
    fn name(&self) -> &'static str {
        "am"
    }

    fn configure(&mut self, sink: &SinkInfo) -> Result<(), String> {
        if sink.format != SampleFormat::Amplitude {
            return Err("the AM mode needs an amplitude-modulating output".into());
        }
        // A file is rendered faster than real time: wait for the decoder
        // rather than inserting silence.
        if let Programme::Audio(a, _) = &mut self.programme {
            a.set_blocking(!sink.realtime);
        }
        let env = EnvelopeGenerator::new(self.cfg.clone(), sink.sample_rate)?;
        if let Programme::Audio(a, _) = &self.programme
            && (a.rate - env.audio_rate()).abs() > 1e-6
        {
            return Err(format!("audio input runs at {} Hz, the modulator needs {} Hz", a.rate, env.audio_rate()));
        }
        info!(
            "AM: audio bandwidth {:.1} kHz, modulation depth {:.0} %, carrier {:.1} dB below full output, audio rate {:.1} Hz",
            self.cfg.bandwidth / 1e3,
            self.cfg.depth * 100.0,
            -txengine_dsp::db(self.cfg.carrier()),
            env.audio_rate()
        );
        self.envelope = Some(env);
        self.fs_nominal = sink.sample_rate;
        self.fs_actual = sink.sample_rate;
        Ok(())
    }

    fn render(&mut self, ctx: &RenderContext, out: &mut [f32]) {
        let Some(env) = self.envelope.as_mut() else {
            out.fill(0.0);
            return;
        };
        if ctx.sample_rate != self.fs_actual {
            // The samples leave at another rate (a crystal error that the
            // output does not correct): keep the programme's playback speed
            // true.
            self.fs_actual = ctx.sample_rate;
            if let Programme::Audio(a, _) = &self.programme {
                a.set_rate_correction_ppm((ctx.sample_rate / self.fs_nominal - 1.0) * 1e6);
            }
        }
        match &mut self.programme {
            Programme::Audio(a, end) => {
                env.render(out, a);
                if *end == EndOfInput::Wait && !self.input_end_logged && a.finished() {
                    self.input_end_logged = true;
                    info!("audio input ended; transmitting the carrier without programme");
                }
            }
            Programme::None => env.render(out, &mut NoAudio),
        }
    }

    fn command(&mut self, line: &str) -> Result<CommandOutput, CommandError> {
        let line = line.trim();
        let (cmd, arg) = line.split_once(char::is_whitespace).unwrap_or((line, ""));
        let arg = arg.trim();
        match cmd.to_ascii_uppercase().as_str() {
            "GAIN" | "VOL" => {
                let db: f64 = arg.trim_end_matches("dB").parse().map_err(|_| invalid("expected a gain in dB"))?;
                AmConfig { gain_db: db, ..self.cfg.clone() }.check().map_err(invalid)?;
                self.cfg.gain_db = db;
                if let Some(e) = self.envelope.as_mut() {
                    e.set_gain_db(db);
                }
                Ok(format!("gain {db} dB"))
            }
            "DEPTH" => {
                let pct: f64 = arg.trim_end_matches('%').parse().map_err(|_| invalid("expected a depth in %"))?;
                AmConfig { depth: pct / 100.0, ..self.cfg.clone() }.check().map_err(invalid)?;
                self.cfg.depth = pct / 100.0;
                if let Some(e) = self.envelope.as_mut() {
                    e.set_depth(self.cfg.depth);
                }
                Ok(format!(
                    "modulation depth {pct} %, carrier {:.1} dB below full output",
                    -txengine_dsp::db(self.cfg.carrier())
                ))
            }
            "STATUS" => Ok(self.status_line()),
            _ => Err(CommandError::Unknown(cmd.to_string())),
        }
    }

    fn status(&self) -> ModulatorStatus {
        let mut f = vec![
            ("depth".into(), format!("{:.0}", self.cfg.depth * 100.0)),
            ("gain".into(), format!("{}", self.cfg.gain_db)),
            ("bandwidth".into(), format!("{:.0}", self.cfg.bandwidth)),
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
            "modulation +{:.0} % / -{:.0} %, limiter {:+.1} dB, clipped {}, audio underruns {}",
            st.peak_positive * 100.0,
            st.peak_negative * 100.0,
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
