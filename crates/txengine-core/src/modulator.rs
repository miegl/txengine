use crate::SinkInfo;

/// Timing context for one [`Modulator::render`] call.
#[derive(Debug, Clone, Copy)]
pub struct RenderContext {
    /// Absolute index of the first sample to render (0 = first sample ever
    /// emitted by the sink).
    pub first_sample: u64,
    /// Rate at which the samples are emitted, in Hz of true time. It is
    /// [`SinkInfo::sample_rate`] corrected for the measured (or configured)
    /// error of the sink's crystal, so it can differ from it by a few ppm
    /// and change slowly while transmitting. Modulators derive absolute
    /// frequencies (subcarriers) and the playback speed of the programme
    /// from it; filters may keep using the nominal rate.
    pub sample_rate: f64,
    /// Estimated wall-clock (UTC, seconds since the Unix epoch) at which the
    /// first sample will leave the transmitter, or `None` when the output is
    /// not real time (e.g. rendering to a file).
    pub emit_time_unix: Option<f64>,
}

impl RenderContext {
    /// Wall-clock emission time of sample `index`, if known.
    pub fn emit_time_of(&self, index: u64) -> Option<f64> {
        let t0 = self.emit_time_unix?;
        Some(t0 + (index as f64 - self.first_sample as f64) / self.sample_rate)
    }
}

/// Successful result of a control command.
pub type CommandOutput = String;

/// Error result of a control command.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CommandError {
    #[error("unknown command '{0}'")]
    Unknown(String),
    #[error("invalid argument: {0}")]
    Invalid(String),
}

/// Point-in-time status of a modulator, for monitoring.
#[derive(Debug, Clone, Default)]
pub struct ModulatorStatus {
    /// Free-form key/value pairs describing the modulator's state.
    pub fields: Vec<(String, String)>,
}

/// A transmission mode.
///
/// Implementations are driven from the real-time transmit thread: `render`
/// must never block (input that is not ready yet has to be substituted, e.g.
/// by silence) and should avoid allocation.
pub trait Modulator: Send {
    /// Short identifier of the mode (e.g. `"wfm"`).
    fn name(&self) -> &'static str;

    /// Prepare for output to the given sink. Called once before rendering.
    fn configure(&mut self, sink: &SinkInfo) -> Result<(), String>;

    /// Render `out.len()` samples in the sink's [`SampleFormat`](crate::SampleFormat).
    fn render(&mut self, ctx: &RenderContext, out: &mut [f32]);

    /// Handle a runtime control command (one line of text, already trimmed).
    fn command(&mut self, line: &str) -> Result<CommandOutput, CommandError>;

    /// Report status for monitoring.
    fn status(&self) -> ModulatorStatus {
        ModulatorStatus::default()
    }

    /// The mode's meters (levels, limiting, input health) since the
    /// previous call, as human-readable text for the periodic status line;
    /// the peak meters start over. Empty if the mode has none.
    fn take_meters(&mut self) -> String {
        String::new()
    }

    /// Whether the programme input has ended (e.g. end of file without looping).
    /// The engine stops transmitting once this returns true.
    fn finished(&self) -> bool {
        false
    }
}
