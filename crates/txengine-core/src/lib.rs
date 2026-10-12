//! Core interfaces of txengine.
//!
//! txengine is split into three independent layers which only meet here:
//!
//! * **Modulators** ([`Modulator`]) implement a transmission mode (wide FM
//!   with RDS, AM). They turn programme input into a stream of RF control
//!   samples ([`SampleFormat`]) at the rate dictated by the output.
//! * **RF sinks** ([`RfSink`]) consume those samples. Hardware backends
//!   (BCM283x/BCM2711 clock manager, RP1 on the Raspberry Pi 5) and software
//!   sinks (file output used for testing) implement this trait.
//! * The **engine** (in the `txengine` binary) connects one modulator to one
//!   sink and keeps the sink's buffer filled.
//!
//! Every sample carries an implicit absolute index (counted from the start of
//! the stream). Combined with the sink's hardware read position this gives a
//! precise mapping between sample indices and wall-clock time
//! ([`RenderContext`]), which is what time-critical features (RDS clock-time,
//! and in the future GPS/PPS-synchronised single frequency networks) build on.

mod modulator;
mod sink;

pub use modulator::{CommandError, CommandOutput, Modulator, ModulatorStatus, RenderContext};
pub use sink::{RfSink, SampleFormat, SinkError, SinkInfo, SinkPoll, interpolate_db};
