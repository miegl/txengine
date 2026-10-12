//! Command line options of the AM mode, and building its modulator.

use clap::{Args, Subcommand};
use txengine_am::envelope::{MAX_BANDWIDTH, MIN_BANDWIDTH};
use txengine_am::{AmConfig, AmModulator};
use txengine_core::{Modulator, SinkInfo};

use crate::cli::{Render, Tx};
use crate::hw::Output;
use crate::mode::Mode;
use crate::programme::{self, ProgrammeArgs};

// Parsed once at start-up: the size of the variants does not matter.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Subcommand)]
pub enum Command {
    /// Transmit.
    Tx(Tx<AmArgs>),
    /// Render the signal to a file instead of transmitting (for testing).
    Render(Render<AmArgs>),
}

/// The broadcast bands (Hz): long wave, medium wave (ITU regions 1 and 3,
/// and region 2), and the short-wave broadcast bands' extent.
const BANDS: [(f64, f64); 3] = [(148.5e3, 283.5e3), (525e3, 1705e3), (2.3e6, 26.1e6)];

// Programme and modulation options of `am tx` and `am render`.
#[derive(Debug, Clone, Args)]
pub struct AmArgs {
    #[command(flatten)]
    pub programme: ProgrammeArgs,

    // Modulation -----------------------------------------------------------
    /// Audio bandwidth in Hz: the highest audio frequency transmitted (the
    /// signal is twice as wide). 4500 fits the 9 kHz channels of Europe,
    /// Africa, Asia and Oceania; 10000 is the limit in the 10 kHz channels
    /// of the Americas.
    #[arg(long, default_value_t = 4_500.0, value_name = "HZ", help_heading = "Modulation")]
    pub bandwidth: f64,
    /// Modulation depth of the programme's peaks in percent. The carrier is
    /// set to 1 / (1 + depth) of the transmitter's full output, so that the
    /// positive peaks reach it.
    #[arg(long, default_value_t = 90.0, value_name = "PERCENT", help_heading = "Modulation")]
    pub depth: f64,
    /// Input gain in dB.
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true, value_name = "DB", help_heading = "Modulation")]
    pub gain: f64,
    /// Disable the limiter; overload is then hard clipped.
    #[arg(long, help_heading = "Modulation")]
    pub no_limiter: bool,
}

impl AmArgs {
    fn config(&self) -> anyhow::Result<AmConfig> {
        let cfg = AmConfig {
            bandwidth: self.bandwidth,
            depth: self.depth / 100.0,
            gain_db: self.gain,
            limiter: !self.no_limiter,
        };
        cfg.check().map_err(|e| {
            anyhow::anyhow!(
                "{e} (--bandwidth {MIN_BANDWIDTH:.0}-{MAX_BANDWIDTH:.0}, --depth up to 100, --gain -60 to 30)"
            )
        })?;
        Ok(cfg)
    }
}

impl Mode for AmArgs {
    fn output(&self) -> Output {
        Output::Amplitude
    }

    fn check(&self, carrier_hz: Option<f64>) -> anyhow::Result<()> {
        self.config()?;
        if let Some(f) = carrier_hz
            && !BANDS.iter().any(|&(lo, hi)| (lo..=hi).contains(&f))
        {
            log::warn!(
                "{:.1} kHz is outside the AM broadcast bands (LW 148.5-283.5 kHz, MW 525-1705 kHz, SW 2.3-26.1 MHz)",
                f / 1e3
            );
        }
        Ok(())
    }

    fn modulator(&self, sink: &SinkInfo) -> anyhow::Result<Box<dyn Modulator>> {
        let programme = programme::open(&self.programme, AmModulator::audio_rate_for(sink.sample_rate), sink)?;
        Ok(Box::new(AmModulator::new(self.config()?, programme).map_err(anyhow::Error::msg)?))
    }
}
