//! Command line options of the NFM mode, and building its modulator.

use anyhow::{Context, bail};
use clap::{Args, Subcommand};
use txengine_core::{Modulator, SinkInfo};
use txengine_nfm::baseband::SUBAUDIO_SHARE;
use txengine_nfm::{DcsCode, NfmConfig, NfmModulator, Subaudio};

use super::MAX_OFFSET_HZ;
use crate::cli::{ProbeArgs, Render, ToneArgs, Tx};
use crate::hw::Output;
use crate::mode::Mode;
use crate::programme::{self, ProgrammeArgs};

// Parsed once at start-up: the size of the variants does not matter.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Subcommand)]
pub enum Command {
    /// Transmit.
    Tx(Tx<NfmArgs>),
    /// Render the signal to a file instead of transmitting (for testing).
    Render(Render<NfmArgs>),
    /// Show the detected hardware and its plan for narrow-band FM, without
    /// transmitting.
    Probe(ProbeArgs),
    /// Transmit test tones as raw frequency deviation (no audio processing,
    /// no compensation) from the transmitter planned for narrow-band FM.
    Tone(ToneArgs),
}

// Programme and modulation options of `nfm tx` and `nfm render`.
#[derive(Debug, Clone, Args)]
pub struct NfmArgs {
    #[command(flatten)]
    pub programme: ProgrammeArgs,

    // Modulation -----------------------------------------------------------
    /// Peak frequency deviation in Hz, voice and CTCSS/DCS together: 2500
    /// in 12.5 kHz channels (PMR446, most land mobile and narrow amateur
    /// FM), 5000 in 25 kHz channels (marine VHF, wide amateur FM), 2000 on
    /// CB.
    #[arg(long, default_value_t = 2_500.0, value_name = "HZ", help_heading = "Modulation")]
    pub deviation: f64,
    /// Audio bandwidth in Hz: the highest audio frequency transmitted (the
    /// voice band starts at 300 Hz).
    #[arg(long, default_value_t = 3_000.0, value_name = "HZ", help_heading = "Modulation")]
    pub bandwidth: f64,
    /// Pre-emphasis: a time constant in µs (750: 6 dB per octave across
    /// the voice band, as receivers de-emphasise it) or "off". A full-scale
    /// 1 kHz tone deviates the full voice deviation either way.
    #[arg(long, default_value = "750", value_name = "US|off", help_heading = "Modulation")]
    pub preemph: String,
    /// CTCSS tone in Hz (67.0-254.1, e.g. 88.5), sent below the voice band
    /// so that receivers with tone squelch open for this transmitter.
    #[arg(long, value_name = "HZ", conflicts_with = "dcs", help_heading = "Squelch signalling")]
    pub ctcss: Option<f64>,
    /// DCS code: three octal digits, normal or inverted (023, 023N, 754I).
    /// A 1 bit deviates the carrier upwards; a receiver that sees the
    /// polarity the other way decodes the inverted code.
    #[arg(long, value_name = "CODE", help_heading = "Squelch signalling")]
    pub dcs: Option<String>,
    /// Peak deviation of the CTCSS tone or the DCS code in Hz [default:
    /// 15 % of --deviation]; the voice gets the rest.
    #[arg(long, value_name = "HZ", help_heading = "Squelch signalling")]
    pub subaudio_deviation: Option<f64>,
    /// Input gain in dB.
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true, value_name = "DB", help_heading = "Modulation")]
    pub gain: f64,
    /// Disable the limiter; overload is then hard clipped.
    #[arg(long, help_heading = "Modulation")]
    pub no_limiter: bool,
}

/// Pre-emphasis option: a time constant in µs, or "off".
fn parse_preemphasis(s: &str) -> anyhow::Result<Option<f64>> {
    let t = s.trim().to_ascii_lowercase();
    if matches!(t.as_str(), "off" | "none" | "0") {
        return Ok(None);
    }
    let us: f64 = t
        .trim_end_matches("us")
        .trim_end_matches("µs")
        .parse()
        .with_context(|| format!("--preemph must be a time constant in µs or off, got '{s}'"))?;
    Ok(Some(us * 1e-6))
}

impl NfmArgs {
    fn config(&self) -> anyhow::Result<NfmConfig> {
        let subaudio = match (self.ctcss, &self.dcs) {
            (Some(f), _) => Subaudio::Ctcss(f),
            (None, Some(d)) => Subaudio::Dcs(DcsCode::parse(d).map_err(anyhow::Error::msg)?),
            (None, None) => Subaudio::Off,
        };
        if self.subaudio_deviation.is_some() && subaudio == Subaudio::Off {
            bail!("--subaudio-deviation needs --ctcss or --dcs");
        }
        let cfg = NfmConfig {
            deviation: self.deviation,
            bandwidth: self.bandwidth,
            preemphasis: parse_preemphasis(&self.preemph)?,
            subaudio,
            subaudio_deviation: self.subaudio_deviation.unwrap_or(SUBAUDIO_SHARE * self.deviation),
            gain_db: self.gain,
            limiter: !self.no_limiter,
        };
        cfg.check().map_err(anyhow::Error::msg)?;
        Ok(cfg)
    }
}

impl Mode for NfmArgs {
    fn output(&self) -> Output {
        Output::Frequency { max_offset_hz: MAX_OFFSET_HZ }
    }

    fn check(&self, _carrier_hz: Option<f64>) -> anyhow::Result<()> {
        let cfg = self.config()?;
        if let Some(w) = txengine_nfm::nonstandard_tone(cfg.subaudio) {
            log::warn!("{w}");
        }
        Ok(())
    }

    fn modulator(&self, sink: &SinkInfo) -> anyhow::Result<Box<dyn Modulator>> {
        let programme = programme::open(&self.programme, NfmModulator::audio_rate_for(sink.sample_rate), sink)?;
        Ok(Box::new(NfmModulator::new(self.config()?, programme).map_err(anyhow::Error::msg)?))
    }
}

#[cfg(test)]
mod tests {
    use txengine_nfm::baseband::DEFAULT_PREEMPHASIS;

    use super::*;

    #[test]
    fn preemphasis_parsing() {
        assert_eq!(parse_preemphasis("off").unwrap(), None);
        assert!((parse_preemphasis("750").unwrap().unwrap() - DEFAULT_PREEMPHASIS).abs() < 1e-12);
        assert!((parse_preemphasis("530us").unwrap().unwrap() - 530e-6).abs() < 1e-12);
        assert!(parse_preemphasis("fast").is_err());
    }
}
