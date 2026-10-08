//! Building the WFM modulator from command line options.

use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, bail};
use txengine_audio::{AudioInput, AudioOptions, EndOfInput, InputSpec};
use txengine_core::Modulator;
use txengine_fm::{MpxConfig, Programme, StereoMode, WfmModulator};
use txengine_rds::encoder::{af_code, encode_rt, encode8};
use txengine_rds::{DecoderInfo, RdsParams};

use super::MAX_OFFSET_HZ;
use super::args::{OnEnd, StereoArg, WfmArgs};
use crate::mode::Mode;

fn flag(v: u8, name: &str) -> anyhow::Result<bool> {
    match v {
        0 => Ok(false),
        1 => Ok(true),
        _ => bail!("--{name} must be 0 or 1"),
    }
}

fn hex(s: &str, what: &str) -> anyhow::Result<u32> {
    u32::from_str_radix(s.trim().trim_start_matches("0x").trim_start_matches("0X"), 16)
        .with_context(|| format!("{what} must be hexadecimal, got '{s}'"))
}

/// Pre-emphasis option: "eu" (50 µs), "us" (75 µs), "off", or a time
/// constant in µs.
fn parse_preemphasis(s: &str) -> anyhow::Result<Option<f64>> {
    Ok(match s.to_ascii_lowercase().as_str() {
        "eu" | "50" => Some(50e-6),
        "us" | "75" => Some(75e-6),
        "off" | "none" | "0" => None,
        other => {
            let v: f64 = other
                .trim_end_matches("us")
                .trim_end_matches("µs")
                .parse()
                .with_context(|| format!("--preemph must be eu, us, off or a time constant in µs, got '{s}'"))?;
            if !(1.0..=1000.0).contains(&v) {
                bail!("--preemph time constant must be 1-1000 µs, got '{s}'");
            }
            Some(v * 1e-6)
        }
    })
}

fn mpx_config(p: &WfmArgs) -> anyhow::Result<MpxConfig> {
    if !(10_000.0..=MAX_OFFSET_HZ).contains(&p.deviation) {
        bail!("--deviation must be 10000-{MAX_OFFSET_HZ} Hz (BS.450 specifies 75000 or 50000)");
    }
    if !(0.0..=7_500.0).contains(&p.rds_level) {
        bail!("--rds-level must be 0-7500 Hz (EN 50067 1.3)");
    }
    if !(0.0..=0.15 * p.deviation).contains(&p.pilot) {
        bail!("--pilot must be at most 15 % of the deviation");
    }
    let stereo = if p.mono {
        StereoMode::Mono
    } else {
        match p.stereo {
            StereoArg::Auto => StereoMode::Auto,
            StereoArg::Mono => StereoMode::Mono,
            StereoArg::Stereo => StereoMode::Stereo,
        }
    };
    Ok(MpxConfig {
        max_deviation: p.deviation,
        audio_deviation: p.audio_deviation,
        pilot_deviation: p.pilot,
        rds_deviation: p.rds_level,
        preemphasis: parse_preemphasis(&p.preemph)?,
        cutoff: p.cutoff,
        stereo,
        gain_db: p.gain,
        limiter: !p.no_limiter,
        hold_compensation: !p.no_hold_compensation,
        // Filled in from the sink when the modulator is configured.
        output_response_db: Vec::new(),
    })
}

fn rds_params(p: &WfmArgs) -> anyhow::Result<RdsParams> {
    let pi = hex(&p.pi, "--pi")?;
    if pi > 0xffff {
        bail!("--pi must be 4 hex digits");
    }
    let di = hex(&p.di, "--di")?;
    if di > 15 {
        bail!("--di must be 0-F");
    }
    let af = p.af.iter().map(|&f| af_code(f)).collect::<Result<Vec<_>, _>>()?;
    if af.len() > 25 {
        bail!("at most 25 alternative frequencies");
    }
    let pty = txengine_rds::pty::parse(&p.pty).with_context(|| format!("--pty: unknown programme type '{}'", p.pty))?;
    let ecc = p.ecc.as_deref().map(|s| hex(s, "--ecc")).transpose()?;
    if ecc.is_some_and(|e| e > 0xff) {
        bail!("--ecc must be one byte");
    }
    let lic = p.lic.as_deref().map(|s| hex(s, "--lic")).transpose()?;
    if lic.is_some_and(|l| l > 0xfff) {
        bail!("--lic must be at most 0xFFF");
    }
    Ok(RdsParams {
        pi: pi as u16,
        ps: encode8(&p.ps).context("--ps")?,
        rt: encode_rt(&p.rt).context("--rt")?,
        pty,
        tp: flag(p.tp, "tp")?,
        ta: flag(p.ta, "ta")?,
        ms: flag(p.ms, "ms")?,
        di: DecoderInfo {
            stereo: false,
            artificial_head: di & 2 != 0,
            compressed: di & 4 != 0,
            dynamic_pty: di & 8 != 0,
        },
        af,
        ptyn: p.ptyn.as_deref().map(encode8).transpose().context("--ptyn")?,
        ecc: ecc.map(|e| e as u8),
        lic: lic.map(|l| l as u16),
        ct: flag(p.ct, "ct")?,
    })
}

fn end_policy(p: &WfmArgs, stdin: bool, realtime: bool) -> EndOfInput {
    if let Some(e) = p.on_end {
        return match e {
            OnEnd::Repeat => EndOfInput::Repeat,
            OnEnd::Wait => EndOfInput::Wait,
            OnEnd::Stop => EndOfInput::Stop,
        };
    }
    // A rendered file ends with the input.
    if !realtime {
        EndOfInput::Stop
    } else if stdin {
        EndOfInput::Wait
    } else {
        EndOfInput::Repeat
    }
}

/// Open the programme input at the modulator's audio rate. `realtime`:
/// for a transmitter, whose crystal error is corrected while running.
fn open_programme(p: &WfmArgs, audio_rate: f64, realtime: bool) -> anyhow::Result<Programme> {
    let (spec, stdin) = match (&p.audio, p.tone) {
        (_, Some(f)) => (InputSpec::Tone { freq: f, amplitude: 0.5, stereo: false }, false),
        (None, None) => return Ok(Programme::None),
        (Some(a), None) if a == "-" => match (p.raw_rate, p.raw_channels) {
            (Some(rate), Some(channels)) => (InputSpec::RawStdin { rate, channels }, true),
            _ => (InputSpec::Stdin, true),
        },
        (Some(a), None) => (InputSpec::File(PathBuf::from(a)), false),
    };
    let end = end_policy(p, stdin, realtime);
    let opts =
        AudioOptions { end, buffer: Duration::from_millis(500), adjustable_rate: realtime, ..Default::default() };
    let input = AudioInput::open(&spec, audio_rate, &opts)?;
    Ok(Programme::Audio(input, end))
}

impl Mode for WfmArgs {
    fn max_offset_hz(&self) -> f64 {
        MAX_OFFSET_HZ
    }

    fn check(&self, carrier_mhz: Option<f64>) -> anyhow::Result<()> {
        mpx_config(self)?;
        rds_params(self)?;
        if let Some(f) = carrier_mhz
            && !(65.0..=108.0).contains(&f)
        {
            log::warn!("{f} MHz is outside the FM broadcast bands (65.8-108 MHz)");
        }
        Ok(())
    }

    fn modulator(&self, sample_rate: f64, realtime: bool) -> anyhow::Result<Box<dyn Modulator>> {
        let cfg = mpx_config(self)?;
        let rds = rds_params(self)?;
        let programme = open_programme(self, WfmModulator::audio_rate_for(sample_rate), realtime)?;
        let mut m = WfmModulator::new(cfg, programme, rds, self.rds != 0).map_err(anyhow::Error::msg)?;
        m.set_local_offset_fn(Box::new(crate::rt::local_offset));
        Ok(Box::new(m))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preemphasis_parsing() {
        assert_eq!(parse_preemphasis("eu").unwrap(), Some(50e-6));
        assert_eq!(parse_preemphasis("US").unwrap(), Some(75e-6));
        assert_eq!(parse_preemphasis("off").unwrap(), None);
        assert!(parse_preemphasis("3185").is_err());
        assert!((parse_preemphasis("75us").unwrap().unwrap() - 75e-6).abs() < 1e-12);
    }
}
