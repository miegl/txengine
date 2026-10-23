//! Transmitter hardware: the backends txengine transmits with, and choosing
//! one (`--device BACKEND[:ARGS]`).
//!
//! A backend is a kind of hardware: this Raspberry Pi's GPIOs, a Pico on
//! USB, ... Each finds the devices of its kind that are present, checks the
//! options that apply to it (rejecting the others), and opens a device as
//! an [`RfSink`]. Without `--device`, txengine uses the only device found.
//! A new backend implements [`Backend`] and is added to [`BACKENDS`].

mod harmonic;
mod pi;
mod pico;

use anyhow::bail;
use log::{info, warn};
use txengine_core::{RfSink, SampleFormat};

use crate::cli::HardwareArgs;

/// What a transmission mode needs from the transmitter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Output {
    /// Frequency modulation (`SampleFormat::FrequencyOffset`), moving the
    /// carrier by up to ±`max_offset_hz`.
    Frequency { max_offset_hz: f64 },
    /// Amplitude modulation (`SampleFormat::Amplitude`).
    Amplitude,
}

impl Output {
    /// The output that makes the `n`-th harmonic's modulation from the
    /// fundamental: offsets `n` times smaller.
    fn at_fundamental(self, n: u32) -> Output {
        match self {
            Output::Frequency { max_offset_hz } => Output::Frequency { max_offset_hz: max_offset_hz / n as f64 },
            Output::Amplitude => Output::Amplitude,
        }
    }

    /// The kind of samples the modulator renders.
    pub fn format(self) -> SampleFormat {
        match self {
            Output::Frequency { .. } => SampleFormat::FrequencyOffset,
            Output::Amplitude => SampleFormat::Amplitude,
        }
    }
}

/// A device that is present.
pub struct Found {
    /// The `--device` value that selects it.
    pub device: String,
    pub description: String,
}

pub trait Backend: Sync {
    /// Its name in `--device`.
    fn name(&self) -> &'static str;
    /// One line for `txengine devices`.
    fn summary(&self) -> &'static str;
    /// The devices of this kind present now.
    fn discover(&self) -> Vec<Found>;
    /// Check the device arguments and that no option is given that does
    /// not apply.
    fn check(&self, args: &str, hw: &HardwareArgs) -> anyhow::Result<()>;
    /// Name of the lock that keeps two txengine processes off one device.
    fn lock_name(&self, args: &str) -> anyhow::Result<String>;
    /// Open the device for transmitting the `output` a mode needs. Called
    /// while the process is still single threaded.
    fn open(&self, args: &str, hw: &HardwareArgs, output: Output) -> anyhow::Result<Box<dyn RfSink>>;
    /// Print the device and its frequency plan, without transmitting.
    fn probe(&self, args: &str, hw: &HardwareArgs, output: Output) -> anyhow::Result<()>;
}

pub const BACKENDS: &[&dyn Backend] = &[&pi::Pi, &pico::Pico];

/// A backend and its arguments.
pub struct Selected {
    backend: &'static dyn Backend,
    args: String,
}

fn by_name(name: &str) -> anyhow::Result<&'static dyn Backend> {
    BACKENDS.iter().copied().find(|b| b.name().eq_ignore_ascii_case(name)).ok_or_else(|| {
        let names: Vec<_> = BACKENDS.iter().map(|b| b.name()).collect();
        anyhow::anyhow!("unknown device '{name}' (backends: {})", names.join(", "))
    })
}

/// Everything present, over all backends.
pub fn discover() -> Vec<Found> {
    BACKENDS.iter().flat_map(|b| b.discover()).collect()
}

/// The device `--device` names, or the only one present.
pub fn select(hw: &HardwareArgs) -> anyhow::Result<Selected> {
    let spec = match hw.device.as_deref() {
        Some(d) if !d.eq_ignore_ascii_case("auto") => d.to_string(),
        _ => {
            let found = discover();
            match found.as_slice() {
                [one] => one.device.clone(),
                [] => bail!(
                    "no transmitter found: run on a Raspberry Pi, or connect a Pico with the txengine firmware \
                     (`txengine devices` lists what is found)"
                ),
                many => {
                    let list: Vec<_> = many.iter().map(|f| format!("{} ({})", f.device, f.description)).collect();
                    bail!("several transmitters found, pick one with --device: {}", list.join(", "))
                }
            }
        }
    };
    let (name, args) = spec.split_once(':').unwrap_or((&spec, ""));
    let s = Selected { backend: by_name(name)?, args: args.to_string() };
    check_common(hw)?;
    s.backend.check(&s.args, hw)?;
    Ok(s)
}

fn check_common(hw: &HardwareArgs) -> anyhow::Result<()> {
    if !(1..=15).contains(&hw.harmonic) {
        bail!("--harmonic must be 1-15");
    }
    if !(100e3..=3e9).contains(&hw.freq_hz) {
        bail!("--freq must be 100 kHz to 3 GHz (e.g. 87.6 in MHz, or 999k)");
    }
    if hw.freq_hz / (hw.harmonic as f64) < 100e3 {
        bail!("--harmonic {} would make {:.1} kHz, below 100 kHz", hw.harmonic, hw.freq_hz / hw.harmonic as f64 / 1e3);
    }
    if hw.power > 7 {
        bail!("--power must be 0-7");
    }
    Ok(())
}

/// The options the hardware is opened with: for `--harmonic N` the
/// fundamental `--freq / N`.
fn fundamental(hw: &HardwareArgs, output: Output) -> anyhow::Result<(HardwareArgs, Output)> {
    if hw.harmonic == 1 {
        return Ok((hw.clone(), output));
    }
    if output == Output::Amplitude {
        bail!("AM is transmitted on the fundamental (--harmonic 1): a pulse's harmonics do not follow its width");
    }
    let mut at = hw.clone();
    at.freq_hz = hw.freq_hz / hw.harmonic as f64;
    Ok((at, output.at_fundamental(hw.harmonic)))
}

impl Selected {
    pub fn lock_name(&self) -> anyhow::Result<String> {
        self.backend.lock_name(&self.args)
    }

    pub fn open(&self, hw: &HardwareArgs, output: Output) -> anyhow::Result<Box<dyn RfSink>> {
        info!("txengine {}", crate::VERSION);
        let (at, output) = fundamental(hw, output)?;
        let sink = self.backend.open(&self.args, &at, output)?;
        if hw.harmonic == 1 {
            return Ok(sink);
        }
        warn!(
            "transmitting on harmonic {} of {:.6} MHz: the fundamental is stronger than {:.6} MHz; band-pass \
             filter the output",
            hw.harmonic,
            sink.info().carrier_hz / 1e6,
            sink.info().carrier_hz * hw.harmonic as f64 / 1e6
        );
        Ok(Box::new(harmonic::HarmonicSink::new(sink, hw.harmonic)))
    }

    pub fn probe(&self, hw: &HardwareArgs, output: Output) -> anyhow::Result<()> {
        let (at, output) = fundamental(hw, output)?;
        if hw.harmonic > 1 {
            println!(
                "Harmonic: {} of {:.6} MHz carries {:.6} MHz (band-pass filter the output)",
                hw.harmonic,
                at.freq_hz / 1e6,
                hw.freq_hz / 1e6
            );
        }
        self.backend.probe(&self.args, &at, output)
    }
}

/// `txengine devices`.
pub fn list_devices() -> anyhow::Result<()> {
    let found = discover();
    if found.is_empty() {
        println!("No transmitter found.");
    } else {
        let w = found.iter().map(|f| f.device.len()).max().unwrap_or(0);
        println!("Transmitters (--device):");
        for f in &found {
            println!("  {:w$}  {}", f.device, f.description);
        }
    }
    println!("\nBackends:");
    for b in BACKENDS {
        println!("  {:6} {}", b.name(), b.summary());
    }
    Ok(())
}
