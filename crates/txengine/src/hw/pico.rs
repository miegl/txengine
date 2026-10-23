//! A Raspberry Pi Pico or Pico 2 on USB running the txengine firmware
//! (`firmware/pico`).

use std::time::Duration;

use anyhow::{Context, bail};
use txengine_core::RfSink;
use txengine_hal::pico;
use txengine_hal::pico::protocol as proto;

use super::{Backend, Found, Output};
use crate::cli::HardwareArgs;

pub struct Pico;

/// Default sample rate of AM: each sample lasts the whole number of carrier
/// cycles that comes closest.
const AM_SAMPLE_RATE: f64 = 100e3;

/// The options every mode takes on the Pico: the device, the GPIO and the
/// pad drive (mA).
fn common(args: &str, hw: &HardwareArgs) -> anyhow::Result<(Option<String>, u8, u8)> {
    if hw.pll.is_some() || hw.pacer.is_some() || hw.dma_channel.is_some() {
        bail!("--pll, --pacer and --dma-channel do not apply to the Pico");
    }
    if !matches!(hw.synthesis.as_deref(), None | Some("pll") | Some("pio")) {
        bail!("--synthesis {}: use pll or pio", hw.synthesis.as_deref().unwrap_or_default());
    }
    let gpio = match hw.gpio.as_slice() {
        [] => 21,
        &[g] => g,
        _ => bail!("the Pico transmits on one GPIO"),
    };
    Ok((
        Some(args.to_string()).filter(|s| !s.is_empty()),
        u8::try_from(gpio)?,
        [2, 2, 4, 4, 8, 8, 12, 12][hw.power as usize],
    ))
}

fn am_config(args: &str, hw: &HardwareArgs) -> anyhow::Result<pico::am::PicoAmConfig> {
    let (device, gpio, drive_ma) = common(args, hw)?;
    if hw.synthesis.as_deref() == Some("pll") {
        bail!("the Pico makes AM with the PIO (--synthesis pio): the PLL method only modulates the frequency");
    }
    Ok(pico::am::PicoAmConfig {
        carrier_hz: hw.freq_hz,
        device,
        gpio,
        drive_ma,
        sample_rate: hw.sample_rate.unwrap_or(AM_SAMPLE_RATE),
        buffer: Duration::from_millis(hw.buffer_ms),
    })
}

fn config(args: &str, hw: &HardwareArgs, max_offset_hz: f64) -> anyhow::Result<pico::fm::PicoFmConfig> {
    let (device, gpio, drive_ma) = common(args, hw)?;
    Ok(pico::fm::PicoFmConfig {
        carrier_hz: hw.freq_hz,
        device,
        gpio,
        drive_ma,
        sample_rate: hw.sample_rate,
        buffer: Duration::from_millis(hw.buffer_ms),
        max_offset_hz,
        interpolate: true,
        phase: hw.pico_phase,
        update_hz: hw.pico_update_rate,
        refdiv: hw.pico_refdiv,
        synthesis: match hw.synthesis.as_deref() {
            Some("pio") => pico::fm::Synthesis::Pio,
            _ => pico::fm::Synthesis::Pll,
        },
    })
}

impl Backend for Pico {
    fn name(&self) -> &'static str {
        "pico"
    }

    fn summary(&self) -> &'static str {
        "a Raspberry Pi Pico or Pico 2 on USB with the txengine firmware; pico:SERIAL or pico:PORT picks one"
    }

    fn discover(&self) -> Vec<Found> {
        pico::list()
            .into_iter()
            .map(|d| {
                let chip = if d.pid == proto::PID_RP2350 { proto::Chip::Rp2350 } else { proto::Chip::Rp2040 };
                Found {
                    device: format!("pico:{}", d.serial),
                    description: format!("Raspberry Pi {} ({}) on USB port {}", chip.board(), chip.name(), d.port()),
                }
            })
            .collect()
    }

    fn check(&self, args: &str, hw: &HardwareArgs) -> anyhow::Result<()> {
        common(args, hw).map(|_| ())
    }

    fn lock_name(&self, args: &str) -> anyhow::Result<String> {
        // By serial number, however the Pico was named.
        let d = pico::find(Some(args).filter(|a| !a.is_empty()))?;
        Ok(format!("txengine-pico-{}", d.serial))
    }

    fn open(&self, args: &str, hw: &HardwareArgs, output: Output) -> anyhow::Result<Box<dyn RfSink>> {
        Ok(match output {
            Output::Frequency { max_offset_hz } => {
                let cfg = config(args, hw, max_offset_hz)?;
                Box::new(pico::fm::PicoFmTransmitter::new(&cfg).context("Pico transmitter")?)
            }
            Output::Amplitude => {
                let cfg = am_config(args, hw)?;
                Box::new(pico::am::PicoAmTransmitter::new(&cfg).context("Pico AM transmitter")?)
            }
        })
    }

    fn probe(&self, args: &str, hw: &HardwareArgs, output: Output) -> anyhow::Result<()> {
        let (device, ..) = common(args, hw)?;
        let usb = pico::find(device.as_deref())?;
        let p = pico::Pico::open(&usb)?;
        let i = &p.info;
        println!("Device:   {}", p.describe());
        println!(
            "Clocks:   crystal {:.3} MHz, system {:.0} MHz, modulator up to {:.0} MHz, buffer {} samples",
            i.xosc_hz as f64 / 1e6,
            i.sys_hz as f64 / 1e6,
            i.max_update_hz as f64 / 1e6,
            i.buffer_samples
        );
        if i.watchdog_note != 0 {
            println!("Note:     the watchdog reset the firmware last time (step {:#010x})", i.watchdog_note);
        }
        let s = p.status()?;
        let fault = if s.fault == proto::Fault::None { String::new() } else { format!(" ({})", s.fault.describe()) };
        println!("State:    {:?}{fault}", s.state);
        let max_offset_hz = match output {
            Output::Frequency { max_offset_hz } => max_offset_hz,
            Output::Amplitude => {
                let cfg = am_config(args, hw)?;
                let p = pico::am::plan(cfg.carrier_hz, cfg.sample_rate, i)?;
                let (lo, hi) = pico::am::carrier_range(i, cfg.sample_rate).unwrap_or_default();
                println!(
                    "Carrier:  {:.4} kHz (AM) on GPIO{} from PIO, one pulse per carrier cycle: system clock \
                     {:.3} MHz (PLL_USB {:.0} MHz x {} / {}x{}) / {:.3}",
                    p.carrier_hz / 1e3,
                    cfg.gpio,
                    p.clock_hz / 1e6,
                    i.xosc_hz as f64 / 1e6,
                    p.clock.fbdiv,
                    p.clock.post_div1,
                    p.clock.post_div2,
                    p.period()
                );
                println!(
                    "Sampling: {:.3} S/s ({} carrier cycles per sample); AM carriers {:.0} kHz to {:.2} MHz",
                    p.sample_rate(),
                    p.cycles,
                    lo / 1e3,
                    hi / 1e6
                );
                return Ok(());
            }
        };
        let cfg = config(args, hw, max_offset_hz)?;
        if cfg.synthesis == pico::fm::Synthesis::Pio {
            let p = pico::fm::pio_plan_for(&cfg, i)?;
            println!(
                "Carrier:  {:.6} MHz on GPIO{} from a PIO bit stream at {:.3} Mbit/s \
                 (PLL_USB {:.0} MHz x {} / {}x{}, also the system clock)",
                p.carrier_hz / 1e6,
                cfg.gpio,
                p.bit_rate / 1e6,
                i.xosc_hz as f64 / 1e6,
                p.fbdiv,
                p.post_div1,
                p.post_div2
            );
            println!(
                "Aliases:  {} for an ideal bit stream (in the FM band the pin's output measures up to about -15 dBc)",
                p.describe_alias()
            );
            println!(
                "Modulation: {:.3} Hz per sample unit, ±{:.0} kHz; {:.1} S/s ({} words per sample)",
                p.resolution_hz,
                p.max_offset_hz() / 1e3,
                p.sample_rate(),
                p.sample_words
            );
            return Ok(());
        }
        let plan = pico::fm::plan_for(&cfg, i.xosc_hz as f64, i.max_update_hz as f64)?;
        println!(
            "Carrier:  {:.6} MHz on GPIO{} from PLL_USB: VCO {:.3} MHz = {:.0} MHz x {:.6}, division {} \
             (post /{}x{}, gpout /{})",
            plan.carrier_hz / 1e6,
            cfg.gpio,
            plan.fref_hz * plan.n / 1e6,
            plan.fref_hz / 1e6,
            plan.n,
            plan.division(),
            plan.post_div1,
            plan.post_div2,
            plan.gpout_div
        );
        println!(
            "Modulation: FBDIV {}..{} dithered at {:.0} MHz ({:.0} kHz per step), {:.3} Hz per sample unit, ±{:.0} kHz",
            plan.base,
            plan.base + 4,
            plan.update_hz() / 1e6,
            plan.step_hz / 1e3,
            plan.resolution_hz,
            plan.max_offset_hz() / 1e3
        );
        println!("Sampling: {:.1} S/s ({} updates per sample)", plan.sample_rate(), plan.sample_updates);
        if let Some(w) = plan.spur_warning(cfg.max_offset_hz) {
            println!("Warning:  {w}");
        }
        Ok(())
    }
}
