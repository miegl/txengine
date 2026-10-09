//! A Raspberry Pi Pico or Pico 2 on USB running the txengine firmware
//! (`firmware/pico`).

use std::time::Duration;

use anyhow::{Context, bail};
use txengine_core::RfSink;
use txengine_hal::pico;
use txengine_hal::pico::protocol as proto;

use super::{Backend, Found};
use crate::cli::HardwareArgs;

pub struct Pico;

fn config(args: &str, hw: &HardwareArgs, max_offset_hz: f64) -> anyhow::Result<pico::fm::PicoFmConfig> {
    if hw.pll.is_some() || hw.pacer.is_some() || hw.dma_channel.is_some() {
        bail!("--pll, --pacer and --dma-channel do not apply to the Pico");
    }
    let gpio = match hw.gpio.as_slice() {
        [] => 21,
        &[g] => g,
        _ => bail!("the Pico transmits on one GPIO"),
    };
    Ok(pico::fm::PicoFmConfig {
        carrier_hz: hw.freq * 1e6,
        device: Some(args.to_string()).filter(|s| !s.is_empty()),
        gpio: u8::try_from(gpio)?,
        drive_ma: [2, 2, 4, 4, 8, 8, 12, 12][hw.power as usize],
        sample_rate: hw.sample_rate,
        buffer: Duration::from_millis(hw.buffer_ms),
        max_offset_hz,
        interpolate: true,
        phase: hw.pico_phase,
        update_hz: hw.pico_update_rate,
        refdiv: hw.pico_refdiv,
        synthesis: match hw.synthesis.as_deref() {
            None | Some("pll") => pico::fm::Synthesis::Pll,
            Some("pio") => pico::fm::Synthesis::Pio,
            Some(other) => bail!("--synthesis {other}: use pll or pio"),
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
        config(args, hw, 0.0).map(|_| ())
    }

    fn lock_name(&self, args: &str) -> anyhow::Result<String> {
        // By serial number, however the Pico was named.
        let d = pico::find(Some(args).filter(|a| !a.is_empty()))?;
        Ok(format!("txengine-pico-{}", d.serial))
    }

    fn open(&self, args: &str, hw: &HardwareArgs, max_offset_hz: f64) -> anyhow::Result<Box<dyn RfSink>> {
        let cfg = config(args, hw, max_offset_hz)?;
        Ok(Box::new(pico::fm::PicoFmTransmitter::new(&cfg).context("Pico transmitter")?))
    }

    fn probe(&self, args: &str, hw: &HardwareArgs, max_offset_hz: f64) -> anyhow::Result<()> {
        let cfg = config(args, hw, max_offset_hz)?;
        let usb = pico::find(cfg.device.as_deref())?;
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
                "Aliases:  the strongest in the FM band {:.1} dBc at {:.2} MHz for an ideal bit stream \
                 (the pin's output measures up to about -15 dBc)",
                p.alias_dbc,
                p.alias_hz / 1e6
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
        Ok(())
    }
}
