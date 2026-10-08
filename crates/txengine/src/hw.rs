//! Selecting and configuring the hardware backend for the running board.

use std::time::Duration;

use anyhow::{Context, bail};
use log::info;
use txengine_core::RfSink;
use txengine_hal::bcm::{self, Bcm};
use txengine_hal::platform::{self, Platform, Soc};
use txengine_hal::rp1;

use crate::cli::HardwareArgs;

fn check_common(hw: &HardwareArgs) -> anyhow::Result<()> {
    if !(1.0..=250.0).contains(&hw.freq) {
        bail!("--freq must be given in MHz (e.g. 87.6)");
    }
    if !(65.0..=108.0).contains(&hw.freq) {
        log::warn!("{} MHz is outside the FM broadcast bands (65.8-108 MHz)", hw.freq);
    }
    if hw.power > 7 {
        bail!("--power must be 0-7");
    }
    Ok(())
}

fn rp1_config(hw: &HardwareArgs) -> anyhow::Result<rp1::fm::Rp1FmConfig> {
    let pll = match hw.pll.as_deref().map(str::to_ascii_lowercase).as_deref() {
        None | Some("audio") => rp1::fm::Rp1Pll::Audio,
        Some("video") => rp1::fm::Rp1Pll::Video,
        Some(other) => bail!("--pll {other}: the Raspberry Pi 5 supports 'audio' and 'video'"),
    };
    let default_gpio = if pll == rp1::fm::Rp1Pll::Video { 6 } else { 4 };
    Ok(rp1::fm::Rp1FmConfig {
        carrier_hz: hw.freq * 1e6,
        gpios: if hw.gpio.is_empty() { vec![default_gpio] } else { hw.gpio.clone() },
        pll,
        drive: rp1::gpio::Drive::from_level(hw.power),
        sample_rate: hw.sample_rate.unwrap_or(250_000.0),
        buffer: Duration::from_millis(hw.buffer_ms),
        max_offset_hz: 100_000.0,
        dma_channel: hw.dma_channel,
        tick: 0,
    })
}

/// Warn about a weak power supply or overheating: both slow the CPU down,
/// and under-voltage can disturb the clocks.
fn warn_power() {
    if let Some(p) = platform::PowerStatus::read() {
        let problems = p.problems();
        if !problems.is_empty() {
            log::warn!("firmware reports {} (get_throttled {:#x}); use a stronger power supply / better cooling", problems.join(", "), p.0);
        }
    }
}

/// Open the transmitter for the detected platform. Must be called while
/// the process is still single threaded (it forks the shutdown guardian).
pub fn open_transmitter(hw: &HardwareArgs) -> anyhow::Result<Box<dyn RfSink>> {
    check_common(hw)?;
    let plat = platform::detect()?;
    info!("{} ({}), crystal {:.3} MHz", plat.model, plat.soc, plat.osc_hz / 1e6);
    warn_power();
    match plat.soc {
        Soc::Bcm2712 => {
            let cfg = rp1_config(hw)?;
            Ok(Box::new(rp1::fm::Rp1FmTransmitter::new(&cfg, plat.osc_hz).context("RP1 transmitter")?))
        }
        Soc::Bcm2835 => bail!("the original Raspberry Pi 1/Zero (ARMv6) is not supported"),
        _ => bcm_transmitter(&plat, hw),
    }
}

fn bcm_config(hw: &HardwareArgs) -> anyhow::Result<bcm::fm::BcmFmConfig> {
    use bcm::cprman::PllId;
    use bcm::fm::{BcmMethod, Pacer};
    let method = match hw.pll.as_deref().map(str::to_ascii_lowercase).as_deref() {
        None | Some("auto") => None,
        Some("a") => Some(BcmMethod::Pll(PllId::A)),
        Some("c") => Some(BcmMethod::Pll(PllId::C)),
        Some("d") => Some(BcmMethod::Pll(PllId::D)),
        Some("clkdiv") => Some(BcmMethod::ClkDiv),
        Some(other) => bail!("--pll {other}: use a, c, d or clkdiv"),
    };
    let pacer = match hw.pacer.as_deref() {
        Some("pcm") => Pacer::Pcm,
        Some("pwm") | None => Pacer::Pwm,
        Some(other) => bail!("--pacer {other}: use pwm or pcm"),
    };
    Ok(bcm::fm::BcmFmConfig {
        carrier_hz: hw.freq * 1e6,
        gpios: if hw.gpio.is_empty() { vec![4] } else { hw.gpio.clone() },
        method,
        pacer,
        drive: hw.power,
        sample_rate: hw.sample_rate,
        buffer: Duration::from_millis(hw.buffer_ms),
        max_offset_hz: 100_000.0,
        dma_channel: hw.dma_channel,
        max_vco_change: 0.03,
    })
}

fn bcm_transmitter(plat: &Platform, hw: &HardwareArgs) -> anyhow::Result<Box<dyn RfSink>> {
    let cfg = bcm_config(hw)?;
    let b = Bcm::open(plat)?;
    Ok(Box::new(bcm::fm::BcmFmTransmitter::new(&cfg, &b).context("BCM transmitter")?))
}

/// Print the detected platform and frequency plan without touching the hardware.
pub fn probe(hw: &HardwareArgs) -> anyhow::Result<()> {
    check_common(hw)?;
    let plat = platform::detect()?;
    println!("Board:    {}", plat.model);
    println!("SoC:      {}", plat.soc);
    println!("Crystal:  {:.3} MHz", plat.osc_hz / 1e6);
    if let Some(p) = platform::PowerStatus::read() {
        let problems = p.problems();
        println!("Power:    {}", if problems.is_empty() { "ok".to_string() } else { format!("{} ({:#x})", problems.join(", "), p.0) });
    }
    match crate::rt::kernel_clock() {
        Some(k) if k.synchronised => println!(
            "Time:     system clock synchronised (NTP frequency correction {:+.3} ppm); --ppm auto measures the crystal",
            k.frequency * 1e6
        ),
        Some(_) => println!("Time:     system clock not synchronised; --ppm auto cannot measure the crystal error"),
        None => {}
    }
    match plat.soc {
        Soc::Bcm2712 => {
            let cfg = rp1_config(hw)?;
            let pci = rp1::find_pci_device()?;
            println!("RP1:      {}", pci.display());
            let gpclk = cfg
                .gpios
                .iter()
                .map(|&g| rp1::gpio::gpclk_for_pin(g).map(|(n, _)| n))
                .collect::<Option<Vec<_>>>()
                .context("unsupported output GPIO")?;
            let xosc = plat.osc_hz;
            let plan = rp1::fm::plan_clocks(cfg.pll, gpclk[0], cfg.carrier_hz, xosc, cfg.max_offset_hz)?;
            println!(
                "Carrier:  {:.6} MHz on GPIO{:?} via clk_gp{} from {}",
                plan.carrier_hz / 1e6,
                cfg.gpios,
                plan.gpclk,
                plan.pll.name()
            );
            println!(
                "PLL:      VCO {:.4} MHz = {:.0} MHz x ({} + {:#08x}/2^24), division {} (post /{}x{}{}, gpclk /{})",
                plan.vco_hz / 1e6,
                plan.ref_hz / 1e6,
                plan.fbdiv_int,
                plan.frac_center as u32,
                plan.n,
                plan.div1,
                plan.div2,
                if plan.phase_div > 1 { ", phase /2" } else { "" },
                plan.gp_div
            );
            println!(
                "Modulation: {:.3} Hz per step, ±{:.0} kHz range",
                1.0 / plan.frac_per_hz,
                plan.max_offset_hz / 1e3
            );
            let cycles = (xosc / cfg.sample_rate).round();
            println!("Sampling: {:.3} S/s (xosc / {cycles}) paced by RP1 DMA tick", xosc / cycles);
            let busy = rp1::dma::Dmac::kernel_channels_in_use();
            println!("DMA:      kernel channels in use {busy:#04x}");
        }
        _ => bcm_probe(&plat, hw)?,
    }
    Ok(())
}

fn bcm_probe(plat: &Platform, hw: &HardwareArgs) -> anyhow::Result<()> {
    use bcm::cprman::{Cprman, Pll, PllId};
    let cfg = bcm_config(hw)?;
    let b = Bcm::open(plat)?;
    let cm = Cprman::new(b.cm.clone());
    let osc = plat.osc_hz;
    let (gpclk, _) = bcm::gpclk_for_pin(cfg.gpios[0]).context("unsupported output GPIO")?;
    let gp_ctl = bcm::regs::cm::gp(gpclk);
    println!("Output:   GPIO{:?} via GPCLK{gpclk}", cfg.gpios);
    for id in [PllId::A, PllId::C, PllId::D] {
        let p = Pll::new(&cm, id, osc, b.bcm2711);
        let running = p.is_powered() && p.is_locked();
        if !running {
            println!("{}:     idle (powered down or unlocked)", id.name());
        } else {
            let u = p.users(gp_ctl);
            println!(
                "{}:     VCO {:.3} MHz, PER {}, users: PER [{}], other channels [{}]{}",
                id.name(),
                p.vco_hz() / 1e6,
                p.per_divider().map_or("off".into(), |d| format!("/{d} = {:.3} MHz", p.vco_hz() / d as f64 / 1e6)),
                u.per.join(", "),
                u.other.join(", "),
                if u.gpu_only && !u.is_empty() { " (GPU only)" } else { "" }
            );
        }
        match bcm::fm::plan_pll(&p, gpclk, cfg.carrier_hz, osc, cfg.max_offset_hz, cfg.max_vco_change) {
            Ok(plan) => println!(
                "          plan: VCO {:.3} MHz{}, PER /{}, GPCLK /{}, carrier {:.6} MHz, step {:.2} Hz{}",
                plan.vco_new / 1e6,
                if plan.vco_old > 0.0 { format!(" ({:+.2} %)", (plan.vco_new / plan.vco_old - 1.0) * 100.0) } else { String::new() },
                plan.per_div,
                plan.gp_div,
                plan.carrier_hz / 1e6,
                plan.step_hz,
                plan.loop_ki.map_or(String::new(), |k| format!(", loop KI {} -> {k}", p.loop_ki()))
            ),
            Err(e) => println!("          plan: {e}"),
        }
    }
    let plld = Pll::new(&cm, PllId::D, osc, b.bcm2711);
    if let Ok(plan) = bcm::fm::plan_clkdiv(&plld, gpclk, cfg.carrier_hz, cfg.max_offset_hz) {
        println!("clkdiv:   GPCLK divider {:.4} from PLLD_PER, step {:.0} Hz (dithered)", plan.center / 4096.0, plan.step_hz);
    }
    Ok(())
}
