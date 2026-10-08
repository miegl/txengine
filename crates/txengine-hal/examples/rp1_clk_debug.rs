//! Dump and exercise RP1 clock generator state (diagnostics).

use txengine_hal::rp1::Rp1;
use txengine_hal::rp1::clocks::{FcSource, Pll, PllId, measure};
use txengine_hal::rp1::regs::{self, clk::*};

fn dump(clocks: &txengine_hal::mmio::Mmio, label: &str) {
    println!("--- {label}");
    println!("GPCLK_OE_CTRL {:#010x}", clocks.read(GPCLK_OE_CTRL));
    for n in 0..3 {
        println!(
            "gp{n}: ctrl {:#010x} div_int {:#x} div_frac {:#x} sel {:#x}",
            clocks.read(gp_ctrl(n)),
            clocks.read(gp_div_int(n)),
            clocks.read(gp_div_frac(n)),
            clocks.read(gp_sel(n))
        );
    }
    for (name, base) in [("sys", PLL_SYS), ("audio", PLL_AUDIO), ("video", PLL_VIDEO)] {
        println!(
            "pll_{name}: cs {:#010x} pwr {:#x} int {} frac {:#x} prim {:#x} sec {:#x}",
            clocks.read(base + PLL_CS),
            clocks.read(base + PLL_PWR),
            clocks.read(base + PLL_FBDIV_INT),
            clocks.read(base + PLL_FBDIV_FRAC),
            clocks.read(base + PLL_PRIM),
            clocks.read(base + PLL_SEC)
        );
    }
    for (name, fc) in [
        ("clk_gp0", FcSource::gp(0)),
        ("clk_gp1", FcSource::gp(1)),
        ("clk_gp2", FcSource::gp(2)),
        ("pll_audio", FcSource::PLL_AUDIO),
        ("pll_video", FcSource::PLL_VIDEO),
        ("pll_sys", FcSource { idx: 0, src: 2 }),
        ("clk_sys", FcSource { idx: 0, src: 4 }),
    ] {
        match measure(clocks, fc, 50e6) {
            Ok(hz) => println!("FC {name}: {:.3} MHz", hz / 1e6),
            Err(e) => println!("FC {name}: {e}"),
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rp1 = Rp1::open()?;
    let clocks = rp1.clocks();
    let set = rp1.block_set(regs::CLOCKS_MAIN);
    let clr = rp1.block_clr(regs::CLOCKS_MAIN);
    dump(&clocks, "initial");
    let arg = std::env::args().nth(1).unwrap_or_default();
    if arg == "xosc" || arg == "audio" {
        let pll = Pll::new(&clocks, PllId::Audio, 50e6);
        let saved = pll.snapshot();
        let gp = txengine_hal::rp1::clocks::ClockGen::new(&clocks, &set, &clr, 0);
        let gsaved = gp.snapshot();
        let (src, div) = if arg == "xosc" { (0, 5) } else { (7, 1) };
        if arg == "audio" {
            pll.configure(31, 0x0089_374b, 6, 3)?;
        }
        let r = gp.configure(src, div, 0);
        println!("configure(auxsrc {src}, div {div}): {r:?}");
        dump(&clocks, "after configure");
        gp.restore(&gsaved);
        pll.restore(&saved)?;
        dump(&clocks, "restored");
    }
    Ok(())
}
