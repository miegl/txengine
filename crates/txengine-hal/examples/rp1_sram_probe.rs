//! Probe the RP1 shared SRAM (Raspberry Pi 5): is it in use, and which
//! system address does the RP1 DMA engine reach it at?
//!
//! Reads the SRAM through PCIe BAR2 twice, two seconds apart, to see whether
//! anything else writes to it; then copies a pattern within the SRAM with a
//! DMA channel using each candidate address, restoring everything afterwards.
//!
//! Run as root on a Pi 5: `rp1_sram_probe`

use std::os::fd::AsRawFd;
use std::time::Duration;

use txengine_hal::mmio::Mmio;
use txengine_hal::rp1::dma::Dmac;
use txengine_hal::rp1::regs::{self, dmac::*, lli};
use txengine_hal::rp1::{Rp1, find_pci_device};
use txengine_hal::sys::{Mapping, open, wait_until};

const LEN: usize = 0x10000;

fn snapshot(m: &Mmio) -> Vec<u32> {
    (0..LEN / 4).map(|i| m.read(i * 4)).collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let pci = find_pci_device()?;
    let res = pci.join("resource2");
    let fd = open(&res, libc::O_RDWR | libc::O_SYNC)?;
    let sram = Mmio::new(Mapping::map(fd.as_raw_fd(), 0, LEN, "RP1 shared SRAM")?, 0);

    let a = snapshot(&sram);
    std::thread::sleep(Duration::from_secs(2));
    let b = snapshot(&sram);
    let nonzero: Vec<usize> = (0..LEN / 4).filter(|&i| a[i] != 0).collect();
    let changed: Vec<usize> = (0..LEN / 4).filter(|&i| a[i] != b[i]).collect();
    println!("non-zero words: {} (first at {:#x?})", nonzero.len(), nonzero.first().map(|i| i * 4));
    let mut ranges: Vec<(usize, usize)> = Vec::new();
    for &i in &nonzero {
        match ranges.last_mut() {
            Some((_, end)) if *end + 16 >= i * 4 => *end = i * 4 + 4,
            _ => ranges.push((i * 4, i * 4 + 4)),
        }
    }
    for (s, e) in ranges.iter().take(20) {
        println!("  used {s:#06x}..{e:#06x}");
    }
    println!(
        "words changed within 2 s: {} {:x?}",
        changed.len(),
        changed.iter().take(16).map(|i| i * 4).collect::<Vec<_>>()
    );

    // DMA copy test in a quiet area.
    let (src, dst, l) = (0x8000usize, 0x9000usize, 0xa000usize);
    let saved: Vec<(usize, u32)> =
        [src, dst, l].iter().flat_map(|&o| (0..16).map(move |k| o + 4 * k)).map(|o| (o, sram.read(o))).collect();
    let rp1 = Rp1::open()?;
    let dmac = Dmac::new(rp1.block(regs::DMA));
    dmac.ensure_running(&rp1.clocks(), &rp1.block_set(regs::CLOCKS_MAIN));
    let ch = dmac.claim(None)?;
    println!("DMA channel {}", ch.ch);
    for base in [0xc0_2000_0000u64, 0xc0_4040_0000u64] {
        for k in 0..16 {
            sram.write(src + 4 * k, 0xa5a5_0000 + k as u32);
            sram.write(dst + 4 * k, 0);
        }
        let ctl = (WIDTH_32 << CTL_SRC_WIDTH_SHIFT)
            | (WIDTH_32 << CTL_DST_WIDTH_SHIFT)
            | (MSIZE_4 << CTL_SRC_MSIZE_SHIFT)
            | (MSIZE_4 << CTL_DST_MSIZE_SHIFT)
            | CTL_LLI_LAST
            | CTL_LLI_VALID;
        for k in 0..16 {
            sram.write(l + 4 * k, 0);
        }
        sram.write64(l + lli::SAR, base + src as u64);
        sram.write64(l + lli::DAR, base + dst as u64);
        sram.write(l + lli::BLOCK_TS, 15);
        sram.write64(l + lli::LLP, 0);
        sram.write64(l + lli::CTL, ctl);
        let cfg = (MULTBLK_LL << CFG_SRC_MULTBLK_SHIFT) | (MULTBLK_LL << CFG_DST_MULTBLK_SHIFT);
        ch.program_linked_list(base + l as u64, cfg)?;
        ch.enable();
        let done = wait_until(Duration::from_millis(50), "DMA copy", || !ch.is_enabled()).is_ok();
        if !done {
            ch.stop()?;
        }
        let ok = (0..16).all(|k| sram.read(dst + 4 * k) == 0xa5a5_0000 + k as u32);
        println!(
            "system address {base:#x}: finished {done}, data {} (INTSTATUS {:#x})",
            if ok { "copied" } else { "NOT copied" },
            ch.intstatus()
        );
        ch.clear_intstatus(0xffff_ffff);
    }
    for (o, v) in saved {
        sram.write(o, v);
    }
    Ok(())
}
