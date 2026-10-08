//! Characterise the RP1 DMA engine with tick pacing (Raspberry Pi 5).
//!
//! Performs a memory-to-memory transfer paced by DMA tick 0 through a
//! circular linked list and reports: the achieved pacing rate, whether LLI
//! write-back clears the VALID bit, data integrity, and the behaviour when
//! the producer stops re-validating LLIs (underrun).
//!
//! Run as root on a Pi 5: `rp1_dma_probe [tick_cycles] [block_words] [blocks]`

use std::time::{Duration, Instant};

use txengine_hal::mem::pinned::PinnedMemory;
use txengine_hal::mem::{DmaMemory, DmaMemoryExt};
use txengine_hal::rp1::Rp1;
use txengine_hal::rp1::dma::Dmac;
use txengine_hal::rp1::regs::{self, dmac::*, lli};
use txengine_hal::rp1::ticks::DmaTick;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let cycles: u32 = args.get(1).map(|s| s.parse()).transpose()?.unwrap_or(200);
    let block_words: usize = args.get(2).map(|s| s.parse()).transpose()?.unwrap_or(1024);
    let blocks: usize = args.get(3).map(|s| s.parse()).transpose()?.unwrap_or(4);

    let rp1 = Rp1::open()?;
    let clocks = rp1.clocks();
    let clocks_set = rp1.block_set(regs::CLOCKS_MAIN);
    let dmac = Dmac::new(rp1.block(regs::DMA));
    println!(
        "DMAC ID {:#x} COMPVER {:#x} CFG {:#x} CHEN {:#x}",
        dmac.id(),
        dmac.compver(),
        rp1.block(regs::DMA).read(CFG),
        dmac.chen()
    );
    println!("CLK_DMA_CTRL {:#x}", clocks.read(regs::clk::CLK_DMA_CTRL));
    println!("kernel channels in use: {:#x}", Dmac::kernel_channels_in_use());
    if dmac.ensure_running(&clocks, &clocks_set) {
        println!("controller was suspended: enabled clk_dma / DMAC_EN");
    }

    let tick = DmaTick::new(rp1.block(regs::TICKS), rp1.block(regs::DMA_TICK), 0);
    println!("tick0 previously in use: {}", tick.in_use());

    let page = txengine_hal::sys::page_size();
    assert!(block_words * 4 <= page && page.is_multiple_of(block_words * 4), "block must fit a page");
    let lli_bytes = txengine_hal::sys::round_up(blocks * lli::SIZE, page);
    let data_bytes = blocks * block_words * 4;
    let mem = PinnedMemory::alloc(lli_bytes + 2 * data_bytes, regs::HOST_MEM_DMA_OFFSET)?;
    println!("{}", mem.describe());
    let src_off = lli_bytes;
    let dst_off = lli_bytes + data_bytes;
    let base = mem.dma_addr_at(0);

    for i in 0..blocks * block_words {
        mem.write_u32(src_off + i * 4, 0x1000_0000 + i as u32);
        mem.write_u32(dst_off + i * 4, 0);
    }
    let ctl = (WIDTH_32 << CTL_SRC_WIDTH_SHIFT)
        | (WIDTH_32 << CTL_DST_WIDTH_SHIFT)
        | (MSIZE_4 << CTL_SRC_MSIZE_SHIFT)
        | (MSIZE_1 << CTL_DST_MSIZE_SHIFT)
        | CTL_ARLEN_EN
        | (3u64 << CTL_ARLEN_SHIFT)
        | CTL_LLI_VALID;
    for b in 0..blocks {
        let l = b * lli::SIZE;
        mem.zero(l, lli::SIZE);
        mem.write_u64(l + lli::SAR, mem.dma_addr_at(src_off + b * block_words * 4));
        mem.write_u64(l + lli::DAR, mem.dma_addr_at(dst_off + b * block_words * 4));
        mem.write_u32(l + lli::BLOCK_TS, block_words as u32 - 1);
        mem.write_u64(l + lli::LLP, mem.dma_addr_at(((b + 1) % blocks) * lli::SIZE));
        mem.write_u64(l + lli::CTL, ctl);
    }
    mem.flush(0, mem.len());

    let ch = dmac.claim(None)?;
    println!("using channel {}", ch.ch);
    let cfg = (MULTBLK_LL << CFG_SRC_MULTBLK_SHIFT)
        | (MULTBLK_LL << CFG_DST_MULTBLK_SHIFT)
        | ((tick.dreq() as u64) << CFG_DST_PER_SHIFT)
        | (TT_FC_MEM_TO_PER_DMAC << CFG_TT_FC_SHIFT)
        | (7u64 << CFG_PRIORITY_SHIFT);
    ch.program_linked_list(base, cfg)?;
    for shift in [49u32, 52] {
        ch.write_cfg(cfg & !CFG_PRIORITY_MASK | (7u64 << shift));
        println!("priority at bit {shift}: read-back CFG {:#018x}", ch.cfg());
    }
    ch.write_cfg(cfg);
    tick.start(cycles)?;
    let expected = 50e6 / cycles as f64;
    println!("tick: {cycles} cycles -> expected {expected:.3} Hz; block {block_words} words x {blocks}");

    let total_words = blocks * block_words;
    let block_addrs: Vec<u64> = (0..blocks).map(|b| mem.dma_addr_at(src_off + b * block_words * 4)).collect();
    let pos = |sar: u64| -> Option<usize> {
        block_addrs.iter().enumerate().find_map(|(b, &a)| {
            let w = sar.checked_sub(a)? as usize / 4;
            (w <= block_words).then_some((b * block_words + w) % total_words)
        })
    };

    let t0 = Instant::now();
    ch.enable();
    // Phase 1: keep re-validating LLIs behind the DMA for ~3 s and measure rate.
    let mut consumed: u64 = 0;
    let mut last = 0usize;
    let mut last_report = Instant::now();
    let mut report_consumed = 0u64;
    let mut wb_cleared = 0;
    let mut wb_seen = 0;
    let mut first_status = None;
    while t0.elapsed() < Duration::from_secs(3) {
        std::thread::sleep(Duration::from_millis(2));
        let sar = ch.sar();
        let Some(p) = pos(sar) else {
            println!("SAR {sar:#x} outside source buffer (LLP {:#x}, en {})", ch.llp(), ch.is_enabled());
            continue;
        };
        let delta = (p + total_words - last) % total_words;
        consumed += delta as u64;
        last = p;
        let cur_block = p / block_words;
        // Re-validate the LLI two blocks behind the current one.
        let behind = (cur_block + blocks - 2) % blocks;
        let l = behind * lli::SIZE;
        mem.invalidate(l, lli::SIZE);
        let c = mem.read_u64(l + lli::CTL);
        if consumed > (2 * total_words) as u64 {
            wb_seen += 1;
            if c & CTL_LLI_VALID == 0 {
                wb_cleared += 1;
            }
            if first_status.is_none() {
                first_status = Some((c, mem.read_u64(l + lli::STATUS)));
            }
        }
        mem.write_u64(l + lli::CTL, ctl);
        mem.flush(l, lli::SIZE);
        if last_report.elapsed() >= Duration::from_millis(500) {
            let dt = last_report.elapsed().as_secs_f64();
            println!(
                "  rate {:.1} Hz (consumed {consumed}), en {} intstat {:#x}",
                (consumed - report_consumed) as f64 / dt,
                ch.is_enabled(),
                ch.intstatus()
            );
            report_consumed = consumed;
            last_report = Instant::now();
        }
    }
    let elapsed = t0.elapsed().as_secs_f64();
    println!("average rate over {elapsed:.3}s: {:.2} Hz (expected {expected:.2})", consumed as f64 / elapsed);
    println!("LLI write-back cleared VALID in {wb_cleared}/{wb_seen} observations");
    if let Some((c, s)) = first_status {
        println!("  written-back CTL {c:#018x} STATUS {s:#018x}");
    }

    // Data integrity: destination must equal source where written.
    mem.invalidate(dst_off, data_bytes);
    let mut bad = 0;
    for i in 0..total_words {
        let d = mem.read_u32(dst_off + i * 4);
        if d != 0x1000_0000 + i as u32 {
            bad += 1;
        }
    }
    println!("data integrity: {bad} mismatching words of {total_words}");

    // Phase 2: where do LLP and SAR point when the engine stalls on an
    // invalid LLI? Invalidate exactly one LLI ahead and watch.
    println!("phase 2: stall position");
    for b in 0..blocks {
        mem.write_u64(b * lli::SIZE + lli::CTL, ctl);
    }
    mem.flush(0, lli_bytes);
    std::thread::sleep(Duration::from_millis(5));
    let cur = pos(ch.sar()).unwrap_or(0) / block_words;
    let bad = (cur + 2) % blocks;
    mem.write_u64(bad * lli::SIZE + lli::CTL, ctl & !CTL_LLI_VALID);
    mem.flush(bad * lli::SIZE, lli::SIZE);
    // Keep the others valid while waiting.
    let t = Instant::now();
    let mut last = (ch.llp(), ch.sar());
    let mut still = Instant::now();
    while t.elapsed() < Duration::from_millis(200) {
        for b in 0..blocks {
            if b != bad {
                mem.write_u64(b * lli::SIZE + lli::CTL, ctl);
            }
        }
        mem.flush(0, lli_bytes);
        let now = (ch.llp(), ch.sar());
        if now != last {
            last = now;
            still = Instant::now();
        } else if still.elapsed() > Duration::from_millis(5) {
            break;
        }
    }
    let (llp, sar) = last;
    let llp_idx = (0..blocks).find(|&b| mem.dma_addr_at(b * lli::SIZE) == llp & !0x3f);
    let sar_info = (0..blocks).find_map(|b| {
        let a = mem.dma_addr_at(src_off + b * block_words * 4);
        let d = sar.checked_sub(a)? as usize;
        (d <= block_words * 4).then_some((b, d / 4))
    });
    println!(
        "  invalid LLI {bad}: stalled with LLP -> LLI {llp_idx:?}, SAR in block/word {sar_info:?} (block {} = {} words)",
        (bad + blocks - 1) % blocks,
        block_words
    );

    ch.stop()?;
    tick.restore();
    println!("stopped: en {} CHEN {:#x}", ch.is_enabled(), dmac.chen());
    Ok(())
}
