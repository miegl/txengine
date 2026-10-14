//! The PIO method: a state machine puts out a bit stream on the pin, one bit
//! per system clock cycle, and the bits are a square wave at the carrier
//! whose edges fall on the nearest bit.
//!
//! While it transmits, the system clock is `PLL_USB` at the bit rate
//! (240-264 MHz, chosen by the host; see [`clock`]). Core 1 looks up one
//! 32-bit word per 32 bits in a table of square-wave patterns indexed by
//! the phase at the word's start, advances the phase by the sample's
//! increment, and DMA feeds the words to the state machine:
//!
//! ```text
//!  table ─► core 1 ─► word ring ──DMA──► PIO TX FIFO ─► `out pins, 1` ─► GPIO
//! ```

use core::ptr::{addr_of, addr_of_mut};

use embassy_rp::pac;
use txengine_pico_protocol::{Fault, PioConfig};

use crate::clock;
use crate::rf::{self, FUNCSEL_NULL, FUNCSEL_PIO0, SM};
use crate::shared::{Command, Progress, SHARED};

/// Words in the ring: all of `rf::WORD_RING` (8192: 1 ms at 252 Mbit/s).
const RING_WORDS: usize = rf::WORD_RING_WORDS;

const TABLE_LEN: usize = 1 << PioConfig::TABLE_BITS;
static mut TABLE: [u32; TABLE_LEN] = [0; TABLE_LEN];

/// DMA channel (a ping-pong pair with the next one on the RP2040).
const CH: usize = 0;

/// Breadcrumbs (see `crate::crumb`).
const STEP: u32 = 0x5049_0000;

#[inline(always)]
fn crumb(step: u32) {
    crate::crumb(STEP | step);
}

/// The bit rate of a configuration.
pub fn bit_rate(cfg: &PioConfig) -> u32 {
    clock::hz(&cfg.clock())
}

pub fn validate(cfg: &PioConfig) -> bool {
    clock::validate(&cfg.clock())
        && cfg.gpio < 30
        && matches!(cfg.drive_ma, 2 | 4 | 8 | 12)
        && cfg.ratio < 1 << 31
        && cfg.scale <= PioConfig::SCALE_MAX
        && cfg.sample_words >= 1
}

/// The word loop's state: the phase at the start of the next word, its
/// advance per word and the change of that per word.
#[repr(C)]
struct Words {
    phase: u32,
    inc: u32,
    dinc: i32,
    table: *const u32,
}

// The word loop for the Cortex-M0+, which the compiler unrolls and spills
// (about 25 of the 32 cycles a word lasts): 11 cycles per word.
// `txengine_pio_words(k, out, words)` writes `words` (at least one) words to
// `out`. Runs from RAM.
#[cfg(feature = "rp2040")]
core::arch::global_asm!(
    ".section .data.txengine_pio_words,\"ax\",%progbits",
    ".global txengine_pio_words",
    ".type txengine_pio_words,%function",
    ".thumb_func",
    ".p2align 2",
    "txengine_pio_words:",
    "    push {{r4-r7, lr}}",
    "    ldr r3, [r0, #0]",  // phase
    "    ldr r4, [r0, #4]",  // inc
    "    ldr r5, [r0, #8]",  // dinc
    "    ldr r6, [r0, #12]", // table
    "1:",
    "    lsrs r7, r3, #{shift}",
    "    lsls r7, r7, #2",
    "    ldr r7, [r6, r7]", // the pattern at this phase
    "    stmia r1!, {{r7}}",
    "    adds r3, r3, r4", // phase += inc
    "    adds r4, r4, r5", // inc += dinc
    "    subs r2, #1",
    "    bne 1b",
    "    str r3, [r0, #0]",
    "    str r4, [r0, #4]",
    "    pop {{r4-r7, pc}}",
    ".size txengine_pio_words, . - txengine_pio_words",
    shift = const 32 - PioConfig::TABLE_BITS,
);

#[cfg(feature = "rp2040")]
unsafe extern "C" {
    fn txengine_pio_words(k: *mut Words, out: *mut u32, words: u32);
}

impl Words {
    /// Write `out.len()` words.
    #[inline(always)]
    fn words(&mut self, out: &mut [u32]) {
        #[cfg(feature = "rp2040")]
        // SAFETY: the loop writes exactly `out.len()` (at least one) words
        // and reads the table, `TABLE_LEN` words.
        unsafe {
            txengine_pio_words(self, out.as_mut_ptr(), out.len() as u32)
        };
        #[cfg(feature = "rp2350")]
        self.words_portable(out);
    }

    /// The same in Rust.
    #[inline(always)]
    fn words_portable(&mut self, out: &mut [u32]) {
        let (mut phase, mut inc) = (self.phase, self.inc);
        for o in out {
            // SAFETY: the index is below `TABLE_LEN`.
            *o = unsafe { self.table.add((phase >> (32 - PioConfig::TABLE_BITS)) as usize).read() };
            phase = phase.wrapping_add(inc);
            inc = inc.wrapping_add_signed(self.dinc);
        }
        (self.phase, self.inc) = (phase, inc);
    }
}

/// Check the assembly word loop against the Rust one. Before core 1 starts
/// (it fills the table with test data).
pub fn self_test() -> bool {
    // SAFETY: core 1 does not run yet.
    let table = unsafe { &mut *addr_of_mut!(TABLE) };
    for (i, t) in table.iter_mut().enumerate() {
        *t = (i as u32).wrapping_mul(0x9e37_79b9);
    }
    let mut a = Words { phase: 0x1234_5678, inc: 0x2345_6789, dinc: 0, table: table.as_ptr() };
    let mut b = Words { ..a };
    let (mut wa, mut wb) = ([0u32; 16], [0u32; 16]);
    for dinc in [0, 12_345, -98_765] {
        a.dinc = dinc;
        b.dinc = dinc;
        a.words(&mut wa);
        b.words_portable(&mut wb);
        if wa != wb || (a.phase, a.inc) != (b.phase, b.inc) {
            return false;
        }
    }
    true
}

/// `n / d` (`d` > 0), rounded toward zero; on the RP2040 with the core's
/// hardware divider (the library routine runs from flash).
#[inline(always)]
fn div(n: i32, d: u32) -> i32 {
    #[cfg(feature = "rp2040")]
    {
        let div = pac::SIO.div();
        div.sdividend().write_value(n as u32);
        div.sdivisor().write_value(d);
        while !div.csr().read().ready() {}
        div.quotient().read() as i32
    }
    #[cfg(feature = "rp2350")]
    {
        n / d as i32
    }
}

pub struct Transmitter {
    cfg: PioConfig,
    /// Phase at the start of the next word, and its advance per word.
    phase: u32,
    inc: u32,
    center: u32,
    scale: u32,
    retune_version: u32,
    written: u32,
    read: u32,
    last_offset: u32,
    taken: u64,
    underruns: u32,
    missing: u32,
    saved: Option<clock::Saved>,
}

impl Transmitter {
    pub fn start(cfg: &PioConfig) -> Result<Transmitter, Fault> {
        // SAFETY: only core 1 touches the table, and it is idle here.
        let table = unsafe { &mut *addr_of_mut!(TABLE) };
        for (bin, t) in table.iter_mut().enumerate() {
            *t = PioConfig::pattern(cfg.ratio, bin as u32);
        }
        crumb(1);
        // A correction sent before the start counts from the start.
        let retune_version = SHARED.retune.version();
        let r = SHARED.retune.get();
        let mut t = Transmitter {
            cfg: *cfg,
            phase: 0,
            inc: r.center,
            center: r.center,
            scale: r.scale.min(PioConfig::SCALE_MAX),
            retune_version,
            written: 0,
            read: 0,
            last_offset: 0,
            taken: 0,
            underruns: 0,
            missing: 0,
            saved: None,
        };
        // The carrier at its centre until the first samples are read.
        let w = cfg.sample_words as u32;
        for _ in 0..(RING_WORDS as u32 - 1) / w {
            t.fill(0, w);
        }
        SHARED.progress.set(Progress { consumed: 0, underruns: 0 });

        crumb(2);
        t.saved = Some(clock::raise(&cfg.clock())?);
        crumb(8);
        start_pio(cfg);
        crumb(9);
        // SAFETY: only the address is taken.
        let ring = unsafe { addr_of_mut!(rf::WORD_RING.0) } as u32;
        start_dma(ring);
        crumb(10);
        pac::PIO0.ctrl().modify(|w| w.set_sm_enable(1 << SM));
        crumb(11);
        Ok(t)
    }

    /// Produce `words` words, the phase advance moving by `dinc` per word.
    #[inline(always)]
    fn fill(&mut self, dinc: i32, words: u32) {
        let table = addr_of!(TABLE) as *const u32;
        let mut k = Words { phase: self.phase, inc: self.inc, dinc, table };
        let mut left = words as usize;
        while left > 0 {
            // Up to the end of the ring, then on from its start.
            let at = self.written as usize % RING_WORDS;
            let n = left.min(RING_WORDS - at);
            // SAFETY: in bounds; core 1 is the only writer, and these words
            // were read by the DMA a ring ago (the caller keeps the lead
            // below a ring).
            let out =
                unsafe { core::slice::from_raw_parts_mut((addr_of_mut!(rf::WORD_RING.0) as *mut u32).add(at), n) };
            k.words(out);
            self.written = self.written.wrapping_add(n as u32);
            left -= n;
        }
        (self.phase, self.inc) = (k.phase, k.inc);
    }

    /// One sample per iteration when there is room (as the PLL method's).
    #[unsafe(link_section = ".data.direct_run")]
    #[inline(never)]
    pub fn run(&mut self) -> Result<(), Fault> {
        let w = self.cfg.sample_words as u32;
        let interpolate = self.cfg.interpolate;
        let host_lost = bit_rate(&self.cfg) / 32 / w;
        let mut min_lead = u32::MAX;
        // The state machine stalls when its FIFO runs dry: a gap in the
        // bit stream. Clear the flag the start left.
        pac::PIO0.fdebug().write(|w| w.set_txstall(1 << SM));
        crumb(12);
        loop {
            if pac::PIO0.fdebug().read().txstall() & (1 << SM) != 0 {
                return Err(Fault::Stall);
            }
            if let Some(offset) = read_offset() {
                self.read = self.read.wrapping_add(offset.wrapping_sub(self.last_offset) % RING_WORDS as u32);
                self.last_offset = offset;
            }
            let lead = self.written.wrapping_sub(self.read) as i32;
            if lead <= 0 {
                return Err(Fault::Late);
            }
            let lead = lead as u32;
            min_lead = min_lead.min(lead);

            if (RING_WORDS as u32 - 1).saturating_sub(lead) < w {
                if min_lead != u32::MAX {
                    let m = SHARED.min_lead.load(core::sync::atomic::Ordering::Relaxed);
                    SHARED.min_lead.store(m.min(min_lead * 32), core::sync::atomic::Ordering::Relaxed);
                    min_lead = u32::MAX;
                }
                let consumed = self.taken.saturating_sub((lead / w) as u64);
                SHARED.progress.set(Progress { consumed, underruns: self.underruns });
                if SHARED.command() == Command::Stop {
                    return Ok(());
                }
                continue;
            }

            if SHARED.retune.version() != self.retune_version {
                self.retune_version = SHARED.retune.version();
                let r = SHARED.retune.get();
                (self.center, self.scale) = (r.center, r.scale.min(PioConfig::SCALE_MAX));
            }
            let target = match SHARED.samples.pop() {
                Some(s) => {
                    self.taken += 1;
                    self.missing = 0;
                    self.center.wrapping_add((s as i32 * self.scale as i32) as u32)
                }
                None => {
                    if self.missing == 0 {
                        self.underruns += 1;
                    }
                    self.missing += 1;
                    if self.missing > host_lost {
                        return Err(Fault::HostLost);
                    }
                    self.center
                }
            };
            let dinc = if interpolate { div(target.wrapping_sub(self.inc) as i32, w) } else { 0 };
            if !interpolate {
                self.inc = target;
            }
            self.fill(dinc, w);
            self.inc = target;
        }
    }

    pub fn stop(&mut self) {
        crumb(20);
        pac::PIO0.ctrl().modify(|w| w.set_sm_enable(0));
        rf::abort_dma(&dma_channels());
        pac::IO_BANK0.gpio(self.cfg.gpio as usize).ctrl().write(|w| w.set_funcsel(FUNCSEL_NULL));
        if let Some(saved) = self.saved.take() {
            clock::restore(&saved);
        }
        pac::WATCHDOG.scratch0().write_value(0);
    }
}

fn start_pio(cfg: &PioConfig) {
    rf::reset_pio0();
    let pio = pac::PIO0;
    let prg = pio::pio_asm!(".wrap_target", "    out pins, 1", ".wrap");
    pio.instr_mem(0).write_value(pac::pio::regs::InstrMem(prg.program.code[0] as u32));
    let sm = pio.sm(SM);
    sm.clkdiv().write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    sm.execctrl().write(|w| {
        w.set_wrap_bottom(0);
        w.set_wrap_top(0);
    });
    sm.shiftctrl().write(|w| {
        // Bit 0 of a word goes out first.
        w.set_out_shiftdir(true);
        w.set_autopull(true);
        w.set_pull_thresh(0); // 32
        w.set_fjoin_tx(true);
    });
    sm.pinctrl().write(|w| {
        w.set_out_base(cfg.gpio);
        w.set_out_count(1);
        w.set_set_base(cfg.gpio);
        w.set_set_count(1);
    });
    let exec = |i: u16| sm.instr().write(|w| w.set_instr(i));
    exec(pio::InstructionOperands::SET { destination: pio::SetDestination::PINDIRS, data: 1 }.encode());
    exec(pio::InstructionOperands::JMP { condition: pio::JmpCondition::Always, address: 0 }.encode());
    rf::configure_pad(cfg.gpio, cfg.drive_ma);
    pac::IO_BANK0.gpio(cfg.gpio as usize).ctrl().write(|w| w.set_funcsel(FUNCSEL_PIO0));
}

fn dma_channels() -> [usize; if cfg!(feature = "rp2040") { 2 } else { 1 }] {
    #[cfg(feature = "rp2040")]
    {
        [CH, CH + 1]
    }
    #[cfg(feature = "rp2350")]
    {
        [CH]
    }
}

fn start_dma(ring: u32) {
    let txf = pac::PIO0.txf(SM).as_ptr() as u32;
    let treq = pac::dma::vals::TreqSel::from_bits(SM as u8);
    rf::start_ring_dma(CH, ring, txf, treq, (RING_WORDS * 4).trailing_zeros() as u8, RING_WORDS as u32);
}

/// The DMA's position in the ring (words), if known right now.
#[inline(always)]
fn read_offset() -> Option<u32> {
    // SAFETY: only the address is taken.
    let ring = unsafe { addr_of_mut!(rf::WORD_RING.0) } as u32;
    rf::ring_read_offset(CH, ring, RING_WORDS as u32)
}
