//! RP1 DMA pacing ticks.
//!
//! The TICKS block divides `clk_ref` (the 50 MHz crystal) by a 9-bit integer;
//! the DMA_TICK block turns each tick into a DMA handshake request. A DMA
//! channel whose destination handshake is the tick then performs exactly one
//! burst per tick – a time base for paced transfers to arbitrary registers.

use std::time::Duration;

use super::regs::{dreq, ticks::*};
use crate::error::{HalError, Result};
use crate::mmio::Mmio;
use crate::sys::wait_until;

#[derive(Debug, Clone, Copy)]
struct TickRegs {
    ctrl: u32,
    cycles: u32,
    en: u32,
    tctrl: u32,
}

/// One of the two DMA tick generators.
#[derive(Debug)]
pub struct DmaTick {
    ticks: Mmio,
    dma_tick: Mmio,
    n: usize,
    saved: TickRegs,
}

impl DmaTick {
    pub fn new(ticks: Mmio, dma_tick: Mmio, n: usize) -> DmaTick {
        assert!(n < 2);
        let saved = TickRegs {
            ctrl: ticks.read(dma_ctrl(n)),
            cycles: ticks.read(dma_cycles(n)),
            en: dma_tick.read(tick_en(n)),
            tctrl: dma_tick.read(tick_ctrl(n)),
        };
        DmaTick { ticks, dma_tick, n, saved }
    }

    /// Handshake number to use as the DMA destination peripheral.
    pub fn dreq(&self) -> u32 {
        if self.n == 0 { dreq::DMA_TICK0 } else { dreq::DMA_TICK1 }
    }

    /// Whether another user already runs this tick generator.
    pub fn in_use(&self) -> bool {
        self.saved.ctrl & CTRL_ENABLE != 0 || self.saved.en & EN_REQ != 0
    }

    /// Start generating one DMA request every `cycles` reference clocks.
    pub fn start(&self, cycles: u32) -> Result<()> {
        if !(2..=CYCLES_MAX).contains(&cycles) {
            return Err(HalError::Config(format!("tick period {cycles} out of range")));
        }
        self.stop();
        // Never auto-stop on the DMA's end-of-transfer strobe; no idle dwell.
        self.dma_tick.write(tick_ctrl(self.n), 0);
        self.ticks.write(dma_cycles(self.n), cycles);
        if self.ticks.read(dma_cycles(self.n)) != cycles {
            return Err(HalError::Hardware("DMA tick period read-back mismatch".into()));
        }
        self.ticks.write(dma_ctrl(self.n), CTRL_ENABLE);
        wait_until(Duration::from_millis(10), "DMA tick generator start", || {
            self.ticks.read(dma_ctrl(self.n)) & CTRL_RUNNING != 0
        })?;
        self.dma_tick.write(tick_en(self.n), EN_REQ);
        Ok(())
    }

    /// Whether the generator and its handshake are both active.
    pub fn is_running(&self) -> bool {
        self.ticks.read(dma_ctrl(self.n)) & CTRL_RUNNING != 0 && self.dma_tick.read(tick_en(self.n)) & EN_REQ != 0
    }

    pub fn stop(&self) {
        self.dma_tick.write(tick_en(self.n), 0);
        self.ticks.write(dma_ctrl(self.n), 0);
    }

    pub fn restore(&self) {
        self.stop();
        self.ticks.write(dma_cycles(self.n), self.saved.cycles);
        self.dma_tick.write(tick_ctrl(self.n), self.saved.tctrl);
        self.ticks.write(dma_ctrl(self.n), self.saved.ctrl);
        self.dma_tick.write(tick_en(self.n), self.saved.en);
    }
}
