//! Legacy BCM283x DMA engine (channels 0–14; on BCM2711 only 0–10 have the
//! legacy register layout).

use std::path::Path;
use std::time::Duration;

use log::{debug, warn};

use super::regs::dma::*;
use crate::error::{HalError, Result};
use crate::mmio::Mmio;
use crate::sys::{read_trimmed, wait_until};

/// A DMA control block (32 bytes, 32-byte aligned, in DMA memory).
#[derive(Debug, Clone, Copy, Default)]
pub struct ControlBlock {
    pub ti: u32,
    pub source: u32,
    pub dest: u32,
    pub length: u32,
    pub stride: u32,
    pub next: u32,
}

impl ControlBlock {
    pub fn words(&self) -> [u32; 8] {
        [self.ti, self.source, self.dest, self.length, self.stride, self.next, 0, 0]
    }
}

/// Channel mask available to Linux (the rest belongs to the VideoCore firmware).
fn linux_channel_mask() -> Option<u32> {
    let soc = Path::new("/proc/device-tree/soc");
    for e in std::fs::read_dir(soc).ok()?.flatten() {
        let name = e.file_name().to_string_lossy().into_owned();
        if name.starts_with("dma") && name.contains("7e007000") {
            let b = std::fs::read(e.path().join("brcm,dma-channel-mask")).ok()?;
            return Some(u32::from_be_bytes(b.get(..4)?.try_into().ok()?));
        }
    }
    None
}

/// Channels currently allocated by the kernel's bcm2835-dma driver.
fn kernel_channels_in_use(mask: u32, bcm2711: bool) -> u32 {
    // The driver registers channels in ascending order of the mask bits,
    // minus channel 0 (legacy API) and, on BCM2711, channel 11 (memcpy).
    let mut registered = mask & !1;
    if bcm2711 {
        registered &= !(1 << 11);
    }
    let order: Vec<u32> = (0..16).filter(|b| registered & (1 << b) != 0).collect();
    let mut used = 0;
    let Ok(dir) = std::fs::read_dir("/sys/class/dma") else { return 0 };
    for e in dir.flatten() {
        let dev = std::fs::canonicalize(e.path().join("device")).unwrap_or_default();
        if !dev.to_string_lossy().ends_with("7e007000.dma") && !dev.to_string_lossy().ends_with("e007000.dma") {
            continue;
        }
        let name = e.file_name().to_string_lossy().into_owned();
        let Some(idx) = name.rsplit("chan").next().and_then(|s| s.parse::<usize>().ok()) else { continue };
        if read_trimmed(e.path().join("in_use")).is_ok_and(|s| s == "1")
            && let Some(&hw) = order.get(idx)
        {
            used |= 1 << hw;
        }
    }
    used
}

/// One legacy DMA channel.
#[derive(Debug, Clone)]
pub struct DmaChannel {
    regs: Mmio,
    global: Mmio,
    pub ch: usize,
}

impl DmaChannel {
    /// Pick a channel: `preferred`, or the highest one that Linux owns but
    /// has not allocated and that is idle.
    pub fn claim(dma_block: &Mmio, preferred: Option<usize>, bcm2711: bool) -> Result<DmaChannel> {
        let max = if bcm2711 { 10 } else { 14 };
        let mask = linux_channel_mask().unwrap_or(if bcm2711 { 0x07f5 } else { 0x7f35 });
        let used = kernel_channels_in_use(mask, bcm2711);
        let active = |ch: usize| dma_block.read(ch * CHANNEL_SIZE + CS) & CS_ACTIVE != 0;
        let ch = match preferred {
            Some(ch) => {
                if ch > max || ch == 0 {
                    return Err(HalError::Config(format!("DMA channel {ch} cannot be used (1-{max})")));
                }
                if mask & (1 << ch) == 0 {
                    warn!(
                        "DMA channel {ch} is reserved for the VideoCore firmware (mask {mask:#06x}); using it anyway"
                    );
                }
                if used & (1 << ch) != 0 || active(ch) {
                    return Err(HalError::Hardware(format!("DMA channel {ch} is in use")));
                }
                ch
            }
            None => {
                (1..=max).rev().find(|&ch| mask & (1 << ch) != 0 && used & (1 << ch) == 0 && !active(ch)).ok_or_else(
                    || HalError::Hardware(format!("no free DMA channel (mask {mask:#06x}, in use {used:#06x})")),
                )?
            }
        };
        debug!("DMA: using channel {ch} (Linux mask {mask:#06x}, kernel in use {used:#06x})");
        Ok(DmaChannel { regs: dma_block.window(ch * CHANNEL_SIZE, CHANNEL_SIZE), global: dma_block.clone(), ch })
    }

    pub fn cs(&self) -> u32 {
        self.regs.read(CS)
    }

    /// Bus address of the control block being executed.
    pub fn current_cb(&self) -> u32 {
        self.regs.read(CONBLK_AD)
    }

    pub fn debug(&self) -> u32 {
        self.regs.read(DEBUG)
    }

    pub fn is_active(&self) -> bool {
        self.cs() & CS_ACTIVE != 0
    }

    /// Reset the channel and wait until it is idle.
    pub fn reset(&self) -> Result<()> {
        self.regs.write(CS, CS_ABORT);
        self.regs.write(CS, CS_RESET);
        wait_until(Duration::from_millis(50), "DMA channel reset", || self.cs() & (CS_ACTIVE | CS_RESET) == 0)?;
        self.regs.write(DEBUG, DEBUG_ERRORS);
        self.regs.write(CS, CS_END | CS_INT);
        Ok(())
    }

    /// Start executing the chain at `first_cb` (bus address).
    pub fn start(&self, first_cb: u32) -> Result<()> {
        if !first_cb.is_multiple_of(CB_SIZE as u32) {
            return Err(HalError::Config("control blocks must be 32-byte aligned".into()));
        }
        // Make sure the channel is enabled in the global enable register.
        let en = self.global.read(ENABLE);
        if en & (1 << self.ch) == 0 {
            self.global.write(ENABLE, en | (1 << self.ch));
        }
        self.reset()?;
        self.regs.write(CONBLK_AD, first_cb);
        self.regs.write(
            CS,
            (8 << CS_PRIORITY_SHIFT)
                | (15 << CS_PANIC_PRIORITY_SHIFT)
                | CS_WAIT_FOR_OUTSTANDING_WRITES
                | CS_DISDEBUG
                | CS_ACTIVE,
        );
        wait_until(Duration::from_millis(10), "DMA channel start", || self.is_active())?;
        Ok(())
    }

    /// Errors latched by the channel, if any.
    pub fn errors(&self) -> Option<String> {
        let cs = self.cs();
        let dbg = self.debug();
        if cs & CS_ERROR != 0 || dbg & DEBUG_ERRORS != 0 { Some(format!("CS {cs:#010x} DEBUG {dbg:#x}")) } else { None }
    }

    /// Stop the channel.
    pub fn stop(&self) -> Result<()> {
        self.reset()
    }
}
