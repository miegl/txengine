//! RP1 DMA controller (Synopsys DesignWare AXI DMAC, 8 channels).
//!
//! The Linux `dw-axi-dmac` driver owns the controller and hands channels to
//! kernel clients on demand. We use a channel the kernel has not allocated
//! (checked through `/sys/class/dma`) and keep its interrupt *status*
//! disabled: the kernel's interrupt handler scans every channel and would
//! otherwise act on (and disable) our channel when it sees status bits.

use std::path::Path;
use std::time::Duration;

use log::{debug, warn};

use super::regs::{clk, dmac::*};
use crate::error::{HalError, Result};
use crate::mmio::Mmio;
use crate::sys::{read_trimmed, wait_until};

/// The DMA controller as a whole.
#[derive(Debug, Clone)]
pub struct Dmac {
    regs: Mmio,
}

impl Dmac {
    pub fn new(regs: Mmio) -> Dmac {
        Dmac { regs }
    }

    pub fn id(&self) -> u32 {
        self.regs.read(ID)
    }

    pub fn compver(&self) -> u32 {
        self.regs.read(COMPVER)
    }

    pub fn is_enabled(&self) -> bool {
        self.regs.read(CFG) & CFG_DMAC_EN != 0
    }

    pub fn chen(&self) -> u32 {
        self.regs.read(CHEN_L)
    }

    /// Make sure the controller is clocked and enabled.
    ///
    /// When no kernel client holds a channel, the kernel runtime-suspends the
    /// controller: it gates `clk_dma` and clears the global enable. Returns
    /// whether anything had to be switched on.
    pub fn ensure_running(&self, clocks: &Mmio, clocks_set: &Mmio) -> bool {
        let mut changed = false;
        if clocks.read(clk::CLK_DMA_CTRL) & clk::CTRL_ENABLE == 0 {
            clocks_set.write(clk::CLK_DMA_CTRL, clk::CTRL_ENABLE);
            changed = true;
        }
        if !self.is_enabled() {
            self.regs.modify(CFG, CFG_DMAC_EN, CFG_DMAC_EN);
            changed = true;
        }
        changed
    }

    /// Channels the kernel's dmaengine currently has allocated.
    pub fn kernel_channels_in_use() -> u32 {
        let mut mask = 0;
        let Ok(dir) = std::fs::read_dir("/sys/class/dma") else { return 0 };
        for e in dir.flatten() {
            let path = e.path();
            let dev = std::fs::canonicalize(path.join("device")).unwrap_or_default();
            if !dev.to_string_lossy().ends_with("188000.dma") {
                continue;
            }
            let name = e.file_name().to_string_lossy().into_owned();
            let Some(idx) = name.rsplit("chan").next().and_then(|s| s.parse::<u32>().ok()) else { continue };
            if read_trimmed(path.join("in_use")).is_ok_and(|s| s == "1") {
                mask |= 1 << idx;
            }
        }
        mask
    }

    /// Claim an idle channel, preferring `preferred`, else the highest-numbered
    /// free one (the kernel allocates from the bottom).
    pub fn claim(&self, preferred: Option<usize>) -> Result<DmaChannel> {
        let kernel = Self::kernel_channels_in_use();
        let busy = kernel | self.chen() & 0xff;
        let pick = match preferred {
            Some(ch) => {
                if ch >= NUM_CHANNELS {
                    return Err(HalError::Config(format!("RP1 DMA channel {ch} does not exist (0-7)")));
                }
                if busy & (1 << ch) != 0 {
                    return Err(HalError::Hardware(format!(
                        "RP1 DMA channel {ch} is busy (kernel in-use mask {kernel:#04x}, enabled {:#04x})",
                        self.chen() & 0xff
                    )));
                }
                ch
            }
            None => (0..NUM_CHANNELS)
                .rev()
                .find(|ch| busy & (1 << ch) == 0)
                .ok_or_else(|| HalError::Hardware("no free RP1 DMA channel".into()))?,
        };
        if !Path::new("/sys/class/dma").exists() {
            warn!("/sys/class/dma not available: cannot verify the kernel is not using DMA channel {pick}");
        }
        debug!("RP1 DMA: using channel {pick} (kernel in-use mask {kernel:#04x})");
        Ok(DmaChannel { regs: self.regs.clone(), ch: pick })
    }
}

/// One DMA channel.
#[derive(Debug, Clone)]
pub struct DmaChannel {
    regs: Mmio,
    pub ch: usize,
}

impl DmaChannel {
    fn r(&self, off: usize) -> usize {
        chan(self.ch) + off
    }

    pub fn is_enabled(&self) -> bool {
        self.regs.read(CHEN_L) & chen_en(self.ch) != 0
    }

    /// Program a linked-list transfer starting at `first_lli` (DMA address)
    /// with the given 64-bit channel configuration. Does not start it.
    pub fn program_linked_list(&self, first_lli: u64, cfg: u64) -> Result<()> {
        if self.is_enabled() {
            return Err(HalError::Hardware(format!("RP1 DMA channel {} is already running", self.ch)));
        }
        if !first_lli.is_multiple_of(64) {
            return Err(HalError::Config("LLI must be 64-byte aligned".into()));
        }
        self.regs.write(self.r(CH_INTSIGNAL_ENA), 0);
        self.regs.write(self.r(CH_INTSTATUS_ENA), 0);
        self.regs.write(self.r(CH_INTCLEAR), 0xffff_ffff);
        self.regs.write64(self.r(CH_CFG), cfg);
        self.regs.write64(self.r(CH_LLP), first_lli);
        let rb = self.regs.read64(self.r(CH_CFG));
        if rb & !CFG_PRIORITY_MASK != cfg & !CFG_PRIORITY_MASK {
            return Err(HalError::Hardware(format!(
                "RP1 DMA channel {}: CFG read-back {rb:#018x} != {cfg:#018x}",
                self.ch
            )));
        }
        Ok(())
    }

    /// Raw access to the channel configuration register (diagnostics).
    pub fn cfg(&self) -> u64 {
        self.regs.read64(self.r(CH_CFG))
    }

    pub fn write_cfg(&self, v: u64) {
        self.regs.write64(self.r(CH_CFG), v);
    }

    pub fn enable(&self) {
        self.regs.write(CHEN_L, chen_en(self.ch) | chen_en_we(self.ch));
    }

    /// Current source address (advances as the channel reads).
    pub fn sar(&self) -> u64 {
        self.regs.read64_consistent(self.r(CH_SAR))
    }

    /// Current destination address.
    pub fn dar(&self) -> u64 {
        self.regs.read64_consistent(self.r(CH_DAR))
    }

    /// Linked list pointer (address of the next LLI to fetch).
    pub fn llp(&self) -> u64 {
        self.regs.read64_consistent(self.r(CH_LLP))
    }

    pub fn ctl(&self) -> u64 {
        self.regs.read64(self.r(CH_CTL))
    }

    pub fn status(&self) -> u64 {
        self.regs.read64(self.r(CH_STATUS))
    }

    pub fn intstatus(&self) -> u32 {
        self.regs.read(self.r(CH_INTSTATUS))
    }

    /// Ask a channel that stalled on an invalid LLI to fetch it again.
    pub fn request_resume(&self) {
        self.regs.write(self.r(CH_BLK_TFR_RESUMEREQ), 1);
    }

    /// Diagnostics: set the interrupt status enable mask.
    pub fn set_intstatus_enable(&self, mask: u32) {
        self.regs.write(self.r(CH_INTSTATUS_ENA), mask);
    }

    pub fn clear_intstatus(&self, mask: u32) {
        self.regs.write(self.r(CH_INTCLEAR), mask);
    }

    /// Stop the channel: request a disable, falling back to an abort.
    pub fn stop(&self) -> Result<()> {
        if !self.is_enabled() {
            return Ok(());
        }
        self.regs.write(CHEN_L, chen_en_we(self.ch));
        if wait_until(Duration::from_millis(20), "DMA channel disable", || !self.is_enabled()).is_ok() {
            return Ok(());
        }
        // A channel waiting for a handshake that never comes does not finish
        // its burst; abort it.
        self.regs.write(CHEN_H, chen_abort(self.ch) | chen_abort_we(self.ch));
        wait_until(Duration::from_millis(50), "DMA channel abort", || !self.is_enabled())?;
        Ok(())
    }
}
