//! RP1 south bridge (Raspberry Pi 5) support.

pub mod am;
pub mod clocks;
pub mod dma;
pub mod fm;
pub mod gpio;
pub mod link;
pub mod regs;
pub mod ticks;

use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};

use log::debug;

use crate::error::{HalError, Result};
use crate::mmio::Mmio;
use crate::sys::{Mapping, open, read_trimmed};

const RP1_VENDOR: u32 = 0x1de4;
const RP1_DEVICE: u32 = 0x0001;

/// Access to the RP1 peripheral space (PCIe BAR1).
#[derive(Clone, Debug)]
pub struct Rp1 {
    bar: Mmio,
    pub pci_path: PathBuf,
}

fn parse_hex(s: &str) -> Option<u32> {
    u32::from_str_radix(s.trim().trim_start_matches("0x"), 16).ok()
}

/// Locate the RP1 PCIe function in sysfs.
pub fn find_pci_device() -> Result<PathBuf> {
    let dir = Path::new("/sys/bus/pci/devices");
    let entries = std::fs::read_dir(dir).map_err(|e| HalError::io("list PCI devices", e))?;
    for e in entries.flatten() {
        let p = e.path();
        let vendor = read_trimmed(p.join("vendor")).ok().and_then(|s| parse_hex(&s));
        let device = read_trimmed(p.join("device")).ok().and_then(|s| parse_hex(&s));
        if vendor == Some(RP1_VENDOR) && device == Some(RP1_DEVICE) {
            return Ok(p);
        }
    }
    Err(HalError::Unsupported("RP1 PCIe device (1de4:0001) not found".into()))
}

impl Rp1 {
    /// Map RP1 peripherals through the PCI sysfs resource file of BAR1.
    pub fn open() -> Result<Rp1> {
        let pci_path = find_pci_device()?;
        let res = pci_path.join("resource1");
        let fd = open(&res, libc::O_RDWR | libc::O_SYNC)?;
        let size = std::fs::metadata(&res).map(|m| m.len() as usize).unwrap_or(0);
        if size < regs::PERIPH_LEN {
            return Err(HalError::Hardware(format!(
                "{} is only {size:#x} bytes, expected at least {:#x}",
                res.display(),
                regs::PERIPH_LEN
            )));
        }
        let map = Mapping::map(fd.as_raw_fd(), 0, regs::PERIPH_LEN, "RP1 BAR1")?;
        debug!("mapped RP1 peripherals from {}", res.display());
        Ok(Rp1 { bar: Mmio::new(map, 0x4000_0000), pci_path })
    }

    /// A register block at `offset` within the peripheral space.
    pub fn block(&self, offset: usize) -> Mmio {
        self.bar.window(offset, regs::BLOCK_LEN)
    }

    /// The atomic SET alias of a register block (write 1s to set bits).
    pub fn block_set(&self, offset: usize) -> Mmio {
        self.bar.window(offset + regs::ALIAS_SET, regs::BLOCK_LEN - regs::ALIAS_SET)
    }

    /// The atomic CLR alias of a register block (write 1s to clear bits).
    pub fn block_clr(&self, offset: usize) -> Mmio {
        self.bar.window(offset + regs::ALIAS_CLR, regs::BLOCK_LEN - regs::ALIAS_CLR)
    }

    /// The clock manager, which spans more than one 16 KiB block (PLLs).
    pub fn clocks(&self) -> Mmio {
        self.bar.window(regs::CLOCKS_MAIN, 0x14000)
    }

    /// Address of a peripheral register as seen by the RP1 DMA engine.
    pub fn dma_addr_of(offset: usize) -> u64 {
        regs::SYS_PERIPH_BASE + offset as u64
    }
}
