//! PCIe link power management of the RP1 link.
//!
//! With ASPM L1 enabled (the default `powersave` policy), the link drops
//! into L1 when the DMA engine is idle for a moment. Waking it up at the
//! next DMA block boundary can take long enough that a DMA tick is lost:
//! the output timeline slips by one sample (4 µs at 250 kS/s), about once
//! every few seconds. Measured on a Pi 5: 3-7 slips per 25 s with L1
//! enabled, none with it disabled. The transmitter therefore disables ASPM
//! on the link while it runs and restores the previous setting afterwards.

use std::path::{Path, PathBuf};

use log::{debug, warn};

/// sysfs link attributes controlling the ASPM states (kernel ≥ 5.5).
const ATTRS: [&str; 6] = ["l0s_aspm", "l1_aspm", "l1_1_aspm", "l1_2_aspm", "l1_1_pcipm", "l1_2_pcipm"];

/// Link power states that were enabled before and have been disabled.
#[derive(Debug, Clone, Default)]
pub struct AspmGuard {
    disabled: Vec<PathBuf>,
}

impl AspmGuard {
    /// Disable every enabled ASPM state of the link of `pci_device`.
    pub fn disable(pci_device: &Path) -> AspmGuard {
        let link = pci_device.join("link");
        let mut disabled = Vec::new();
        for a in ATTRS {
            let p = link.join(a);
            match std::fs::read_to_string(&p) {
                Ok(v) if v.trim() == "1" => match std::fs::write(&p, "0") {
                    Ok(()) => disabled.push(p),
                    Err(e) => warn!("cannot disable PCIe {a} on the RP1 link ({e}); expect occasional 1-sample timing slips"),
                },
                _ => {}
            }
        }
        if disabled.is_empty() {
            debug!("RP1 PCIe link: no ASPM states enabled");
        } else {
            let names: Vec<_> = disabled.iter().filter_map(|p| p.file_name()?.to_str()).collect();
            debug!("RP1 PCIe link: disabled {} while transmitting", names.join(", "));
        }
        AspmGuard { disabled }
    }

    /// Re-enable what [`AspmGuard::disable`] turned off.
    pub fn restore(&self) {
        for p in &self.disabled {
            if let Err(e) = std::fs::write(p, "1") {
                warn!("cannot restore {}: {e}", p.display());
            }
        }
    }
}
