//! Holding the CPU clock steady while a BCM283x/BCM2711 PLL is modulated.
//!
//! The VideoCore firmware changes the ARM clock (on every cpufreq step) by
//! reprogramming PLLB through the A2W interface, the same slow register bus
//! the DMA engine keeps busy writing the carrier PLL's fraction (one write
//! every ~5 µs, each taking ~4.3 µs). When the two collide, the carrier
//! PLL's configuration is destroyed. Observed on a Raspberry Pi Zero 2 W
//! (BCM2837) with the `schedutil` governor at 185 kS/s: PLLA's control
//! register read 0, its PER divider 4 and it lost lock, after anywhere
//! between a few and ~200 ARM clock changes; the carrier vanished. Never
//! with the clock fixed (90 s), and not at 120 kS/s (23 changes), nor
//! without the DMA engine running.
//!
//! The transmitter therefore selects the `performance` governor while it
//! runs (no cpufreq steps) and restores the previous governor afterwards
//! (also from the shutdown guardian).

use std::path::{Path, PathBuf};

use log::{debug, info, warn};

const CPUFREQ: &str = "/sys/devices/system/cpu/cpufreq";

/// cpufreq policies whose governor was changed, with the previous governor.
#[derive(Debug, Clone, Default)]
pub struct CpuClockPin {
    changed: Vec<(PathBuf, String)>,
}

impl CpuClockPin {
    /// Nothing pinned.
    pub fn none() -> CpuClockPin {
        CpuClockPin::default()
    }

    /// Select the `performance` governor on every cpufreq policy.
    pub fn pin() -> CpuClockPin {
        let mut changed = Vec::new();
        let mut policies: Vec<PathBuf> = match std::fs::read_dir(CPUFREQ) {
            Ok(d) => d
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.starts_with("policy")))
                .collect(),
            Err(_) => Vec::new(),
        };
        policies.sort();
        if policies.is_empty() {
            debug!("no cpufreq policies: the CPU clock is not scaled by Linux");
        }
        for p in policies {
            let gov = p.join("scaling_governor");
            let Ok(old) = std::fs::read_to_string(&gov).map(|s| s.trim().to_string()) else { continue };
            if old == "performance" {
                continue;
            }
            let available = std::fs::read_to_string(p.join("scaling_available_governors")).unwrap_or_default();
            if !available.split_whitespace().any(|g| g == "performance") {
                warn!("{}: no `performance` cpufreq governor; CPU clock changes may disturb the carrier", p.display());
                continue;
            }
            match std::fs::write(&gov, "performance") {
                Ok(()) => changed.push((gov, old)),
                Err(e) => warn!(
                    "cannot fix the CPU clock ({}: {e}); CPU clock changes may disturb the carrier",
                    gov.display()
                ),
            }
        }
        if let Some((_, old)) = changed.first() {
            info!("CPU clock held at its maximum while transmitting (cpufreq governor {old} -> performance)");
        }
        CpuClockPin { changed }
    }

    /// Restore the governors [`CpuClockPin::pin`] replaced.
    pub fn restore(&self) {
        for (gov, old) in &self.changed {
            if let Err(e) = std::fs::write(gov, old) {
                warn!("cannot restore the cpufreq governor {old} ({}: {e})", Path::new(gov).display());
            }
        }
    }
}
