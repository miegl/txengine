//! Holding the CPU clock steady while a BCM283x/BCM2711 transmitter runs.
//!
//! The VideoCore firmware changes the ARM clock (on every cpufreq step) by
//! reprogramming PLLB through the A2W interface. In FM that is the slow
//! register bus the DMA engine keeps busy writing the carrier PLL's
//! fraction (one write every ~5 µs, each taking ~4.3 µs). When the two
//! collide, the carrier PLL's configuration is destroyed. Observed on a Raspberry Pi Zero 2 W
//! (BCM2837) with the `schedutil` governor at 185 kS/s: PLLA's control
//! register read 0, its PER divider 4 and it lost lock, after anywhere
//! between a few and ~200 ARM clock changes; the carrier vanished. Never
//! with the clock fixed (90 s), and not at 120 kS/s (23 changes), nor
//! without the DMA engine running.
//!
//! The AM transmitter (PWM fed by DMA) needs the same for another reason:
//! during the clock changes the DMA engine stalls for longer than the words
//! in the PWM's FIFO last (8 to 11: 4 µs at 1 MHz), and every stall drops
//! carrier cycles, which shifts the carrier's phase. On a Zero 2 W with the
//! `schedutil` governor the PWM missed 0.4 % (500 kHz) to 1 % (3 MHz) of
//! its words; with the clock fixed, none.
//!
//! The transmitters therefore select the `performance` governor while they
//! run (no cpufreq steps) and restore the previous governor afterwards
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

/// Restores the CPU clock when dropped, unless the pin has been taken over
/// (by a transmitter's restorer): for constructors that can fail after
/// pinning it.
#[derive(Debug)]
pub struct CpuPinGuard(Option<CpuClockPin>);

impl CpuPinGuard {
    pub fn new(pin: Option<CpuClockPin>) -> CpuPinGuard {
        CpuPinGuard(pin)
    }

    /// Take the pin over (nothing pinned if there was none).
    pub fn take(&mut self) -> CpuClockPin {
        self.0.take().unwrap_or_default()
    }
}

impl Drop for CpuPinGuard {
    fn drop(&mut self) {
        if let Some(p) = self.0.take() {
            p.restore();
        }
    }
}
