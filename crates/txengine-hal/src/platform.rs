//! Board and SoC detection.

use std::fmt;
use std::path::Path;

use crate::error::{HalError, Result};

/// The application processor family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Soc {
    /// Raspberry Pi 1, Zero, Zero W (ARMv6; detected but not a build target).
    Bcm2835,
    /// Raspberry Pi 2 Model B v1.1.
    Bcm2836,
    /// Raspberry Pi 2 v1.2, 3, 3+, Zero 2 W, CM3.
    Bcm2837,
    /// Raspberry Pi 4, 400, CM4.
    Bcm2711,
    /// Raspberry Pi 5, 500, CM5 (peripherals live in the RP1 south bridge).
    Bcm2712,
}

impl Soc {
    /// Whether the GPIO/clock peripherals are the classic BCM283x/BCM2711 ones.
    pub fn has_bcm_peripherals(self) -> bool {
        !matches!(self, Soc::Bcm2712)
    }
}

impl fmt::Display for Soc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Soc::Bcm2835 => "BCM2835",
            Soc::Bcm2836 => "BCM2836",
            Soc::Bcm2837 => "BCM2837",
            Soc::Bcm2711 => "BCM2711",
            Soc::Bcm2712 => "BCM2712",
        };
        f.write_str(s)
    }
}

/// Description of the running board.
#[derive(Debug, Clone)]
pub struct Platform {
    pub soc: Soc,
    /// Model string from the device tree, e.g. "Raspberry Pi 5 Model B Rev 1.0".
    pub model: String,
    /// ARM physical address of the 0x7e000000 peripheral bus window
    /// (BCM283x/BCM2711 only).
    pub periph_phys_base: u64,
    /// Crystal oscillator frequency feeding the clock manager / PLLs, in Hz.
    pub osc_hz: f64,
}

const DT: &str = "/proc/device-tree";

fn dt_strings(path: &str) -> Vec<String> {
    std::fs::read(Path::new(DT).join(path))
        .map(|b| {
            b.split(|&c| c == 0)
                .filter(|s| !s.is_empty())
                .map(|s| String::from_utf8_lossy(s).into_owned())
                .collect()
        })
        .unwrap_or_default()
}

fn dt_u32(path: &str) -> Option<u32> {
    let b = std::fs::read(Path::new(DT).join(path)).ok()?;
    let b: [u8; 4] = b.get(..4)?.try_into().ok()?;
    Some(u32::from_be_bytes(b))
}

fn soc_from_compatible(compat: &[String]) -> Option<Soc> {
    for c in compat {
        let soc = match c.as_str() {
            "brcm,bcm2712" => Soc::Bcm2712,
            "brcm,bcm2711" => Soc::Bcm2711,
            "brcm,bcm2837" | "brcm,bcm2710" => Soc::Bcm2837,
            "brcm,bcm2836" | "brcm,bcm2709" => Soc::Bcm2836,
            "brcm,bcm2835" | "brcm,bcm2708" => Soc::Bcm2835,
            _ => continue,
        };
        return Some(soc);
    }
    None
}

/// Fallback for kernels without /proc/device-tree: the revision code in /proc/cpuinfo.
fn soc_from_cpuinfo() -> Option<Soc> {
    let info = std::fs::read_to_string("/proc/cpuinfo").ok()?;
    let rev = info
        .lines()
        .find(|l| l.starts_with("Revision"))
        .and_then(|l| l.split(':').nth(1))
        .and_then(|v| u32::from_str_radix(v.trim(), 16).ok())?;
    if rev & (1 << 23) == 0 {
        return Some(Soc::Bcm2835); // old-style revision codes are all Pi 1
    }
    Some(match (rev >> 12) & 0xf {
        0 => Soc::Bcm2835,
        1 => Soc::Bcm2836,
        2 => Soc::Bcm2837,
        3 => Soc::Bcm2711,
        4 => Soc::Bcm2712,
        _ => return None,
    })
}

/// Peripheral base from the `soc` node's `ranges` (like bcm_host_get_peripheral_address).
fn periph_base_from_dt() -> Option<u64> {
    let r = std::fs::read(Path::new(DT).join("soc/ranges")).ok()?;
    let word = |i: usize| -> Option<u32> {
        Some(u32::from_be_bytes(r.get(i * 4..i * 4 + 4)?.try_into().ok()?))
    };
    // <child-bus-addr parent-addr... size>; the parent address is one cell on
    // BCM283x and two cells (high, low) on BCM2711.
    if word(0)? != 0x7e00_0000 {
        return None;
    }
    let p1 = word(1)?;
    if p1 != 0 {
        Some(p1 as u64)
    } else {
        let p2 = word(2)?;
        Some(p2 as u64)
    }
}

/// Detect the running platform.
pub fn detect() -> Result<Platform> {
    let compat = dt_strings("compatible");
    let soc = soc_from_compatible(&compat).or_else(soc_from_cpuinfo).ok_or_else(|| {
        HalError::Unsupported(format!(
            "not a supported Raspberry Pi (device-tree compatible: {compat:?})"
        ))
    })?;
    let model = dt_strings("model").into_iter().next().unwrap_or_else(|| format!("unknown {soc} board"));

    let periph_phys_base = match soc {
        Soc::Bcm2835 => periph_base_from_dt().unwrap_or(0x2000_0000),
        Soc::Bcm2836 | Soc::Bcm2837 => periph_base_from_dt().unwrap_or(0x3f00_0000),
        Soc::Bcm2711 => periph_base_from_dt().unwrap_or(0xfe00_0000),
        Soc::Bcm2712 => 0,
    };

    let osc_hz = match soc {
        Soc::Bcm2712 => dt_u32("clocks/clk_xosc/clock-frequency").unwrap_or(50_000_000),
        Soc::Bcm2711 => dt_u32("clocks/clk-osc/clock-frequency").unwrap_or(54_000_000),
        _ => dt_u32("clocks/clk-osc/clock-frequency").unwrap_or(19_200_000),
    } as f64;

    Ok(Platform { soc, model, periph_phys_base, osc_hz })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compatible_parsing() {
        let c = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(soc_from_compatible(&c(&["raspberrypi,5-model-b", "brcm,bcm2712"])), Some(Soc::Bcm2712));
        assert_eq!(soc_from_compatible(&c(&["raspberrypi,model-zero-2-w", "brcm,bcm2837"])), Some(Soc::Bcm2837));
        assert_eq!(soc_from_compatible(&c(&["raspberrypi,2-model-b", "brcm,bcm2836"])), Some(Soc::Bcm2836));
        assert_eq!(soc_from_compatible(&c(&["raspberrypi,4-model-b", "brcm,bcm2711"])), Some(Soc::Bcm2711));
        assert_eq!(soc_from_compatible(&c(&["foo,bar"])), None);
    }
}

/// Power supply and thermal state reported by the firmware
/// (`vcgencmd get_throttled`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PowerStatus(pub u32);

impl PowerStatus {
    /// Query the firmware; `None` if the mailbox is unavailable.
    pub fn read() -> Option<PowerStatus> {
        crate::mem::mailbox::Mailbox::open().ok()?.throttled().ok().map(PowerStatus)
    }

    /// Problems now or since boot, as text (empty if none).
    pub fn problems(&self) -> Vec<String> {
        const NAMES: [&str; 4] = ["under-voltage", "ARM frequency capped", "CPU throttled", "soft temperature limit"];
        let mut out = Vec::new();
        for (bit, name) in NAMES.iter().enumerate() {
            if self.0 & (1 << bit) != 0 {
                out.push(format!("{name} now"));
            } else if self.0 & (1 << (bit + 16)) != 0 {
                out.push(format!("{name} since boot"));
            }
        }
        out
    }
}
