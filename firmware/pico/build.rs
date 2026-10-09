//! Puts the memory layout of the selected chip where the linker finds it,
//! and works out the firmware's version as txengine's build script does:
//! `git describe --tags --always --dirty` of the source tree, overridden by
//! `TXENGINE_VERSION` in the environment, else the one in Cargo.toml.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    let chip = match (env::var_os("CARGO_FEATURE_RP2040").is_some(), env::var_os("CARGO_FEATURE_RP2350").is_some()) {
        (true, false) => "rp2040",
        (false, true) => "rp2350",
        _ => panic!("enable exactly one of the features rp2040 and rp2350 (cargo rp2040 / cargo rp2350)"),
    };
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy(format!("memory-{chip}.x"), out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory-{chip}.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    if chip == "rp2040" {
        println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    }

    println!("cargo:rerun-if-env-changed=TXENGINE_VERSION");
    // Commits, checkouts, new tags and edited sources change the version.
    for p in ["HEAD", "index", "refs", "packed-refs"] {
        watch(git(&["rev-parse", "--path-format=absolute", "--git-path", p]));
    }
    for p in ["src", "Cargo.toml", "Cargo.lock", "../../crates/txengine-pico-protocol"] {
        watch(Some(p.into()));
    }
    let version = env::var("TXENGINE_VERSION")
        .ok()
        .or_else(|| git(&["--no-optional-locks", "describe", "--tags", "--always", "--dirty"]))
        .unwrap_or_else(|| env!("CARGO_PKG_VERSION").into());
    println!("cargo:rustc-env=TXENGINE_VERSION={version}");
}

/// Run the build script again when `path` changes (a path that does not
/// exist would run it every time).
fn watch(path: Option<String>) {
    if let Some(p) = path.filter(|p| Path::new(p).exists()) {
        println!("cargo:rerun-if-changed={p}");
    }
}

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    out.status.success().then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}
