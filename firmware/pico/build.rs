//! Puts the memory layout of the selected chip where the linker finds it.

use std::env;
use std::fs;
use std::path::PathBuf;

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
}
