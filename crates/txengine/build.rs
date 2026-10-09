//! The version txengine reports (`txengine -V`): `git describe --tags
//! --always --dirty` of the source tree. `TXENGINE_VERSION` in the
//! environment overrides it, for builds without git; without either it is
//! the one in Cargo.toml. The Pico firmware's build script does the same.

use std::path::Path;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-env-changed=TXENGINE_VERSION");
    // Commits, checkouts, new tags and edited sources change the version.
    for p in ["HEAD", "index", "refs", "packed-refs"] {
        watch(git(&["rev-parse", "--path-format=absolute", "--git-path", p]));
    }
    for p in ["../../Cargo.toml", "../../Cargo.lock", "../../crates"] {
        watch(Some(p.into()));
    }
    let version = std::env::var("TXENGINE_VERSION")
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
