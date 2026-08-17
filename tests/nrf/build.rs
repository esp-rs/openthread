//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    // Which flash layout the firmware is linked for. A board flashed through a
    // debug probe owns all of flash; one flashed as UF2 has a bootloader (and
    // usually a SoftDevice) sitting at the bottom of it, so the application
    // has to start above them - see `memory-uf2.x`.
    let layout: &[u8] = if env::var_os("CARGO_FEATURE_UF2").is_some() {
        include_bytes!("memory-uf2.x")
    } else {
        include_bytes!("memory.x")
    };

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(layout)
        .unwrap();
    println!("cargo::rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo::rerun-if-changed=memory.x");
    println!("cargo::rerun-if-changed=memory-uf2.x");

    println!("cargo::rustc-link-arg-bins=--nmagic");
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rustc-link-arg-bins=-Tdefmt.x");
}
