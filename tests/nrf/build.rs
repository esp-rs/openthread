use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn linker_data() -> &'static [u8] {
    #[cfg(feature = "nrf52840")]
    return include_bytes!("memory-nrf52840.x");
    #[cfg(feature = "nrf54l15")]
    return include_bytes!("memory-nrf54l15.x");
}

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(linker_data())
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory-nrf52840.x");
    println!("cargo:rerun-if-changed=memory-nrf54l15.x");
    println!("cargo:rerun-if-changed=build.rs");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
