use std::{env, path::PathBuf};

use anyhow::Result;

#[path = "gen/builder.rs"]
mod builder;

fn main() -> Result<()> {
    let crate_root_path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());

    builder::OpenThreadBuilder::track(&crate_root_path.join("gen"));
    builder::OpenThreadBuilder::track(&crate_root_path.join("openthread"));

    // If `custom` is enabled, we need to re-build the bindings on-the-fly even if there are
    // pre-generated bindings for the target triple

    let host = env::var("HOST").unwrap();
    let target = env::var("TARGET").unwrap();

    let force_esp_riscv_toolchain = env::var("CARGO_FEATURE_FORCE_ESP_RISCV_TOOLCHAIN").is_ok();
    let full_thread_device = cfg!(feature = "full-thread-device");

    let pregen_bindings = env::var("CARGO_FEATURE_FORCE_GENERATE_BINDINGS").is_err();
    let pregen_bindings_rs_file = crate_root_path
        .join("src")
        .join("include")
        .join(format!("{target}.rs"));
    let pregen_libs_dir = crate_root_path.join("libs").join(&target);

    let dirs = if pregen_bindings && pregen_bindings_rs_file.exists() {
        // Use the pre-generated bindings
        Some((pregen_bindings_rs_file, pregen_libs_dir))
    } else if target.ends_with("-espidf") {
        // Nothing to do for ESP-IDF, `esp-idf-sys` will do everything for us
        None
    } else {
        // Need to do on-the-fly build and bindings' generation
        let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

        let builder = builder::OpenThreadBuilder::new(
            crate_root_path.clone(),
            Some(target),
            Some(host),
            None,
            None,
            None,
            force_esp_riscv_toolchain,
            full_thread_device,
        );

        let libs_dir = builder.compile(&out, None)?;
        let bindings = builder.generate_bindings(&out, None)?;

        Some((bindings, libs_dir))
    };

    if let Some((bindings, libs_dir)) = dirs {
        println!(
            "cargo::rustc-env=OPENTHREAD_SYS_BINDINGS_FILE={}",
            bindings.display()
        );

        println!("cargo:rustc-link-search={}", libs_dir.display());

        for entry in std::fs::read_dir(libs_dir)? {
            let entry = entry?;

            let file_name = entry.file_name();
            let file_name = file_name.to_str().unwrap();
            if file_name.ends_with(".a") || file_name.to_ascii_lowercase().ends_with(".lib") {
                let lib_name = if file_name.ends_with(".a") {
                    file_name.trim_start_matches("lib").trim_end_matches(".a")
                } else {
                    file_name.trim_end_matches(".lib")
                };

                let is_ftd_specific = lib_name.contains("-ftd");
                let is_mtd_specific = lib_name.contains("-mtd");
                let is_general = !is_ftd_specific && !is_mtd_specific;
                let should_link = is_general
                    || (is_ftd_specific && full_thread_device)
                    || (is_mtd_specific && !full_thread_device);

                if should_link {
                    println!("cargo:rustc-link-lib=static={lib_name}");
                }
            }
        }
    }

    Ok(())
}
