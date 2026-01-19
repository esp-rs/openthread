#![deny(unexpected_cfgs)]

use std::{env, path::PathBuf};

use anyhow::Result;

use crate::builder::OpenThreadConfig;
use crate::paths::PreGenerationPaths;

#[path = "gen/builder.rs"]
mod builder;
#[path = "gen/pregen_paths.rs"]
mod paths;

fn main() -> Result<()> {
    let crate_root_path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());

    builder::OpenThreadBuilder::track(&crate_root_path.join("gen"));
    builder::OpenThreadBuilder::track(&crate_root_path.join("openthread"));
    builder::OpenThreadBuilder::track(&crate_root_path.join("CMakeLists.txt"));

    // If `custom` is enabled, we need to re-build the bindings on-the-fly even if there are
    // pre-generated bindings for the target triple

    let host = env::var("HOST").unwrap();
    let target = env::var("TARGET").unwrap();

    let force_esp_riscv_toolchain = cfg!(feature = "force-esp-riscv-toolchain");

    let mut openthread_config = OpenThreadConfig::default();
    set_config_from_features(&mut openthread_config);

    let force_generate_bindings = cfg!(feature = "force-generate-bindings");
    let paths = PreGenerationPaths::derive(&crate_root_path, &target, &openthread_config);

    let use_pregen_bindings = !force_generate_bindings && paths.bindings_rs_file.exists();

    let dirs = if use_pregen_bindings {
        Some((paths.bindings_rs_file, paths.libs_dir))
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
            openthread_config,
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

                println!("cargo:rustc-link-lib=static={lib_name}");
            }
        }
    }

    Ok(())
}

fn set_config_from_features(config: &mut OpenThreadConfig) {
    if cfg!(feature = "full-thread-device") {
        config.ftd(true);
    }
}
