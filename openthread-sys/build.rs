use std::{env, path::PathBuf};

use anyhow::Result;

#[path = "gen/builder.rs"]
mod builder;

fn main() -> Result<()> {
    let crate_root_path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());

    builder::OpenThreadBuilder::track(&crate_root_path.join("gen"));
    builder::OpenThreadBuilder::track(&crate_root_path.join("openthread"));

    let host = env::var("HOST").unwrap();
    let target = env::var("TARGET").unwrap();

    let use_gcc = env::var("CARGO_FEATURE_USE_GCC").is_ok();
    let force_esp_riscv_gcc = env::var("CARGO_FEATURE_FORCE_ESP_RISCV_GCC").is_ok();

    // If `force-generate-bindings` is enabled, we need to re-build the bindings on-the-fly even if there are
    // pre-generated bindings for the target triple
    let pregen_bindings = env::var("CARGO_FEATURE_FORCE_GENERATE_BINDINGS").is_err();

    let pregen_bindings_rs_file = crate_root_path
        .join("src")
        .join("include")
        .join(format!("{target}.rs"));
    let pregen_libs_dir = crate_root_path.join("libs").join(&target);

    // Desync guard: when the `prebuilt` profile itself is active (i.e. `xtask`
    // generating the committed artifacts), the active feature set MUST equal the
    // prebuilt reference, or the `matter` bundle in `Cargo.toml` and
    // `features::PREBUILT_FEATURES` have drifted apart.
    if env::var_os("CARGO_FEATURE_PREBUILT").is_some() {
        if let Err(delta) = builder::features::prebuilt_validity() {
            panic!(
                "BUG: `prebuilt` profile active but the selected knobs do not match \
                 `features::PREBUILT_FEATURES`. The `matter`/`prebuilt` bundle in Cargo.toml \
                 and PREBUILT_FEATURES have drifted. Delta: {delta}"
            );
        }
    }

    // The committed prebuilt libraries and bindings are produced with the
    // `prebuilt` (= `matter`) feature profile. They are valid only if the active
    // features select exactly the same OpenThread knobs; otherwise we must
    // rebuild on the fly (`--gc-sections` cannot recover the difference).
    let prebuilt_validity = builder::features::prebuilt_validity();

    let dirs = if pregen_bindings && pregen_bindings_rs_file.exists() && prebuilt_validity.is_ok() {
        // Use the pre-generated bindings
        Some((pregen_bindings_rs_file, pregen_libs_dir))
    } else if target.ends_with("-espidf") {
        // Nothing to do for ESP-IDF, `esp-idf-sys` will do everything for us
        None
    } else {
        if pregen_bindings_rs_file.exists() {
            if !pregen_bindings {
                println!("cargo::warning=Forcing on-the-fly OpenThread build for target {target} as bindings are not available.");
            } else if let Err(delta) = &prebuilt_validity {
                println!("cargo::warning=Forcing on-the-fly OpenThread build for {target}: the selected features differ from the prebuilt config by: {delta}.");
            }
        }

        let clang_sysroot = if use_gcc {
            None
        } else {
            // For clang, we can use our own cross-platform sysroot.
            let path = crate_root_path.join("gen").join("sysroot");
            builder::OpenThreadBuilder::track(&path);
            Some(path)
        };

        // Need to do on-the-fly build and bindings' generation
        let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

        let builder = builder::OpenThreadBuilder::new(
            !use_gcc,
            crate_root_path.clone(),
            Some(target),
            Some(host),
            None,
            clang_sysroot,
            None,
            force_esp_riscv_gcc,
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

        println!("cargo::rustc-link-search={}", libs_dir.display());

        // Link only the archives for the selected device type / co-processor
        // role (see `features::device_link_libs`). All flavors are built and
        // shipped, but linking all of them would collide (every core flavor
        // defines `otInstanceInitSingle`); we select exactly one core stack.
        for lib_name in builder::features::device_link_libs() {
            // Tolerate names that don't exist for a given target/layout (e.g.
            // `.lib` on MSVC, or a flavor a target didn't produce).
            let exists = std::fs::read_dir(&libs_dir)?.any(|entry| {
                entry
                    .ok()
                    .and_then(|e| e.file_name().into_string().ok())
                    .is_some_and(|f| {
                        f == format!("lib{lib_name}.a") || f == format!("{lib_name}.lib")
                    })
            });
            if exists {
                println!("cargo::rustc-link-lib=static={lib_name}");
            } else {
                println!(
                    "cargo::warning=openthread-sys: expected library `{lib_name}` not found in {}",
                    libs_dir.display()
                );
            }
        }
    }

    Ok(())
}
