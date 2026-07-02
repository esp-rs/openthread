use std::{
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{anyhow, Result};
use bindgen::Builder;
use cmake::Config;

#[path = "features.rs"]
pub mod features;

pub struct OpenThreadBuilder {
    crate_root_path: PathBuf,
    cmake_configurer: CMakeConfigurer,
    clang_path: Option<PathBuf>,
    clang_sysroot_path: Option<PathBuf>,
    clang_target: Option<String>,
}

impl OpenThreadBuilder {
    /// Create a new OpenThreadBuilder
    ///
    /// Arguments:
    /// - `force_clang`: If true, force the use of Clang as the compiler.
    /// - `crate_root_path`: Path to the root of the crate
    /// - `cmake_rust_target`: Optional target for CMake when building Openthread, with Rust target-triple syntax. If not specified, the "TARGET" env variable will be used
    /// - `cmake_host_rust_target`: Optional host target for the build
    /// - `clang_path`: Optional path to the Clang compiler. If not specified, the system Clang will be used for generating bindings,
    ///   and the system compiler (likely GCC) would be used for building the OpenThread C/C++ code itself
    /// - `clang_sysroot_path`: Optional path to the compiler sysroot directory. If not specified, the host sysroot will be used
    /// - `clang_target`: Optional target for Clang when generating bindings. If not specified, the "TARGET" env variable target will be used
    /// - `force_esp_riscv_gcc`: If true, and if the target is a riscv32 target, force the use of the Espressif RISCV GCC toolchain
    ///   (`riscv32-esp-elf-gcc`) rather than the derived `riscv32-unknown-elf-gcc` toolchain which is the "official" RISC-V one
    ///   (https://github.com/riscv-collab/riscv-gnu-toolchain)
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        force_clang: bool,
        crate_root_path: PathBuf,
        cmake_rust_target: Option<String>,
        cmake_host_rust_target: Option<String>,
        clang_path: Option<PathBuf>,
        clang_sysroot_path: Option<PathBuf>,
        clang_target: Option<String>,
        force_esp_riscv_gcc: bool,
    ) -> Self {
        Self {
            cmake_configurer: CMakeConfigurer::new(
                force_clang,
                clang_sysroot_path.clone(),
                crate_root_path.clone(),
                cmake_rust_target,
                cmake_host_rust_target,
                force_esp_riscv_gcc,
                crate_root_path.join("gen").join("toolchain.cmake"),
            ),
            crate_root_path,
            clang_path,
            clang_sysroot_path,
            clang_target,
        }
    }

    /// Generate bindings for openthread-sys
    ///
    /// Arguments:
    /// - `out_path`: Path to write the bindings to
    pub fn generate_bindings(
        &self,
        out_path: &Path,
        copy_file_path: Option<&Path>,
    ) -> Result<PathBuf> {
        log::info!("Generating OpenThread bindings");

        if let Some(clang_path) = &self.clang_path {
            // For bindgen
            std::env::set_var("CLANG_PATH", clang_path);
        }

        if let Some(cmake_rust_target) = &self.cmake_configurer.cmake_rust_target {
            // Necessary for bindgen. See this:
            // https://github.com/rust-lang/rust-bindgen/blob/af7fd38d5e80514406fb6a8bba2d407d252c30b9/bindgen/lib.rs#L711
            std::env::set_var("TARGET", cmake_rust_target);
        }

        let canon = |path: &Path| {
            // TODO: Is this really necessary?
            path.display()
                .to_string()
                .replace('\\', "/")
                .replace("//?/C:", "")
        };

        // Generate the bindings using `bindgen`:
        log::info!("Generating bindings");
        let mut builder = Builder::default()
            .use_core()
            .enable_function_attribute_detection()
            .derive_debug(false)
            .derive_default(true)
            .layout_tests(false)
            .allowlist_item("ot.*")
            .allowlist_item("OT_.*")
            .header(
                self.crate_root_path
                    .join("gen")
                    .join("include")
                    .join("include.h")
                    .to_string_lossy(),
            )
            .clang_args([&format!(
                "-I{}",
                canon(&self.crate_root_path.join("openthread").join("include"))
            )]);

        if self.short_enums() {
            builder = builder.clang_arg("-fshort-enums");
        }

        if let Some(sysroot_path) = self
            .clang_sysroot_path
            .clone()
            .or_else(|| self.cmake_configurer.derive_sysroot())
        {
            builder = builder.clang_args([
                &format!("-I{}", canon(&sysroot_path.join("include"))),
                &format!("--sysroot={}", canon(&sysroot_path)),
            ]);
        }

        if let Some(target) = &self.clang_target {
            builder = builder.clang_arg(format!("--target={target}"));
        }

        let bindings = builder
            .generate()
            .map_err(|_| anyhow!("Failed to generate bindings"))?;

        let bindings_file = out_path.join("bindings.rs");

        // Write out the bindings to the appropriate path:
        log::info!("Writing out bindings to: {}", bindings_file.display());
        bindings.write_to_file(&bindings_file)?;

        // Format the bindings:
        Command::new("rustfmt")
            .arg(bindings_file.to_string_lossy().to_string())
            .arg("--config")
            .arg("normalize_doc_attributes=true")
            .output()?;

        if let Some(copy_file_path) = copy_file_path {
            log::info!("Copying bindings to {}", copy_file_path.display());
            std::fs::create_dir_all(copy_file_path.parent().unwrap())?;
            std::fs::copy(&bindings_file, copy_file_path)?;
        }

        Ok(bindings_file)
    }

    /// Compile OpenThread
    ///
    /// Arguments:
    /// - `out_path`: Path to use as a build space
    /// - `copy_path`: Optional path to copy the generated libraries to
    pub fn compile(&self, out_path: &Path, copy_path: Option<&Path>) -> Result<PathBuf> {
        // Whether to build OpenThread against the external MbedTLS from
        // `mbedtls-rs-sys`
        let use_external_mbedtls = std::env::var_os("CARGO_FEATURE_MBEDTLS_RS_SYS").is_some();

        let target_dir = out_path.join("openthread").join("build");
        std::fs::create_dir_all(&target_dir)?;

        let target_lib_dir = out_path.join("openthread").join("lib");

        let lib_dir = copy_path.unwrap_or(&target_lib_dir);
        std::fs::create_dir_all(lib_dir)?;

        // Compile OpenThread and generate libraries to link against
        log::info!("Compiling OpenThread");

        let mut config = self.cmake_configurer.configure(Some(lib_dir));

        // Increase message buffers for Matter + SRP (default 44 is too small)
        config
            .cflag("-DOPENTHREAD_CONFIG_NUM_MESSAGE_BUFFERS=128")
            .cxxflag("-DOPENTHREAD_CONFIG_NUM_MESSAGE_BUFFERS=128");

        // When using the external MbedTLS provided by `mbedtls-rs-sys`, add its
        // include directories so OpenThread compiles against that config. When
        // the feature is off, OpenThread builds its own bundled MbedTLS (the
        // `third_party/mbedtls` subtree) and needs no external includes.
        if use_external_mbedtls {
            let mbedtls_include = std::env::var_os("DEP_MBEDTLS_INCLUDE")
                .expect("mbedtls-rs-sys should set the 'include' metadata");
            std::env::split_paths(&mbedtls_include).for_each(|include_dir| {
                config.cflag(format!("-I{}", include_dir.display()));
                config.cxxflag(format!("-I{}", include_dir.display()));
            });
        }

        config
            .define("OT_THREAD_VERSION", "1.1")
            .define("OT_LOG_LEVEL", "NOTE")
            // Build BOTH device types so the prebuilt cache covers MTD and FTD.
            // The actual archives shipped/linked are chosen by the umbrella
            // target in our wrapper `CMakeLists.txt` and the `ftd`/`rcp`/`tcp`
            // features (see `gen/features.rs::device_link_libs`); the consumer's
            // linker pulls only the archive(s) its firmware references, so unused
            // archives cost zero bytes in the final image.
            //
            // OT_RCP here builds OpenThread's RCP *firmware* (radio-only image),
            // which we don't ship. Remote-radio support on the *host* side is
            // the separate `radio-spinel`/`hdlc` libraries (selected by the
            // `rcp` feature), which build regardless of this flag. The NCP role
            // (`openthread-ncp-*`) is omitted from the umbrella entirely.
            .define("OT_FTD", "ON")
            .define("OT_MTD", "ON")
            .define("OT_RCP", "OFF")
            // Compile the minimal spinel codec shim (`gen/support/src/spinel_codec.c`)
            // into the `support` library when the `rcp` feature is active. It
            // re-exports the spinel packed-uint codec used by the Rust
            // `SpinelRadio` driver (`crate::rcp`).
            .define(
                "OT_RCP_HOST_SHIM",
                if std::env::var_os("CARGO_FEATURE_RCP").is_some() {
                    "ON"
                } else {
                    "OFF"
                },
            )
            // Do not change from here below
            .define("OT_LOG_OUTPUT", "PLATFORM_DEFINED")
            .define("OT_PLATFORM", "external")
            .define("OT_SETTINGS_RAM", "OFF")
            //.define("OT_COMPILE_WARNING_AS_ERROR", "ON "$@" "${OT_SRCDIR}"")
            // ... or else the build would fail with `arm-none-eabi-gcc` during the linking phase
            // with "undefined symbol `__exit`" error
            .define("BUILD_TESTING", "OFF")
            .define("OT_BUILD_EXECUTABLES", "OFF")
            .define("CMAKE_POLICY_VERSION_MINIMUM", "3.5") // For MbedTLS
            .profile("MinSizeRel")
            .out_dir(&target_dir)
            // Build only the `openthread-sys-libs` umbrella target (defined in
            // our wrapper `CMakeLists.txt`), not CMake's default `ALL`. This
            // both skips the install target — which pulls in pkg-config / CMake
            // helper files and 3rdparty install bits we don't need, and fails on
            // some host setups — AND avoids building OpenThread's CLI / RCP /
            // radio example libraries (registered unconditionally upstream, and
            // not buildable against our trimmed MbedTLS profile). We harvest the
            // `.a` files straight from `target_dir` via the CMAKE_*_OUTPUT_DIRECTORY
            // defines.
            .build_target("openthread-sys-libs");

        // MbedTLS backend selection (the `mbedtls-rs-sys` feature, on by
        // default). When enabled, OpenThread is built against the external
        // MbedTLS provided by `mbedtls-rs-sys` (whose includes were added
        // above). When disabled, OpenThread compiles its own bundled MbedTLS
        // (the `third_party/mbedtls` subtree) — `OT_EXTERNAL_MBEDTLS` is simply
        // not set, which is OpenThread's default.
        if use_external_mbedtls {
            config.define("OT_EXTERNAL_MBEDTLS", "mbedtls");
        }

        // Heap configuration (the `heap-*` features; see `features::HeapConfig`).
        // OpenThread has ONE fixed-size internal buffer shared by its own code
        // and — when builtin MbedTLS management is on — by MbedTLS. The three
        // axes below are independent: each ext toggle redirects one consumer to
        // the global allocator, and the size knob sizes the internal buffer for
        // whoever still uses it.
        let heap = features::heap_config();

        // `heap-int-<N>`: resize the internal buffer. Override both the DTLS and
        // non-DTLS sizes (OpenThread picks one or the other depending on whether
        // a DTLS feature is compiled in) so the selected size applies regardless.
        // Applied whenever set, independently of the ext toggles.
        if let Some(size) = heap.int_size {
            for define in [
                "OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE",
                "OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE_NO_DTLS",
            ] {
                config
                    .cflag(format!("-D{define}={size}"))
                    .cxxflag(format!("-D{define}={size}"));
            }
        }

        // `heap-ext-ot`: redirect OpenThread's *own* heap usage to the global
        // allocator (`OPENTHREAD_CONFIG_HEAP_EXTERNAL_ENABLE=1`). This is a core
        // OpenThread knob, independent of the MbedTLS backend. The consumer must
        // provide `otPlatCAlloc`/`otPlatFree` at link time (forwarding to the
        // firmware's global allocator), exactly like MbedTLS's `calloc`/`free`
        // contract — the crate does not supply them.
        if heap.ext_ot {
            config
                .cflag("-DOPENTHREAD_CONFIG_HEAP_EXTERNAL_ENABLE=1")
                .cxxflag("-DOPENTHREAD_CONFIG_HEAP_EXTERNAL_ENABLE=1");
        }

        // `heap-ext-mbedtls`: turn OpenThread's builtin MbedTLS management off
        // (`OT_BUILTIN_MBEDTLS_MANAGEMENT=OFF`) so MbedTLS allocates through the
        // global `calloc`/`free` rather than the internal buffer. Only meaningful
        // with the external MbedTLS.
        //
        // Note: with the external MbedTLS, OpenThread's own default for
        // `OT_BUILTIN_MBEDTLS_MANAGEMENT` is already OFF (it tracks
        // `OPENTHREAD_CONFIG_ENABLE_BUILTIN_MBEDTLS`, which is 0 for external), so
        // when the feature is NOT set we must request `ON` explicitly to keep the
        // historical shared-buffer behaviour.
        if use_external_mbedtls {
            config.define(
                "OT_BUILTIN_MBEDTLS_MANAGEMENT",
                if heap.ext_mbedtls { "OFF" } else { "ON" },
            );
        } else if heap.ext_mbedtls {
            // The bundled-MbedTLS path is not exercised here and would route
            // MbedTLS to global `calloc`/`free` symbols the consumer may not
            // provide. Warn rather than silently ignore the selection.
            println!(
                "cargo::warning=openthread-sys: the `heap-ext-mbedtls` feature is ignored \
                 without the `mbedtls-rs-sys` feature (external MbedTLS); using OpenThread's \
                 bundled MbedTLS with its default memory management."
            );
        }

        // Apply the feature-driven `OT_*` knobs: every exposed knob is reset to
        // `OFF` (overriding OpenThread's own, sometimes-`ON`, defaults), then
        // turned `ON` per active cargo feature. See `gen/features.rs`.
        for setting in features::active_knob_settings() {
            config.define(setting.knob, if setting.on { "ON" } else { "OFF" });
        }

        // OpenThread's DTLS code (`secure_transport`) gates its key-export
        // callback declaration on `#ifdef MBEDTLS_SSL_EXPORT_KEYS`, but invokes
        // `mbedtls_ssl_set_export_keys_cb` unconditionally on MbedTLS >= 3.0
        // (where the symbol is always available and the config option was
        // removed). Define it for OpenThread's own compilation so the
        // declaration matches, when a DTLS feature is active. Only relevant for
        // the external MbedTLS (the bundled one is the matching upstream version).
        if use_external_mbedtls && features::dtls_active() {
            config
                .cflag("-DMBEDTLS_SSL_EXPORT_KEYS")
                .cxxflag("-DMBEDTLS_SSL_EXPORT_KEYS");
        }

        config.build();

        Ok(lib_dir.to_path_buf())
    }

    /// Re-run the build script if the file or directory has changed.
    #[allow(unused)]
    pub fn track(file_or_dir: &Path) {
        println!("cargo::rerun-if-changed={}", file_or_dir.display())
    }

    /// A heuristics (we don't have anything better) to signal to `bindgen` whether the GCC toolchain
    /// for the target emits short enums or not.
    ///
    /// This is necessary for `bindgen` to generate correct bindings for OpenThread.
    /// See https://github.com/rust-lang/rust-bindgen/issues/711
    fn short_enums(&self) -> bool {
        let target = std::env::var("TARGET").unwrap();

        target.ends_with("-eabi") || target.ends_with("-eabihf")
    }
}

// TODO: Move to `embuild`
#[derive(Clone)]
pub struct CMakeConfigurer {
    pub force_clang: bool,
    pub clang_sysroot_path: Option<PathBuf>,
    pub project_path: PathBuf,
    pub cmake_rust_target: Option<String>,
    pub cmake_host_rust_target: Option<String>,
    pub force_esp_riscv_gcc: bool,
    pub empty_toolchain_file: PathBuf,
}

impl CMakeConfigurer {
    /// Create a new OpenThreadBuilder
    ///
    /// Arguments:
    /// - `force_clang`: If true, force the use of Clang as the compiler.
    /// - `clang_sysroot_path`: Optional path to a sysroot directory. Only used if `force_clang` is true.
    /// - `project_path`: Path to the root of the CMake project
    /// - `cmake_rust_target`: Optional target for CMake when building Openthread, with Rust target-triple syntax. If not specified, the "TARGET" env variable will be used
    /// - `cmake_host_rust_target`: Optional host target for the build
    /// - `force_esp_riscv_gcc`: If true, and if the target is a riscv32 target, force the use of the Espressif RISCV GCC toolchain
    ///   (`riscv32-esp-elf-gcc`) rather than the derived `riscv32-unknown-elf-gcc` toolchain which is the "official" RISC-V one
    ///   (https://github.com/riscv-collab/riscv-gnu-toolchain)
    pub const fn new(
        force_clang: bool,
        clang_sysroot_path: Option<PathBuf>,
        project_path: PathBuf,
        cmake_rust_target: Option<String>,
        cmake_host_rust_target: Option<String>,
        force_esp_riscv_gcc: bool,
        empty_toolchain_file: PathBuf,
    ) -> Self {
        Self {
            force_clang,
            clang_sysroot_path,
            project_path,
            cmake_rust_target,
            cmake_host_rust_target,
            force_esp_riscv_gcc,
            empty_toolchain_file,
        }
    }

    pub fn configure(&self, target_dir: Option<&Path>) -> Config {
        if let Some(cmake_rust_target) = &self.cmake_rust_target {
            // For `cc-rs`
            std::env::set_var("TARGET", cmake_rust_target);
        }

        let mut config = Config::new(&self.project_path);

        config
            // OpenThread's own CMake doesn't run Python, but the bundled MbedTLS
            // (built when the `mbedtls-rs-sys` feature is OFF) does, and Python
            // would drop `__pycache__/*.pyc` caches into the crate source tree.
            // `cargo publish` aborts when build.rs modifies the extracted source,
            // so disable bytecode caching to keep a bundled-MbedTLS publish clean.
            .env("PYTHONDONTWRITEBYTECODE", "1")
            // ... or else the build would fail with `arm-none-eabi-gcc` when testing the compiler
            .define("CMAKE_TRY_COMPILE_TARGET_TYPE", "STATIC_LIBRARY")
            .define("CMAKE_EXPORT_COMPILE_COMMANDS", "ON")
            .define("CMAKE_BUILD_TYPE", "MinSizeRel");

        if let Some(target_dir) = target_dir {
            config
                .define("CMAKE_ARCHIVE_OUTPUT_DIRECTORY", target_dir)
                .define("CMAKE_LIBRARY_OUTPUT_DIRECTORY", target_dir)
                .define("CMAKE_RUNTIME_OUTPUT_DIRECTORY", target_dir)
                // Multi-config generators (Ninja Multi-Config, Visual Studio, Xcode)
                // ignore `CMAKE_BUILD_TYPE` and the unsuffixed *_OUTPUT_DIRECTORY
                // vars at build time. Restrict the generated configs to
                // `MinSizeRel` (matching the single-config `CMAKE_BUILD_TYPE`
                // above and `compile()`'s `.profile("MinSizeRel")`) and pin the
                // per-config output dirs to `target_dir` so the build script can
                // locate the produced static libs.
                .define("CMAKE_CONFIGURATION_TYPES", "MinSizeRel")
                .define("CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL", target_dir)
                .define("CMAKE_LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL", target_dir)
                .define("CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL", target_dir);
        }

        if let Some((compiler, _)) = self.derive_forced_c_compiler() {
            let mut cfg = cc::Build::new();
            cfg.compiler(&compiler);

            config
                .init_c_cfg(cfg.clone())
                .init_cxx_cfg(cfg)
                .define("CMAKE_C_COMPILER", &compiler)
                .define("CMAKE_CXX_COMPILER", compiler)
                .define("CMAKE_TOOLCHAIN_FILE", &self.empty_toolchain_file);
        } else if let Some(target) = &self.cmake_rust_target {
            let mut split = target.split('-');
            let target_arch = split.next().unwrap();
            let target_os = split.next().unwrap();

            let mut target_vendor = "unknown";
            let mut target_env = split.next().unwrap();

            if let Some(next) = split.next() {
                target_vendor = target_env;
                target_env = next;
            }

            std::env::set_var("CARGO_CFG_TARGET_ARCH", target_arch);
            std::env::set_var("CARGO_CFG_TARGET_OS", target_os);
            std::env::set_var("CARGO_CFG_TARGET_VENDOR", target_vendor);
            std::env::set_var("CARGO_CFG_TARGET_ENV", target_env);
        }

        for arg in self.derive_c_args() {
            config.cflag(&arg).cxxflag(arg);
        }

        if let Some(target) = &self.cmake_rust_target {
            config.target(target);
        }

        if let Some(host) = &self.cmake_host_rust_target {
            config.host(host);
        }

        config
    }

    pub fn derive_sysroot(&self) -> Option<PathBuf> {
        if self.force_clang {
            if let Some(clang_sysroot_path) = self.clang_sysroot_path.clone() {
                // If clang is used and there is a pre-defined sysroot path for it, use it
                return Some(clang_sysroot_path);
            }
        }

        // Only GCC has a sysroot, so try to locate the sysroot using GCC first
        let unforce_clang = Self {
            force_clang: false,
            ..self.clone()
        };

        let (compiler, gnu) = unforce_clang.derive_c_compiler();

        if gnu {
            let output = Command::new(&compiler)
                .arg("-print-sysroot")
                .output()
                .ok()?;

            if output.status.success() {
                let sysroot = String::from_utf8(output.stdout).ok()?.trim().to_string();

                if !sysroot.is_empty() {
                    return Some(PathBuf::from(sysroot));
                }
            }

            // Some packaged GCC cross-toolchains (e.g. PlatformIO's
            // `toolchain-riscv32-esp` / `toolchain-xtensa-esp32s3`, both based
            // on crosstool-NG esp-2021r2-patch5 / GCC 8.4.0) print an empty
            // string for `-print-sysroot` even though the sysroot is present
            // on disk. Fall back to deriving the prefix from
            // `-print-search-dirs` (the `install:` line points at
            // `<prefix>/lib/gcc/<triple>/<version>/`) and then joining the
            // target triple.
            let install_dir = Command::new(compiler)
                .arg("-print-search-dirs")
                .output()
                .ok()
                .and_then(|o| String::from_utf8(o.stdout).ok())
                .and_then(|s| {
                    s.lines()
                        .find(|l| l.starts_with("install:"))
                        .map(|l| PathBuf::from(l.trim_start_matches("install:").trim()))
                });

            if let Some(install_dir) = install_dir {
                // install_dir = <prefix>/lib/gcc/<triple>/<version>/
                // Walk up 4 levels (version -> triple -> gcc -> lib) to recover <prefix>.
                if let Some(prefix) = install_dir
                    .parent() // <version>
                    .and_then(|p| p.parent()) // <triple>
                    .and_then(|p| p.parent()) // gcc
                    .and_then(|p| p.parent())
                // lib  ->  <prefix>
                {
                    if let Some(triple) = self.derive_gcc_target_triple() {
                        // Two common layouts:
                        //   - crosstool-NG / PlatformIO (ESP toolchains):
                        //       `<prefix>/<triple>/include/`
                        //   - Debian / Ubuntu native packaging of
                        //     `gcc-arm-none-eabi`:
                        //       `<prefix>/lib/<triple>/include/`
                        // Probe each in turn, accept the first whose
                        // `include/stdio.h` exists.
                        for candidate in [prefix.join(triple), prefix.join("lib").join(triple)] {
                            if candidate.join("include").join("stdio.h").exists() {
                                return Some(candidate);
                            }
                        }
                    }
                }
            }

            None
        } else {
            None
        }
    }

    /// Returns the GCC cross-toolchain triple for ESP cross targets, mirroring
    /// the compiler binary names in `derive_forced_c_compiler`. Returns `None`
    /// for host GCC or any target where no fixed cross-triple is known
    /// (e.g. unforced RISC-V, where `cc-rs` selects the compiler).
    fn derive_gcc_target_triple(&self) -> Option<&'static str> {
        match self.target().as_str() {
            "xtensa-esp32-none-elf" | "xtensa-esp32-espidf" => Some("xtensa-esp32-elf"),
            "xtensa-esp32s2-none-elf" | "xtensa-esp32s2-espidf" => Some("xtensa-esp32s2-elf"),
            "xtensa-esp32s3-none-elf" | "xtensa-esp32s3-espidf" => Some("xtensa-esp32s3-elf"),
            "riscv32imc-unknown-none-elf"
            | "riscv32imc-esp-espidf"
            | "riscv32imac-unknown-none-elf"
            | "riscv32imac-esp-espidf"
            | "riscv32imafc-unknown-none-elf"
            | "riscv32imafc-esp-espidf"
                if self.force_esp_riscv_gcc =>
            {
                Some("riscv32-esp-elf")
            }
            // ARM bare-metal Rust targets all map to the same `arm-none-eabi`
            // GCC cross-toolchain (Cortex-M architecture is selected via
            // `-mcpu` / `-march` flags, not via separate compilers).
            "thumbv6m-none-eabi"
            | "thumbv7m-none-eabi"
            | "thumbv7em-none-eabi"
            | "thumbv7em-none-eabihf"
            | "thumbv8m.base-none-eabi"
            | "thumbv8m.main-none-eabi"
            | "thumbv8m.main-none-eabihf" => Some("arm-none-eabi"),
            _ => None,
        }
    }

    fn derive_c_compiler(&self) -> (PathBuf, bool) {
        if let Some((compiler, gnu)) = self.derive_forced_c_compiler() {
            return (compiler, gnu);
        }

        let mut build = cc::Build::new();
        build.opt_level(0);

        if let Some(target) = self.cmake_rust_target.as_ref() {
            build.target(target);
        }

        if let Some(host) = self.cmake_host_rust_target.as_ref() {
            build.host(host);
        }

        let compiler = build.get_compiler();

        (compiler.path().to_path_buf(), compiler.is_like_gnu())
    }

    fn derive_forced_c_compiler(&self) -> Option<(PathBuf, bool)> {
        if self.force_clang {
            Some((PathBuf::from("clang"), false))
        } else {
            match self.target().as_str() {
                "xtensa-esp32-none-elf" | "xtensa-esp32-espidf" => {
                    Some((PathBuf::from("xtensa-esp32-elf-gcc"), true))
                }
                "xtensa-esp32s2-none-elf" | "xtensa-esp32s2-espidf" => {
                    Some((PathBuf::from("xtensa-esp32s2-elf-gcc"), true))
                }
                "xtensa-esp32s3-none-elf" | "xtensa-esp32s3-espidf" => {
                    Some((PathBuf::from("xtensa-esp32s3-elf-gcc"), true))
                }
                "riscv32imc-unknown-none-elf"
                | "riscv32imc-esp-espidf"
                | "riscv32imac-unknown-none-elf"
                | "riscv32imac-esp-espidf"
                | "riscv32imafc-unknown-none-elf"
                | "riscv32imafc-esp-espidf" => {
                    if self.force_esp_riscv_gcc {
                        Some((PathBuf::from("riscv32-esp-elf-gcc"), true))
                    } else {
                        None
                    }
                }
                _ => None,
            }
        }
    }

    fn derive_c_args(&self) -> Vec<String> {
        let mut args = Vec::new();

        args.extend(
            self.derive_c_target_args()
                .iter()
                .map(|arg| arg.to_string()),
        );

        if self.force_clang {
            if let Some(sysroot_path) = self.derive_sysroot() {
                args.push("-fbuiltin".to_string());
                // OpenThread uses C++, but does not use any of the C++ standard
                // library features, instead preferring to use the C standard library.
                // This is very helpful for us because it means we don't have to
                // provide any STL headers in our sysroot.
                args.push("-nostdinc++".to_string());
                args.push(format!("-I{}", sysroot_path.join("include").display()));
                args.push(format!("--sysroot={}", sysroot_path.display()));
            }
        }

        args
    }

    fn derive_c_target_args(&self) -> &[&str] {
        if self.force_clang {
            match self.target().as_str() {
                "riscv32imc-unknown-none-elf" | "riscv32imc-esp-espidf" => {
                    &["--target=riscv32-esp-elf", "-march=rv32imc", "-mabi=ilp32"]
                }
                "riscv32imac-unknown-none-elf" | "riscv32imac-esp-espidf" => {
                    &["--target=riscv32-esp-elf", "-march=rv32imac", "-mabi=ilp32"]
                }
                "riscv32imafc-unknown-none-elf" | "riscv32imafc-esp-espidf" => &[
                    "--target=riscv32-esp-elf",
                    "-march=rv32imafc",
                    "-mabi=ilp32",
                ],
                "xtensa-esp32-none-elf" | "xtensa-esp32-espidf" => {
                    &["--target=xtensa-esp-elf", "-mcpu=esp32"]
                }
                "xtensa-esp32s2-none-elf" | "xtensa-esp32s2-espidf" => {
                    &["--target=xtensa-esp-elf", "-mcpu=esp32s2"]
                }
                "xtensa-esp32s3-none-elf" | "xtensa-esp32s3-espidf" => {
                    &["--target=xtensa-esp-elf", "-mcpu=esp32s3"]
                }
                _ => &[],
            }
        } else {
            match self.target().as_str() {
                "riscv32imc-unknown-none-elf" | "riscv32imc-esp-espidf" => {
                    &["-march=rv32imc", "-mabi=ilp32"]
                }
                "riscv32imac-unknown-none-elf" | "riscv32imac-esp-espidf" => {
                    &["-march=rv32imac", "-mabi=ilp32"]
                }
                "riscv32imafc-unknown-none-elf" | "riscv32imafc-esp-espidf" => {
                    &["-march=rv32imafc", "-mabi=ilp32"]
                }
                "xtensa-esp32-none-elf" | "xtensa-esp32-espidf" => &["-mlongcalls"],
                "xtensa-esp32s2-none-elf" | "xtensa-esp32s2-espidf" => &["-mlongcalls"],
                "xtensa-esp32s3-none-elf" | "xtensa-esp32s3-espidf" => &["-mlongcalls"],
                _ => &[],
            }
        }
    }

    fn target(&self) -> String {
        self.cmake_rust_target
            .clone()
            .unwrap_or_else(|| std::env::var("TARGET").unwrap().to_string())
    }
}
