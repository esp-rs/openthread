use std::{
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{anyhow, Result};
use bindgen::Builder;
use cmake::Config;

pub struct OpenThreadBuilder {
    crate_root_path: PathBuf,
    cmake_configurer: CMakeConfigurer,
    clang_path: Option<PathBuf>,
    clang_sysroot_path: Option<PathBuf>,
    clang_target: Option<String>,
    full_thread_device: bool,
}

impl OpenThreadBuilder {
    /// Create a new OpenThreadBuilder
    ///
    /// Arguments:
    /// - `crate_root_path`: Path to the root of the crate
    /// - `cmake_rust_target`: Optional target for CMake when building Openthread, with Rust target-triple syntax. If not specified, the "TARGET" env variable will be used
    /// - `cmake_host_rust_target`: Optional host target for the build
    /// - `clang_path`: Optional path to the Clang compiler. If not specified, the system Clang will be used for generating bindings,
    ///   and the system compiler (likely GCC) would be used for building the OpenThread C/C++ code itself
    /// - `clang_sysroot_path`: Optional path to the compiler sysroot directory. If not specified, the host sysroot will be used
    /// - `clang_target`: Optional target for Clang when generating bindings. If not specified, the "TARGET" env variable target will be used
    /// - `force_esp_riscv_toolchain`: If true, and if the target is a riscv32 target, force the use of the Espressif RISCV GCC toolchain
    ///   (`riscv32-esp-elf-gcc`) rather than the derived `riscv32-unknown-elf-gcc` toolchain which is the "official" RISC-V one
    ///   (https://github.com/riscv-collab/riscv-gnu-toolchain)
    pub fn new(
        crate_root_path: PathBuf,
        cmake_rust_target: Option<String>,
        cmake_host_rust_target: Option<String>,
        clang_path: Option<PathBuf>,
        clang_sysroot_path: Option<PathBuf>,
        clang_target: Option<String>,
        force_esp_riscv_toolchain: bool,
        full_thread_device: bool,
    ) -> Self {
        Self {
            cmake_configurer: CMakeConfigurer::new(
                crate_root_path.clone(),
                cmake_rust_target,
                cmake_host_rust_target,
                force_esp_riscv_toolchain,
                crate_root_path.join("gen").join("toolchain.cmake"),
            ),
            crate_root_path,
            clang_path,
            clang_sysroot_path,
            clang_target,
            full_thread_device,
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
            .blocklist_function("strtold")
            .blocklist_function("_strtold_r")
            .blocklist_function("v.*printf")
            .blocklist_function("v.*scanf")
            .blocklist_function("_v.*printf_r")
            .blocklist_function("_v.*scanf_r")
            .blocklist_function("q.*cvt")
            .blocklist_function("q.*cvt_r")
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
        let target_dir = out_path.join("openthread").join("build");
        std::fs::create_dir_all(&target_dir)?;

        let target_lib_dir = out_path.join("openthread").join("lib");

        let lib_dir = copy_path.unwrap_or(&target_lib_dir);
        std::fs::create_dir_all(lib_dir)?;

        // Compile OpenThread and generate libraries to link against
        log::info!("Compiling OpenThread");

        let mut config = self.cmake_configurer.configure(Some(lib_dir));

        config
            .define("OT_LOG_LEVEL", "NOTE")
            .define("OT_FTD", if self.full_thread_device { "ON" } else { "OFF" })
            .define("OT_MTD", if self.full_thread_device { "OFF" } else { "ON" })
            .define("OT_RCP", "OFF")
            .define("OT_TCP", "OFF")
            .define("OT_APP_CLI", "OFF")
            .define("OT_APP_NCP", "OFF")
            .define("OT_APP_RCP", "OFF")
            .define("OT_BORDER_ROUTER", "OFF")
            .define("OT_BORDER_ROUTING", "OFF")
            .define("OT_SRP_CLIENT", "ON")
            .define("OT_SLAAC", "ON")
            .define("OT_ECDSA", "ON")
            .define("OT_PING_SENDER", "ON")
            // Do not change from here below
            .define("OT_LOG_OUTPUT", "PLATFORM_DEFINED")
            .define("OT_PLATFORM", "external")
            .define("OT_SETTINGS_RAM", "OFF")
            //.define("OT_COMPILE_WARNING_AS_ERROR", "ON "$@" "${OT_SRCDIR}"")
            // ... or else the build would fail with `arm-none-eabi-gcc` during the linking phase
            // with "undefined symbol `__exit`" error
            .define("BUILD_TESTING", "OFF")
            .define("CMAKE_POLICY_VERSION_MINIMUM", "3.5") // For MbedTLS
            .profile("Release")
            .out_dir(&target_dir);

        config.build();

        Ok(lib_dir.to_path_buf())
    }

    /// Re-run the build script if the file or directory has changed.
    #[allow(unused)]
    pub fn track(file_or_dir: &Path) {
        println!("cargo:rerun-if-changed={}", file_or_dir.display())
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
pub struct CMakeConfigurer {
    pub project_path: PathBuf,
    pub cmake_rust_target: Option<String>,
    pub cmake_host_rust_target: Option<String>,
    pub force_esp_riscv_toolchain: bool,
    pub empty_toolchain_file: PathBuf,
}

impl CMakeConfigurer {
    /// Create a new OpenThreadBuilder
    ///
    /// Arguments:
    /// - `project_path`: Path to the root of the CMake project
    /// - `cmake_rust_target`: Optional target for CMake when building Openthread, with Rust target-triple syntax. If not specified, the "TARGET" env variable will be used
    /// - `cmake_host_rust_target`: Optional host target for the build
    /// - `force_esp_riscv_toolchain`: If true, and if the target is a riscv32 target, force the use of the Espressif RISCV GCC toolchain
    ///   (`riscv32-esp-elf-gcc`) rather than the derived `riscv32-unknown-elf-gcc` toolchain which is the "official" RISC-V one
    ///   (https://github.com/riscv-collab/riscv-gnu-toolchain)
    pub const fn new(
        project_path: PathBuf,
        cmake_rust_target: Option<String>,
        cmake_host_rust_target: Option<String>,
        force_esp_riscv_toolchain: bool,
        empty_toolchain_file: PathBuf,
    ) -> Self {
        Self {
            project_path,
            cmake_rust_target,
            cmake_host_rust_target,
            force_esp_riscv_toolchain,
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
            // ... or else the build would fail with `arm-none-eabi-gcc` when testing the compiler
            .define("CMAKE_TRY_COMPILE_TARGET_TYPE", "STATIC_LIBRARY")
            .define("CMAKE_EXPORT_COMPILE_COMMANDS", "ON")
            .define("CMAKE_BUILD_TYPE", "MinSizeRel");

        if let Some(target_dir) = target_dir {
            config
                .define("CMAKE_ARCHIVE_OUTPUT_DIRECTORY", target_dir)
                .define("CMAKE_LIBRARY_OUTPUT_DIRECTORY", target_dir)
                .define("CMAKE_RUNTIME_OUTPUT_DIRECTORY", target_dir);
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
            config.cflag(arg).cxxflag(arg);
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
        let (compiler, gnu) = self.derive_c_compiler();

        if gnu {
            let output = Command::new(compiler).arg("-print-sysroot").output().ok()?;

            if output.status.success() {
                let sysroot = String::from_utf8(output.stdout).ok()?.trim().to_string();

                (!sysroot.is_empty()).then_some(PathBuf::from(sysroot))
            } else {
                None
            }
        } else {
            None
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
        match self.target().as_str() {
            "xtensa-esp32-none-elf"
            | "xtensa-esp32-espidf"
            | "xtensa-esp32s2-none-elf"
            | "xtensa-esp32s2-espidf"
            | "xtensa-esp32s3-none-elf"
            | "xtensa-esp32s3-espidf" => Some((PathBuf::from("xtensa-esp-elf-gcc"), true)),
            "riscv32imc-unknown-none-elf"
            | "riscv32imc-esp-espidf"
            | "riscv32imac-unknown-none-elf"
            | "riscv32imac-esp-espidf"
            | "riscv32imafc-unknown-none-elf"
            | "riscv32imafc-esp-espidf" => {
                if self.force_esp_riscv_toolchain {
                    Some((PathBuf::from("riscv32-esp-elf-gcc"), true))
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn derive_c_args(&self) -> &[&str] {
        match self.target().as_str() {
            "xtensa-esp32-none-elf" | "xtensa-esp32-espidf" => {
                &["target=xtensa-esp-elf", "-mcpu=esp32"]
            }
            "xtensa-esp32s2-none-elf" | "xtensa-esp32s2-espidf" => {
                &["target=xtensa-esp-elf", "-mcpu=esp32s2"]
            }
            "xtensa-esp32s3-none-elf" | "xtensa-esp32s3-espidf" => {
                &["target=xtensa-esp-elf", "-mcpu=esp32s3"]
            }
            _ => &[],
        }
    }

    fn target(&self) -> String {
        self.cmake_rust_target
            .clone()
            .unwrap_or_else(|| std::env::var("TARGET").unwrap().to_string())
    }
}
