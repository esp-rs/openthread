use std::env;
use std::path::PathBuf;

use anyhow::Result;

use clap::{Parser, Subcommand};

use log::LevelFilter;

use tempdir::TempDir;

#[path = "../../openthread-sys/gen/builder.rs"]
mod builder;

// Arguments
#[derive(Parser, Debug)]
#[command(author, version, about = "Compile and generate bindings for OpenThread to be used in Rust.", long_about = None, subcommand_required = true)]
struct Args {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Generate Rust bindings for openthread and generate .a libraries
    Gen {
        /// If the target is a riscv32 target, force the use of the Espressif RISCV GCC toolchain
        /// (`riscv32-esp-elf-gcc`) rather than the derived `riscv32-unknown-elf-gcc` toolchain which is the "official" RISC-V one
        /// (https://github.com/riscv-collab/riscv-gnu-toolchain)
        #[arg(short = 'e', long)]
        force_esp_riscv_toolchain: bool,

        /// If specified, builds OpenThread as a Full Thread Device (FTD), otherwise builds as a Minimal Thread Device (MTD)
        #[arg(long)]
        full_thread_device: bool,

        /// Target triple for which to generate bindings and `.a` libraries
        target: String,
    },
}

fn main() -> Result<()> {
    env_logger::Builder::new()
        .filter_module("xtask", LevelFilter::Info)
        .init();

    // The directory containing the cargo manifest for the 'xtask' package is a
    // subdirectory
    let workspace = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace = workspace.parent().unwrap().canonicalize()?;

    let sys_crate_root_path = workspace.join("openthread-sys");

    let args = Args::parse();

    if let Some(Commands::Gen {
        target,
        force_esp_riscv_toolchain,
        full_thread_device,
    }) = args.command
    {
        let builder = builder::OpenThreadBuilder::new(
            sys_crate_root_path.clone(),
            Some(target.clone()),
            // Fake host, but we do need to pass something to CMake
            Some("x86_64-unknown-linux-gnu".into()),
            None,
            None,
            None,
            force_esp_riscv_toolchain,
            full_thread_device,
        );

        let out = TempDir::new("openthread-sys-libs")?;

        builder.compile(
            out.path(),
            Some(&sys_crate_root_path.join("libs").join(&target)),
        )?;

        let out = TempDir::new("openthread-sys-bindings")?;

        builder.generate_bindings(
            out.path(),
            Some(
                &sys_crate_root_path
                    .join("src")
                    .join("include")
                    .join(format!("{target}.rs")),
            ),
        )?;
    }

    Ok(())
}
