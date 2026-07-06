use std::env;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use anyhow::{anyhow, bail, ensure, Context, Result};

use clap::{Parser, Subcommand};

use log::{info, LevelFilter};

use tempfile::TempDir;

mod ping_stress;

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "Compile and generate bindings for OpenThread to be used in Rust.",
    long_about = None,
    subcommand_required = true,
)]
struct Args {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Generate Rust bindings and the matching `.a` static libraries for a
    /// given target.
    ///
    /// Delegates the actual build to `cargo build -p openthread-sys --target
    /// <target> --features force-generate-bindings[,...]`, so the OpenThread
    /// C build runs through the regular Cargo dep graph (notably picking up
    /// `DEP_MBEDTLS_INCLUDE` from `mbedtls-rs-sys`'s build script). The
    /// resulting `.a` libraries and `bindings.rs` are then copied from the
    /// `openthread-sys` build script's `OUT_DIR` to the canonical
    /// `openthread-sys/libs/<target>/` and
    /// `openthread-sys/src/include/<target>.rs` paths.
    Gen {
        /// Use GCC instead of clang to build the C OpenThread code.
        ///
        /// Note that - for non-host targets - the user is expected to have the
        /// corresponding GCC cross-toolchain installed.
        #[arg(short = 'g', long = "gcc")]
        use_gcc: bool,

        /// If the target is a riscv32 target, force the use of the Espressif RISCV GCC toolchain
        /// (`riscv32-esp-elf-gcc`) rather than the derived `riscv32-unknown-elf-gcc` toolchain which is the "official" RISC-V one
        /// (https://github.com/riscv-collab/riscv-gnu-toolchain).
        ///
        /// Implies `--gcc`.
        #[arg(short = 'e', long = "force-esp-riscv-gcc")]
        force_esp_riscv_gcc: bool,

        /// Target triple for which to generate bindings and `.a` libraries.
        target: String,

        /// Extra arguments to forward verbatim to the underlying
        /// `cargo build` invocation. Specify after a `--` separator.
        ///
        /// Notably useful for `-Zbuild-std=core,alloc,panic_abort` when
        /// building for Tier-3 targets like Xtensa, where rustup doesn't
        /// ship a pre-compiled `core`. Such a build also requires the
        /// matching toolchain to be active (e.g. `cargo +esp xtask gen
        /// xtensa-esp32-none-elf -- -Zbuild-std=core,alloc,panic_abort`);
        /// the xtask itself stays toolchain-agnostic.
        #[arg(last = true, allow_hyphen_values = true)]
        cargo_args: Vec<String>,
    },

    /// Load/recovery-test a live OpenThread device with escalating ICMPv6
    /// echo traffic (via the system `ping`).
    ///
    /// Sweeps a payload-size × interval matrix against the device and checks
    /// that loss grows gracefully with the offered load and that the device
    /// answers a clean probe promptly after each burst. Works against any
    /// OpenThread node reachable over IPv6 — the host RCP driver as well as
    /// an MCU with its native radio. See `xtask/src/ping_stress.rs`.
    PingStress(ping_stress::PingStressArgs),
}

fn main() -> Result<()> {
    env_logger::Builder::new()
        .filter_module("xtask", LevelFilter::Info)
        .init();

    // The directory containing the cargo manifest for the 'xtask' package is a
    // subdirectory of the workspace root.
    let workspace = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace = workspace.parent().unwrap().canonicalize()?;

    let sys_crate_root_path = workspace.join("openthread-sys");

    let args = Args::parse();

    if let Some(Commands::PingStress(ping_args)) = &args.command {
        return ping_stress::run(ping_args);
    }

    if let Some(Commands::Gen {
        target,
        use_gcc,
        force_esp_riscv_gcc,
        cargo_args,
    }) = args.command
    {
        // Validate before constructing `libs/<target>` and
        // `src/include/<target>.rs`; `libs_dst` is later passed to
        // `fs::remove_dir_all`. The `ensure!` below is defense-in-depth and
        // survives release builds (unlike `debug_assert!`).
        validate_target_triple(&target).context("validating --target argument")?;

        let libs_root = sys_crate_root_path.join("libs");
        fs::create_dir_all(&libs_root)
            .with_context(|| format!("creating {}", libs_root.display()))?;
        let canonical_libs_root = libs_root
            .canonicalize()
            .with_context(|| format!("canonicalizing {}", libs_root.display()))?;
        let libs_dst = canonical_libs_root.join(&target);
        ensure!(
            libs_dst.starts_with(&canonical_libs_root),
            "BUG: {} escaped {}",
            libs_dst.display(),
            canonical_libs_root.display(),
        );

        let bindings_dst = sys_crate_root_path
            .join("src")
            .join("include")
            .join(format!("{target}.rs"));

        // `_target_dir` is kept in scope for the duration of `harvest` so the
        // scratch CARGO_TARGET_DIR (and therefore `out_dir`, which lives inside
        // it) is not yet cleaned up. It's dropped at the end of this scope.
        let (_target_dir, out_dir) = run_cargo(
            &workspace,
            &target,
            use_gcc,
            force_esp_riscv_gcc,
            &cargo_args,
        )?;

        harvest(&out_dir, &libs_dst, &bindings_dst)?;
    }

    Ok(())
}

/// Spawn `cargo build` on `openthread-sys` for `target`, parse the
/// `--message-format=json-render-diagnostics` event stream and return the
/// `OUT_DIR` reported by `openthread-sys`' build script.
fn run_cargo(
    workspace: &Path,
    target: &str,
    use_gcc: bool,
    force_esp_riscv_gcc: bool,
    cargo_args: &[String],
) -> Result<(TempDir, PathBuf)> {
    // Build the prebuilt artifacts with exactly the `prebuilt` (= `matter`)
    // feature profile, decoupled from `default`, so the committed per-target
    // `.a` libraries correspond to one explicit, named knob set. `openthread-sys`'s
    // build script validates consumer builds against this same profile.
    let mut features: Vec<&str> = vec!["prebuilt", "force-generate-bindings"];
    if use_gcc {
        features.push("use-gcc");
    }
    if force_esp_riscv_gcc {
        // `force-esp-riscv-gcc` already implies `use-gcc` via Cargo's feature
        // dependency, but adding both is harmless and self-documenting.
        features.push("force-esp-riscv-gcc");
    }
    let features_arg = features.join(",");

    // Use a scratch CARGO_TARGET_DIR so every `xtask gen` invocation is a
    // guaranteed-clean build. The OpenThread + MbedTLS C compile is the
    // expensive part anyway; pretending to cache it across invocations would
    // risk shipping pre-generated artifacts that aren't actually consistent
    // with the current source.
    let target_dir = TempDir::with_prefix("openthread-sys-xtask-")
        .context("creating scratch CARGO_TARGET_DIR")?;

    info!(
        "Building openthread-sys for {target} (features: {features_arg}, \
         scratch dir: {})",
        target_dir.path().display(),
    );

    let cargo = env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let mut child = Command::new(&cargo)
        .arg("build")
        .arg("--release")
        .arg("-p")
        .arg("openthread-sys")
        .arg("--target")
        .arg(target)
        // Pin the feature profile to `prebuilt`; don't let `default` leak in.
        .arg("--no-default-features")
        .arg("--features")
        .arg(&features_arg)
        // JSON on stdout for programmatic consumption; human-readable
        // diagnostics still rendered on stderr.
        .arg("--message-format=json-render-diagnostics")
        // Forward any user-supplied extra args (e.g.
        // `-Zbuild-std=core,alloc,panic_abort` for Xtensa) verbatim.
        .args(cargo_args)
        .current_dir(workspace)
        .env("CARGO_TARGET_DIR", target_dir.path())
        .stdout(Stdio::piped())
        .spawn()
        .context("spawning `cargo build`")?;

    let stdout = child.stdout.take().expect("stdout is piped");

    let mut out_dir: Option<PathBuf> = None;
    for line in BufReader::new(stdout).lines() {
        let line = line.context("reading cargo stdout")?;
        let msg: serde_json::Value = match serde_json::from_str(&line) {
            Ok(v) => v,
            // Non-JSON output gets ignored; cargo only emits JSON in this mode,
            // but be defensive.
            Err(_) => continue,
        };
        if msg.get("reason").and_then(|v| v.as_str()) != Some("build-script-executed") {
            continue;
        }
        let pkg = msg
            .get("package_id")
            .and_then(|v| v.as_str())
            .unwrap_or_default();
        // Both `openthread-sys` and `mbedtls-rs-sys` produce
        // build-script-executed messages; we only care about the former.
        // `mbedtls-rs-sys` doesn't contain the substring "openthread-sys",
        // so a `contains` match is sufficient.
        if !pkg.contains("openthread-sys") {
            continue;
        }
        if let Some(od) = msg.get("out_dir").and_then(|v| v.as_str()) {
            out_dir = Some(PathBuf::from(od));
        }
    }

    let status = child.wait().context("waiting for cargo")?;
    if !status.success() {
        bail!("`cargo build` failed");
    }

    let out_dir = out_dir.ok_or_else(|| {
        anyhow!(
            "no `build-script-executed` event for openthread-sys in cargo output \
             - cannot locate OUT_DIR"
        )
    })?;

    Ok((target_dir, out_dir))
}

/// Copy the build script's outputs to the canonical pre-generated paths.
fn harvest(out_dir: &Path, libs_dst: &Path, bindings_dst: &Path) -> Result<()> {
    // `openthread-sys/gen/builder.rs::compile` lands the `.a` files under
    // `<OUT_DIR>/openthread/lib/`.
    let src_libs = out_dir.join("openthread").join("lib");
    if !src_libs.is_dir() {
        bail!("expected `{}` to exist after the build", src_libs.display(),);
    }

    // Clear any prior contents so libraries removed or renamed in the current
    // openthread-sys configuration don't linger as orphans.
    if libs_dst.exists() {
        fs::remove_dir_all(libs_dst).with_context(|| format!("clearing {}", libs_dst.display()))?;
    }

    fs::create_dir_all(libs_dst).with_context(|| format!("creating {}", libs_dst.display()))?;

    let mut count = 0usize;
    for entry in
        fs::read_dir(&src_libs).with_context(|| format!("reading {}", src_libs.display()))?
    {
        let entry = entry?;

        let name = entry.file_name();
        let name_str = name.to_string_lossy();

        let lower = name_str.to_ascii_lowercase();

        if lower.ends_with(".a") || lower.ends_with(".lib") {
            let dst = libs_dst.join(&*name_str);

            fs::copy(entry.path(), &dst).with_context(|| {
                format!("copying {} -> {}", entry.path().display(), dst.display(),)
            })?;

            count += 1;
        }
    }

    info!("Copied {count} static libraries to {}", libs_dst.display());

    // `openthread-sys/gen/builder.rs::generate_bindings` writes the bindings
    // to `<OUT_DIR>/bindings.rs`.
    let bindings_src = out_dir.join("bindings.rs");
    if !bindings_src.is_file() {
        bail!(
            "expected `{}` to exist after the build",
            bindings_src.display(),
        );
    }

    if let Some(parent) = bindings_dst.parent() {
        fs::create_dir_all(parent).with_context(|| format!("creating {}", parent.display()))?;
    }

    fs::copy(&bindings_src, bindings_dst).with_context(|| {
        format!(
            "copying {} -> {}",
            bindings_src.display(),
            bindings_dst.display(),
        )
    })?;

    info!("Copied bindings to {}", bindings_dst.display());

    Ok(())
}

/// Validates that `target` is a well-formed Rust target triple. Each `-`
/// separated segment must match `[a-z0-9_.]+` and start with `[a-z0-9_]`
/// (no leading dot); `..` is rejected anywhere; at least one `-`.
///
/// `libs/<target>` is later fed to `remove_dir_all`, so an unvalidated
/// `target` containing `/`, `\`, an absolute prefix, or `..` could escape
/// the `openthread-sys/libs/` root. Real targets that must pass include
/// `x86_64-unknown-linux-gnu`, `riscv32imac-unknown-none-elf`,
/// `thumbv6m-none-eabi`, and `thumbv7em-none-eabi`.
fn validate_target_triple(target: &str) -> Result<()> {
    if target.is_empty() {
        bail!("target triple is empty");
    }
    if target.len() > 64 {
        bail!(
            "target triple {target:?} too long ({} chars, max 64)",
            target.len()
        );
    }
    if target.contains("..") {
        bail!("invalid target triple {target:?}: contains `..`");
    }

    let segs: Vec<&str> = target.split('-').collect();
    if segs.len() < 2 {
        bail!("invalid target triple {target:?}: must contain at least one `-`");
    }
    for seg in &segs {
        if seg.is_empty() {
            bail!("invalid target triple {target:?}: empty segment");
        }
        let bytes = seg.as_bytes();
        let first = bytes[0];
        if !(first.is_ascii_lowercase() || first.is_ascii_digit() || first == b'_') {
            bail!("invalid target triple {target:?}: segment {seg:?} must start with [a-z0-9_]");
        }
        if !bytes.iter().all(|&b| is_target_byte(b)) {
            bail!("invalid target triple {target:?}: segment {seg:?} must match [a-z0-9_.]+");
        }
    }
    Ok(())
}

#[inline]
fn is_target_byte(b: u8) -> bool {
    b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_' || b == b'.'
}

#[cfg(test)]
mod tests {
    use super::validate_target_triple;

    #[test]
    fn accepts_real_targets() {
        for t in [
            "x86_64-unknown-linux-gnu",
            "thumbv6m-none-eabi",
            "thumbv7em-none-eabi",
            "riscv32imac-unknown-none-elf",
        ] {
            assert!(
                validate_target_triple(t).is_ok(),
                "rejected legit target {t:?}"
            );
        }
    }

    #[test]
    fn rejects_path_traversal_and_malformed() {
        let too_long = "a-".repeat(40);
        for bad in [
            "../etc",
            "../../etc/passwd",
            "/etc/passwd",
            ".",
            "..",
            "foo..bar",
            ".foo-bar",
            "foo-.bar",
            "foo/bar",
            "foo\\bar",
            "FOO-BAR",
            "foo bar",
            "foo\nbar",
            "-leading-dash",
            "trailing-",
            "single",
            "",
            "c:/tmp",
            too_long.as_str(),
        ] {
            assert!(
                validate_target_triple(bad).is_err(),
                "accepted malicious target {bad:?}"
            );
        }
    }
}
