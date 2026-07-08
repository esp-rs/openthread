//! Feature-driven OpenThread configuration.
//!
//! OpenThread's footprint is dominated by the fact that `ot::Instance` holds
//! *every* enabled optional feature as a direct member object (`mDnsClient`,
//! `mDhcp6Client`, `mApplicationCoap`, …), each guarded by an
//! `OPENTHREAD_CONFIG_*_ENABLE` `#if`. The single, always-live `Instance`
//! constructor constructs all of them, so their code is reachable and the
//! linker's `--gc-sections` cannot drop an unused-but-enabled feature. The only
//! way to keep a feature's member (and code) out of the binary is to compile it
//! out via its knob. So — exactly like `mbedtls-rs-sys` — algorithm/feature
//! selection must be a *compile-time* decision, modeled here as additive cargo
//! features.
//!
//! Each feature maps to one or more OpenThread CMake `OT_*` options, which the
//! `ot_option` macro turns into `OPENTHREAD_CONFIG_*_ENABLE=0/1` compile
//! definitions applied to the shared core sources (compiled into both the MTD
//! and FTD libraries).
//!
//! Note: OpenThread's *own* defaults for these knobs are mixed (e.g.
//! `OPENTHREAD_CONFIG_TCP_ENABLE` defaults to **1**), so we cannot rely on
//! "feature off ⇒ knob off". [`apply_features`] therefore explicitly passes
//! `OT_X=OFF` for every knob in [`KNOB_UNIVERSE`] first, then `OT_X=ON` for the
//! knobs the active features request — a deterministic reset, like
//! `mbedtls-rs-sys`'s `#undef` baseline.
//!
//! Device type (MTD/FTD) and co-processor role (NCP/RCP) are **not** modeled
//! here: they are separate `.a` archives built from separate source sets, all
//! produced unconditionally. The consumer's linker selects which archive(s) to
//! pull; the others cost zero bytes in the final firmware. This module only
//! governs the shared `OPENTHREAD_CONFIG_*` knobs.

/// Every OpenThread `OT_*` knob this crate exposes as a feature. All of these
/// are forced `OFF` at the start of [`apply_features`] (resetting OpenThread's
/// own, sometimes-`ON`, defaults), then re-enabled per active feature.
pub const KNOB_UNIVERSE: &[&str] = &[
    "OT_TCP",
    "OT_COAP",
    "OT_COAP_BLOCK",
    "OT_COAP_OBSERVE",
    "OT_COAPS",
    "OT_SRP_CLIENT",
    "OT_SRP_SERVER",
    "OT_DNS_CLIENT",
    "OT_DNSSD_SERVER",
    "OT_MDNS",
    "OT_SERVICE",
    "OT_NETDATA_PUBLISHER",
    "OT_SLAAC",
    "OT_DHCP6_CLIENT",
    "OT_DHCP6_SERVER",
    "OT_NAT64_TRANSLATOR",
    "OT_JOINER",
    "OT_COMMISSIONER",
    "OT_BORDER_ROUTER",
    "OT_BORDER_ROUTING",
    "OT_PING_SENDER",
    "OT_LINK_METRICS_INITIATOR",
    "OT_LINK_METRICS_SUBJECT",
    "OT_MAC_FILTER",
    "OT_JAM_DETECTION",
    "OT_CHILD_SUPERVISION",
    "OT_MESH_DIAG",
    "OT_HISTORY_TRACKER",
    "OT_DATASET_UPDATER",
    "OT_ECDSA",
    "OT_SNTP_CLIENT",
    "OT_TREL",
];

/// Maps each public cargo feature (by its `CARGO_FEATURE_*` env-var suffix) to
/// the OpenThread `OT_*` knob(s) it enables. The first tuple element is the
/// uppercased, underscored feature name as Cargo exposes it
/// (`srp-client` -> `SRP_CLIENT`).
///
/// Dependencies that OpenThread requires together are folded into the owning
/// feature (e.g. CoAP sub-options imply the CoAP API).
pub const FEATURE_DEFINES: &[(&str, &[&str])] = &[
    // Transport / application protocols
    ("TCP", &["OT_TCP"]),
    ("COAP", &["OT_COAP"]),
    ("COAP_BLOCK", &["OT_COAP", "OT_COAP_BLOCK"]),
    ("COAP_OBSERVE", &["OT_COAP", "OT_COAP_OBSERVE"]),
    ("COAPS", &["OT_COAPS"]),
    // Service discovery / naming
    ("SRP_CLIENT", &["OT_SRP_CLIENT"]),
    ("SRP_SERVER", &["OT_SRP_SERVER"]),
    ("DNS_CLIENT", &["OT_DNS_CLIENT"]),
    ("DNSSD_SERVER", &["OT_DNSSD_SERVER"]),
    ("MDNS", &["OT_MDNS"]),
    ("SERVICE", &["OT_SERVICE"]),
    ("NETDATA_PUBLISHER", &["OT_NETDATA_PUBLISHER"]),
    // IPv6 / addressing
    ("SLAAC", &["OT_SLAAC"]),
    ("DHCP6_CLIENT", &["OT_DHCP6_CLIENT"]),
    ("DHCP6_SERVER", &["OT_DHCP6_SERVER"]),
    ("NAT64", &["OT_NAT64_TRANSLATOR"]),
    // Commissioning
    ("JOINER", &["OT_JOINER"]),
    ("COMMISSIONER", &["OT_COMMISSIONER"]),
    // Border functionality
    ("BORDER_ROUTER", &["OT_BORDER_ROUTER"]),
    ("BORDER_ROUTING", &["OT_BORDER_ROUTING"]),
    // Diagnostics / management
    ("PING_SENDER", &["OT_PING_SENDER"]),
    ("LINK_METRICS_INITIATOR", &["OT_LINK_METRICS_INITIATOR"]),
    ("LINK_METRICS_SUBJECT", &["OT_LINK_METRICS_SUBJECT"]),
    ("MAC_FILTER", &["OT_MAC_FILTER"]),
    ("JAM_DETECTION", &["OT_JAM_DETECTION"]),
    ("CHILD_SUPERVISION", &["OT_CHILD_SUPERVISION"]),
    ("MESH_DIAG", &["OT_MESH_DIAG"]),
    ("HISTORY_TRACKER", &["OT_HISTORY_TRACKER"]),
    ("DATASET_UPDATER", &["OT_DATASET_UPDATER"]),
    // Misc
    ("ECDSA", &["OT_ECDSA"]),
    ("SNTP_CLIENT", &["OT_SNTP_CLIENT"]),
    ("TREL", &["OT_TREL"]),
];

/// The `FEATURE_DEFINES` keys that the `prebuilt` profile (= `matter`) enables —
/// i.e. the exact knob set the committed per-target `.a` libraries are built
/// with (see `xtask`). Mirrors the `matter` bundle in `Cargo.toml`; keep in
/// sync (the build script's desync guard enforces this).
pub const PREBUILT_FEATURES: &[&str] =
    &["SRP_CLIENT", "SLAAC", "ECDSA", "PING_SENDER", "DNS_CLIENT"];

/// An OpenThread knob and its on/off state, ready to be passed to CMake.
pub struct KnobSetting {
    pub knob: &'static str,
    pub on: bool,
}

/// Compute the full set of OpenThread knob settings for the given active-feature
/// predicate: every knob in [`KNOB_UNIVERSE`] reset to `OFF`, then turned `ON`
/// for each active feature's knobs. Returned sorted by knob name for stable
/// comparison.
fn knob_settings(is_active: impl Fn(&str) -> bool) -> Vec<KnobSetting> {
    use std::collections::BTreeMap;

    // 1. Reset every exposed knob to OFF (deterministic baseline).
    let mut state: BTreeMap<&'static str, bool> =
        KNOB_UNIVERSE.iter().map(|k| (*k, false)).collect();

    // 2. Enable knobs for each active feature.
    for (feature, knobs) in FEATURE_DEFINES {
        if is_active(feature) {
            for knob in *knobs {
                state.insert(knob, true);
            }
        }
    }

    state
        .into_iter()
        .map(|(knob, on)| KnobSetting { knob, on })
        .collect()
}

/// The knob settings for the currently enabled `CARGO_FEATURE_*` environment
/// variables. Must only be called from within the build script.
pub fn active_knob_settings() -> Vec<KnobSetting> {
    knob_settings(|feature| std::env::var_os(format!("CARGO_FEATURE_{feature}")).is_some())
}

/// The knob settings the *prebuilt* artifacts correspond to ([`PREBUILT_FEATURES`]).
pub fn prebuilt_knob_settings() -> Vec<KnobSetting> {
    knob_settings(|feature| PREBUILT_FEATURES.contains(&feature))
}

/// Whether any DTLS-based OpenThread feature (secure CoAP / commissioning) is
/// active. These pull `mbedtls-rs-sys`'s DTLS features via the internal
/// `_ot-dtls` cargo feature; see `Cargo.toml`.
///
/// Used to apply an OpenThread-only compatibility define (see builder).
pub fn dtls_active() -> bool {
    ["COAPS", "JOINER", "COMMISSIONER"]
        .iter()
        .any(|f| std::env::var_os(format!("CARGO_FEATURE_{f}")).is_some())
}

/// Whether OpenThread is built against the external MbedTLS (`mbedtls-rs-sys`
/// feature) rather than its own bundled MbedTLS. This is part of the prebuilt
/// fingerprint: the committed libraries are built with the BUNDLED MbedTLS (the
/// `prebuilt` profile does NOT include `mbedtls-rs-sys`), so a build that uses
/// the external MbedTLS must not reuse them.
pub fn use_external_mbedtls() -> bool {
    std::env::var_os("CARGO_FEATURE_MBEDTLS_RS_SYS").is_some()
}

/// OpenThread keeps ONE general-purpose heap: a single fixed-size buffer baked
/// into the firmware (`Instance::sHeap`, sized by
/// `OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE[_NO_DTLS]`), with OpenThread's own block
/// allocator on top. Two consumers draw from it:
///
///  - OpenThread itself (`HeapData`/`HeapString`, SRP tables, network data, …),
///    via `ot::Heap::CAlloc`.
///  - MbedTLS, *if* OpenThread takes over MbedTLS's allocator
///    (`OT_BUILTIN_MBEDTLS_MANAGEMENT=ON`, the default), which rewires
///    `mbedtls_calloc`/`free` to that same `ot::Heap`.
///
/// So the buffer is small, non-growable, and SHARED. In memory-hungry scenarios
/// (e.g. Matter + Thread coexistence, where MbedTLS is shared between
/// `rs-matter` and OpenThread, and ECDSA scratch competes with OpenThread's own
/// allocations) it runs out of room.
///
/// Three independent feature axes tune this (see the accessors below). They are
/// orthogonal: each ext toggle redirects ONE consumer away from the internal
/// buffer to the global allocator, and the size knob sizes the internal buffer
/// for whoever still uses it. None overrides another.
pub struct HeapConfig {
    /// `heap-ext-ot`: redirect OpenThread's *own* heap usage to the global
    /// allocator (`OPENTHREAD_CONFIG_HEAP_EXTERNAL_ENABLE=1`). The consumer must
    /// then provide `otPlatCAlloc`/`otPlatFree` (forwarding to whatever global
    /// allocator the firmware has), exactly like MbedTLS's `calloc`/`free`
    /// contract.
    pub ext_ot: bool,
    /// `heap-ext-mbedtls`: turn OpenThread's builtin MbedTLS management OFF
    /// (`OT_BUILTIN_MBEDTLS_MANAGEMENT=OFF`), so MbedTLS allocates via the plain
    /// `calloc`/`free` symbols (the firmware's global allocator) rather than
    /// OpenThread's internal buffer. Only meaningful with the external MbedTLS.
    pub ext_mbedtls: bool,
    /// `heap-int-<N>`: resize the internal buffer to `Some(N)` bytes (both the
    /// DTLS and NO_DTLS variants), or `None` to keep OpenThread's own default.
    /// Applies whenever set, independently of the ext toggles, since the
    /// internal buffer may still be used by whichever consumer is NOT redirected
    /// to the global allocator.
    pub int_size: Option<u32>,
}

/// The internal-buffer sizes (in bytes) exposed as `heap-int-<N>` cargo
/// features. Keep in sync with the `heap-int-*` features in `Cargo.toml`.
pub const HEAP_INT_SIZES: &[u32] = &[4096, 6144, 8192, 12288, 16384, 32768, 49152, 65536];

/// Resolve the active heap configuration from the `CARGO_FEATURE_*` environment.
/// The three axes are independent (see [`HeapConfig`]); the only "additive"
/// rule is that, among multiple `heap-int-<N>`, the *largest* size wins (cargo
/// features union across the dependency graph and cannot be made exclusive).
/// Must only be called from within the build script.
pub fn heap_config() -> HeapConfig {
    HeapConfig {
        ext_ot: std::env::var_os("CARGO_FEATURE_HEAP_EXT_OT").is_some(),
        ext_mbedtls: std::env::var_os("CARGO_FEATURE_HEAP_EXT_MBEDTLS").is_some(),
        int_size: HEAP_INT_SIZES
            .iter()
            .copied()
            .filter(|n| std::env::var_os(format!("CARGO_FEATURE_HEAP_INT_{n}")).is_some())
            .max(),
    }
}

/// The OpenThread static libraries to link, selected by the device-type
/// (`ftd`) and radio-location (`rcp`) features. All archives are always *built*;
/// this picks the subset the firmware actually links so the core-stack archives
/// don't collide (both `openthread-mtd` and `openthread-ftd` define
/// `otInstanceInitSingle`, etc.).
///
/// Returns library names without the `lib` prefix / `.a` suffix, as expected by
/// `cargo::rustc-link-lib=static=`. Names not present for a given target are
/// silently skipped by the caller.
pub fn device_link_libs() -> Vec<&'static str> {
    let ftd = std::env::var_os("CARGO_FEATURE_FTD").is_some();
    let rcp = std::env::var_os("CARGO_FEATURE_RCP").is_some();
    let tcp = std::env::var_os("CARGO_FEATURE_TCP").is_some();

    let core = if ftd {
        "openthread-ftd"
    } else {
        "openthread-mtd"
    };

    // Static-archive link order matters: a referencing archive must precede the
    // one defining the symbol. The radio-spinel client references the core stack
    // (and the common spinel codec), so it goes first; the core follows.
    let mut libs = Vec::new();

    if rcp {
        // Remote radio (RCP) over a spinel transport: this MCU runs the stack
        // but drives a separate radio chip. The Rust `SpinelRadio` driver
        // (`openthread::rcp`) implements the spinel/HDLC wire protocol itself and
        // does NOT use OpenThread's synchronous C++ `RadioSpinel` client. The one
        // C dependency is the variable-length packed-uint codec from `spinel.c`
        // (in `openthread-spinel-rcp`), reached via the `spinel_codec.c` shim
        // compiled into `support`. So — unlike OpenThread's POSIX RCP host — we
        // link neither `openthread-radio-spinel` (the blocking client) nor
        // `openthread-hdlc` (framing is done in Rust); only the codec archive.
        libs.push("openthread-spinel-rcp");
    }

    libs.push(core);

    // TCPlp (OpenThread's TCP implementation) is its own archive and is only
    // referenced by the core stack when `OPENTHREAD_CONFIG_TCP_ENABLE` is set
    // (the `tcp` feature). Link it only then.
    if tcp {
        libs.push(if ftd { "tcplp-ftd" } else { "tcplp-mtd" });
    }

    // Role-agnostic archives, always linked, last (most-depended-upon).
    libs.push("openthread-platform");
    libs.push("openthread-platform-utils-static");
    libs.push("support");

    // Bundled MbedTLS (the default): OpenThread compiled its own vendored MbedTLS
    // into these archives (from `third_party/mbedtls`). They define the
    // `mbedtls_*` / `psa_*` symbols the core stack references, so they must be
    // linked LAST (most-depended-upon). With the external `mbedtls-rs-sys` those
    // symbols come from that crate's own linkage instead, and these archives are
    // neither built nor linked. Order within the group: mbedtls (TLS/X.509
    // callers) -> mbedx509 -> mbedcrypto (the leaf), then the crypto backends
    // mbedcrypto references (everest = Curve25519, p256m = P-256).
    if !use_external_mbedtls() {
        for lib in ["mbedtls", "mbedx509", "mbedcrypto", "everest", "p256m"] {
            libs.push(lib);
        }
    }

    libs
}

/// Whether the committed prebuilt libraries are valid for the active features.
///
/// Returns `Ok(())` if the active build matches the prebuilt one, or `Err(delta)`
/// describing the difference so the caller can rebuild and explain why. The
/// fingerprint is the `OT_*` knob set (`+OT_X`/`-OT_X` for a knob enabled here
/// but not in the prebuilt, or vice-versa) *and* the MbedTLS backend (the
/// prebuilt uses the bundled MbedTLS; an external-`mbedtls-rs-sys` build cannot
/// reuse it even if the knobs match).
pub fn prebuilt_validity() -> Result<(), String> {
    let active = active_knob_settings();
    let prebuilt = prebuilt_knob_settings();

    // Both are sorted by knob (BTreeMap); compare on/off per knob.
    let mut parts = Vec::new();
    for (a, p) in active.iter().zip(prebuilt.iter()) {
        debug_assert_eq!(a.knob, p.knob);
        if a.on != p.on {
            parts.push(if a.on {
                format!("+{}", a.knob)
            } else {
                format!("-{}", a.knob)
            });
        }
    }

    // The prebuilt is built with the bundled MbedTLS (`prebuilt` does NOT include
    // `mbedtls-rs-sys`); an external-MbedTLS build is a different OpenThread
    // compilation (different config headers; the vendored mbedtls archives are
    // absent) and must not reuse it.
    if use_external_mbedtls() {
        parts.push("+mbedtls-rs-sys (external MbedTLS)".to_string());
    }

    // The `rcp` feature compiles the spinel codec shim (`spinel_codec.c`,
    // `OT_RCP_HOST_SHIM`) into `libsupport.a` and links `openthread-spinel-rcp`.
    // The prebuilt profile is `matter` (no `rcp`), so the committed `libsupport.a`
    // lacks the shim — a `rcp` build must be produced on the fly, or the final
    // firmware link would fail with undefined `ot_spinel_*` symbols.
    if std::env::var_os("CARGO_FEATURE_RCP").is_some() {
        parts.push("+rcp (spinel codec shim)".to_string());
    }

    // The prebuilt is built with OpenThread's default heap configuration (no
    // `heap-*` feature). Any override changes the compiled `.a` (the allocator
    // wiring and/or the internal heap size), so it must force an on-the-fly
    // rebuild.
    let heap = heap_config();
    if heap.ext_ot {
        parts.push("+heap-ext-ot".to_string());
    }
    if heap.ext_mbedtls {
        parts.push("+heap-ext-mbedtls".to_string());
    }
    if let Some(n) = heap.int_size {
        parts.push(format!("+heap-int-{n}"));
    }

    if parts.is_empty() {
        Ok(())
    } else {
        Err(parts.join(", "))
    }
}
