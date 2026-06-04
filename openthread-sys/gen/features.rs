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
pub const PREBUILT_FEATURES: &[&str] = &["SRP_CLIENT", "SLAAC", "ECDSA", "PING_SENDER"];

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
/// fingerprint: the committed libraries are built with the external MbedTLS (the
/// `prebuilt` profile includes `mbedtls-rs-sys`), so a build that uses the
/// bundled MbedTLS must not reuse them.
pub fn use_external_mbedtls() -> bool {
    std::env::var_os("CARGO_FEATURE_MBEDTLS_RS_SYS").is_some()
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
        // but drives a separate radio chip. Mirrors OpenThread's POSIX RCP host
        // (src/posix/platform/CMakeLists.txt): radio-spinel (the RadioSpinel
        // client) + spinel-rcp (common spinel codec) + hdlc (framing).
        libs.push("openthread-radio-spinel");
        libs.push("openthread-spinel-rcp");
        libs.push("openthread-hdlc");
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

    libs
}

/// Whether the committed prebuilt libraries are valid for the active features.
///
/// Returns `Ok(())` if the active build matches the prebuilt one, or `Err(delta)`
/// describing the difference so the caller can rebuild and explain why. The
/// fingerprint is the `OT_*` knob set (`+OT_X`/`-OT_X` for a knob enabled here
/// but not in the prebuilt, or vice-versa) *and* the MbedTLS backend (the
/// prebuilt uses the external `mbedtls-rs-sys`; an internal-MbedTLS build cannot
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

    // The prebuilt is built with the external MbedTLS (`prebuilt` ⊇
    // `mbedtls-rs-sys`); a bundled-MbedTLS build is a different OpenThread
    // compilation and must not reuse it.
    if !use_external_mbedtls() {
        parts.push("-mbedtls-rs-sys (bundled MbedTLS)".to_string());
    }

    if parts.is_empty() {
        Ok(())
    } else {
        Err(parts.join(", "))
    }
}
