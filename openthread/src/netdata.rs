use core::{
    fmt::{self, Display},
    net::Ipv6Addr,
};

use openthread_sys::{
    otBorderRouterConfig, otError_OT_ERROR_NONE, otNetDataGetNextOnMeshPrefix,
    otRoutePreference_OT_ROUTE_PREFERENCE_HIGH, otRoutePreference_OT_ROUTE_PREFERENCE_LOW,
    otRoutePreference_OT_ROUTE_PREFERENCE_MED, OT_NETWORK_DATA_ITERATOR_INIT,
};

use crate::{OpenThread, OtError};

#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub enum OtRoutePreference {
    /// Low route preference
    OtRoutePreferenceLow = -1,
    /// Medium route preference
    OtRoutePreferenceMed = 0,
    /// High route preference
    OtRoutePreferenceHigh = 1,
    Unkown = 2,
}

#[allow(non_upper_case_globals, clippy::unnecessary_cast)]
impl OtRoutePreference {
    fn from_ot_int(input: i32) -> Self {
        // These cast are needed as otRoutePreference_OT_ROUTE_PREFERENCE_* are i8 for thumbv*
        // but i32 for riscv32im*
        const value_low: i32 = otRoutePreference_OT_ROUTE_PREFERENCE_LOW as i32;
        const value_med: i32 = otRoutePreference_OT_ROUTE_PREFERENCE_MED as i32;
        const value_high: i32 = otRoutePreference_OT_ROUTE_PREFERENCE_HIGH as i32;
        match input {
            value_low => OtRoutePreference::OtRoutePreferenceLow,
            value_med => OtRoutePreference::OtRoutePreferenceMed,
            value_high => OtRoutePreference::OtRoutePreferenceHigh,
            _ => OtRoutePreference::Unkown,
        }
    }
}

impl Display for OtRoutePreference {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            OtRoutePreference::OtRoutePreferenceLow => "Low",
            OtRoutePreference::OtRoutePreferenceMed => "Medium",
            OtRoutePreference::OtRoutePreferenceHigh => "High",
            OtRoutePreference::Unkown => "Unknown",
        };
        write!(f, "OtRoutePreference: {}", s)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for OtRoutePreference {
    fn format(&self, fmt: defmt::Formatter) {
        let s = match self {
            OtRoutePreference::OtRoutePreferenceLow => "Low",
            OtRoutePreference::OtRoutePreferenceMed => "Medium",
            OtRoutePreference::OtRoutePreferenceHigh => "High",
            OtRoutePreference::Unkown => "Unknown",
        };
        defmt::write!(fmt, "OtRoutePreference: {}", s)
    }
}

/// Represents a Border Router configuration
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct OtBorderRouterConfig {
    /// The IPv6 prefix
    pub prefix: (Ipv6Addr, u8),
    /// A 2-bit signed int preference
    pub preference: OtRoutePreference,
    /// Whether prefix is preferred
    pub prefered: bool,
    /// Whether prefix can be used for address auto-configuration (SLAAC)
    pub slaac: bool,
    /// Whether border router is DHCPv6 Agent
    pub dhcp: bool,
    /// Whether DHCPv6 Agent supplying other config data
    pub configure: bool,
    /// Whether border router is a default router for prefix
    pub default_route: bool,
    /// Whether this prefix is considered on-mesh
    pub on_mesh: bool,
    /// Whether this configuration is considered Stable Network Data
    pub stable: bool,
    /// Whether this border router can supply DNS information via ND
    pub nd_dns: bool,
    /// Whether prefix is a Thread Domain Prefix (added since Thread 1.2)
    pub domain_prefix: bool,
    /// The border router's RLOC16 (value ignored on config add)
    pub rloc16: u16,
}

impl OtBorderRouterConfig {
    fn from_ot(config: otBorderRouterConfig) -> Self {
        Self {
            prefix: (
                unsafe { config.mPrefix.mPrefix.mFields.m8 }.into(),
                config.mPrefix.mLength,
            ),
            preference: OtRoutePreference::from_ot_int(config.mPreference()),
            prefered: config.mPreferred(),
            slaac: config.mSlaac(),
            dhcp: config.mDhcp(),
            configure: config.mConfigure(),
            default_route: config.mDefaultRoute(),
            on_mesh: config.mOnMesh(),
            stable: config.mStable(),
            nd_dns: config.mNdDns(),
            domain_prefix: config.mDp(),
            rloc16: config.mRloc16,
        }
    }
}

impl fmt::Display for OtBorderRouterConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "OtBorderRouterConfig {{
        prefix: ({}, {}),
        preference: {},
        preferred: {},
        slaac: {},
        dhcp: {},
        configure: {},
        default_route: {},
        on_mesh: {},
        stable: {},
        nd_dns: {},
        domain_prefix: {},
        rloc16: {}
    }}",
            self.prefix.0,
            self.prefix.1,
            self.preference,
            self.prefered,
            self.slaac,
            self.dhcp,
            self.configure,
            self.default_route,
            self.on_mesh,
            self.stable,
            self.nd_dns,
            self.domain_prefix,
            self.rloc16
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for OtBorderRouterConfig {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "OtBorderRouterConfig {{
        prefix: ({}, {}),
        preference: {},
        preferred: {},
        slaac: {},
        dhcp: {},
        configure: {},
        default_route: {},
        on_mesh: {},
        stable: {},
        nd_dns: {},
        domain_prefix: {},
        rloc16: {}
    }}",
            self.prefix.0,
            self.prefix.1,
            self.preference,
            self.prefered,
            self.slaac,
            self.dhcp,
            self.configure,
            self.default_route,
            self.on_mesh,
            self.stable,
            self.nd_dns,
            self.domain_prefix,
            self.rloc16
        )
    }
}

impl<'a> OpenThread<'a> {
    /// Gets the list of all on mesh prefixes
    ///
    /// Arguments:
    /// - `f`: A closure that will be called for each mesh prefix with the corresponding
    ///   `OtBorderRouterConfig`. Once called for all prefixes,
    ///   the closure will be called with `None`.
    pub fn netdata_get_on_mesh_prefixes<F>(&self, mut f: F) -> Result<(), OtError>
    where
        F: FnMut(Option<OtBorderRouterConfig>) -> Result<(), OtError>,
    {
        let mut ot = self.activate();
        let state = ot.state();

        let mut network_data_iterator = OT_NETWORK_DATA_ITERATOR_INIT;
        let mut a_config = otBorderRouterConfig::default();

        while unsafe {
            otNetDataGetNextOnMeshPrefix(
                state.ot.instance,
                &mut network_data_iterator,
                &mut a_config,
            )
        } == otError_OT_ERROR_NONE
        {
            f(Some(OtBorderRouterConfig::from_ot(a_config)))?;
        }

        f(None)
    }
}
