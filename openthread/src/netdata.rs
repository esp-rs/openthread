use core::{fmt, net::Ipv6Addr};

use openthread_sys::{
    otBorderRouterConfig, otError_OT_ERROR_NONE, otNetDataGetNextOnMeshPrefix,
    OT_NETWORK_DATA_ITERATOR_INIT,
};

use crate::{OpenThread, OtError};

pub struct OtBorderRouterConfig {
    pub prefix: (Ipv6Addr, u8),
    pub preference: i32,
    pub prefered: bool,
    pub slaac: bool,
    pub dhcp: bool,
    pub configure: bool,
    pub default_route: bool,
    pub on_mesh: bool,
    pub stable: bool,
    pub nd_dns: bool,
    pub domain_prefix: bool,
    pub rloc16: u16,
}

impl OtBorderRouterConfig {
    fn from_ot(config: otBorderRouterConfig) -> Self {
        Self {
            prefix: (
                unsafe { config.mPrefix.mPrefix.mFields.m8 }.into(),
                config.mPrefix.mLength,
            ),
            preference: config.mPreference(),
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

impl<'a> OpenThread<'a> {
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
