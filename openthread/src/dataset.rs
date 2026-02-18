//! The module contains the `Dataset` type and its related types.
//!
//! Basically, a way to configure the Thread network settings.

use crate::sys::{
    otDatasetParseTlvs, otDatasetSetActive, otDatasetSetActiveTlvs, otDatasetSetPending,
    otDatasetSetPendingTlvs, otError_OT_ERROR_INVALID_ARGS, otError_OT_ERROR_NO_BUFS,
    otExtendedPanId, otMeshLocalPrefix, otNetworkKey, otOperationalDataset,
    otOperationalDatasetTlvs, otPskc, otTimestamp,
};
use crate::{ot, OpenThread, OtActiveState, OtError};

/// Active or Pending Operational Dataset
#[derive(Debug, Clone, Default, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OperationalDataset<'a> {
    /// Active Timestamp
    pub active_timestamp: Option<ThreadTimestamp>,
    /// Pending Timestamp
    pub pending_timestamp: Option<ThreadTimestamp>,
    /// Network Key
    pub network_key: Option<[u8; 16]>,
    /// Network name
    pub network_name: Option<&'a str>,
    /// Extended PAN ID
    pub extended_pan_id: Option<[u8; 8]>,
    /// Mesh Local Prefix
    pub mesh_local_prefix: Option<[u8; 8]>,
    /// Delay Timer
    pub delay: Option<u32>,
    /// PAN ID
    pub pan_id: Option<u16>,
    /// Channel
    pub channel: Option<u16>,
    /// PSKc
    pub pskc: Option<[u8; 16]>,
    /// Security Policy.
    pub security_policy: Option<SecurityPolicy>,
    /// Channel Mask
    pub channel_mask: Option<u32>,
}

impl OperationalDataset<'_> {
    /// Store the dataset in the format the `OpenThread` C library expects.
    ///
    /// Arguments:
    /// - `raw_dataset`: A mutable reference to the `otOperationalDataset` struct where the dataset needs to be stored.
    pub(crate) fn store_raw(&self, raw_dataset: &mut otOperationalDataset) {
        let dataset_slice = unsafe {
            core::slice::from_raw_parts_mut(
                raw_dataset as *mut _ as *mut u8,
                core::mem::size_of::<otOperationalDataset>(),
            )
        };
        dataset_slice.fill(0);

        let mut active_timestamp_present = false;
        let mut pending_timestamp_present = false;
        let mut network_key_present = false;
        let mut network_name_present = false;
        let mut extended_pan_present = false;
        let mut mesh_local_prefix_present = false;
        let mut delay_present = false;
        let mut pan_id_present = false;
        let mut channel_present = false;
        let mut pskc_present = false;
        let mut security_policy_present = false;
        let mut channel_mask_present = false;

        let dataset = self;

        if let Some(active_timestamp) = dataset.active_timestamp {
            raw_dataset.mActiveTimestamp = otTimestamp {
                mSeconds: active_timestamp.seconds,
                mTicks: active_timestamp.ticks,
                mAuthoritative: active_timestamp.authoritative,
            };
            active_timestamp_present = true;
        }

        if let Some(pending_timestamp) = dataset.pending_timestamp {
            raw_dataset.mActiveTimestamp = otTimestamp {
                mSeconds: pending_timestamp.seconds,
                mTicks: pending_timestamp.ticks,
                mAuthoritative: pending_timestamp.authoritative,
            };
            pending_timestamp_present = true;
        }

        if let Some(network_key) = dataset.network_key {
            raw_dataset.mNetworkKey = otNetworkKey { m8: network_key };
            network_key_present = true;
        }

        if let Some(network_name) = dataset.network_name {
            let src = network_name.as_bytes();
            let dst = &mut raw_dataset.mNetworkName.m8;

            if src.len() < dst.len() {
                dst.fill(0);
                dst[..src.len()].copy_from_slice(unsafe {
                    core::slice::from_raw_parts(src.as_ptr() as *const _, src.len())
                });
                network_name_present = true;
            }
        }

        if let Some(extended_pan_id) = dataset.extended_pan_id {
            raw_dataset.mExtendedPanId = otExtendedPanId {
                m8: extended_pan_id,
            };
            extended_pan_present = true;
        }

        if let Some(mesh_local_prefix) = dataset.mesh_local_prefix {
            raw_dataset.mMeshLocalPrefix = otMeshLocalPrefix {
                m8: mesh_local_prefix,
            };
            mesh_local_prefix_present = true;
        }

        if let Some(delay) = dataset.delay {
            raw_dataset.mDelay = delay;
            delay_present = true;
        }

        if let Some(pan_id) = dataset.pan_id {
            raw_dataset.mPanId = pan_id;
            pan_id_present = true;
        }

        if let Some(channel) = dataset.channel {
            raw_dataset.mChannel = channel;
            channel_present = true;
        }

        if let Some(pskc) = dataset.pskc {
            raw_dataset.mPskc = otPskc { m8: pskc };
            pskc_present = true;
        }

        if let Some(security_policy) = &dataset.security_policy {
            raw_dataset.mSecurityPolicy = Default::default();

            let s = &mut raw_dataset.mSecurityPolicy;
            s.mRotationTime = security_policy.rotation_time;
            s.set_mObtainNetworkKeyEnabled(security_policy.obtain_network_key_enabled);
            s.set_mNativeCommissioningEnabled(security_policy.native_commissioning_enabled);
            s.set_mRoutersEnabled(security_policy.routers_enabled);
            s.set_mExternalCommissioningEnabled(security_policy.external_commissioning_enabled);
            s.set_mCommercialCommissioningEnabled(security_policy.commercial_commissioning_enabled);
            s.set_mAutonomousEnrollmentEnabled(security_policy.autonomous_enrollment_enabled);
            s.set_mNetworkKeyProvisioningEnabled(security_policy.network_key_provisioning_enabled);
            s.set_mTobleLinkEnabled(security_policy.toble_link_enabled);
            s.set_mNonCcmRoutersEnabled(security_policy.non_ccm_routers_enabled);
            s.set_mVersionThresholdForRouting(security_policy.version_threshold_for_routing);

            security_policy_present = true;
        }

        if let Some(channel_mask) = dataset.channel_mask {
            raw_dataset.mChannelMask = channel_mask;
            channel_mask_present = true;
        }

        raw_dataset.mComponents = Default::default();

        let c = &mut raw_dataset.mComponents;
        c.mIsActiveTimestampPresent = active_timestamp_present;
        c.mIsPendingTimestampPresent = pending_timestamp_present;
        c.mIsNetworkKeyPresent = network_key_present;
        c.mIsNetworkNamePresent = network_name_present;
        c.mIsExtendedPanIdPresent = extended_pan_present;
        c.mIsMeshLocalPrefixPresent = mesh_local_prefix_present;
        c.mIsDelayPresent = delay_present;
        c.mIsPanIdPresent = pan_id_present;
        c.mIsChannelPresent = channel_present;
        c.mIsPskcPresent = pskc_present;
        c.mIsSecurityPolicyPresent = security_policy_present;
        c.mIsChannelMaskPresent = channel_mask_present;
    }

    /// Extract the regular and extended PAN IDs from the raw dataset.
    pub(crate) fn get_pan_ids(
        raw_dataset: &otOperationalDataset,
    ) -> (Option<u16>, Option<[u8; 8]>) {
        (
            raw_dataset
                .mComponents
                .mIsPanIdPresent
                .then_some(raw_dataset.mPanId),
            raw_dataset
                .mComponents
                .mIsExtendedPanIdPresent
                .then_some(raw_dataset.mExtendedPanId.m8),
        )
    }

    /// Parse the Thread TLV dataset and store it in the raw dataset.
    pub(crate) fn parse_tlv(
        tlv: &[u8],
        ot_tlv: &mut otOperationalDatasetTlvs,
        raw_dataset: &mut otOperationalDataset,
    ) -> Result<(), OtError> {
        if tlv.len() > ot_tlv.mTlvs.len() {
            Err(OtError::new(otError_OT_ERROR_NO_BUFS))?;
        }

        ot_tlv.mTlvs[..tlv.len()].copy_from_slice(tlv);
        ot_tlv.mLength = tlv.len() as _;

        ot!(unsafe { otDatasetParseTlvs(ot_tlv, raw_dataset) })
    }
}

/// Security Policy
#[derive(Debug, Clone, Default, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SecurityPolicy {
    /// The value for thrKeyRotation in units of hours.
    pub rotation_time: u16,
    /// Autonomous Enrollment is enabled.
    pub autonomous_enrollment_enabled: bool,
    /// Commercial Commissioning is enabled.
    pub commercial_commissioning_enabled: bool,
    /// External Commissioner authentication is allowed.
    pub external_commissioning_enabled: bool,
    /// Native Commissioning using PSKc is allowed.
    pub native_commissioning_enabled: bool,
    /// Network Key Provisioning is enabled.
    pub network_key_provisioning_enabled: bool,
    /// Non-CCM Routers enabled.
    pub non_ccm_routers_enabled: bool,
    /// Obtaining the Network Key for out-of-band commissioning is enabled.
    pub obtain_network_key_enabled: bool,
    /// Thread 1.0/1.1.x Routers are enabled.
    pub routers_enabled: bool,
    /// ToBLE link is enabled.
    pub toble_link_enabled: bool,
    /// Version-threshold for Routing.
    pub version_threshold_for_routing: u8,
}

/// Thread Dataset timestamp
// TODO: Do we need both "seconds" and "ticks"? Revisit this later.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ThreadTimestamp {
    pub seconds: u64,
    pub ticks: u16,
    pub authoritative: bool,
}

impl OpenThread<'_> {
    /// Extract the regular and the extended PAN IDs from a Thread TLV dataset.
    ///
    /// Arguments:
    /// - `tlv`: The Thread TLV dataset.
    ///
    /// Returns:
    /// - A tuple containing the regular and the extended PAN IDs.
    pub fn get_tlv_pan_ids(&self, tlv: &[u8]) -> Result<(Option<u16>, Option<[u8; 8]>), OtError> {
        let mut ot = self.activate();
        let state = ot.state();
        let resources = &mut state.ot.dataset_resources;

        OperationalDataset::parse_tlv(tlv, &mut resources.dataset_tlv, &mut resources.dataset)?;

        Ok(OperationalDataset::get_pan_ids(&resources.dataset))
    }

    /// Set a new active dataset in the OpenThread stack.
    ///
    /// Arguments:
    /// - `dataset`: A reference to the new dataset to be set.
    pub fn set_active_dataset(&self, dataset: &OperationalDataset<'_>) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        dataset.store_raw(&mut state.ot.dataset_resources.dataset);

        ot!(unsafe { otDatasetSetActive(state.ot.instance, &state.ot.dataset_resources.dataset) })
    }

    /// Set a new active dataset in the OpenThread stack.
    ///
    /// The dataset should be in Thread TLV format.
    ///
    /// Arguments:
    /// - `dataset`: A reference to the new dataset to be set.
    pub fn set_active_dataset_tlv(&self, dataset: &[u8]) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        Self::fill_dataset_tlv(state, dataset)?;

        ot!(unsafe {
            otDatasetSetActiveTlvs(state.ot.instance, &state.ot.dataset_resources.dataset_tlv)
        })
    }

    /// Set a new active dataset in the OpenThread stack.
    ///
    /// The dataset should be in Thread TLV format, encoded as a HEX string.
    ///
    /// Arguments:
    /// - `dataset`: A reference to the new dataset to be set.
    pub fn set_active_dataset_tlv_hexstr(&self, dataset: &str) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        Self::fill_dataset_tlv_hexstr(state, dataset)?;

        ot!(unsafe {
            otDatasetSetActiveTlvs(state.ot.instance, &state.ot.dataset_resources.dataset_tlv)
        })
    }

    /// Set a new pending dataset in the OpenThread stack.
    ///
    /// Arguments:
    /// - `dataset`: A reference to the new dataset to be set.
    pub fn set_pending_dataset(&self, dataset: &OperationalDataset<'_>) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        dataset.store_raw(&mut state.ot.dataset_resources.dataset);

        ot!(unsafe { otDatasetSetPending(state.ot.instance, &state.ot.dataset_resources.dataset) })
    }

    /// Set a new pending dataset in the OpenThread stack.
    ///
    /// The dataset should be in Thread TLV format.
    ///
    /// Arguments:
    /// - `dataset`: A reference to the new dataset to be set.
    pub fn set_pending_dataset_tlv(&self, dataset: &[u8]) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        Self::fill_dataset_tlv(state, dataset)?;

        ot!(unsafe {
            otDatasetSetPendingTlvs(state.ot.instance, &state.ot.dataset_resources.dataset_tlv)
        })
    }

    /// Set a new pending dataset in the OpenThread stack.
    ///
    /// The dataset should be in Thread TLV format, encoded as a HEX string.
    ///
    /// Arguments:
    /// - `dataset`: A reference to the new dataset to be set.
    pub fn set_pending_dataset_tlv_hexstr(&self, dataset: &str) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        Self::fill_dataset_tlv_hexstr(state, dataset)?;

        ot!(unsafe {
            otDatasetSetPendingTlvs(state.ot.instance, &state.ot.dataset_resources.dataset_tlv)
        })
    }

    /// Populates the internal OT TLV dataset structure with the given dataset in TLV slice format.
    fn fill_dataset_tlv(state: &mut OtActiveState<'_>, dataset: &[u8]) -> Result<(), OtError> {
        if state.ot.dataset_resources.dataset_tlv.mTlvs.len() < dataset.len() {
            Err(OtError::new(otError_OT_ERROR_NO_BUFS))?;
        }

        state.ot.dataset_resources.dataset_tlv.mTlvs[..dataset.len()].copy_from_slice(dataset);
        state.ot.dataset_resources.dataset_tlv.mLength = dataset.len() as _;

        Ok(())
    }

    /// Populates the internal OT TLV dataset structure with the given dataset in HEX-TLV str format.
    fn fill_dataset_tlv_hexstr(
        state: &mut OtActiveState<'_>,
        dataset: &str,
    ) -> Result<(), OtError> {
        let dataset = dataset.trim();
        let mut offset = 0;

        for (chf, chs) in dataset
            .chars()
            .step_by(2)
            .zip(dataset.chars().skip(1).step_by(2))
        {
            let byte = (chf
                .to_digit(16)
                .ok_or(OtError::new(otError_OT_ERROR_INVALID_ARGS))?
                << 4)
                | chs
                    .to_digit(16)
                    .ok_or(OtError::new(otError_OT_ERROR_INVALID_ARGS))?;

            if offset >= state.ot.dataset_resources.dataset_tlv.mTlvs.len() {
                Err(OtError::new(otError_OT_ERROR_NO_BUFS))?;
            }

            state.ot.dataset_resources.dataset_tlv.mTlvs[offset] = byte as _;
            offset += 1;
        }

        state.ot.dataset_resources.dataset_tlv.mLength = offset as _;

        Ok(())
    }
}
