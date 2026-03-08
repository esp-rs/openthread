//! The module contains the `Setting` trait and its related types.
//!
//! Basically, a way for OpenThread to persist its settings.

use core::cell::RefCell;
use core::task::{Context, Poll};

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::blocking_mutex::Mutex;

use crate::fmt::Bytes;
use crate::signal::Signal;
use crate::sys::{
    OT_ERROR_INVALID_ARGS, OT_ERROR_NOT_IMPLEMENTED, OT_ERROR_NO_BUFS,
    OT_SETTINGS_KEY_ACTIVE_DATASET, OT_SETTINGS_KEY_BORDER_AGENT_ID,
    OT_SETTINGS_KEY_BR_ON_LINK_PREFIXES, OT_SETTINGS_KEY_BR_ULA_PREFIX, OT_SETTINGS_KEY_CHILD_INFO,
    OT_SETTINGS_KEY_DAD_INFO, OT_SETTINGS_KEY_NETWORK_INFO, OT_SETTINGS_KEY_PARENT_INFO,
    OT_SETTINGS_KEY_PENDING_DATASET, OT_SETTINGS_KEY_SLAAC_IID_SECRET_KEY,
    OT_SETTINGS_KEY_SRP_CLIENT_INFO, OT_SETTINGS_KEY_SRP_ECDSA_KEY,
    OT_SETTINGS_KEY_SRP_SERVER_INFO,
};
use crate::OtError;

/// Settings error
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SettingsError {
    /// Not enough space
    NoBufs,
    /// Invalid data
    InvalidData,
    /// Not implemented
    NotImplemented,
}

impl From<SettingsError> for OtError {
    fn from(error: SettingsError) -> OtError {
        match error {
            SettingsError::NoBufs => OtError::new(OT_ERROR_NO_BUFS),
            SettingsError::InvalidData => OtError::new(OT_ERROR_INVALID_ARGS),
            SettingsError::NotImplemented => OtError::new(OT_ERROR_NOT_IMPLEMENTED),
        }
    }
}

/// A non-exhaustive list of keys used by OpenThread. The list might grow when new versions of OpenThread are released.
///
/// Keys in the range 0 - 0x7fff are reserved for OpenThread
/// Keys in the range 0x8000 - 0xffff can be used by vendors
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum SettingsKey {
    /// Active Operational Dataset
    ActiveDataset = OT_SETTINGS_KEY_ACTIVE_DATASET as _,
    /// Pending Operational Dataset
    PendingDataset = OT_SETTINGS_KEY_PENDING_DATASET as _,
    /// Thread network information
    NetworkInfo = OT_SETTINGS_KEY_NETWORK_INFO as _,
    /// Parent information
    ParentInfo = OT_SETTINGS_KEY_PARENT_INFO as _,
    /// Child information
    ChildInfo = OT_SETTINGS_KEY_CHILD_INFO as _,
    /// SLAAC key to generate semantically opaque IID
    KeySlaacIidSecretKey = OT_SETTINGS_KEY_SLAAC_IID_SECRET_KEY as _,
    /// Duplicate Address Detection (DAD) information
    DadInfo = OT_SETTINGS_KEY_DAD_INFO as _,
    /// SRP client ECDSA public/private key pair
    SrpEcdsaKey = OT_SETTINGS_KEY_SRP_ECDSA_KEY as _,
    /// SRP client info (selected SRP server address)
    SrpClientInfo = OT_SETTINGS_KEY_SRP_CLIENT_INFO as _,
    /// SRP server info (UDP port)
    SrpServerInfo = OT_SETTINGS_KEY_SRP_SERVER_INFO as _,
    /// BR ULA prefix
    BrUlaPrefix = OT_SETTINGS_KEY_BR_ULA_PREFIX as _,
    /// BR on-link prefixes
    BrOnLinkPrefixes = OT_SETTINGS_KEY_BR_ON_LINK_PREFIXES as _,
    /// Unique Border Agent/Router ID
    BorderAgentId = OT_SETTINGS_KEY_BORDER_AGENT_ID as _,
}

impl TryFrom<u16> for SettingsKey {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let key = match value as _ {
            OT_SETTINGS_KEY_ACTIVE_DATASET => Self::ActiveDataset,
            OT_SETTINGS_KEY_PENDING_DATASET => Self::PendingDataset,
            OT_SETTINGS_KEY_NETWORK_INFO => Self::NetworkInfo,
            OT_SETTINGS_KEY_PARENT_INFO => Self::ParentInfo,
            OT_SETTINGS_KEY_CHILD_INFO => Self::ChildInfo,
            OT_SETTINGS_KEY_SLAAC_IID_SECRET_KEY => Self::KeySlaacIidSecretKey,
            OT_SETTINGS_KEY_DAD_INFO => Self::DadInfo,
            OT_SETTINGS_KEY_SRP_ECDSA_KEY => Self::SrpEcdsaKey,
            OT_SETTINGS_KEY_SRP_CLIENT_INFO => SettingsKey::SrpClientInfo,
            OT_SETTINGS_KEY_SRP_SERVER_INFO => Self::SrpServerInfo,
            OT_SETTINGS_KEY_BR_ULA_PREFIX => Self::BrUlaPrefix,
            OT_SETTINGS_KEY_BR_ON_LINK_PREFIXES => Self::BrOnLinkPrefixes,
            OT_SETTINGS_KEY_BORDER_AGENT_ID => Self::BorderAgentId,
            _ => Err(())?,
        };

        Ok(key)
    }
}

/// A trait for abstracting the notion of OpenThread Platform settings
///
/// Each platform should provide an implementation of the `Settings` trait, where
/// ideally - at least a portion of the settings (like the SRP key) - are saved to non-volatile storage.
///
/// User might just utilize the `RamSettings` struct, especially in cases where the non-volatile storage
/// is behind an async API. See `RamSettings` for more information.
pub trait Settings {
    /// Initialize the settings
    ///
    /// # Arguments
    /// - `sensitive_keys`: A list of sensitive keys that should ideally be saved in encrypted storage
    fn init(&mut self, sensitive_keys: &[u16]);

    /// Get a setting
    ///
    /// # Arguments
    /// - `key`: The key of the setting
    /// - `index`: The index of the setting
    /// - `buf`: The buffer to write the setting to
    ///
    /// # Returns
    /// - `Ok(Some(len))`: The setting was found and written to the buffer
    /// - `Ok(None)`: The setting was not found
    /// - `Err(_)`: An error occurred
    fn get(
        &mut self,
        key: u16,
        index: usize,
        buf: &mut [u8],
    ) -> Result<Option<usize>, SettingsError>;

    /// Add a setting
    ///
    /// Settings with the same key are not overwritten but added as a new setting with the same key and a new index.
    ///
    /// # Arguments
    /// - `key`: The key of the setting
    /// - `value`: The value of the setting
    ///
    /// # Returns
    /// - `Ok(())`: The setting was added
    /// - `Err(_)`: An error occurred, like out of storage space
    fn add(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError>;

    /// Remove a setting
    ///
    /// # Arguments
    /// - `key`: The key of the setting
    /// - `index`: The index of the setting to remove, or `None` to remove all settings with the same key
    ///
    /// # Returns
    /// - `Ok(true)`: The setting was removed
    /// - `Ok(false)`: The setting was not found
    /// - `Err(_)`: An error occurred
    fn remove(&mut self, key: u16, index: Option<usize>) -> Result<bool, SettingsError>;

    /// Update a setting
    ///
    /// This method will remove all settings with the same key prior to adding the new setting.
    ///
    /// # Arguments
    /// - `key`: The key of the setting
    /// - `value`: The value of the setting
    ///
    /// # Returns
    /// - `Ok(())`: The setting was updated
    /// - `Err(_)`: An error occurred, like out of storage space
    fn set(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError>;

    /// Clear all settings
    fn clear(&mut self) -> Result<(), SettingsError>;

    /// Deinitialize the settings
    fn deinit(&mut self);
}

impl<T> Settings for &mut T
where
    T: Settings,
{
    fn init(&mut self, sensitive_keys: &[u16]) {
        (**self).init(sensitive_keys)
    }

    fn get(
        &mut self,
        key: u16,
        index: usize,
        buf: &mut [u8],
    ) -> Result<Option<usize>, SettingsError> {
        (**self).get(key, index, buf)
    }

    fn add(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        (**self).add(key, value)
    }

    fn remove(&mut self, key: u16, index: Option<usize>) -> Result<bool, SettingsError> {
        (**self).remove(key, index)
    }

    fn set(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        (**self).set(key, value)
    }

    fn clear(&mut self) -> Result<(), SettingsError> {
        (**self).clear()
    }

    fn deinit(&mut self) {
        (**self).deinit()
    }
}

impl Settings for () {
    fn init(&mut self, _: &[u16]) {}

    fn get(&mut self, _: u16, _: usize, _: &mut [u8]) -> Result<Option<usize>, SettingsError> {
        Err(SettingsError::NotImplemented)
    }

    fn add(&mut self, _: u16, _: &[u8]) -> Result<(), SettingsError> {
        Err(SettingsError::NotImplemented)
    }

    fn remove(&mut self, _: u16, _: Option<usize>) -> Result<bool, SettingsError> {
        Err(SettingsError::NotImplemented)
    }

    fn set(&mut self, _: u16, _: &[u8]) -> Result<(), SettingsError> {
        Err(SettingsError::NotImplemented)
    }

    fn clear(&mut self) -> Result<(), SettingsError> {
        Err(SettingsError::NotImplemented)
    }

    fn deinit(&mut self) {}
}

/// A type alias of `RamSettings` for the case where the signal changes' closure is a `fn`
pub type SimpleRamSettings<'a> = RamSettings<'a, fn(RamSettingsChange) -> bool>;

impl<'a> SimpleRamSettings<'a> {
    /// Create a new `RamSettings` instance which never signals changes.
    ///
    /// # Arguments
    /// - `buffer`: The RAM buffer where settings are cached
    pub const fn new(buffer: &'a mut [u8]) -> Self {
        Self::new_with_signal_change(buffer, |_| false)
    }
}

/// A type implementing the `Settings` trait that stores settings in RAM
///
/// Useful in cases where storing the settings in non-volatile storage is not important
/// (e.g. examples), or when the non-volatile storage is behind an async API.
///
/// Since the `Settings` trait is synchronous, the `RamSettings` struct provides a way to
/// service settings requests coming from `OpenThread` immediately by using RAM storage.
/// However, the `RamSettings` struct can signal when settings have changed, so that
/// (a subset of) these can be persisted to non-volatile storage.
pub struct RamSettings<'a, T> {
    /// The RAM buffer where settings are cached
    buffer: &'a mut [u8],
    /// The length of the settings in the buffer
    len: usize,
    /// A closure to evaluate if to signal depending on the change type
    signal_change: T,
    /// A signal to notify the user when the settings have changed
    changed_signal: Signal<()>,
}

impl<'a, T> RamSettings<'a, T>
where
    T: FnMut(RamSettingsChange) -> bool,
{
    /// Create a new `RamSettings` instance
    ///
    /// # Arguments
    /// - `buffer`: The RAM buffer where settings are cached
    /// - `signal_change`: A closure that would be called-back if the settings change.
    ///   Returning `true` from this closure will signal the `Signal` instance.
    pub const fn new_with_signal_change(buffer: &'a mut [u8], signal_change: T) -> Self {
        Self {
            buffer,
            len: 0,
            signal_change,
            changed_signal: Signal::new(),
        }
    }

    /// Get a reference to the settings signal that indicates changes.
    /// The signal will be signaled if the user-provided closure returns `true`
    pub fn changed_signal(&mut self) -> &mut Signal<()> {
        &mut self.changed_signal
    }

    /// Return an iterator over the settings
    pub fn iter(&self) -> RamSettingsIter<'_> {
        RamSettingsIter {
            buffer: &self.buffer[..self.len],
        }
    }

    /// Get a setting
    ///
    /// # Arguments
    /// - `key`: The key of the setting
    /// - `index`: The index of the setting
    /// - `buf`: The buffer to write the setting to
    ///
    /// # Returns
    /// - `Ok(Some(len))`: The setting was found and written to the buffer
    /// - `Ok(None)`: The setting was not found
    /// - `Err(_)`: An error occurred
    pub fn get(
        &self,
        key: u16,
        index: usize,
        buf: &mut [u8],
    ) -> Result<Option<usize>, SettingsError> {
        trace!("Getting key: {}, index: {}", key, index);

        let for_key = self.iter().filter(|setting| setting.0 == key);
        let setting = for_key
            .enumerate()
            .find(|(i, _)| *i == index)
            .map(|(_, setting)| setting);

        if let Some(setting) = setting {
            let len = setting.1.len().min(buf.len());
            buf[..len].copy_from_slice(setting.1);

            trace!(
                "Got key: {}, index: {}, value: {}",
                key,
                index,
                Bytes(&buf[..len])
            );
            Ok(Some(len))
        } else {
            trace!("Key not found: {}, index: {}", key, index);
            Ok(None)
        }
    }

    /// Add a setting
    ///
    /// Settings with the same key are not overwritten but added as a new setting with the same key and a new index.
    ///
    /// On successful add, the new setting will be marked as changed, and the user-provided signal will be called.
    ///
    /// # Arguments
    /// - `key`: The key of the setting
    /// - `value`: The value of the setting
    ///
    /// # Returns
    /// - `Ok(())`: The setting was added
    /// - `Err(_)`: An error occurred, like out of storage space
    pub fn add(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        trace!("Adding key: {}, value: {}", key, Bytes(value));

        let len = RamSetting::HDR_LEN + value.len();
        if self.buffer.len() - self.len < len {
            warn!("Adding key: {} failed, no space", key);
            Err(SettingsError::NoBufs)?;
        }

        let setting = RamSetting {
            key,
            value_len: value.len() as _,
            value,
        };

        self.buffer[self.len + RamSetting::HDR_LEN..self.len + len].copy_from_slice(value);
        self.buffer[self.len..self.len + RamSetting::HDR_LEN].copy_from_slice(&setting.as_bytes());
        self.len += len;

        let index = self.iter().filter(|setting| setting.0 == key).count() - 1;

        if (self.signal_change)(RamSettingsChange::Added { key, index }) {
            self.changed_signal.signal(());
        }

        trace!("Added key: {}", key);
        Ok(())
    }

    /// Remove a setting
    ///
    /// On successful remove, the overall settings will be marked as changed, and the user-provided signal will be called.
    ///
    /// # Arguments
    /// - `key`: The key of the setting
    /// - `index`: The index of the setting to remove, or `None` to remove all settings with the same key
    ///
    /// # Returns
    /// - `Ok(true)`: The setting was removed
    /// - `Ok(false)`: The setting was not found
    /// - `Err(_)`: An error occurred
    pub fn remove(&mut self, key: u16, index: Option<usize>) -> Result<bool, SettingsError> {
        trace!("Removing key: {}, index: {:?}", key, index);

        let mut found = false;
        let mut buf = &mut self.buffer[..self.len];

        let mut current = 0;

        while !buf.is_empty() {
            let setting = RamSetting::from_bytes(buf);
            let len = RamSetting::HDR_LEN + setting.value_len as usize;

            if setting.key == key {
                if index.map(|index| index == current).unwrap_or(true) {
                    buf.copy_within(len.., 0);
                    self.len -= len;

                    // Shrink the buffer pointer
                    let new_buf_len = buf.len() - len;
                    buf = &mut buf[..new_buf_len];

                    if (self.signal_change)(RamSettingsChange::Removed {
                        key,
                        index: current,
                    }) {
                        self.changed_signal.signal(());
                    }

                    trace!("Removed key: {}, index: {:?}", key, index);
                    found = true;

                    current += 1;
                    continue;
                }

                current += 1;
            }

            buf = &mut buf[len..];
        }

        if !found {
            trace!("Key not found: {}, index: {:?}", key, index);
        }

        Ok(found)
    }

    /// Update a setting
    ///
    /// This method will remove all settings with the same key prior to adding the new setting.
    ///
    /// On successful update, the setting will be marked as changed, and the user-provided signal will be called.
    ///
    /// # Arguments
    /// - `key`: The key of the setting
    /// - `value`: The value of the setting
    ///
    /// # Returns
    /// - `Ok(())`: The setting was updated
    /// - `Err(_)`: An error occurred, like out of storage space
    pub fn set(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        self.remove(key, None)?;
        self.add(key, value)
    }

    /// Clear all settings
    pub fn clear(&mut self) {
        debug!("Clearing settings");

        self.len = 0;

        if (self.signal_change)(RamSettingsChange::Clear) {
            self.changed_signal.signal(());
        }
    }
}

impl<T> Settings for RamSettings<'_, T>
where
    T: FnMut(RamSettingsChange) -> bool,
{
    fn init(&mut self, _: &[u16]) {}

    fn get(
        &mut self,
        key: u16,
        index: usize,
        buf: &mut [u8],
    ) -> Result<Option<usize>, SettingsError> {
        RamSettings::get(self, key, index, buf)
    }

    fn add(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        RamSettings::add(self, key, value)
    }

    fn remove(&mut self, key: u16, index: Option<usize>) -> Result<bool, SettingsError> {
        RamSettings::remove(self, key, index)
    }

    fn set(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        RamSettings::set(self, key, value)
    }

    fn clear(&mut self) -> Result<(), SettingsError> {
        RamSettings::clear(self);
        Ok(())
    }

    fn deinit(&mut self) {}
}

/// An iterator over the settings in a `RamSettings` instance
pub struct RamSettingsIter<'a> {
    buffer: &'a [u8],
}

impl<'a> Iterator for RamSettingsIter<'a> {
    type Item = (u16, &'a [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        if self.buffer.is_empty() {
            return None;
        }

        let setting = RamSetting::from_bytes(self.buffer);
        let len = RamSetting::HDR_LEN + setting.value_len as usize;

        self.buffer = &self.buffer[len..];

        Some((setting.key, setting.value))
    }
}

/// A notification for a change in the settings
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum RamSettingsChange {
    /// All settings were removed
    Clear,
    /// A setting was added
    Added { key: u16, index: usize },
    /// A setting was removed
    Removed { key: u16, index: usize },
}

/// A setting in RAM
#[derive(Debug)]
struct RamSetting<'a> {
    /// The key of the setting
    key: u16,
    /// The length of the value
    value_len: u16,
    /// The value
    value: &'a [u8],
}

impl<'a> RamSetting<'a> {
    /// The length of the header of the setting when stored in the RAM buffer:
    /// - 2 bytes for the key (u16 LE)
    /// - 2 bytes for the value length (u16 LE)
    const HDR_LEN: usize = 4;

    /// Create a new `RamSetting` instance from a slice of bytes
    fn from_bytes(data: &'a [u8]) -> Self {
        let key = u16::from_le_bytes([data[0], data[1]]);
        let value_len = u16::from_le_bytes([data[2], data[3]]);

        Self {
            key,
            value_len,
            value: &data[Self::HDR_LEN..Self::HDR_LEN + value_len as usize],
        }
    }

    /// Return the setting as a slice of bytes
    fn as_bytes(&self) -> [u8; 4] {
        let mut bytes = [0; Self::HDR_LEN];
        bytes[..2].copy_from_slice(&self.key.to_le_bytes());
        bytes[2..].copy_from_slice(&self.value_len.to_le_bytes());

        bytes
    }
}

/// A shared `RamSettings` instance with interior mutability
/// that can be shared between `openthread` and user code
pub struct SharedRamSettings<'a, M, T>(Mutex<M, RefCell<RamSettings<'a, T>>>);

impl<'a, M, T> SharedRamSettings<'a, M, T>
where
    M: RawMutex,
    T: FnMut(RamSettingsChange) -> bool,
{
    /// Create a new `SharedRamSettings` instance
    ///
    /// # Arguments
    /// - `settings`: The `RamSettings` instance to share
    pub const fn new(settings: RamSettings<'a, T>) -> Self {
        Self(Mutex::new(RefCell::new(settings)))
    }

    /// Get access to the `RamSettings` instance
    pub fn with<F, R>(&self, mut f: F) -> R
    where
        F: FnMut(&mut RamSettings<'a, T>) -> R,
    {
        self.0.lock(|settings| f(&mut *settings.borrow_mut()))
    }

    /// Poll for changes in the settings
    pub fn poll_changed(&self, cx: &mut Context<'_>) -> Poll<()> {
        self.with(|settings| settings.changed_signal().poll_wait(cx))
    }
}

impl<M, T> Settings for &SharedRamSettings<'_, M, T>
where
    M: RawMutex,
    T: FnMut(RamSettingsChange) -> bool,
{
    fn init(&mut self, sensitive_keys: &[u16]) {
        self.0
            .lock(|settings| settings.borrow_mut().init(sensitive_keys))
    }

    fn get(
        &mut self,
        key: u16,
        index: usize,
        buf: &mut [u8],
    ) -> Result<Option<usize>, SettingsError> {
        self.0
            .lock(|settings| settings.borrow_mut().get(key, index, buf))
    }

    fn add(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        self.0
            .lock(|settings| settings.borrow_mut().add(key, value))
    }

    fn remove(&mut self, key: u16, index: Option<usize>) -> Result<bool, SettingsError> {
        self.0
            .lock(|settings| settings.borrow_mut().remove(key, index))
    }

    fn set(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        self.0
            .lock(|settings| settings.borrow_mut().set(key, value))
    }

    fn clear(&mut self) -> Result<(), SettingsError> {
        self.0.lock(|settings| settings.borrow_mut().clear());

        Ok(())
    }

    fn deinit(&mut self) {
        self.0.lock(|settings| settings.borrow_mut().deinit());
    }
}
