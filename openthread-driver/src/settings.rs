//! The storage of OpenThread's settings: the [`Settings`] trait.

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

/// A trait for abstracting the notion of OpenThread Platform settings
///
/// Each platform should provide an implementation of the `Settings` trait, where
/// ideally - at least a portion of the settings (like the SRP key) - are saved to non-volatile storage.
///
/// User might just utilize the `openthread::RamSettings` struct, especially in cases where the non-volatile storage
/// is behind an async API. See `openthread::RamSettings` for more information.
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
