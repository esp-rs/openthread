//! File-backed OpenThread settings for the simulation DUT.
//!
//! The upstream simulation binaries persist settings in per-node flash files
//! under the working directory, which is what gives the CLI `reset` command
//! its real semantics: the process restarts, the dataset survives. This is
//! the same contract for the Rust DUT: every mutation is flushed to a small
//! per-node file, a fresh process loads it back, and `factoryreset` (or an
//! `otPlatSettingsWipe` from the stack) deletes it.
//!
//! The store itself is a plain in-memory record list - at host scale there is
//! no reason for anything cleverer - serialized as repeated
//! `[key: u16 LE][len: u16 LE][value]` records. A missing or truncated file
//! deserializes to "no settings" (the file is written atomically via a
//! rename, so truncation would take a crash mid-rename).

use std::fs;
use std::path::{Path, PathBuf};

use log::{trace, warn};

use openthread::{Settings, SettingsError};

/// A [`Settings`] implementation persisting to a single file.
pub struct FileSettings {
    path: PathBuf,
    records: Vec<(u16, Vec<u8>)>,
}

impl FileSettings {
    /// Create the store, loading any settings a previous incarnation of this
    /// node persisted at `path`.
    pub fn new(path: PathBuf) -> Self {
        let records = Self::load(&path);

        trace!(
            "Settings: loaded {} record(s) from {}",
            records.len(),
            path.display()
        );

        Self { path, records }
    }

    fn load(path: &Path) -> Vec<(u16, Vec<u8>)> {
        let Ok(data) = fs::read(path) else {
            return Vec::new();
        };

        let mut records = Vec::new();
        let mut offs = 0;

        while data.len() >= offs + 4 {
            let key = u16::from_le_bytes([data[offs], data[offs + 1]]);
            let len = u16::from_le_bytes([data[offs + 2], data[offs + 3]]) as usize;
            offs += 4;

            if data.len() < offs + len {
                warn!("Settings: truncated record in {}", path.display());
                break;
            }

            records.push((key, data[offs..offs + len].to_vec()));
            offs += len;
        }

        records
    }

    /// Flush the whole store to its file, atomically (write-then-rename).
    fn persist(&self) {
        let mut data = Vec::new();

        for (key, value) in &self.records {
            data.extend_from_slice(&key.to_le_bytes());
            data.extend_from_slice(&(value.len() as u16).to_le_bytes());
            data.extend_from_slice(value);
        }

        let tmp = self.path.with_extension("tmp");

        if let Err(err) = fs::write(&tmp, &data).and_then(|_| fs::rename(&tmp, &self.path)) {
            warn!(
                "Settings: persisting to {} failed: {err}",
                self.path.display()
            );
        }
    }
}

impl Settings for FileSettings {
    fn init(&mut self, _sensitive_keys: &[u16]) {}

    fn get(
        &mut self,
        key: u16,
        index: usize,
        buf: &mut [u8],
    ) -> Result<Option<usize>, SettingsError> {
        let setting = self
            .records
            .iter()
            .filter(|(k, _)| *k == key)
            .nth(index)
            .map(|(_, v)| v);

        if let Some(value) = setting {
            let len = value.len().min(buf.len());
            buf[..len].copy_from_slice(&value[..len]);

            Ok(Some(len))
        } else {
            Ok(None)
        }
    }

    fn add(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        self.records.push((key, value.to_vec()));
        self.persist();

        Ok(())
    }

    fn remove(&mut self, key: u16, index: Option<usize>) -> Result<bool, SettingsError> {
        let found = match index {
            Some(index) => {
                let pos = self
                    .records
                    .iter()
                    .enumerate()
                    .filter(|(_, (k, _))| *k == key)
                    .map(|(pos, _)| pos)
                    .nth(index);

                if let Some(pos) = pos {
                    self.records.remove(pos);
                    true
                } else {
                    false
                }
            }
            None => {
                let before = self.records.len();
                self.records.retain(|(k, _)| *k != key);
                self.records.len() != before
            }
        };

        if found {
            self.persist();
        }

        Ok(found)
    }

    fn set(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        self.records.retain(|(k, _)| *k != key);
        self.records.push((key, value.to_vec()));
        self.persist();

        Ok(())
    }

    fn clear(&mut self) -> Result<(), SettingsError> {
        self.records.clear();

        // A wipe means "forget this node ever existed" - remove the file
        // rather than persisting an empty one.
        if let Err(err) = fs::remove_file(&self.path) {
            if err.kind() != std::io::ErrorKind::NotFound {
                warn!("Settings: removing {} failed: {err}", self.path.display());
            }
        }

        Ok(())
    }

    fn deinit(&mut self) {}
}
