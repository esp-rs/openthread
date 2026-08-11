//! Flash persistence for the OpenThread settings.
//!
//! The `Settings` trait is synchronous and so is `esp-storage`'s flash
//! access, which makes write-through the natural shape: every mutation lands
//! in a RAM working copy first (`SimpleRamSettings`), and the whole store is
//! then serialized back to flash in one image. Reads never touch the flash.
//!
//! Whole-image rather than a log, because the store is small (it caps at the
//! RAM buffer size, well under one sector), `esp-storage` does the
//! erase/read-modify-write dance internally, and OpenThread's write rate is
//! tame - network state changes on attach and dataset events, and the frame
//! counters are persisted in increments of ~1000 frames, not per frame. Wear
//! and write latency are both non-issues at this scale.
//!
//! A flash operation stalls execute-in-place (and with it the radio driver's
//! interrupt handling) for its duration. The MAC's hardware ACK path is
//! unaffected, and a delayed RX/TX event at worst costs a retransmit - a
//! trade the test tier can afford for durable `reset` semantics.
//!
//! # Image layout
//!
//! At `offset` (the start of the board's NVS partition):
//!
//! ```text
//! [ magic u32 | payload-len u32 | FNV-1a u32 | payload ]
//! payload: [ key u16 | value-len u16 | value bytes ]*
//! ```
//!
//! All integers little-endian. Anything that does not parse - erased flash,
//! a bad checksum from a torn write - reads as an empty store: for a test
//! node, losing settings is a visible failure, while trusting corrupt ones
//! is a debugging session.

use embedded_storage::{ReadStorage, Storage};

use esp_storage::FlashStorage;

use log::warn;

use openthread::{Settings, SettingsError, SimpleRamSettings};

/// "OTS1"
const MAGIC: u32 = 0x4f54_5331;

/// Magic + payload length + checksum.
const HDR_LEN: usize = 12;

/// The largest payload the image may carry.
///
/// A serialized record is 4 bytes of header plus the value - the same shape
/// (and thus the same size) as `RamSettings`' in-memory records, so a RAM
/// buffer no larger than this cap can never produce an oversized image.
const PAYLOAD_MAX: usize = 1024;

/// The `Settings` implementation of this firmware: a RAM working copy,
/// written through to a flash image on every mutation.
///
/// `factoryreset` needs no special handling here: OpenThread's factory-reset
/// path clears the settings via the `Settings` trait, and the write-through
/// makes that durable before the chip reboots.
pub struct FlashSettings {
    ram: SimpleRamSettings<'static>,
    flash: FlashStorage<'static>,
    /// Flash byte offset of the image.
    offset: u32,
    /// Assembly buffer for the serialized image.
    img: [u8; HDR_LEN + PAYLOAD_MAX],
}

impl FlashSettings {
    /// Create the settings over `ram_buf` as the working copy, persisting at
    /// `offset` (sector-aligned, e.g. the start of the NVS partition).
    pub fn new(flash: FlashStorage<'static>, offset: u32, ram_buf: &'static mut [u8]) -> Self {
        assert!(ram_buf.len() <= PAYLOAD_MAX);

        Self {
            ram: SimpleRamSettings::new(ram_buf),
            flash,
            offset,
            img: [0; HDR_LEN + PAYLOAD_MAX],
        }
    }

    /// Seed the RAM working copy from the flash image, if a valid one is
    /// present.
    fn load(&mut self) {
        let mut hdr = [0; HDR_LEN];
        if self.flash.read(self.offset, &mut hdr).is_err() {
            warn!("Settings: flash read failed; starting empty");
            return;
        }

        let magic = u32::from_le_bytes(hdr[0..4].try_into().unwrap());
        let len = u32::from_le_bytes(hdr[4..8].try_into().unwrap()) as usize;
        let crc = u32::from_le_bytes(hdr[8..12].try_into().unwrap());

        if magic != MAGIC || len > PAYLOAD_MAX {
            // Erased flash or a foreign image: factory-fresh.
            return;
        }

        let payload = &mut self.img[..len];
        if self
            .flash
            .read(self.offset + HDR_LEN as u32, payload)
            .is_err()
            || checksum(payload) != crc
        {
            warn!("Settings: flash image corrupt; starting empty");
            return;
        }

        let mut pos = 0;
        while pos + 4 <= len {
            let key = u16::from_le_bytes(self.img[pos..pos + 2].try_into().unwrap());
            let vlen = u16::from_le_bytes(self.img[pos + 2..pos + 4].try_into().unwrap()) as usize;
            pos += 4;

            if pos + vlen > len {
                warn!("Settings: flash image truncated; loaded what parses");
                break;
            }

            let _ = Settings::add(&mut self.ram, key, &self.img[pos..pos + vlen]);
            pos += vlen;
        }
    }

    /// Serialize the RAM working copy and write the image back.
    fn persist(&mut self) {
        let mut pos = HDR_LEN;
        for (key, value) in self.ram.iter() {
            self.img[pos..pos + 2].copy_from_slice(&key.to_le_bytes());
            self.img[pos + 2..pos + 4].copy_from_slice(&(value.len() as u16).to_le_bytes());
            self.img[pos + 4..pos + 4 + value.len()].copy_from_slice(value);
            pos += 4 + value.len();
        }

        let crc = checksum(&self.img[HDR_LEN..pos]);
        self.img[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        self.img[4..8].copy_from_slice(&((pos - HDR_LEN) as u32).to_le_bytes());
        self.img[8..12].copy_from_slice(&crc.to_le_bytes());

        // A failed write leaves the settings RAM-only: the node keeps
        // working, and the loss surfaces (loudly) only if it resets.
        if self.flash.write(self.offset, &self.img[..pos]).is_err() {
            warn!("Settings: flash write failed; settings not persisted");
        }
    }
}

impl Settings for FlashSettings {
    fn init(&mut self, sensitive_keys: &[u16]) {
        Settings::init(&mut self.ram, sensitive_keys);
        self.load();
    }

    fn get(
        &mut self,
        key: u16,
        index: usize,
        buf: &mut [u8],
    ) -> Result<Option<usize>, SettingsError> {
        Settings::get(&mut self.ram, key, index, buf)
    }

    fn add(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        Settings::add(&mut self.ram, key, value)?;
        self.persist();
        Ok(())
    }

    fn remove(&mut self, key: u16, index: Option<usize>) -> Result<bool, SettingsError> {
        let removed = Settings::remove(&mut self.ram, key, index)?;
        if removed {
            self.persist();
        }
        Ok(removed)
    }

    fn set(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        Settings::set(&mut self.ram, key, value)?;
        self.persist();
        Ok(())
    }

    fn clear(&mut self) -> Result<(), SettingsError> {
        Settings::clear(&mut self.ram)?;
        self.persist();
        Ok(())
    }

    fn deinit(&mut self) {
        Settings::deinit(&mut self.ram);
    }
}

/// FNV-1a, 32-bit.
fn checksum(data: &[u8]) -> u32 {
    let mut hash = 0x811c_9dc5u32;

    for byte in data {
        hash ^= *byte as u32;
        hash = hash.wrapping_mul(0x0100_0193);
    }

    hash
}
