//! Flash persistence for the OpenThread settings, over the MPSL-coordinated
//! flash driver.
//!
//! With MPSL scheduling the radio, flash operations must go through
//! [`nrf_mpsl::Flash`], whose erase/write are `async` (they run in radio-free
//! timeslots). The `Settings` trait is synchronous, so persistence becomes **write-behind**: every mutation
//! lands in the RAM working copy and bumps a generation counter, and a
//! dedicated task serializes and writes the image out. [`FlashSettings::flush`]
//! awaits "everything mutated so far is on flash" - the reset paths call it
//! before rebooting the chip, which is where durability actually matters.
//!
//! # Image layout
//!
//! At `offset` (the page `memory.x` carves off the top of the application
//! flash):
//!
//! ```text
//! [ magic u32 | payload-len u32 | FNV-1a u32 | payload ]
//! payload: [ key u16 | value-len u16 | value bytes ]*
//! ```
//!
//! All integers little-endian. Anything that does not parse - erased flash,
//! a bad checksum from a torn write - reads as an empty store.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

use embedded_storage_async::nor_flash::NorFlash;

use nrf_mpsl::Flash;

use openthread::{Settings, SettingsError, SimpleRamSettings};

/// "OTS1"
const MAGIC: u32 = 0x4f54_5331;

/// Magic + payload length + checksum.
const HDR_LEN: usize = 12;

/// The largest payload the image may carry; must not exceed the RAM buffer.
const PAYLOAD_MAX: usize = 1024;

/// One flash page, which is what `memory.x` reserves for the image.
const PAGE_SIZE: usize = 4096;

/// The granularity a write must come in, which is not the same everywhere:
/// one 32-bit word on nRF52's NVMC, but a 128-bit line on nRF54L's RRAM. It
/// matters because `nrf_mpsl::Flash` rejects a misaligned length outright
/// rather than padding it for us - so hard-coding the nRF52 figure left the
/// nRF54L node silently RAM-only, its settings gone on every reset.
const WRITE_SIZE: usize = <Flash<'static> as NorFlash>::WRITE_SIZE;

/// The serialization buffer, rounded up to whole write units so that even a
/// maximum-size image has room for its padding.
const IMG_LEN: usize = (HDR_LEN + PAYLOAD_MAX).next_multiple_of(WRITE_SIZE);

/// The shared state between the [`FlashSettings`] handle (driven
/// synchronously by OpenThread) and [`run`] (the persist task).
pub struct FlashSettingsState {
    /// The RAM working copy.
    ram: Mutex<CriticalSectionRawMutex, RefCell<Option<SimpleRamSettings<'static>>>>,
    /// Bumped on every mutation; what the persist task owes to flash.
    dirty_gen: AtomicU32,
    /// The last generation fully written to flash.
    persisted_gen: AtomicU32,
    /// Wakes the persist task.
    dirty: Signal<CriticalSectionRawMutex, ()>,
    /// Wakes `flush` waiters.
    persisted: Signal<CriticalSectionRawMutex, ()>,
    /// Set while a `flush` is waiting: persist now, skip the settle delay.
    flush_pending: AtomicBool,
    /// Wakes the persist task out of its settle delay.
    flush_now: Signal<CriticalSectionRawMutex, ()>,
}

impl FlashSettingsState {
    /// Create the state; `run` and `FlashSettings::new` bring it to life.
    pub const fn new() -> Self {
        Self {
            ram: Mutex::new(RefCell::new(None)),
            dirty_gen: AtomicU32::new(0),
            persisted_gen: AtomicU32::new(0),
            dirty: Signal::new(),
            persisted: Signal::new(),
            flush_pending: AtomicBool::new(false),
            flush_now: Signal::new(),
        }
    }

    fn with_ram<R>(&self, f: impl FnOnce(&mut SimpleRamSettings<'static>) -> R) -> R {
        self.ram.lock(|ram| {
            let mut ram = ram.borrow_mut();
            f(ram.as_mut().unwrap())
        })
    }

    fn mark_dirty(&self) {
        self.dirty_gen.fetch_add(1, Ordering::Relaxed);
        self.dirty.signal(());
    }
}

impl Default for FlashSettingsState {
    fn default() -> Self {
        Self::new()
    }
}

/// The `Settings` implementation of this firmware: a RAM working copy,
/// persisted write-behind by the [`run`] task.
///
/// `factoryreset` needs no special handling here: OpenThread's factory-reset
/// path clears the settings via the `Settings` trait, and the reset paths
/// `flush` before rebooting, which makes the wipe durable.
pub struct FlashSettings {
    state: &'static FlashSettingsState,
}

impl FlashSettings {
    /// Create the settings over `ram_buf` as the working copy, seeding it
    /// from the image at flash `offset` (read synchronously - reads need no
    /// MPSL timeslot).
    pub fn new(
        state: &'static FlashSettingsState,
        flash: &mut Flash<'_>,
        offset: u32,
        ram_buf: &'static mut [u8],
    ) -> Self {
        assert!(ram_buf.len() <= PAYLOAD_MAX);
        assert!((offset as usize).is_multiple_of(PAGE_SIZE));

        let mut ram = SimpleRamSettings::new(ram_buf);
        load(&mut ram, flash, offset);

        state.ram.lock(|cell| *cell.borrow_mut() = Some(ram));

        Self { state }
    }
}

/// Await everything mutated so far being on flash.
///
/// The reset paths call this before rebooting; in steady state the persist
/// task keeps up on its own. A free function over the shared state, because
/// the mutable [`FlashSettings`] handle itself is owned by the OpenThread
/// instance.
pub async fn flush(state: &FlashSettingsState) {
    state.flush_pending.store(true, Ordering::Relaxed);
    state.flush_now.signal(());

    loop {
        let dirty = state.dirty_gen.load(Ordering::Relaxed);
        if state.persisted_gen.load(Ordering::Relaxed) >= dirty {
            break;
        }

        state.persisted.wait().await;
    }

    state.flush_pending.store(false, Ordering::Relaxed);
}

/// How long the persist task lets a mutation settle before writing it out.
///
/// Persisting costs the radio: the page erase (~85 ms on nRF52) runs in MPSL
/// timeslots the 802.15.4 driver has to vacate, and a peer gives up on a
/// frame after four attempts spanning ~30 ms. Mutations arrive in bursts
/// (an attach, a key change) and the traffic they trigger follows at once -
/// a key change is answered with a ping in the certification scripts - so
/// the write waits for that to pass. `flush` skips the wait.
const PERSIST_SETTLE: Duration = Duration::from_millis(500);

impl Settings for FlashSettings {
    fn init(&mut self, sensitive_keys: &[u16]) {
        // The store itself was seeded in `new`; this only records the
        // sensitive-key set (`SimpleRamSettings::init` does not clear).
        self.state
            .with_ram(|ram| Settings::init(ram, sensitive_keys));
    }

    fn get(
        &mut self,
        key: u16,
        index: usize,
        buf: &mut [u8],
    ) -> Result<Option<usize>, SettingsError> {
        self.state
            .with_ram(|ram| Settings::get(ram, key, index, buf))
    }

    fn add(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        defmt::debug!("Settings: add key {} len {}", key, value.len());
        self.state.with_ram(|ram| Settings::add(ram, key, value))?;
        self.state.mark_dirty();
        Ok(())
    }

    fn remove(&mut self, key: u16, index: Option<usize>) -> Result<bool, SettingsError> {
        let removed = self
            .state
            .with_ram(|ram| Settings::remove(ram, key, index))?;
        if removed {
            self.state.mark_dirty();
        }
        Ok(removed)
    }

    fn set(&mut self, key: u16, value: &[u8]) -> Result<(), SettingsError> {
        defmt::debug!("Settings: set key {} len {}", key, value.len());
        self.state.with_ram(|ram| Settings::set(ram, key, value))?;
        self.state.mark_dirty();
        Ok(())
    }

    fn clear(&mut self) -> Result<(), SettingsError> {
        defmt::debug!("Settings: clear");
        self.state.with_ram(Settings::clear)?;
        self.state.mark_dirty();
        Ok(())
    }

    fn deinit(&mut self) {
        self.state.with_ram(Settings::deinit);
    }
}

/// The persist task: serialize the RAM working copy and write it out
/// whenever it changes. Run this on the main executor for the life of the
/// node.
pub async fn run(state: &'static FlashSettingsState, mut flash: Flash<'static>, offset: u32) -> ! {
    let mut img = [0; IMG_LEN];

    loop {
        state.dirty.wait().await;

        if !state.flush_pending.load(Ordering::Relaxed) {
            let _ = select(Timer::after(PERSIST_SETTLE), state.flush_now.wait()).await;
        }

        // The generation this pass covers; anything newer re-raises `dirty`.
        let dirty_gen = state.dirty_gen.load(Ordering::Relaxed);

        let len = state.with_ram(|ram| serialize(ram, &mut img));
        defmt::debug!("Settings: persisting gen {} image len {}", dirty_gen, len);

        // Writes are granular (see `WRITE_SIZE`); the pad bytes never parse,
        // as the length field bounds them out.
        let len = len.next_multiple_of(WRITE_SIZE);

        // A failed write leaves the settings RAM-only: the node keeps
        // working, and the loss surfaces (loudly) only if it resets.
        let done = flash.erase(offset, offset + PAGE_SIZE as u32).await.is_ok()
            && flash.write(offset, &img[..len]).await.is_ok();

        if !done {
            defmt::warn!("Settings: flash write failed; settings not persisted");
        }

        defmt::debug!("Settings: persisted gen {} ok {}", dirty_gen, done);
        state.persisted_gen.store(dirty_gen, Ordering::Relaxed);
        state.persisted.signal(());
    }
}

/// Seed `ram` from the image at `offset`, if a valid one is present.
fn load(ram: &mut SimpleRamSettings<'static>, flash: &mut Flash<'_>, offset: u32) {
    let mut hdr = [0; HDR_LEN];
    if flash.read(offset, &mut hdr).is_err() {
        defmt::warn!("Settings: flash read failed; starting empty");
        return;
    }

    let magic = u32::from_le_bytes(hdr[0..4].try_into().unwrap());
    let len = u32::from_le_bytes(hdr[4..8].try_into().unwrap()) as usize;
    let crc = u32::from_le_bytes(hdr[8..12].try_into().unwrap());

    if magic != MAGIC || len > PAYLOAD_MAX {
        // Erased flash or a foreign image: factory-fresh.
        return;
    }

    let mut payload = [0; PAYLOAD_MAX];
    let payload = &mut payload[..len];
    if flash.read(offset + HDR_LEN as u32, payload).is_err() || checksum(payload) != crc {
        defmt::warn!("Settings: flash image corrupt; starting empty");
        return;
    }

    let mut pos = 0;
    while pos + 4 <= len {
        let key = u16::from_le_bytes(payload[pos..pos + 2].try_into().unwrap());
        let vlen = u16::from_le_bytes(payload[pos + 2..pos + 4].try_into().unwrap()) as usize;
        pos += 4;

        if pos + vlen > len {
            defmt::warn!("Settings: flash image truncated; loaded what parses");
            break;
        }

        let _ = Settings::add(ram, key, &payload[pos..pos + vlen]);
        pos += vlen;
    }
}

/// Serialize `ram` into `img`, returning the total image length.
fn serialize(ram: &SimpleRamSettings<'static>, img: &mut [u8; IMG_LEN]) -> usize {
    let mut pos = HDR_LEN;
    for (key, value) in ram.iter() {
        img[pos..pos + 2].copy_from_slice(&key.to_le_bytes());
        img[pos + 2..pos + 4].copy_from_slice(&(value.len() as u16).to_le_bytes());
        img[pos + 4..pos + 4 + value.len()].copy_from_slice(value);
        pos += 4 + value.len();
    }

    let crc = checksum(&img[HDR_LEN..pos]);
    img[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    img[4..8].copy_from_slice(&((pos - HDR_LEN) as u32).to_le_bytes());
    img[8..12].copy_from_slice(&crc.to_le_bytes());

    pos
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
