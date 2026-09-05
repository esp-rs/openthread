//! The ESP32-XX firmware node of the HIL tier:
//! Same purpose as the host / STD `cli_node`, but running on the MCU with its own radio.
//!
//! The upstream harness drives this exactly as it drives every other node -
//! CLI lines in, CLI output back - except the pipe is the chip's
//! USB-Serial-JTAG console rather than a process's stdin/stdout.
//! `openthread-tests`' `serial_bridge` is what makes that substitution
//! invisible to the harness.
//!
//! # What only this tier exercises
//!
//! [`EspRadio`] driving the chip's IEEE 802.15.4 peripheral, on a real clock,
//! under the unmodified upstream scenarios.
//!
//! # Console and logs
//!
//! The CLI console is the USB-Serial-JTAG serial side (`/dev/ttyACM*` on the
//! host). Logs would land on the same channel the harness parses, so test
//! builds compile them out (`ESP_LOG=off` in `.cargo/config.toml`); panics
//! still print there via `esp-backtrace`, which is where a dying node's last
//! words belong - they surface in the failing test's log tail.
//!
//! # Reset
//!
//! The CLI `reset`/`factoryreset` commands are intercepted (the C stack
//! cannot re-create itself in place - see the crate's `otPlatReset`) and
//! honored with a chip `software_reset`. The settings persist in flash (see
//! `settings`), so a `reset` node comes back with its dataset and
//! network state intact and rejoins on its own. `factoryreset` is first
//! forwarded to the stack, whose factory-reset path clears the settings -
//! durably, thanks to the write-through - before the chip reboots.
//!
//! # Node identity
//!
//! Flashed firmware cannot be told which node it is - the harness passes that
//! on a command line only the bridge sees. So the EUI-64 derives from the
//! chip's factory-programmed base MAC (unique per chip, stable across
//! resets), and the node-id-to-board mapping lives entirely in the bridge's
//! port map.

#![no_std]
#![no_main]

use embassy_executor::Spawner;

use esp_hal::rng::{Trng, TrngSource};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;
use esp_radio::ieee802154::Ieee802154;
use {esp_backtrace as _, esp_println as _};

use embedded_io_async::{Read, Write};

/// The console's two ends: the USB-Serial-JTAG serial side.
type ConsoleRx = UsbSerialJtagRx<'static, Async>;
type ConsoleTx = UsbSerialJtagTx<'static, Async>;

use esp_bootloader_esp_idf::partitions::{
    read_partition_table, DataPartitionSubType, PartitionType, PARTITION_TABLE_MAX_LEN,
};
use esp_storage::FlashStorage;

use openthread::esp::EspRadio;
use openthread::{OpenThread, OtResources};

use tinyrlibc as _;

#[path = "../../../shared/console.rs"]
mod console;

#[path = "../settings.rs"]
mod settings;

use settings::FlashSettings;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // For tinyrlibc's malloc/calloc, which the C CLI library may reach for.
    esp_alloc::heap_allocator!(size: 4096);

    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT)
            .software_interrupt0,
    );

    // The chip's factory base MAC, expanded to an EUI-64 the standard way
    // (OUI half ++ FF:FE ++ device half): unique per chip, stable across
    // resets - the closest thing firmware has to the node id the host DUT
    // gets on its command line.
    let ieee_eui64 = ieee_eui64();

    // OpenThread requires a cryptographically secure RNG; the TRNG is only
    // truly random while its entropy source (RNG + SAR ADC) stays enabled
    let _trng_source = mk_static!(
        TrngSource<'static>,
        TrngSource::new(peripherals.RNG, peripherals.ADC1)
    );
    let rng = mk_static!(Trng, Trng::try_new().unwrap());

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);

    // The settings image lives at the start of the NVS partition - present in
    // the (default) partition table `espflash` writes. A board without one
    // panics here, onto the console where the failing test's log tail is.
    let mut flash = FlashStorage::new(peripherals.FLASH);
    let pt_buf = mk_static!([u8; PARTITION_TABLE_MAX_LEN], [0; PARTITION_TABLE_MAX_LEN]);
    let nvs_offset = read_partition_table(&mut flash, pt_buf)
        .unwrap()
        .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
        .unwrap()
        .unwrap()
        .offset();

    let ot_settings = mk_static!(
        FlashSettings,
        FlashSettings::new(flash, nvs_offset, ot_settings_buf)
    );

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    spawner.spawn(
        run_ot(
            ot.clone(),
            EspRadio::new(Ieee802154::new(peripherals.IEEE802154)),
        )
        .unwrap(),
    );

    let (console_rx, console_tx) = UsbSerialJtag::new(peripherals.USB_DEVICE)
        .into_async()
        .split();

    spawner.spawn(run_console_out(console_tx).unwrap());

    ot.cli_init(console::out);

    // The prompt the harness waits for on connect.
    console::out(b"\r\n> ");

    run_cli(ot, console_rx).await
}

/// Read CLI lines off the console and hand them to the interpreter.
///
/// `reset`/`factoryreset` are handled here with a chip reset - see the module
/// docs.
async fn run_cli(ot: OpenThread<'static>, mut console_rx: ConsoleRx) -> ! {
    let mut reader = console::LineReader::new();
    let mut buf = [0; 64];

    loop {
        // Irrefutable: the USB-Serial-JTAG read cannot fail.
        let Ok(len) = console_rx.read(&mut buf).await;

        for byte in &buf[..len] {
            if reader.push(*byte) {
                let line = reader.line().trim();

                match line {
                    // Drain before rebooting, or the reply races the reset:
                    // whether it reaches the wire comes down to timing, and
                    // the harness is left waiting on a response that was
                    // never sent - which it reports as a device that went
                    // quiet, a long way from the cause.
                    "reset" => {
                        console::drained().await;
                        esp_hal::system::software_reset()
                    }
                    "factoryreset" => {
                        let _ = ot.cli_input_line(line);
                        console::drained().await;
                        esp_hal::system::software_reset();
                    }
                    "" => (),
                    _ => {
                        let _ = ot.cli_input_line(line);
                    }
                }

                reader.clear();

                console::drained().await;
            }
        }
    }
}

/// The chip's factory base MAC as an EUI-64.
fn ieee_eui64() -> [u8; 8] {
    let mac_address = esp_hal::efuse::base_mac_address();
    let mac = mac_address.as_bytes();

    [mac[0], mac[1], mac[2], 0xff, 0xfe, mac[3], mac[4], mac[5]]
}

/// Drain pending CLI output to the console.
///
/// A task rather than a direct write from the output callback, because that
/// callback is synchronous while the console is not - see `console`.
#[embassy_executor::task]
async fn run_console_out(mut console_tx: ConsoleTx) -> ! {
    // Big chunks: the driver splits into USB packets itself, and fewer
    // task round-trips is what keeps the drain ahead of the CLI.
    let mut buf = [0; 512];

    loop {
        let len = console::read_out(&mut buf).await;
        // No await between taking the bytes and claiming them - see `tx_begin`.
        console::tx_begin();

        let _ = console_tx.write_all(&buf[..len]).await;
        let _ = console_tx.flush().await;

        console::tx_end();
    }
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: EspRadio<'static>) -> ! {
    ot.run(radio).await
}
