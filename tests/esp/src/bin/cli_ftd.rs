//! The ESP32-C6/H2 firmware node of the hardware-in-the-loop tier: the same
//! DUT shape as the host `cli_ftd`, but running on the MCU with its own radio.
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
//! under the unmodified upstream scenarios. Unlike the nRF node there is no
//! software MAC in the picture: `esp-radio` offloads the whole MAC set (bar
//! source matching), so the radio goes straight into `OpenThread::run` - which
//! also makes this firmware the *simple* end of the MCU tier.
//!
//! # Why this board is the automation-friendly one
//!
//! The USB-Serial-JTAG peripheral makes one USB port do everything:
//! `espflash` resets the chip into download mode over it (no buttons), flashes,
//! resets back into the app, and the same port then carries the CLI console -
//! so a flash/test/fix loop needs no human hands at all.
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
//! honored with a chip `software_reset`. With RAM-backed settings a reboot
//! comes back factory-fresh: exactly `factoryreset`'s semantics, and an
//! approximation of `reset` that loses the dataset - so scenarios that
//! `reset` a node and expect it to rejoin stay off this tier's list until
//! flash-backed settings land.
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

use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;
use esp_radio::ieee802154::Ieee802154;
use {esp_backtrace as _, esp_println as _};

use embedded_io_async::{Read, Write};

/// The console's two ends: the USB-Serial-JTAG serial side.
type ConsoleRx = UsbSerialJtagRx<'static, Async>;
type ConsoleTx = UsbSerialJtagTx<'static, Async>;

use openthread::esp::EspRadio;
use openthread::{OpenThread, OtResources, SimpleRamSettings};

use tinyrlibc as _;

#[path = "../../../shared/console.rs"]
mod console;

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

    let rng = mk_static!(Rng, Rng::new());

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);
    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    // A full hardware MAC (bar source matching) - no `MacRadio`, no
    // `ProxyRadio`: the radio can run on the main executor.
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
        let Ok(len) = console_rx.read(&mut buf).await else {
            continue;
        };

        for byte in &buf[..len] {
            if reader.push(*byte) {
                let line = reader.line().trim();

                if matches!(line, "reset" | "factoryreset") {
                    esp_hal::system::software_reset();
                }

                if !line.is_empty() {
                    let _ = ot.cli_input_line(line);
                }

                reader.clear();
            }
        }
    }
}

/// Drain pending CLI output to the console.
///
/// A task rather than a direct write from the output callback, because that
/// callback is synchronous while the console is not - see `console`.
#[embassy_executor::task]
async fn run_console_out(mut console_tx: ConsoleTx) -> ! {
    let mut buf = [0; 64];

    loop {
        let len = console::read_out(&mut buf).await;
        let _ = console_tx.write_all(&buf[..len]).await;
        let _ = console_tx.flush().await;
    }
}

/// The chip's factory base MAC as an EUI-64.
fn ieee_eui64() -> [u8; 8] {
    let mac_address = esp_hal::efuse::base_mac_address();
    let mac = mac_address.as_bytes();

    [mac[0], mac[1], mac[2], 0xff, 0xfe, mac[3], mac[4], mac[5]]
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: EspRadio<'static>) -> ! {
    ot.run(radio).await
}
