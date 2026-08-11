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
//! honored with a chip `software_reset`. The settings persist in flash (see
//! `flash_settings`), so a `reset` node comes back with its dataset and
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

use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;
use esp_radio::ieee802154::Ieee802154;
use {esp_backtrace as _, esp_println as _};

use embassy_futures::select::{select, Either};

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

#[path = "../flash_settings.rs"]
mod flash_settings;

use flash_settings::FlashSettings;

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
    spawner.spawn(heartbeat().unwrap());

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
        // Watchdogged for the same lost-event race as the output task: bytes
        // can sit in the RX FIFO with their wakeup gone. Re-entering `read`
        // drains the FIFO before waiting, so a periodic re-poll recovers
        // them; the read itself cannot fail (its error type is uninhabited).
        let len = loop {
            match select(
                console_rx.read(&mut buf),
                embassy_time::Timer::after(embassy_time::Duration::from_millis(200)),
            )
            .await
            {
                Either::First(Ok(len)) => break len,
                Either::Second(()) => continue,
            }
        };

        for byte in &buf[..len] {
            if reader.push(*byte) {
                let line = reader.line().trim();

                match line {
                    // The chip reset both need happens here - see the module
                    // docs. `factoryreset` goes through the stack first: its
                    // factory-reset path is what wipes the settings store.
                    "reset" => esp_hal::system::software_reset(),
                    "factoryreset" => {
                        let _ = ot.cli_input_line(line);
                        esp_hal::system::software_reset();
                    }
                    "" => (),
                    _ => {
                        let _ = ot.cli_input_line(line);
                    }
                }

                reader.clear();

                // Let the response drain fully before accepting the next
                // command - see `console::drained`.
                console::drained().await;
            }
        }
    }
}

/// Drain pending CLI output to the console.
///
/// A task rather than a direct write from the output callback, because that
/// callback is synchronous while the console is not - see `console`.
///
/// The writes are watchdogged: the USB-Serial-JTAG driver's async TX waits
/// on an edge-latched "endpoint empty" event, and that event can be lost to
/// an interrupt race, leaving a write future parked forever on bytes that
/// long since reached the host - the console then freezes until the next
/// *incoming* byte's interrupt shakes it loose (observed as command output
/// arriving in one burst seconds late, exactly when the harness gives up
/// and sends its next command). Re-polling on a timeout re-reads the FIFO
/// state and completes; the chunk size stays within one 64-byte endpoint
/// fill so a stranded wait never leaves part of a chunk unsent.
#[embassy_executor::task]
async fn run_console_out(mut console_tx: ConsoleTx) -> ! {
    // One endpoint fill per write: a chunk this size is committed to the
    // hardware in one go, so a timed-out wait below can only ever mean
    // "completion lost or host slow", never "partially unsent".
    let mut buf = [0; 64];

    loop {
        let len = console::read_out(&mut buf).await;

        // A timeout here does NOT warrant re-sending: the bytes are already
        // in the endpoint. Recovery happens in the flush loop below, whose
        // entry re-reads the endpoint state instead of trusting the (lost)
        // event.
        let _ = select(
            console_tx.write_all(&buf[..len]),
            embassy_time::Timer::after(embassy_time::Duration::from_millis(50)),
        )
        .await;

        // Wait until the endpoint has actually drained - re-polling on a
        // timeout so a lost completion costs 50ms, not a frozen console.
        // The next write may only start against an empty endpoint.
        while matches!(
            select(
                console_tx.flush(),
                embassy_time::Timer::after(embassy_time::Duration::from_millis(50)),
            )
            .await,
            Either::Second(())
        ) {}
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

/// Console keep-alive: a blank line every 2s.
///
/// The USB-Serial-JTAG driver's async TX waits on an edge-latched "endpoint
/// empty" event that an interrupt race can lose, parking the output task on
/// bytes the host long since read - the console then freezes until the next
/// write cycles fresh events through the endpoint. This heartbeat IS that
/// next write: it bounds any such freeze at one period, well inside every
/// harness timeout, and an empty line is invisible to the harness's line
/// matchers. (A watchdog inside `run_console_out` is not enough on its own:
/// its recovery path can park on the same lost event class it guards
/// against - traffic through the endpoint is what reliably clears it.)
#[embassy_executor::task]
async fn heartbeat() -> ! {
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;

        console::out(b"\r\n");
    }
}
