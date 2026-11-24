//! The nRF firmware node of the HIL test tier, with the `NrfRadio` of
//! `openthread-driver-nrf` over the `nrf-802154` driver.
//!
//! The upstream harness drives this exactly as it drives every other node -
//! CLI lines in over the board's console, CLI output back. The
//! `serial_bridge` of the `tests` crate makes that substitution invisible
//! to the harness:
//!
//! ```sh
//! # in the root of this repo, with this firmware flashed:
//! cargo xtask itest --hw-port /dev/serial/by-id/<this-board>=mcu \
//!                   --hw-port /dev/serial/by-id/<ot-rcp-dongle>=rcp
//! ```
//!
//! # Reset
//!
//! The CLI `reset`/`factoryreset` commands are intercepted (the C stack
//! cannot re-create itself in place) and honored with a chip reset - after
//! awaiting the settings flush, since persistence is write-behind here (see
//! `settings`).
//!
//! # Node identity
//!
//! The EUI-64 comes from the chip's factory device address
//! (`FICR.DEVICEADDR`), unique per board and stable across resets; the
//! node-id-to-board mapping lives entirely in the bridge's port map.

#![no_std]
#![no_main]

use embassy_executor::Spawner;

#[cfg(not(feature = "console-usb"))]
use embassy_nrf::buffered_uarte::{BufferedUarteRx, BufferedUarteTx};
#[cfg(all(feature = "nrf54l15", not(feature = "console-usb")))]
use embassy_nrf::peripherals;
#[cfg(not(feature = "console-usb"))]
use embassy_nrf::uarte;
#[cfg(feature = "console-usb")]
use embassy_nrf::{peripherals, usb};

#[cfg(feature = "console-usb")]
use embassy_usb::class::cdc_acm::{self, CdcAcmClass};
#[cfg(feature = "console-usb")]
use embassy_usb::UsbDevice;

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::{Flash, MultiprotocolServiceLayer, SessionMem};

use openthread::{OpenThread, OtResources, Radio as _};
use openthread_driver_nrf::NrfRadio;

use openthread_tests_nrf::{console, ieee_eui64, settings, Irqs, Rng};

use static_cell::StaticCell;

use tinyrlibc as _;

use {defmt_rtt as _, panic_probe as _};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

/// The flash page `memory.x` carves off the top of the application region -
/// out of the linker's FLASH, so the firmware can never grow into it.
#[cfg(feature = "nrf52840")]
const SETTINGS_OFFSET: u32 = 0x000f_f000;
/// The RRAM page `memory.x` carves off the top of the application region.
#[cfg(feature = "nrf54l15")]
const SETTINGS_OFFSET: u32 = 0x0017_c000;

/// The console's two ends: the board's own USB peripheral (CDC-ACM).
#[cfg(feature = "console-usb")]
type UsbDriver = usb::Driver<'static, &'static usb::vbus_detect::SoftwareVbusDetect>;
#[cfg(feature = "console-usb")]
type ConsoleTx = cdc_acm::Sender<'static, UsbDriver>;
#[cfg(feature = "console-usb")]
type ConsoleRx = cdc_acm::Receiver<'static, UsbDriver>;

/// The console's two ends: a UART, reached through a debug probe's bridge.
/// (embassy's pre-nRF54L buffered UART is not generic over the instance.)
#[cfg(all(feature = "nrf52840", not(feature = "console-usb")))]
type ConsoleTx = BufferedUarteTx<'static>;
#[cfg(all(feature = "nrf52840", not(feature = "console-usb")))]
type ConsoleRx = BufferedUarteRx<'static>;

/// The console's two ends: UART20, which the board's onboard CMSIS-DAP probe
/// bridges to a USB CDC port on the host.
#[cfg(feature = "nrf54l15")]
type ConsoleTx = BufferedUarteTx<'static, peripherals::SERIAL20>;
#[cfg(feature = "nrf54l15")]
type ConsoleRx = BufferedUarteRx<'static, peripherals::SERIAL20>;

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();
static SETTINGS_STATE: settings::FlashSettingsState = settings::FlashSettingsState::new();

// Only needed for tinyrlibc's alloc functions, which are not called at runtime.
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = openthread_tests_nrf::init();

    // The LP clock. On the nRF52840 it has to be the 32.768 kHz crystal: there
    // the driver timestamps frames with a 16 MHz timer that it re-aligns with
    // the LP clock only every ~17 s, and against the RC oscillator - which MPSL
    // recalibrates every few seconds, changing its rate in steps - the two
    // drift apart by hundreds of µs in between: enough for a CSL parent to miss
    // its children's receive windows. The nRF54L timestamps off the LP clock
    // (GRTC) itself, so the RC oscillator does there.
    #[cfg(feature = "nrf52840")]
    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 20,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    #[cfg(not(feature = "nrf52840"))]
    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    let lfclk_accuracy_ppm = lfclk_cfg.accuracy_ppm;

    let mpsl_p = openthread_tests_nrf::mpsl_peripherals!(p);
    // `with_timeslots` rather than `new`: the flash driver schedules its
    // erases/writes as MPSL timeslots, and without session memory allocated
    // here, every `Flash` operation fails at `mpsl_timeslot_session_open` -
    // which silently degrades the settings to RAM-only (see `settings`).
    // One session: the flash driver opens a single session per operation.
    let mpsl = MPSL.init(
        MultiprotocolServiceLayer::with_timeslots(
            mpsl_p,
            Irqs,
            lfclk_cfg,
            mk_static!(SessionMem<1>, SessionMem::new()),
        )
        .unwrap(),
    );
    spawner.spawn(run_mpsl(mpsl).unwrap());

    // Flash access must be MPSL-coordinated (it runs in radio-free
    // timeslots), which is what makes settings persistence write-behind -
    // see `settings`.
    let mut flash = Flash::take(mpsl, openthread_tests_nrf::flash_peripheral!(p));

    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);
    let ot_settings = mk_static!(
        settings::FlashSettings,
        settings::FlashSettings::new(
            &SETTINGS_STATE,
            &mut flash,
            SETTINGS_OFFSET,
            ot_settings_buf
        )
    );

    spawner.spawn(run_settings(flash).unwrap());

    let rng = mk_static!(Rng<'static>, openthread_tests_nrf::rng!(p));

    let ieee_eui64 = ieee_eui64();

    // The full-MAC radio goes straight to the stack: no software MAC, no
    // executor split.
    //
    // The CSL clock accuracy reported to peers is the LP clock's (see
    // `lfclk_cfg`), not the crystal the radio defaults to: with the RC
    // oscillator a CSL peer would otherwise size its windows for a fraction of
    // the drift it gets.
    let mut radio = NrfRadio::new(Radio::new(
        p.RADIO,
        openthread_tests_nrf::radio_peripherals!(p),
        Irqs,
        mpsl,
    ))
    .with_csl_timing(
        lfclk_accuracy_ppm.min(u8::MAX as u16) as u8,
        NrfRadio::DEFAULT_CSL_UNCERTAINTY,
    );
    let radio_caps = radio.init().await.unwrap();

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot = OpenThread::new(ieee_eui64, &radio_caps, rng, ot_settings, ot_resources).unwrap();

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    #[cfg(feature = "console-usb")]
    let (console_tx, console_rx) = {
        let (console_tx, console_rx, usb) = build_usb_console(p.USBD);
        spawner.spawn(run_usb(usb).unwrap());
        (console_tx, console_rx)
    };

    #[cfg(not(feature = "console-usb"))]
    let (console_tx, console_rx) = {
        let mut uarte_config = uarte::Config::default();
        uarte_config.baudrate = uarte::Baudrate::Baud115200;
        uarte_config.parity = uarte::Parity::Excluded;

        // Which UART, on which pins, is board-specific - see `console_uart`.
        let (console_rx, console_tx) = openthread_tests_nrf::console_uart!(
            p,
            uarte_config,
            mk_static!([u8; 1024], [0; 1024]),
            mk_static!([u8; 1024], [0; 1024])
        );
        (console_tx, console_rx)
    };

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
        // USB is packetized and only readable while the host has the port
        // open; the UART is a plain byte stream, one byte at a time.
        #[cfg(feature = "console-usb")]
        let read = {
            console_rx.wait_connection().await;
            console_rx.read_packet(&mut buf).await
        };
        #[cfg(not(feature = "console-usb"))]
        let read = console_rx.read(&mut buf).await;

        let Ok(len) = read else {
            continue;
        };

        for byte in &buf[..len] {
            if reader.push(*byte) {
                let line = reader.line().trim();

                match line {
                    // The chip reset both need happens here - see the module
                    // docs. `factoryreset` goes through the stack first: its
                    // factory-reset path is what wipes the settings store.
                    // Persistence is write-behind, so await the flush before
                    // rebooting - that is what makes either reset durable.
                    "reset" => {
                        settings::flush(&SETTINGS_STATE).await;
                        // Drain before rebooting, or the reply races the
                        // reset: whether it reaches the wire comes down to how
                        // long the flush happened to take, and the harness is
                        // left waiting on a response that was never sent.
                        console::drained().await;
                        cortex_m::peripheral::SCB::sys_reset()
                    }
                    "factoryreset" => {
                        let _ = ot.cli_input_line(line);
                        settings::flush(&SETTINGS_STATE).await;
                        console::drained().await;
                        cortex_m::peripheral::SCB::sys_reset()
                    }
                    // An empty line included: the CLI answers it with a bare
                    // prompt, which is how the harness (and `serial_bridge`)
                    // probe whether the node is up.
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
#[embassy_executor::task]
async fn run_console_out(mut console_tx: ConsoleTx) -> ! {
    // Big chunks off the pipe: fewer task round-trips keeps the drain ahead
    // of the CLI; USB is packetized below.
    let mut buf = [0; 512];

    loop {
        #[cfg(feature = "console-usb")]
        console_tx.wait_connection().await;

        let len = console::read_out(&mut buf).await;
        // No await between taking the bytes and claiming them - see `tx_begin`.
        console::tx_begin();

        // Write *all* of it: a buffered write reports how many bytes it took,
        // and a burst larger than the TX ring buffer is accepted in pieces.
        // Discarding that count truncates CLI output mid-line, which the
        // harness sees as a command that never finished answering.
        #[cfg(not(feature = "console-usb"))]
        {
            let mut sent = 0;
            while sent < len {
                match console_tx.write(&buf[sent..len]).await {
                    Ok(0) | Err(_) => break,
                    Ok(n) => sent += n,
                }
            }

            // Past the ring buffer and onto the wire. Without this, `drained`
            // would report bytes delivered while they are still queued, and a
            // reset following a reply would truncate it.
            let _ = console_tx.flush().await;
        }

        #[cfg(feature = "console-usb")]
        {
            // How long a packet write may sit unaccepted before the output is
            // dropped. Generous next to USB speeds, so it only fires when nobody
            // is reading at all.
            const TX_STALL: embassy_time::Duration = embassy_time::Duration::from_millis(500);

            // One packet per `write_packet` - it does not split - and a
            // zero-length packet after a final full-size one: a bulk transfer
            // only ends on a *short* packet. The writes are bounded because the
            // host may not be reading at all; blocking forever would back up
            // `drained` and the command loop with it.
            let mps = console_tx.max_packet_size() as usize;

            let mut sent = 0;
            while sent < len {
                let chunk = &buf[sent..(sent + mps).min(len)];

                match embassy_time::with_timeout(TX_STALL, console_tx.write_packet(chunk)).await {
                    Ok(Ok(())) => sent += chunk.len(),
                    _ => break,
                }
            }

            if sent == len && sent % mps == 0 {
                let _ = embassy_time::with_timeout(TX_STALL, console_tx.write_packet(&[])).await;
            }
        }

        console::tx_end();
    }
}

#[cfg(feature = "console-usb")]
/// Build the USB CDC console: the board's own USB peripheral presented to
/// the host as a serial port.
///
/// `SoftwareVbusDetect` rather than the hardware one: the `POWER` events
/// belong to MPSL here (see `Irqs`) - and this board is USB-powered, so
/// VBUS is present by definition.
fn build_usb_console(
    usbd: embassy_nrf::Peri<'static, peripherals::USBD>,
) -> (ConsoleTx, ConsoleRx, UsbDevice<'static, UsbDriver>) {
    let vbus = mk_static!(
        usb::vbus_detect::SoftwareVbusDetect,
        usb::vbus_detect::SoftwareVbusDetect::new(true, true)
    );

    let driver = usb::Driver::new(usbd, Irqs, &*vbus);

    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("openthread");
    config.product = Some("openthread test node");
    config.serial_number = None;
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        mk_static!([u8; 256], [0; 256]),
        mk_static!([u8; 256], [0; 256]),
        mk_static!([u8; 128], [0; 128]),
        mk_static!([u8; 128], [0; 128]),
    );

    let class = CdcAcmClass::new(
        &mut builder,
        mk_static!(cdc_acm::State, cdc_acm::State::new()),
        64,
    );

    let usb = builder.build();
    let (console_tx, console_rx) = class.split();

    (console_tx, console_rx, usb)
}

#[embassy_executor::task]
async fn run_mpsl(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::task]
async fn run_settings(flash: Flash<'static>) -> ! {
    settings::run(&SETTINGS_STATE, flash, SETTINGS_OFFSET).await
}

#[cfg(feature = "console-usb")]
#[embassy_executor::task]
async fn run_usb(mut usb: UsbDevice<'static, UsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: NrfRadio<'static>) -> ! {
    ot.run(radio).await
}
