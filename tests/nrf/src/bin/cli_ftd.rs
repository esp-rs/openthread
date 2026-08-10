//! The nRF firmware node of the hardware-in-the-loop tier: the same DUT shape
//! as the host `cli_ftd`, but running on an nRF52840 with its own radio.
//!
//! The upstream harness drives this exactly as it drives every other node -
//! CLI lines in, CLI output back - except the pipe is the board's serial
//! console rather than a process's stdin/stdout. `openthread-tests`'
//! `serial_bridge` is what makes that substitution invisible to the harness.
//!
//! # What only this tier exercises
//!
//! The simulation tiers cover the stack above the radio, and the RCP tier adds
//! real RF under a co-processor's firmware. Here the crate's *own* radio path
//! runs where it is meant to:
//!
//! - [`NrfRadio`] driving the nRF's IEEE 802.15.4 peripheral;
//! - [`MacRadio`], whose software ACKs have to make the inter-frame gap on a
//!   real clock rather than a simulated one;
//! - [`ProxyRadio`] / [`PhyRadioRunner`], whose whole reason to exist is
//!   putting that soft-MAC on a higher-priority executor so it can.
//!
//! Any other `Radio` implementation (the `nrf-802154` crate's, say) drops into
//! the same place - which is the point of testing the contract rather than one
//! driver.
//!
//! # Board
//!
//! An nRF52840-DK: the CLI console is `UARTE0` on the DK's J-Link virtual COM
//! port (P0.06 TX, P0.08 RX at 115200), which enumerates on the host as
//! `/dev/ttyACM*`. `defmt` logs go out over RTT instead, so they never touch
//! the console the harness is parsing - the same separation the host DUT keeps
//! between its logs and its CLI.
//!
//! A USB-only board (the nRF52840 *dongle*) has no such UART and would need a
//! USB CDC console instead; that is a different console binding, not a
//! different node.
//!
//! # Node identity
//!
//! Flashed firmware cannot be told which node it is - the harness passes that
//! on a command line only the bridge sees. So the EUI-64 comes from the chip's
//! own factory device address (`FICR.DEVICEADDR`), which is unique per board
//! and stable across resets. The node-id-to-board mapping lives entirely in
//! the bridge's port map.

#![no_std]
#![no_main]

use cortex_m_rt::entry;

use embassy_executor::{Executor, InterruptExecutor, Spawner};

use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::mode::Blocking;
use embassy_nrf::radio::ieee802154::Radio as Ieee802154;
use embassy_nrf::rng::Rng;
#[cfg(not(feature = "console-usb"))]
use embassy_nrf::uarte::{self, UarteRx, UarteTx};
#[cfg(feature = "console-usb")]
use embassy_nrf::usb;
use embassy_nrf::{bind_interrupts, interrupt, peripherals, radio};

#[cfg(feature = "console-usb")]
use embassy_usb::class::cdc_acm::{self, CdcAcmClass};
#[cfg(feature = "console-usb")]
use embassy_usb::UsbDevice;

use openthread::nrf::NrfRadio;
use openthread::{
    EmbassyTimeTimer, MacRadio, MacRadioResources, OpenThread, OtResources, PhyRadioRunner,
    ProxyRadio, ProxyRadioResources, SimpleRamSettings,
};

use tinyrlibc as _;

#[path = "../../../shared/console.rs"]
mod console;

use panic_rtt_target as _;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

#[cfg(not(feature = "console-usb"))]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    UARTE0 => uarte::InterruptHandler<peripherals::UARTE0>;
});

#[cfg(feature = "console-usb")]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

/// The console's two ends, whichever peripheral is underneath.
#[cfg(not(feature = "console-usb"))]
type ConsoleTx = UarteTx<'static>;
#[cfg(not(feature = "console-usb"))]
type ConsoleRx = UarteRx<'static>;

#[cfg(feature = "console-usb")]
type UsbDriver = usb::Driver<'static, usb::vbus_detect::HardwareVbusDetect>;
#[cfg(feature = "console-usb")]
type ConsoleTx = cdc_acm::Sender<'static, UsbDriver>;
#[cfg(feature = "console-usb")]
type ConsoleRx = cdc_acm::Receiver<'static, UsbDriver>;

#[interrupt]
unsafe fn EGU0_SWI0() {
    EXECUTOR_HIGH.on_interrupt()
}

/// The radio's executor: higher priority than everything else, so the software
/// MAC's ACK deadlines are met (see [`ProxyRadio`]).
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

// Only needed for tinyrlibc's alloc functions, which are not called at runtime.
#[global_allocator]
static HEAP: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let p = embassy_nrf::init(Default::default());

    #[cfg(not(feature = "console-usb"))]
    let (console_tx, console_rx) = {
        let mut uarte_config = uarte::Config::default();
        uarte_config.baudrate = uarte::Baudrate::BAUD115200;
        uarte_config.parity = uarte::Parity::EXCLUDED;

        // The DK's J-Link virtual COM port.
        uarte::Uarte::new(p.UARTE0, p.P0_08, p.P0_06, Irqs, uarte_config).split()
    };

    #[cfg(feature = "console-usb")]
    let (console_tx, console_rx, usb) = build_usb_console(p.USBD);

    // The radio's executor is started before the main one so the runner is
    // already serving by the time the stack comes up.
    interrupt::EGU0_SWI0.set_priority(Priority::P7);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::EGU0_SWI0);

    let proxy_radio_resources = mk_static!(ProxyRadioResources, ProxyRadioResources::new());
    let (proxy_radio, phy_radio_runner) = ProxyRadio::new(proxy_radio_resources);

    let radio = NrfRadio::new(Ieee802154::new(p.RADIO, Irqs));

    spawner_high.spawn(run_radio(phy_radio_runner, radio).unwrap());

    let rng = mk_static!(Rng<'static, Blocking>, Rng::new_blocking(p.RNG));

    let executor = mk_static!(Executor, Executor::new());

    executor.run(|spawner| {
        #[cfg(feature = "console-usb")]
        spawner.spawn(run_usb(usb).unwrap());

        spawner.spawn(run_console_out(console_tx).unwrap());
        spawner.spawn(run_node(spawner, proxy_radio, console_rx, rng).unwrap());
    })
}

#[embassy_executor::task]
async fn run_node(
    spawner: Spawner,
    radio: ProxyRadio<'static>,
    console_rx: ConsoleRx,
    rng: &'static mut Rng<'static, Blocking>,
) -> ! {
    // The chip's factory device address: unique per board, stable across
    // resets - the closest thing firmware has to the node id the host DUT
    // gets on its command line.
    let ieee_eui64 = ieee_eui64();

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);
    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    ot.cli_init(console::out);

    // The prompt the harness waits for on connect.
    console::out(b"\r\n> ");

    run_cli(ot, console_rx).await
}

/// Read CLI lines off the console and hand them to the interpreter.
async fn run_cli(ot: OpenThread<'static>, mut console_rx: ConsoleRx) -> ! {
    let mut reader = console::LineReader::new();
    let mut buf = [0; 64];

    loop {
        #[cfg(not(feature = "console-usb"))]
        let read = console_rx.read(&mut buf[..1]).await.map(|()| 1);
        #[cfg(feature = "console-usb")]
        let read = {
            console_rx.wait_connection().await;
            console_rx.read_packet(&mut buf).await
        };

        let Ok(len) = read else {
            continue;
        };

        for byte in &buf[..len] {
            if reader.push(*byte) {
                let _ = ot.cli_input_line(reader.line());
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
    // The UART writes take whatever length; the USB binding splits into
    // packets itself. Fewer task round-trips keeps the drain ahead of the CLI.
    let mut buf = [0; 512];

    loop {
        #[cfg(feature = "console-usb")]
        console_tx.wait_connection().await;

        let len = console::read_out(&mut buf).await;

        #[cfg(not(feature = "console-usb"))]
        let _ = console_tx.write(&buf[..len]).await;
        #[cfg(feature = "console-usb")]
        let _ = console_tx.write_packet(&buf[..len]).await;
    }
}

/// Build the USB CDC console: the nRF's own USB peripheral presented to the
/// host as a serial port, for boards that have no UART bridge.
#[cfg(feature = "console-usb")]
fn build_usb_console(
    usbd: embassy_nrf::Peri<'static, peripherals::USBD>,
) -> (ConsoleTx, ConsoleRx, UsbDevice<'static, UsbDriver>) {
    let driver = usb::Driver::new(usbd, Irqs, usb::vbus_detect::HardwareVbusDetect::new(Irqs));

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

/// Drive the USB device stack.
#[cfg(feature = "console-usb")]
#[embassy_executor::task]
async fn run_usb(mut usb: UsbDevice<'static, UsbDriver>) -> ! {
    usb.run().await
}

/// The chip's factory device address, as an EUI-64.
fn ieee_eui64() -> [u8; 8] {
    let ficr = embassy_nrf::pac::FICR;

    let low = ficr.deviceaddr(0).read();
    let high = ficr.deviceaddr(1).read();

    let mut eui64 = [0; 8];
    eui64[..4].copy_from_slice(&low.to_be_bytes());
    eui64[4..].copy_from_slice(&high.to_be_bytes());

    // Locally administered, unicast - this is a device address, not an OUI one.
    eui64[0] = (eui64[0] & 0xfe) | 0x02;

    eui64
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: ProxyRadio<'static>) -> ! {
    ot.run(radio).await
}

#[embassy_executor::task]
async fn run_radio(mut runner: PhyRadioRunner<'static>, radio: NrfRadio<'static>) -> ! {
    // A bare PHY, so the software MAC goes on here - on this high-priority
    // executor, where its ACK deadlines are meetable.
    let mac_radio_resources = mk_static!(MacRadioResources, MacRadioResources::new());

    runner
        .run(MacRadio::new(radio, EmbassyTimeTimer, mac_radio_resources))
        .await
}
