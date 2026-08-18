//! The nRF firmware node of the HIL tier:
//! Same purpose as the host / STD `cli_node`, but running on the MCU with its own radio.
//!
//! The upstream harness drives this exactly as it drives every other node -
//! CLI lines in, CLI output back - except the pipe is the chip's serial
//! console rather than a process's stdin/stdout. `openthread-tests`'
//! `serial_bridge` is what makes that substitution invisible to the harness.
//!
//! # What only this tier exercises
//!
//! - [`NrfRadio`] driving the nRF's IEEE 802.15.4 peripheral;
//! - [`MacRadio`], whose software ACKs have to make the inter-frame gap on a
//!   real clock rather than a simulated one;
//! - [`ProxyRadio`] / [`PhyRadioRunner`], whose whole reason to exist is
//!   putting that soft-MAC on a higher-priority executor so it can.
//!
//! # Reset
//!
//! The CLI `reset`/`factoryreset` commands are intercepted (the C stack
//! cannot re-create itself in place - see the crate's `otPlatReset`) and
//! honored with a chip reset. The settings persist in flash (see
//! `settings`), so a `reset` node comes back with its dataset and network
//! state intact and rejoins on its own. `factoryreset` is first forwarded to
//! the stack, whose factory-reset path clears the settings - durably, thanks
//! to the write-through - before the chip reboots.
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

#[cfg(not(feature = "console-usb"))]
use embassy_nrf::buffered_uarte::{BufferedUarte, BufferedUarteRx, BufferedUarteTx};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::mode::Blocking;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::radio::ieee802154::Radio as Ieee802154;
use embassy_nrf::rng::Rng;
#[cfg(not(feature = "console-usb"))]
use embassy_nrf::uarte;
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
    ProxyRadio, ProxyRadioResources,
};

use tinyrlibc as _;

#[path = "../../../shared/console.rs"]
mod console;

#[path = "../settings.rs"]
mod settings;

use settings::FlashSettings;

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
    UARTE0 => embassy_nrf::buffered_uarte::InterruptHandler<peripherals::UARTE0>;
});

#[cfg(feature = "console-usb")]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

/// The console's two ends, whichever peripheral is underneath.
#[cfg(not(feature = "console-usb"))]
type ConsoleTx = BufferedUarteTx<'static>;
#[cfg(not(feature = "console-usb"))]
type ConsoleRx = BufferedUarteRx<'static>;

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

    defmt::info!("boot");

    // The external crystal, not the default internal RC: the USB peripheral
    // requires it outright, and the radio's timing is better served by it -
    // and every nRF52840 board has one.
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    #[cfg(not(feature = "console-usb"))]
    let (console_tx, console_rx) = {
        let mut uarte_config = uarte::Config::default();
        uarte_config.baudrate = uarte::Baudrate::Baud115200;
        uarte_config.parity = uarte::Parity::Excluded;

        // The DK's J-Link virtual COM port by default; `console-uart-xiao`
        // picks a XIAO nRF52840's D6/D7 pads instead, for a XIAO wired to an
        // external debug probe's UART bridge.
        #[cfg(not(feature = "console-uart-xiao"))]
        let (rxd, txd) = (p.P0_08, p.P0_06);
        #[cfg(feature = "console-uart-xiao")]
        let (rxd, txd) = (p.P1_12, p.P1_11);

        // Buffered rather than a raw `Uarte`: a raw `UarteRx::read` only arms
        // EasyDMA for the duration of the call, so a byte arriving while the
        // node echoes the previous one is lost, and the harness sees commands
        // mangled into `InvalidCommand`. The buffered driver keeps RX armed
        // into a ring buffer. Its TIMER, PPI channels and PPI group are free
        // here: this node drives the radio through `embassy-nrf` alone, with
        // no MPSL and no Nordic C driver bidding for them.
        let (console_rx, console_tx) = BufferedUarte::new(
            p.UARTE0,
            p.TIMER1,
            p.PPI_CH0,
            p.PPI_CH1,
            p.PPI_GROUP3,
            rxd,
            txd,
            Irqs,
            uarte_config,
            mk_static!([u8; 1024], [0; 1024]),
            mk_static!([u8; 1024], [0; 1024]),
        )
        .split();
        (console_tx, console_rx)
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

    // The settings image lives in the flash page both `memory*.x` layouts
    // carve off the top of the application region - out of the linker's
    // FLASH, so the firmware can never grow into it.
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);
    let ot_settings = mk_static!(
        FlashSettings,
        FlashSettings::new(Nvmc::new(p.NVMC), 0x000f_f000, ot_settings_buf)
    );

    defmt::info!("peripherals up, starting executor");

    let executor = mk_static!(Executor, Executor::new());

    executor.run(|spawner| {
        #[cfg(feature = "console-usb")]
        spawner.spawn(run_usb(usb).unwrap());

        spawner.spawn(run_console_out(console_tx).unwrap());
        spawner.spawn(run_node(spawner, proxy_radio, console_rx, rng, ot_settings).unwrap());
    })
}

#[embassy_executor::task]
async fn run_node(
    spawner: Spawner,
    radio: ProxyRadio<'static>,
    console_rx: ConsoleRx,
    rng: &'static mut Rng<'static, Blocking>,
    ot_settings: &'static mut FlashSettings,
) -> ! {
    // The chip's factory device address: unique per board, stable across
    // resets - the closest thing firmware has to the node id the host DUT
    // gets on its command line.
    let ieee_eui64 = ieee_eui64();

    let ot_resources = mk_static!(OtResources, OtResources::new());

    defmt::info!("ot init");

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    defmt::info!("ot up");

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    ot.cli_init(console::out);

    // The prompt the harness waits for on connect.
    console::out(b"\r\n> ");

    defmt::info!("cli ready");

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
        #[cfg(not(feature = "console-usb"))]
        let read = console_rx.read(&mut buf).await;
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
                let line = reader.line().trim();

                match line {
                    // Drain before rebooting, or the reply races the reset:
                    // whether it reaches the wire comes down to timing, and
                    // the harness is left waiting on a response that was
                    // never sent - which it reports as a device that went
                    // quiet, a long way from the cause.
                    "reset" => {
                        console::drained().await;
                        cortex_m::peripheral::SCB::sys_reset()
                    }
                    "factoryreset" => {
                        let _ = ot.cli_input_line(line);
                        console::drained().await;
                        cortex_m::peripheral::SCB::sys_reset();
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

/// Drain pending CLI output to the console.
///
/// A task rather than a direct write from the output callback, because that
/// callback is synchronous while the console is not - see `console`.
#[embassy_executor::task]
async fn run_console_out(mut console_tx: ConsoleTx) -> ! {
    // Big chunks off the pipe: fewer task round-trips keeps the drain ahead
    // of the CLI. The UART takes whatever length; USB is packetized below.
    let mut buf = [0; 512];

    loop {
        #[cfg(feature = "console-usb")]
        console_tx.wait_connection().await;

        let len = console::read_out(&mut buf).await;
        // No await between taking the bytes and claiming them - see `tx_begin`.
        console::tx_begin();

        #[cfg(not(feature = "console-usb"))]
        let _ = console_tx.write(&buf[..len]).await;

        #[cfg(feature = "console-usb")]
        {
            // How long a packet write may sit unaccepted before the output
            // is dropped. Generous next to USB speeds, so it only fires
            // when nobody is reading at all.
            const TX_STALL: embassy_time::Duration = embassy_time::Duration::from_millis(500);

            // One packet per `write_packet` - it does not split - and a
            // zero-length packet after a final full-size one: a bulk
            // transfer only ends on a *short* packet, so without the ZLP a
            // host reading in multi-packet units sits on the data until
            // more output happens to push it out.
            //
            // The writes are bounded because the host may not be reading at
            // all (no process has the port open - or worse, ModemManager
            // opened it, probed, and left). Blocking here forever would
            // back up `drained`, and with it the command loop - which then
            // stops reading the OUT endpoint, NAKing every host write, all
            // the way up to a `tcsetattr` that never returns. Dropping
            // output with nobody listening is the console's documented
            // contract anyway.
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
