//! Shared plumbing for the `cli_node` test firmware.
//!
//! The node itself is chip-agnostic; everything that differs between the
//! nRF52840 and the nRF54L15 — the interrupt bindings, which peripherals MPSL
//! and the 802.15.4 driver claim, and which peripheral the console rides on —
//! is collected here.
//!
//! The console is a UART by default and the board's own USB peripheral under
//! `console-usb`. Which one a board needs is a wiring question: a XIAO
//! plugged straight into the host has no serial port but USB, while a board
//! reached through a debug probe gets its console over the probe's UART
//! bridge. Either way the harness sees a USB CDC
//! port on the host, so the substitution is invisible to it.
//!
//! `console-usb` is nRF52840-only — the nRF54L15 has no USB peripheral at all.

#![no_std]

#[cfg(all(feature = "console-usb", not(feature = "nrf52840")))]
compile_error!("`console-usb` needs a USB peripheral, which only the nRF52840 target has here");

use embassy_nrf::bind_interrupts;

#[path = "../../shared/console.rs"]
pub mod console;
pub mod settings;

#[cfg(feature = "nrf52840")]
bind_interrupts!(pub struct Irqs {
    // MPSL low-priority and 802.15.4 EGU handler share EGU0_SWI0
    EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler, nrf_802154::EguInterruptHandler;
    // MPSL high-priority handlers
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    // MPSL clock handler (this is why the USB console uses
    // `SoftwareVbusDetect`: `CLOCK_POWER` belongs to MPSL here)
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    // 802.15.4 LP timer (RTC2 on nRF52840)
    RTC2 => nrf_802154::LpTimerInterruptHandler;
    // The console
    #[cfg(feature = "console-usb")]
    USBD => embassy_nrf::usb::InterruptHandler<embassy_nrf::peripherals::USBD>;
    #[cfg(not(feature = "console-usb"))]
    UARTE0 => embassy_nrf::buffered_uarte::InterruptHandler<embassy_nrf::peripherals::UARTE0>;
});

#[cfg(feature = "nrf54l15")]
bind_interrupts!(pub struct Irqs {
    // MPSL low-priority handler. Unlike nRF52, nothing is shared with the
    // 802.15.4 driver here: it has its own EGU instance (EGU10).
    SWI00 => nrf_mpsl::LowPrioInterruptHandler;
    // MPSL high-priority handlers
    RADIO_0 => nrf_mpsl::HighPrioInterruptHandler;
    TIMER10 => nrf_mpsl::HighPrioInterruptHandler;
    GRTC_3 => nrf_mpsl::HighPrioInterruptHandler;
    // MPSL clock handler
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    // 802.15.4 notifications
    EGU10 => nrf_802154::EguInterruptHandler;
    // 802.15.4 LP timer (GRTC interrupt group 0)
    GRTC_0 => nrf_802154::LpTimerInterruptHandler;
    // 802.15.4 frame encryption offload
    AAR00_CCM00 => nrf_802154::CcmInterruptHandler;
    // The UART console. Buffered, so RX stays armed between reads - see the
    // console construction in `cli_node`.
    SERIAL20 => embassy_nrf::buffered_uarte::InterruptHandler<embassy_nrf::peripherals::SERIAL20>;
});

/// The blocking hardware RNG. nRF54L has no standalone `RNG` peripheral; its
/// entropy comes out of the CRACEN crypto engine instead.
#[cfg(feature = "nrf52840")]
pub type Rng<'d> = embassy_nrf::rng::Rng<'d, embassy_nrf::mode::Blocking>;

/// The blocking hardware RNG. nRF54L has no standalone `RNG` peripheral; its
/// entropy comes out of the CRACEN crypto engine instead.
#[cfg(feature = "nrf54l15")]
pub type Rng<'d> = embassy_nrf::cracen::Cracen<'d, embassy_nrf::mode::Blocking>;

/// Initialize embassy-nrf with a configuration MPSL and the 802.15.4 driver can
/// live with.
///
/// On nRF54L that means forcing the 128 MHz PLL: embassy-nrf defaults to 64 MHz,
/// but MPSL asserts on 128 MHz at startup, and 128 MHz is the only frequency the
/// 802.15.4 driver supports on this series.
pub fn init() -> embassy_nrf::Peripherals {
    #[cfg(feature = "nrf54l15")]
    scrub_grtc();

    #[allow(unused_mut)]
    let mut config: embassy_nrf::config::Config = Default::default();
    #[cfg(feature = "nrf54l15")]
    {
        config.clock_speed = embassy_nrf::config::ClockSpeed::CK128;
    }
    embassy_nrf::init(config)
}

/// Drop any GRTC interrupt state inherited from the firmware that ran before us.
///
/// The GRTC is in the always-on domain, so a soft reset — which is what a
/// debugger's `SYSRESETREQ`, `probe-rs run`, and this node's own CLI `reset`
/// all issue — leaves its compare events latched and its per-domain interrupt
/// enables set. The NVIC *is* reset, so nothing fires until someone re-enables
/// the line; embassy's GRTC time driver does exactly that at the end of
/// `embassy_nrf::init`.
///
/// From there an inherited enable is fatal: embassy's ISR only ever clears its
/// own channel (CC[11]), so a stale event on any other channel of the same
/// domain re-fires the instant the handler returns, and `init` never comes back.
/// That matters more here than in the examples, because `reset` is part of this
/// node's contract with the harness and runs on every test.
///
/// Cheap insurance, and it has to happen before `embassy_nrf::init`.
#[cfg(feature = "nrf54l15")]
fn scrub_grtc() {
    let r = embassy_nrf::pac::GRTC;

    // Every domain, not just ours: at this point in boot nothing else — not
    // MPSL, not the time driver — has claimed one yet.
    for group in 0..4 {
        r.intenclr(group).write(|w| w.0 = u32::MAX);
    }
    for cc in 0..12 {
        r.events_compare(cc).write_value(0);
    }
}

/// Claim the peripherals MPSL needs out of [`init`]'s `Peripherals`.
#[cfg(feature = "nrf52840")]
#[macro_export]
macro_rules! mpsl_peripherals {
    ($p:expr) => {
        nrf_mpsl::Peripherals::new(
            $p.RTC0,
            $p.TIMER0,
            $p.TEMP,
            $p.PPI_CH19,
            $p.PPI_CH30,
            $p.PPI_CH31,
        )
    };
}

/// Claim the peripherals MPSL needs out of [`init`]'s `Peripherals`.
#[cfg(feature = "nrf54l15")]
#[macro_export]
macro_rules! mpsl_peripherals {
    ($p:expr) => {
        nrf_mpsl::Peripherals::new(
            $p.GRTC_CH7,
            $p.GRTC_CH8,
            $p.GRTC_CH9,
            $p.GRTC_CH10,
            $p.GRTC_CH11,
            $p.TIMER10,
            $p.TIMER20,
            $p.TEMP,
            $p.PPI10_CH0,
            $p.PPI20_CH1,
            $p.PPIB11_CH0,
            $p.PPIB21_CH0,
        )
    };
}

/// Claim the peripherals the 802.15.4 driver needs out of [`init`]'s
/// `Peripherals`.
#[cfg(feature = "nrf52840")]
#[macro_export]
macro_rules! radio_peripherals {
    ($p:expr) => {
        nrf_802154::RadioPeripherals::new($p.EGU0, $p.TIMER2, $p.RTC2)
    };
}

/// Claim the peripherals the 802.15.4 driver needs out of [`init`]'s
/// `Peripherals`.
#[cfg(feature = "nrf54l15")]
#[macro_export]
macro_rules! radio_peripherals {
    ($p:expr) => {
        nrf_802154::RadioPeripherals::new(
            $p.GRTC_CH3,
            $p.GRTC_CH4,
            $p.GRTC_CH5,
            $p.PPI20_CH2,
            $p.PPI20_CH3,
            $p.PPIB11_CH1,
            $p.PPIB21_CH1,
            $p.PPIB11_CH2,
            $p.PPIB21_CH2,
        )
    };
}

/// Claim the hardware RNG out of [`init`]'s `Peripherals`.
#[cfg(feature = "nrf52840")]
#[macro_export]
macro_rules! rng {
    ($p:expr) => {
        embassy_nrf::rng::Rng::new_blocking($p.RNG)
    };
}

/// Claim the hardware RNG out of [`init`]'s `Peripherals`.
#[cfg(feature = "nrf54l15")]
#[macro_export]
macro_rules! rng {
    ($p:expr) => {
        embassy_nrf::cracen::Cracen::new_blocking($p.CRACEN)
    };
}

/// Claim the flash peripheral the settings image lives in.
///
/// nRF52 has NVMC over its flash; nRF54L has RRAMC over its RRAM. `nrf-mpsl`'s
/// timeslot-coordinated `Flash` supports both.
#[cfg(feature = "nrf52840")]
#[macro_export]
macro_rules! flash_peripheral {
    ($p:expr) => {
        $p.NVMC
    };
}

/// Claim the flash peripheral the settings image lives in.
#[cfg(feature = "nrf54l15")]
#[macro_export]
macro_rules! flash_peripheral {
    ($p:expr) => {
        $p.RRAMC
    };
}

/// The chip's factory device identity, as an EUI-64: unique per board and
/// stable across resets, which is what makes it usable as the node identity.
///
/// nRF52 exposes `FICR.DEVICEADDR`; the nRF54L FICR has no such register, so
/// the device ID under `FICR.INFO` stands in.
pub fn ieee_eui64() -> [u8; 8] {
    let ficr = embassy_nrf::pac::FICR;

    #[cfg(feature = "nrf52840")]
    let (low, high) = (ficr.deviceaddr(0).read(), ficr.deviceaddr(1).read());
    #[cfg(feature = "nrf54l15")]
    let (low, high) = (
        ficr.info().deviceid(0).read(),
        ficr.info().deviceid(1).read(),
    );

    let mut eui64 = [0; 8];
    eui64[..4].copy_from_slice(&low.to_be_bytes());
    eui64[4..].copy_from_slice(&high.to_be_bytes());

    // Locally administered, unicast - this is a device address, not an OUI one.
    eui64[0] = (eui64[0] & 0xfe) | 0x02;

    eui64
}

/// The console UART's `(rxd, txd)` pins: an nRF52840-DK's J-Link virtual COM
/// port by default, a XIAO nRF52840's D6/D7 pads under `console-uart-xiao`.
#[cfg(all(feature = "nrf52840", not(feature = "console-uart-xiao")))]
#[macro_export]
macro_rules! console_uart_pins {
    ($p:expr) => {
        ($p.P0_08, $p.P0_06)
    };
}

/// The console UART's `(rxd, txd)` pins - see the other arm.
#[cfg(all(feature = "nrf52840", feature = "console-uart-xiao"))]
#[macro_export]
macro_rules! console_uart_pins {
    ($p:expr) => {
        ($p.P1_12, $p.P1_11)
    };
}

/// Build the console UART, returning `(rx, tx)`.
///
/// nRF52840: `UARTE0` on the pins `console_uart_pins` selects.
///
/// Buffered rather than a raw `Uarte`: a raw `UarteRx::read` only arms EasyDMA
/// for the duration of the call, so bytes arriving while the node echoes the
/// previous one are lost, and the harness sees commands mangled into
/// `InvalidCommand`. The buffered driver keeps RX armed into a ring buffer.
///
/// The TIMER, PPI channels and PPI group are ones neither MPSL (RTC0, TIMER0,
/// TEMP, PPI 19/30/31) nor the 802.15.4 driver (PPI 6-11 and 13/14, PPI groups
/// 0-2) claims.
#[cfg(all(feature = "nrf52840", not(feature = "console-usb")))]
#[macro_export]
macro_rules! console_uart {
    ($p:expr, $config:expr, $rx_buf:expr, $tx_buf:expr) => {{
        let (rxd, txd) = $crate::console_uart_pins!($p);
        embassy_nrf::buffered_uarte::BufferedUarte::new(
            $p.UARTE0,
            $p.TIMER1,
            $p.PPI_CH0,
            $p.PPI_CH1,
            $p.PPI_GROUP3,
            rxd,
            txd,
            $crate::Irqs,
            $config,
            $rx_buf,
            $tx_buf,
        )
        .split()
    }};
}

/// Build the console UART, returning `(rx, tx)`.
///
/// nRF54L15: `UART20` on P1.08/P1.09, as the XIAO nRF54L15's onboard SAMD11
/// CMSIS-DAP probe bridges it. See the nRF52840 arm for why this is buffered.
#[cfg(feature = "nrf54l15")]
#[macro_export]
macro_rules! console_uart {
    ($p:expr, $config:expr, $rx_buf:expr, $tx_buf:expr) => {
        embassy_nrf::buffered_uarte::BufferedUarte::new(
            $p.SERIAL20,
            $p.P1_08,
            $p.P1_09,
            $crate::Irqs,
            $config,
            $rx_buf,
            $tx_buf,
        )
        .split()
    };
}
