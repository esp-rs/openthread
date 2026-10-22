//! An implementation of an OpenThread Driver for `embassy-net` using the `embassy-net-driver-channel` crate.

use core::pin::pin;

use embassy_futures::select::{select3, Either3};
use embassy_net_driver_channel::{driver::HardwareAddress, RxRunner, TxRunner};

use crate::{OpenThread, Radio};

pub use embassy_net_driver_channel::{
    driver::LinkState as EnetLinkState, Device as EnetDriver, State as EnetDriverState,
    StateRunner as EnetStateRunner,
};

/// Create a new OpenThread driver for `embassy-net`, by combining the provided `OpenThread` instance with the `embassy-net-driver-channel` runner types.
///
/// The driver is communicating with `embassy-net` and `smoltcp` using naked Ipv6 frames, without
/// any hardware address and any additional framing (like e.g. Ethernet) attached.
///
/// All details about the network stack (i.e. that it is based on IEEE 802.15.4) are abstracted away and
/// invisible to `embassy-net` and `smoltcp`.
///
/// Arguments:
/// - `ot`: The OpenThread instance.
/// - `state`: A mutable reference to the `embassy-net-driver-channel` state resources.
///
/// Returns:
///   - The `embassy-net-driver-channel` state runner (note: this is not really a "runner" per se, but more of a controller to switch on/off the Driver)
///   - A runner that runs both the `openthread` stack as well as the `embassy-net` driver stack
///   - The `embassy-net` Driver for OpenThread
pub fn new<'d, const MTU: usize, const N_RX: usize, const N_TX: usize>(
    ot: OpenThread<'d>,
    state: &'d mut EnetDriverState<MTU, N_RX, N_TX>,
) -> (
    EnetStateRunner<'d>,
    EnetRunner<'d, MTU>,
    EnetDriver<'d, MTU>,
) {
    let (runner, device) = embassy_net_driver_channel::new(state, HardwareAddress::Ip);

    let (state_runner, rx_runner, tx_runner) = runner.split();

    (
        state_runner,
        EnetRunner {
            ot,
            rx_runner,
            tx_runner,
        },
        device,
    )
}

/// A runner that runs both the `openthread` stack runner as well as the `embassy-net-driver-channel` runner.
///
/// The runner also does the Ipv6 packets' ingress/egress to/from the `embassy-net` stack and to/from `openthread`.
pub struct EnetRunner<'d, const MTU: usize> {
    ot: OpenThread<'d>,
    rx_runner: RxRunner<'d, MTU>,
    tx_runner: TxRunner<'d, MTU>,
}

impl<const MTU: usize> EnetRunner<'_, MTU> {
    /// Run the OpenThread stack and the `embassy-net-driver-channel` runner.
    ///
    /// Arguments:
    /// - `radio`: The radio to be used by the OpenThread stack: the one whose capabilities
    ///   ([`Radio::init`]) were passed to the `OpenThread` constructor.
    pub async fn run<R>(&mut self, mut radio: R) -> !
    where
        R: Radio,
    {
        self.ot.enable_ipv6_rx(true);
        let _guard = scopeguard::guard((), |_| self.ot.enable_ipv6_rx(false));

        let mut rx = pin!(Self::run_rx(&self.ot, &mut self.rx_runner));
        let mut tx = pin!(Self::run_tx(&self.ot, &mut self.tx_runner));
        let mut ot = pin!(Self::run_ot(&self.ot, &mut radio));

        match select3(&mut rx, &mut tx, &mut ot).await {
            Either3::First(r) | Either3::Second(r) | Either3::Third(r) => r,
        }
    }

    async fn run_rx(rx: &OpenThread<'_>, rx_runner: &mut RxRunner<'_, MTU>) -> ! {
        loop {
            unwrap!(rx.wait_rx_available().await);

            let buf = rx_runner.rx_buf().await;

            let len = unwrap!(rx.rx(buf).await);

            rx_runner.rx_done(len);
        }
    }

    async fn run_tx(tx: &OpenThread<'_>, tx_runner: &mut TxRunner<'_, MTU>) -> ! {
        loop {
            let buf = tx_runner.tx_buf().await;

            unwrap!(tx.tx(buf));

            tx_runner.tx_done();
        }
    }

    async fn run_ot<R>(runner: &OpenThread<'_>, radio: R) -> !
    where
        R: Radio,
    {
        runner.run(radio).await
    }
}
