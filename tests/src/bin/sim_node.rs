//! A simulation node: the full `openthread` stack on the Rust platform
//! (embassy alarm, tasklet pumping, software MAC via [`MacRadio`]) with the
//! UDP-multicast [`SimRadio`] as its 802.15.4 "RF".
//!
//! Usage: `sim_node <node id>` - the same invocation shape as the upstream
//! `ot-cli-ftd <node id>` simulation binary, so that the upstream harness can
//! eventually spawn it as a drop-in node. The port base comes from the
//! `PORT_BASE`/`PORT_OFFSET` env vars (harness convention), the Operational
//! Dataset from `SIM_NODE_DATASET` (TLV hex).
//!
//! The node speaks a minimal line protocol on stdout (one token per line,
//! flushed): `ready` once the stack runs, then `role: <role>` on every device
//! role change. Logs go to stderr. A CLI front-end (the upstream C CLI over
//! stdio) is the planned replacement of this protocol; the smoke tests only
//! need role observations.

use embassy_executor::Spawner;

use log::info;

use openthread::{
    EmbassyTimeTimer, MacRadio, MacRadioResources, OpenThread, OtResources, SimpleRamSettings,
};

use openthread_tests::executor::{self, Mode};
use openthread_tests::sim_radio::SimRadio;

use rand::rngs::StdRng;
use rand::SeedableRng;

use static_cell::StaticCell;

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let node_id = std::env::args()
        .nth(1)
        .and_then(|arg| arg.parse::<u16>().ok())
        .expect("usage: sim_node <node id>");

    executor::run(Mode::RealTime, move |spawner| {
        spawner.spawn(main_task(spawner, node_id).unwrap())
    });
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner, node_id: u16) {
    info!("Simulation node {node_id} starting");

    static RNG: StaticCell<StdRng> = StaticCell::new();
    let rng = RNG.init(StdRng::from_os_rng());

    // Deterministic, node-unique EUI64 (the node id in the last two bytes).
    let mut ieee_eui64 = [0x18, 0xb4, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00];
    ieee_eui64[6..].copy_from_slice(&node_id.to_be_bytes());

    static OT_RESOURCES: StaticCell<OtResources> = StaticCell::new();
    static OT_SETTINGS_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static OT_SETTINGS: StaticCell<SimpleRamSettings> = StaticCell::new();

    let ot_resources = OT_RESOURCES.init(OtResources::new());
    let ot_settings_buf = OT_SETTINGS_BUF.init([0; 1024]);
    let ot_settings = OT_SETTINGS.init(SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    // Bare radio: the runner task below adds the `MacRadio` software MAC.
    let radio = SimRadio::new(node_id).expect("create simulation radio");

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    println!("ready");

    if let Ok(dataset) = std::env::var("SIM_NODE_DATASET") {
        info!("Dataset: {dataset}");

        ot.set_active_dataset_tlv_hexstr(&dataset).unwrap();
        ot.enable_ipv6(true).unwrap();
        ot.enable_thread(true).unwrap();
    }

    let mut role = None;
    loop {
        let new_role = ot.device_role();
        if role != Some(new_role) {
            role = Some(new_role);
            println!("role: {new_role:?}");
        }

        ot.wait_changed().await;
    }
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: SimRadio) -> ! {
    static MAC_RADIO_RESOURCES: StaticCell<MacRadioResources> = StaticCell::new();
    let mac_radio_resources = MAC_RADIO_RESOURCES.init(MacRadioResources::new());

    ot.run(MacRadio::new(radio, EmbassyTimeTimer, mac_radio_resources))
        .await
}
