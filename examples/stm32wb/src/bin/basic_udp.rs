#![no_std]
#![no_main]

use core::net::{Ipv6Addr, SocketAddrV6};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::ipcc::{Config, ReceiveInterruptHandler, TransmitInterruptHandler};
use embassy_stm32::peripherals::RNG;
use embassy_stm32::rcc::Config as RccConfig;
use embassy_stm32::rng::Rng;
use embassy_stm32::{bind_interrupts, rng};
use embassy_stm32_wpan::TlMbox;
use embassy_stm32_wpan::sub::thread::Runner;
use embassy_stm32_wpan::sub::{mm, thread};
use openthread::{BytesFmt, OpenThread, OtResources, OtUdpResources, SimpleRamSettings, UdpSocket};
use rand_core::RngCore;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs{
    IPCC_C1_RX => ReceiveInterruptHandler;
    IPCC_C1_TX => TransmitInterruptHandler;
    RNG => rng::InterruptHandler<RNG>;
});

#[embassy_executor::task]
async fn run_mm_queue(mut memory_manager: mm::MemoryManager<'static>) {
    memory_manager.run_queue().await;
}

#[embassy_executor::task]
async fn run_wpan_thread(mut thread: thread::Runner<'static>) {
    thread.run_stack().await;
}

use tinyrlibc as _;

macro_rules! mk_static {
    ($t:ty) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit();
        x
    }};
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

const BOUND_PORT: u16 = 1212;

const UDP_SOCKETS_BUF: usize = 1280;
const UDP_MAX_SOCKETS: usize = 2;

const THREAD_DATASET: &str = if let Some(dataset) = option_env!("THREAD_DATASET") {
    dataset
} else {
    "0e080000000000010000000300000b35060004001fffe002083a90e3a319a904940708fd1fa298dbd1e3290510fe0458f7db96354eaa6041b880ea9c0f030f4f70656e5468726561642d35386431010258d10410888f813c61972446ab616ee3c556a5910c0402a0f7f8"
};

#[embassy_executor::main(
    executor = "embassy_stm32::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(spawner: Spawner) {
    /*
        How to make this work:

        - Obtain a NUCLEO-STM32WB55 from your preferred supplier.
        - Download and Install STM32CubeProgrammer.
        - Download stm32wb5x_FUS_fw.bin, stm32wb5x_Thread_MTD_fw.bin, and Release_Notes.html from
          gh:STMicroelectronics/STM32CubeWB@2234d97/Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x
        - Open STM32CubeProgrammer
        - On the right-hand pane, click "firmware upgrade" to upgrade the st-link firmware.
        - Once complete, click connect to connect to the device.
        - On the left hand pane, click the RSS signal icon to open "Firmware Upgrade Services".
        - In the Release_Notes.html, find the memory address that corresponds to your device for the stm32wb5x_FUS_fw.bin file
        - Select that file, the memory address, "verify download", and then "Firmware Upgrade".
        - Once complete, in the Release_Notes.html, find the memory address that corresponds to your device for the
          stm32wb5x_BLE_Mac_802_15_4_fw.bin file. It should not be the same memory address.
        - Select that file, the memory address, "verify download", and then "Firmware Upgrade".
        - Select "Start Wireless Stack".
        - Disconnect from the device.
        - Run this example.

        Note: extended stack versions are not supported at this time. Do not attempt to install a stack with "extended" in the name.
    */

    let mut config = embassy_stm32::Config::default();
    config.rcc = RccConfig::new_wpan();
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    let config = Config::default();
    let (thread, mm) = TlMbox::wait_ready(p.IPCC, Irqs, config)
        .await
        .unwrap()
        .init_thread()
        .await
        .unwrap();

    let runner = Runner::new(thread);

    spawner.spawn(run_mm_queue(mm).unwrap());

    spawner.spawn(run_wpan_thread(runner).unwrap());

    info!("Starting...");

    let rng = mk_static!(Rng<'static, RNG>, Rng::new(p.RNG, Irqs));

    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(rng, &mut ieee_eui64);

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_udp_resources =
        mk_static!(OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>, OtUdpResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);

    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new_with_udp(ieee_eui64, rng, ot_settings, ot_resources, ot_udp_resources)
        .unwrap();

    info!("About to spawn OT IP info");

    spawner.spawn(run_ot_ip_info(ot.clone()).unwrap());

    info!("Dataset: {}", THREAD_DATASET);

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    let socket = UdpSocket::bind(
        ot,
        &SocketAddrV6::new(Ipv6Addr::UNSPECIFIED, BOUND_PORT, 0, 0),
    )
    .unwrap();

    info!(
        "Opened socket on port {} and waiting for packets...",
        BOUND_PORT
    );

    let buf: &mut [u8] = unsafe { mk_static!([u8; UDP_SOCKETS_BUF]).assume_init_mut() };

    loop {
        let (len, local, remote) = socket.recv(buf).await.unwrap();

        info!("Got {} from {} on {}", BytesFmt(&buf[..len]), remote, local);

        socket.send(b"Hello", Some(&local), &remote).await.unwrap();
        info!("Sent `b\"Hello\"`");
    }
}

#[embassy_executor::task]
async fn run_ot_ip_info(ot: OpenThread<'static>) -> ! {
    let mut cur_addrs = heapless::Vec::<(Ipv6Addr, u8), 4>::new();

    loop {
        let mut addrs = heapless::Vec::<(Ipv6Addr, u8), 4>::new();
        ot.ipv6_addrs(|addr| {
            if let Some(addr) = addr {
                let _ = addrs.push(addr);
            }

            Ok(())
        })
        .unwrap();

        if cur_addrs != addrs {
            info!("Got new IPv6 address(es) from OpenThread: {:?}", addrs);

            cur_addrs = addrs;

            info!("Waiting for OpenThread changes signal...");
        }

        ot.wait_changed().await;
    }
}
