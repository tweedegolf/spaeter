use core::task::Poll;

use crate::statime_wrapper::PtpClock;
use defmt::unwrap;
use futures::future::poll_fn;
use ieee802_3_miim::{
    phy::{PhySpeed, LAN8742A},
    Phy,
};
use minimq::{broker::IpBroker, embedded_nal, Minimq};
use rtic::Mutex;
use rtic_monotonics::{systick::Systick, Monotonic};
use smoltcp::{
    iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage},
    socket::{dhcpv4, tcp, udp},
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
};
use stm32_eth::{
    dma::{EthernetDMA, PacketId, PacketIdNotFound, RxRingEntry, TxRingEntry},
    mac,
    mac::{EthernetMACWithMii, MdcPin, MdioPin},
    ptp::Timestamp,
};
use stm32f7xx_hal::signature::Uid;

pub struct DmaResources {
    pub rx_ring: [RxRingEntry; 2],
    pub tx_ring: [TxRingEntry; 2],
}

impl DmaResources {
    pub const fn new() -> Self {
        Self {
            rx_ring: [RxRingEntry::new(), RxRingEntry::new()],
            tx_ring: [TxRingEntry::new(), TxRingEntry::new()],
        }
    }
}

#[derive(Copy, Clone)]
pub struct UdpSocketResources {
    pub rx_meta_storage: [udp::PacketMetadata; 8],
    pub rx_payload_storage: [u8; 8192],
    pub tx_meta_storage: [udp::PacketMetadata; 8],
    pub tx_payload_storage: [u8; 8192],
}

impl UdpSocketResources {
    pub const fn new() -> Self {
        Self {
            rx_meta_storage: [udp::PacketMetadata::EMPTY; 8],
            rx_payload_storage: [0; 8192],
            tx_meta_storage: [udp::PacketMetadata::EMPTY; 8],
            tx_payload_storage: [0; 8192],
        }
    }
}

#[derive(Copy, Clone)]
pub struct TcpSocketResources {
    pub rx_payload_storage: [u8; 8192],
    pub tx_payload_storage: [u8; 8192],
}

impl TcpSocketResources {
    pub const fn new() -> Self {
        Self {
            rx_payload_storage: [0; 8192],
            tx_payload_storage: [0; 8192],
        }
    }
}

pub type Nal = smoltcp_nal::NetworkStack<
    'static,
    &'static mut EthernetDMA<'static, 'static>,
    &'static PtpClock,
>;

pub type MiniMq = Minimq<'static, Nal, &'static PtpClock, IpBroker>;

pub struct NetworkStack {
    pub mqtt: MiniMq,
}

impl NetworkStack {
    pub fn new(
        dma: &'static mut EthernetDMA<'static, 'static>,
        sockets: SocketSet<'static>,
        interface: Interface,
        ptp_clock: &'static PtpClock,
        mq_buffer: &'static mut [u8],
        _mqtt_tcp_socket: SocketHandle,
        // _dhcp_socket: SocketHandle,
    ) -> Self {
        let nal_stack = smoltcp_nal::NetworkStack::new(interface, dma, sockets, ptp_clock);

        let mq_broker_addr = embedded_nal::IpAddr::V4(embedded_nal::Ipv4Addr::new(10, 0, 0, 1));
        let mqtt: Minimq<
            '_,
            smoltcp_nal::NetworkStack<'_, &mut EthernetDMA<'_, '_>, &PtpClock>,
            &PtpClock,
            _,
        > = Minimq::new(
            nal_stack,
            ptp_clock,
            minimq::ConfigBuilder::new(mq_broker_addr.into(), mq_buffer)
                .client_id("spaeter")
                .unwrap(),
        );

        Self { mqtt }
    }

    pub fn nal(&mut self) -> &mut Nal {
        self.mqtt.client().stack_mut()
    }

    pub fn poll(&mut self) {
        self.nal().poll().unwrap();
    }

    pub fn poll_delay(&mut self) -> Option<smoltcp::time::Duration> {
        self.nal().smoltcp_poll_delay(now())
    }
}

fn now() -> smoltcp::time::Instant {
    let now_millis = Systick::now().ticks();
    // TODO handle case where systick is not 1kHz
    smoltcp::time::Instant::from_millis(i64::try_from(now_millis).unwrap())
}

/// Initialize the PHY, wait for link and set the speed
///
/// This function will *block* until the ethernet link is up!
pub fn setup_phy<MDIO: MdioPin, MDC: MdcPin>(mac: EthernetMACWithMii<MDIO, MDC>) {
    // Setup PHY
    let mut phy = LAN8742A::new(mac, 0);

    phy.phy_init();

    defmt::info!("Waiting for link up.");

    while !phy.phy_link_up() {}

    defmt::info!("Link up.");

    if let Some(speed) = phy.link_speed().map(|s| match s {
        PhySpeed::HalfDuplexBase10T => mac::Speed::HalfDuplexBase10T,
        PhySpeed::FullDuplexBase10T => mac::Speed::FullDuplexBase10T,
        PhySpeed::HalfDuplexBase100Tx => mac::Speed::HalfDuplexBase100Tx,
        PhySpeed::FullDuplexBase100Tx => mac::Speed::FullDuplexBase100Tx,
    }) {
        phy.get_miim().set_speed(speed);
        defmt::info!("Detected link speed: {}", speed);
    } else {
        defmt::warn!("Failed to detect link speed.");
    }
}

pub fn setup_smoltcp(
    sockets: &'static mut [SocketStorage],
    mut dma: &mut EthernetDMA,
    mac_address: [u8; 6],
) -> (Interface, SocketSet<'static>) {
    // Setup smoltcp
    let cfg = Config::new(EthernetAddress(mac_address).into());

    let mut interface = Interface::new(cfg, &mut dma, smoltcp::time::Instant::ZERO);

    interface.update_ip_addrs(|a| {
        unwrap!(a.push(IpCidr::new(IpAddress::v4(10, 0, 0, 2), 24)));
    });

    unwrap!(interface.join_multicast_group(
        &mut dma,
        Ipv4Address::new(224, 0, 1, 129),
        smoltcp::time::Instant::ZERO
    ));
    unwrap!(interface.join_multicast_group(
        &mut dma,
        Ipv4Address::new(224, 0, 0, 107),
        smoltcp::time::Instant::ZERO
    ));

    defmt::info!("Set IPs to: {}", interface.ip_addrs());

    // Register socket
    let sockets = SocketSet::new(sockets);

    (interface, sockets)
}

pub fn setup_udp_socket(
    socket_set: &mut SocketSet,
    resources: &'static mut UdpSocketResources,
    port: u16,
) -> SocketHandle {
    let UdpSocketResources {
        rx_meta_storage,
        rx_payload_storage,
        tx_meta_storage,
        tx_payload_storage,
    } = resources;

    let rx_buffer = udp::PacketBuffer::new(&mut rx_meta_storage[..], &mut rx_payload_storage[..]);
    let tx_buffer = udp::PacketBuffer::new(&mut tx_meta_storage[..], &mut tx_payload_storage[..]);
    let mut socket = udp::Socket::new(rx_buffer, tx_buffer);
    unwrap!(socket.bind(port));

    socket_set.add(socket)
}

pub fn setup_tcp_socket(
    socket_set: &mut SocketSet,
    resources: &'static mut TcpSocketResources,
) -> SocketHandle {
    let TcpSocketResources {
        rx_payload_storage,
        tx_payload_storage,
    } = resources;

    let rx_buffer = tcp::SocketBuffer::new(&mut rx_payload_storage[..]);
    let tx_buffer = tcp::SocketBuffer::new(&mut tx_payload_storage[..]);
    let socket = tcp::Socket::new(rx_buffer, tx_buffer);

    socket_set.add(socket)
}

pub fn setup_dhcp_socket(socket_set: &mut SocketSet) -> SocketHandle {
    let dhcp_socket = dhcpv4::Socket::new();
    socket_set.add(dhcp_socket)
}

pub async fn recv_slice(
    net: &mut impl Mutex<T = NetworkStack>,
    mut socket: SocketHandle,
    buffer: &mut [u8],
) -> Result<(usize, Timestamp), RecvError> {
    poll_fn(|_cx| {
        let result = net.lock(|net| {
            // Get next packet (if any)

            let (len, meta) = net.nal().smoltcp_recv_udp(&mut socket, buffer)?;
            // Get the timestamp
            let packet_id = PacketId::from(meta.meta);
            let timestamp = match net.nal().device().rx_timestamp(&packet_id) {
                Ok(Some(ts)) => ts,
                Ok(None) => return Err(RecvError::NoTimestampRecorded),
                Err(e) => return Err(e.into()),
            };

            // Return the buffer length and timestamp
            Ok((len, timestamp))
        });

        match result {
            Ok(r) => Poll::Ready(Ok(r)),
            Err(RecvError::Exhausted) => Poll::Pending,
            e @ Err(_) => Poll::Ready(e),
        }
    })
    .await
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum RecvError {
    Exhausted,
    PacketIdNotFound(PacketIdNotFound),
    NoTimestampRecorded,
}

impl From<PacketIdNotFound> for RecvError {
    fn from(value: PacketIdNotFound) -> Self {
        Self::PacketIdNotFound(value)
    }
}

impl From<udp::RecvError> for RecvError {
    fn from(value: udp::RecvError) -> Self {
        match value {
            udp::RecvError::Exhausted => Self::Exhausted,
        }
    }
}

/// Generate a mac based on the UID of the chip.
///
/// *Note: This is not the proper way to do it.
/// You're supposed to buy a mac address or buy a phy that includes a mac and
/// use that one*
pub fn generate_mac_address() -> [u8; 6] {
    let mut hasher = adler::Adler32::new();

    // Form the basis of our OUI octets
    let bin_name = env!("CARGO_CRATE_NAME").as_bytes();
    hasher.write_slice(bin_name);
    let oui = hasher.checksum().to_ne_bytes();

    // Form the basis of our NIC octets
    let uid: [u8; 12] =
        unsafe { core::mem::transmute_copy::<_, [u8; core::mem::size_of::<Uid>()]>(Uid::get()) };
    hasher.write_slice(&uid);
    let nic = hasher.checksum().to_ne_bytes();

    // To make it adhere to EUI-48, we set it to be a unicast locally administered
    // address
    [
        oui[0] & 0b1111_1100 | 0b0000_0010,
        oui[1],
        oui[2],
        nic[0],
        nic[1],
        nic[2],
    ]
}
