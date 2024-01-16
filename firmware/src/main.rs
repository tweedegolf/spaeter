#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use core::{pin::pin, task::Poll};

use defmt::unwrap;
use defmt_rtt as _;
use embassy_sync::waitqueue::WakerRegistration;
use futures::future::FutureExt;
use panic_probe as _;
use rtic::{app, Mutex};
use rtic_monotonics::systick::{ExtU64, Systick};
use rtic_sync::{channel::Receiver, make_channel};
use smoltcp::iface::{SocketHandle, SocketStorage};
use statime::{filters::BasicFilter, PtpInstance};
use stm32_eth::{dma::PacketId, EthPins, Parts, PartsIn};
use stm32f7xx_hal::{
    gpio::{Output, Pin, Speed},
    prelude::*,
    rng::RngExt,
};

use spaeter_firmware::{
    adc_capture::{AdcCapture, AdcCaptureBuffer},
    ethernet::{
        self, generate_mac_address, recv_slice, DmaResources, NetworkStack, TcpSocketResources,
        UdpSocketResources,
    },
    port::{self, setup_statime, TimerName},
    ptp_clock::{stm_time_to_statime, PtpClock},
};

use defmt::Debug2Format;
use static_cell::StaticCell;
use stm32_eth::ptp::{EthernetPTP, Subseconds, Timestamp};
use stm32f7xx_hal::pac;

defmt::timestamp!("{=u64:iso8601ms}", {
    let time = stm32_eth::ptp::EthernetPTP::get_time();
    time.seconds() as u64 * 1_000 + (time.subseconds().nanos() / 1000000) as u64
});

#[app(device = stm32f7xx_hal::pac, dispatchers = [CAN1_RX0])]
mod app {
    use super::*;

    static ADC_CONVERSION_DATA: StaticCell<AdcCaptureBuffer> = StaticCell::new();

    #[shared]
    struct Shared {
        net: NetworkStack,
        ptp_port: port::Port,
        tx_waker: WakerRegistration,
        adc_capture: AdcCapture,
    }

    #[local]
    struct Local {}

    #[init(local = [
        dma_resources: DmaResources = DmaResources::new(),
        sockets: [SocketStorage<'static>; 8] = [SocketStorage::EMPTY; 8],
        udp_resources: [UdpSocketResources; 2] = [UdpSocketResources::new(); 2],
        tcp_resources: TcpSocketResources = TcpSocketResources::new(),
        dma: core::cell::OnceCell<stm32_eth::dma::EthernetDMA<'static,'static> >  = core::cell::OnceCell::new(),
        mq_buffer: [u8; 256] = [0; 256],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = cx.device;

        defmt::println!("HELLO!!O!O!");

        let mut rcc = p.RCC.constrain();
        // Setup clocks
        let clocks = {
            let clocks = rcc.cfgr.sysclk(216.MHz()).hclk(216.MHz());
            // .pclk1(27.MHz())
            // .pclk2(27.MHz());

            clocks.freeze()
        };
        // Make sure pclk1 and pclk2 run on frequencies that are multiples of each other,
        // so ADC1 and TIM2 run in sync
        let tim2_freq = clocks.timclk1();
        let adc1_freq = clocks.pclk2();
        defmt::println!("Clocks: {:?}", Debug2Format(&clocks));
        assert!(tim2_freq.to_Hz() % adc1_freq.to_Hz() == 0);

        // Setup systick to be used for delays
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, clocks.sysclk().to_Hz(), systick_token);

        // Uncomment to see the statime logs at the cost of quite a bit of extra flash
        // usage
        //
        // log_to_defmt::setup();

        // Setup GPIO
        let (led_pin, pps, eth_pins, mdio, mdc) = {
            let gpioa = p.GPIOA.split();
            let gpiob = p.GPIOB.split();
            let gpioc = p.GPIOC.split();
            let gpiog = p.GPIOG.split();

            let _adc1_in = gpioa.pa3.into_analog();
            let _tim2_it1: Pin<'A', 5, stm32f7xx_hal::gpio::Alternate<1>> =
                gpioa.pa5.into_alternate();

            let led_pin = gpiob.pb7.into_push_pull_output();
            let pps = gpiob.pb5.into_push_pull_output();

            let ref_clk = gpioa.pa1.into_floating_input();
            let crs = gpioa.pa7.into_floating_input();
            let tx_d1 = gpiob.pb13.into_floating_input();
            let rx_d0 = gpioc.pc4.into_floating_input();
            let rx_d1 = gpioc.pc5.into_floating_input();

            let (tx_en, tx_d0) = {
                (
                    gpiog.pg11.into_floating_input(),
                    gpiog.pg13.into_floating_input(),
                )
            };

            let (mdio, mdc) = (
                gpioa.pa2.into_alternate().set_speed(Speed::VeryHigh),
                gpioc.pc1.into_alternate().set_speed(Speed::VeryHigh),
            );

            let eth_pins = EthPins {
                ref_clk,
                crs,
                tx_en,
                tx_d0,
                tx_d1,
                rx_d0,
                rx_d1,
            };

            (led_pin, pps, eth_pins, mdio, mdc)
        };

        defmt::println!("Going to do scary stuff now");

        let adc_conversion_data = ADC_CONVERSION_DATA.init_with(|| core::array::from_fn(|_| 0));
        let adc_capture = AdcCapture::init(
            adc_conversion_data,
            p.ADC1,
            p.TIM2,
            p.DMA2,
            &mut rcc.apb1,
            &mut rcc.apb2,
            &mut rcc.ahb1,
        );
        defmt::println!("👻");

        // Setup Ethernet
        let Parts { dma, mac, mut ptp } = {
            let ethernet = PartsIn {
                dma: p.ETHERNET_DMA,
                mac: p.ETHERNET_MAC,
                mmc: p.ETHERNET_MMC,
                ptp: p.ETHERNET_PTP,
            };

            let DmaResources { rx_ring, tx_ring } = cx.local.dma_resources;

            unwrap!(stm32_eth::new_with_mii(
                ethernet, rx_ring, tx_ring, clocks, eth_pins, mdio, mdc
            )
            .ok())
        };

        cx.local
            .dma
            .set(dma)
            .unwrap_or_else(|_| panic!("Unable to set DMA"));
        let dma = cx.local.dma.get_mut().unwrap();

        defmt::trace!("Enabling DMA interrupts");
        dma.enable_interrupt();

        // Setup PPS
        ptp.enable_pps(pps);
        ptp.set_pps_freq(4);

        // Setup PHY
        ethernet::setup_phy(mac);

        // Setup smoltcp as our network stack
        let mac_address = generate_mac_address();
        let (interface, mut sockets) = ethernet::setup_smoltcp(cx.local.sockets, dma, mac_address);

        // Create sockets
        let [tc_res, g_res] = cx.local.udp_resources;

        let event_socket = ethernet::setup_udp_socket(&mut sockets, tc_res, 319);
        let general_socket = ethernet::setup_udp_socket(&mut sockets, g_res, 320);

        // Setup DHCP. Smoltcp_nal should handle it when polled
        // let dhcp_socket = crate::ethernet::setup_dhcp_socket(&mut sockets);

        // Setup TCP socket for MQTT
        let mqtt_res = cx.local.tcp_resources;
        let mqtt_tcp_socket = ethernet::setup_tcp_socket(&mut sockets, mqtt_res);

        // Setup statime
        let rng = p.RNG.init();
        let (ptp_instance, ptp_port, ptp_clock) = setup_statime(ptp, mac_address, rng);

        // Setup network stack
        let net = NetworkStack::new(
            dma,
            sockets,
            interface,
            ptp_clock,
            cx.local.mq_buffer,
            mqtt_tcp_socket,
            // dhcp_socket,
        );

        // Setup message channels
        type TimerMsg = (TimerName, core::time::Duration);
        let (timer_sender, timer_receiver) = make_channel!(TimerMsg, 4);

        type PacketIdMsg = (statime::port::TimestampContext, PacketId);
        let (packet_id_sender, packet_id_receiver) = make_channel!(PacketIdMsg, 16);

        // Setup context for event handling around the `ptp_port`
        let ptp_port = port::Port::new(
            timer_sender,
            packet_id_sender,
            event_socket,
            general_socket,
            ptp_port,
        );

        // Start tasks
        {
            // Blink LED
            blinky::spawn(led_pin, ptp_clock)
                .unwrap_or_else(|_| defmt::panic!("Failed to start blinky"));

            // Listen on sockets
            event_listen::spawn().unwrap_or_else(|_| defmt::panic!("Failed to start event_listen"));
            general_listen::spawn()
                .unwrap_or_else(|_| defmt::panic!("Failed to start general_listen"));

            // Listen for transmit timestamps
            tx_timestamp_listener::spawn(packet_id_receiver)
                .unwrap_or_else(|_| defmt::panic!("Failed to start send_timestamp_grabber"));

            // Listen for timer events
            statime_timers::spawn(timer_receiver)
                .unwrap_or_else(|_| defmt::panic!("Failed to start timers"));

            // Handle BMCA phase for statime
            instance_bmca::spawn(ptp_instance)
                .unwrap_or_else(|_| defmt::panic!("Failed to start instance bmca"));

            // Poll network interfaces
            poll_smoltcp::spawn().unwrap_or_else(|_| defmt::panic!("Failed to start poll_smoltcp"));

            // Poll mqtt client
            poll_mqtt::spawn().unwrap_or_else(|_| defmt::panic!("Failed to start poll_mqtt"));
        }

        (
            Shared {
                net,
                ptp_port,
                tx_waker: WakerRegistration::new(),
                adc_capture,
            },
            Local {},
        )
    }

    /// Task that runs the BMCA every required interval
    #[task(shared = [net, ptp_port], priority = 1)]
    async fn instance_bmca(
        mut cx: instance_bmca::Context,
        ptp_instance: &'static PtpInstance<BasicFilter>,
    ) {
        let net = &mut cx.shared.net;
        let ptp_port = &mut cx.shared.ptp_port;

        loop {
            // Run the BMCA with our single port
            ptp_port.lock(|ptp_port| {
                ptp_port.perform_bmca(
                    |bmca_port| {
                        ptp_instance.bmca(&mut [bmca_port]);
                    },
                    net,
                );
            });

            // Wait for the given time before running again
            let wait_duration = ptp_instance.bmca_interval();
            Systick::delay((wait_duration.as_millis() as u64).millis()).await;
        }
    }

    /// Task that runs the timers and lets the port handle the expired timers.
    /// The channel is used for resetting the timers (which comes from the port
    /// actions and get sent here).
    #[task(shared = [net, ptp_port], priority = 0)]
    async fn statime_timers(
        mut cx: statime_timers::Context,
        mut timer_resets: Receiver<'static, (TimerName, core::time::Duration), 4>,
    ) {
        let net = &mut cx.shared.net;
        let ptp_port = &mut cx.shared.ptp_port;

        let mut announce_timer_delay = pin!(Systick::delay(24u64.hours()).fuse());
        let mut sync_timer_delay = pin!(Systick::delay(24u64.hours()).fuse());
        let mut delay_request_timer_delay = pin!(Systick::delay(24u64.hours()).fuse());
        let mut announce_receipt_timer_delay = pin!(Systick::delay(24u64.hours()).fuse());
        let mut filter_update_timer_delay = pin!(Systick::delay(24u64.hours()).fuse());

        loop {
            futures::select_biased! {
                _ = announce_timer_delay => {
                    ptp_port.lock(|port| port.handle_timer(TimerName::Announce, net));
                }
                _ = sync_timer_delay => {
                    ptp_port.lock(|port| port.handle_timer(TimerName::Sync, net));
                }
                _ = delay_request_timer_delay => {
                    ptp_port.lock(|port| port.handle_timer(TimerName::DelayRequest, net));
                }
                _ = announce_receipt_timer_delay => {
                    ptp_port.lock(|port| port.handle_timer(TimerName::AnnounceReceipt, net));
                }
                _ = filter_update_timer_delay => {
                    ptp_port.lock(|port| port.handle_timer(TimerName::FilterUpdate, net));
                }
                reset = timer_resets.recv().fuse() => {
                    let (timer, delay_time) = unwrap!(reset.ok());

                    let delay = match timer {
                        TimerName::Announce => &mut announce_timer_delay,
                        TimerName::Sync => &mut sync_timer_delay,
                        TimerName::DelayRequest => &mut delay_request_timer_delay,
                        TimerName::AnnounceReceipt => &mut announce_receipt_timer_delay,
                        TimerName::FilterUpdate => &mut filter_update_timer_delay,
                    };

                    delay.set(Systick::delay((delay_time.as_millis() as u64).millis()).fuse());
                }
            }
        }
    }

    /// Listen for new transmission timestamps
    ///
    /// This waits for new packet IDs of send packets for which a timestamp
    /// should be collected and fetches them from the ethernet peripheral. In
    /// case the packet ID is not known yet it will retry a few times to handle
    /// the case where a packet is not send directly (e.g. because ARP is
    /// fetching the receivers mac address).
    #[task(shared = [net, ptp_port, tx_waker], priority = 0)]
    async fn tx_timestamp_listener(
        mut cx: tx_timestamp_listener::Context,
        mut packet_id_receiver: Receiver<'static, (statime::port::TimestampContext, PacketId), 16>,
    ) {
        // Extract state to keep code more readable
        let tx_waker = &mut cx.shared.tx_waker;
        let net = &mut cx.shared.net;
        let ptp_port = &mut cx.shared.ptp_port;

        loop {
            // Wait for the next (smoltcp) packet id and its (statime) timestamp context
            let (timestamp_context, packet_id) = unwrap!(packet_id_receiver.recv().await.ok());

            // We try a limited amount of times since the queued packet might not be sent
            // first (e.g. in case ARP needs to run first)
            let mut tries = 10;

            let timestamp = core::future::poll_fn(|ctx| {
                // Register to wake up after every tx packet has been sent
                tx_waker.lock(|tx_waker| tx_waker.register(ctx.waker()));

                // Keep polling as long as we have tries left
                match net.lock(|net| net.nal().device().poll_tx_timestamp(&packet_id)) {
                    Poll::Ready(Ok(ts)) => Poll::Ready(ts),
                    Poll::Ready(Err(_)) | Poll::Pending => {
                        if tries > 0 {
                            tries -= 1;
                            Poll::Pending
                        } else {
                            Poll::Ready(None)
                        }
                    }
                }
            })
            .await;

            match timestamp {
                Some(timestamp) => ptp_port.lock(|port| {
                    // Inform statime about the timestamp we collected
                    port.handle_send_timestamp(
                        timestamp_context,
                        stm_time_to_statime(timestamp),
                        net,
                    );
                }),
                None => defmt::error!("Failed to get timestamp for packet id {}", packet_id),
            }
        }
    }

    /// Hello world blinky
    ///
    /// Blinks the blue LED on the Nucleo board to indicate that the program is
    /// running
    #[task(priority = 0)]
    async fn blinky(
        _cx: blinky::Context,
        mut led: Pin<'B', 7, Output>,
        ptp_clock: &'static PtpClock,
    ) {
        const PTP_SYNC_INTERVAL: Timestamp = Timestamp::new(
            false,
            0,
            match Subseconds::new_from_nanos(1000 * 1000 * 500) {
                Some(s) => s,
                None => unreachable!(),
            },
        );
        // defmt::println!("PTP now: {}; target: {}", now, ptp_target_time);
        loop {
            let now = EthernetPTP::now();
            let ptp_target_time = now + PTP_SYNC_INTERVAL;
            ptp_clock.access(|clock| clock.configure_target_time_interrupt(ptp_target_time));
            let dma2 = unsafe { &*pac::DMA2::ptr() };
            let lisr = dma2.lisr.read().bits();
            let cfg = dma2.st[0].cr.read().bits();
            let ndtr = dma2.st[0].ndtr.read().bits();
            // defmt::info!(
            //     "cfg={=u32:032b}\tndtr={=u32}\tlisr={=u32:032b}",
            //     cfg,
            //     ndtr,
            //     lisr
            // );
            // unsafe{&*pac::TIM2::ptr()}.egr.write(|w| w.cc1g().trigger());
            Systick::delay(500u64.millis()).await;
            led.set_high();
            Systick::delay(500u64.millis()).await;
            led.set_low();
        }
    }

    /// Listen for packets on the event udp socket
    #[task(shared = [net, ptp_port], priority = 1)]
    async fn event_listen(mut cx: event_listen::Context) {
        let socket = cx.shared.ptp_port.lock(|ptp_port| ptp_port.event_socket());

        listen_and_handle::<true>(&mut cx.shared.net, socket, &mut cx.shared.ptp_port).await
    }

    /// Listen for packets on the general udp socket
    #[task(shared = [net, ptp_port], priority = 0)]
    async fn general_listen(mut cx: general_listen::Context) {
        let socket = cx
            .shared
            .ptp_port
            .lock(|ptp_port| ptp_port.general_socket());

        listen_and_handle::<false>(&mut cx.shared.net, socket, &mut cx.shared.ptp_port).await
    }

    /// Listen for packets on the given socket
    ///
    /// The handling for both event and general sockets is basically the
    /// same and only differs in which `handle_*` function needs to be
    /// called.
    async fn listen_and_handle<const IS_EVENT: bool>(
        net: &mut impl Mutex<T = NetworkStack>,
        socket: SocketHandle,
        port: &mut impl Mutex<T = port::Port>,
    ) {
        // Get a local buffer to store the received packet
        // This is needed because we want to send and receive on the same socket at the
        // same time which both requires a `&mut` to the socket.
        let mut buffer = [0u8; 1500];
        loop {
            // Receive the next packet into the buffer
            let (len, timestamp) = match recv_slice(net, socket, &mut buffer).await {
                Ok(ok) => ok,
                Err(e) => {
                    defmt::error!("Failed to receive a packet because: {}", e);
                    continue;
                }
            };
            let data = &buffer[..len];

            // Inform statime about the new packet
            port.lock(|port| {
                if IS_EVENT {
                    port.handle_event_receive(data, stm_time_to_statime(timestamp), net);
                } else {
                    port.handle_general_receive(data, net);
                };
            });
        }
    }

    /// Poll smoltcp
    ///
    /// Smoltcp needs to be regularly polled to handle its state machines
    /// So we poll it after the delay it indicates.
    #[task(shared = [net], priority = 0)]
    async fn poll_smoltcp(mut cx: poll_smoltcp::Context) {
        loop {
            // Let smoltcp handle its things
            let delay_millis = cx
                .shared
                .net
                .lock(|net| {
                    net.poll();
                    net.poll_delay().map(|d| d.total_millis())
                })
                .unwrap_or(50);

            // TODO this could wait longer if we were notified about any other calls to
            // poll, would be an optimization for later to go to sleep longer
            Systick::delay(delay_millis.millis()).await;
        }
    }

    #[task(shared = [net], priority = 0)]
    async fn poll_mqtt(mut cx: poll_mqtt::Context) {
        loop {
            let did_recv = cx.shared.net.lock(|net| {
                defmt::info!(
                    "Polling MQTT. Connected: {}",
                    net.mqtt
                        .client()
                        .is_connected()
                        .then_some("✅")
                        .unwrap_or("❌")
                );
                match net.mqtt.poll(|_, topic, payload, properties| {
                    defmt::info!(
                        "Received MQTT message. Topic: {}, payload: '{}', properties: {:?}",
                        topic,
                        core::str::from_utf8(payload).unwrap_or("<GIBBERISH>"),
                        Debug2Format(&properties)
                    );
                }) {
                    Ok(res) => res.is_some(),
                    Err(e) => {
                        defmt::error!("MQTT Poll error: {:?}", Debug2Format(&e));
                        false
                    }
                }
            });
            if !did_recv {
                Systick::delay(500u64.millis()).await;
            }
        }
    }

    #[task(binds = DMA2_STREAM0, shared = [ adc_capture ], priority = 1)]
    fn on_dma2_stream0(mut cx: on_dma2_stream0::Context) {
        cx.shared
            .adc_capture
            .lock(|capture| capture.dma_interrupt_handler());

        // TODO: Handle new data
    }

    #[task(binds = TIM2, priority = 1)]
    fn on_tim2_update(mut cx: on_tim2_update::Context) {
        let tim2 = unsafe { &*pac::TIM2::ptr() };
        let sr = tim2.sr.read();
        tim2.sr.write(|w| unsafe { w.bits(!sr.bits()) });

        // if sr.cc1if().bit_is_set() {
        // }

        let cc1 = tim2.ccr1.read().ccr().bits();
        defmt::println!("TIM2 CC1: {}", cc1);
        defmt::println!("TIM2 {=u32:032b}!", sr.bits());
    }

    #[task(binds = ADC, priority = 1)]
    fn on_adc1_conversion(mut cx: on_adc1_conversion::Context) {
        let adc1 = unsafe { &*pac::ADC1::ptr() };
        let sr = adc1.sr.read().bits();
        defmt::println!("ADC1 SR: {=u32:032b}", sr);
        adc1.sr
            .modify(|_, w| w.eoc().not_complete().strt().not_started());

        let overrun = adc1.sr.read().ovr().bit_is_set();
        if overrun {
            defmt::println!("ADC! Overrun: {}", overrun);
            adc1.sr.modify(|_, w| w.ovr().clear_bit());
        }
        // defmt::println!("ADC1");
    }

    /// Handle the interrupt of the ethernet peripheral
    #[task(binds = ETH, shared = [net, tx_waker], priority = 2)]
    fn eth_interrupt(mut cx: eth_interrupt::Context) {
        let reason = stm32_eth::eth_interrupt_handler();

        // Receiving a tx event wakes the task waiting for tx timestamps
        if reason.tx {
            cx.shared.tx_waker.lock(|tx_waker| tx_waker.wake());
        }

        if reason.time_passed {
            defmt::info!("Timestamp trigger !!@#!@#");
        }

        // Let smoltcp handle any new packets
        cx.shared.net.lock(|net| {
            net.poll();
        });
    }
}
