#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
// This lint produces false positives in this project with the nightly-2023-09-19 compiler
#![allow(clippy::needless_pass_by_ref_mut)]

use core::{pin::pin, task::Poll};

use defmt::unwrap;
use defmt_rtt as _;
use embassy_sync::waitqueue::WakerRegistration;
use ethernet::{DmaResources, NetworkStack};
use futures::future::FutureExt;
use panic_probe as _;
use rtic::{app, Mutex};
use rtic_monotonics::systick::{ExtU64, Systick};
use rtic_sync::{channel::Receiver, make_channel};
use smoltcp::{
    iface::{SocketHandle, SocketStorage},
    socket::dhcpv4,
    wire::{IpCidr, Ipv4Address, Ipv4Cidr},
};
use statime::{BasicFilter, PtpInstance};
use stm32_eth::{dma::PacketId, EthPins, Parts, PartsIn};
use stm32f7xx_hal::{
    gpio::{Output, Pin, Speed},
    prelude::*,
    rng::RngExt,
};

use crate::{
    ethernet::{generate_mac_address, recv_slice, UdpSocketResources},
    port::setup_statime,
    ptp_clock::stm_time_to_statime,
};

mod ethernet;
mod port;
mod ptp_clock;

defmt::timestamp!("{=u64:iso8601ms}", {
    let time = stm32_eth::ptp::EthernetPTP::get_time();
    time.seconds() as u64 * 1_000 + (time.subseconds().nanos() / 1000000) as u64
});

#[app(device = stm32f7xx_hal::pac, dispatchers = [CAN1_RX0])]
mod app {
    use defmt::Debug2Format;
    use stm32_eth::ptp::{EthernetPTP, Subseconds, Timestamp};
    use stm32f7xx_hal::{gpio::Input, pac, rcc::Reset};

    use super::*;
    use crate::{port::TimerName, ptp_clock::PtpClock};

    const DMA_CHUNK_NUM_CONVERSIONS: u16 = 1024 * 8;
    const DMA_CHUNK_SIZE: usize = 4;
    static mut ADC_CONVERSION_DATA: [[u16; DMA_CHUNK_NUM_CONVERSIONS as usize]; DMA_CHUNK_SIZE] =
        [[0; DMA_CHUNK_NUM_CONVERSIONS as usize]; DMA_CHUNK_SIZE];

    #[shared]
    struct Shared {
        net: NetworkStack,
        ptp_port: port::Port,
        tx_waker: WakerRegistration,
    }

    #[local]
    struct Local {}

    #[init(local = [
        dma_resources: DmaResources = DmaResources::new(),
        sockets: [SocketStorage<'static>; 8] = [SocketStorage::EMPTY; 8],
        udp_resources: [UdpSocketResources; 2] = [UdpSocketResources::new(); 2]
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = cx.device;

        // Enable TIM2, ADC1 and DMA2 clocks
        p.RCC.apb1enr.modify(|_, w| w.tim2en().enabled());
        p.RCC.apb2enr.modify(|_, w| w.adc1en().enabled());
        p.RCC.ahb1enr.modify(|_, w| w.dma2en().enabled());

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
        // Setup ADC1 and DMA2
        {
            // let p = unsafe { pac::Peripherals::steal() };

            /*
               Configure DMA2 Stream 0 to read 16-bit conversions from ADC1
               and write them into ADC_CONVERSION_DATA
               in double-buffer mode
            */

            // Disable DMA2 Stream 0
            p.DMA2.st[0].cr.modify(|_, w| w.en().disabled());

            // Choose channel 0 for DMA2 Stream 0
            p.DMA2.st[0].cr.modify(|_, w| {
                // Select channel 0 (ADC1)
                w.chsel()
                    .bits(0)
                    // Enable Double-buffer mode
                    .dbm()
                    .enabled()
                    // Disable circular mode
                    .circ()
                    .disabled()
                    // Set data size to 16 bits at memory side
                    .msize()
                    .bits16()
                    // Set data size to 16 bits at peripheral side
                    .psize()
                    .bits16()
                    // Increment memory pointer after each read
                    .minc()
                    .incremented()
                    // Do not increment peripheral data pointer
                    .pinc()
                    .fixed()
                    // Write from peripheral to memory
                    .dir()
                    .peripheral_to_memory()
                    // DMA controls when transfer ends (which is never due to circular mode)
                    .pfctrl()
                    .dma()
                    // Enable Transfer Complete Interrupt
                    .tcie()
                    .enabled()
                    // Enable Transfer Error Interrupt
                    .teie()
                    .enabled()
                    // Enable Direct Mode Error Interrupt
                    .dmeie()
                    .enabled()
                    // Select Memory 0 to start
                    .ct()
                    .memory0()
            });

            // Set buffer size
            p.DMA2.st[0]
                .ndtr
                .modify(|_, w| w.ndt().bits(DMA_CHUNK_NUM_CONVERSIONS));

            // Set peripheral address to ADC1 data register
            p.DMA2.st[0]
                .par
                .write(|w| unsafe { w.pa().bits(p.ADC1.dr.as_ptr() as u32) });

            // Point DMA Memory 0 to first chunk of ADC_CONVERSION_DATA
            p.DMA2.st[0]
                .m0ar
                .write(|w| unsafe { w.m0a().bits(ADC_CONVERSION_DATA[0].as_ptr() as u32) });
            // Point DMA Memory 1 to second chunk of ADC_CONVERSION_DATA
            p.DMA2.st[0]
                .m1ar
                .write(|w| unsafe { w.m1a().bits(ADC_CONVERSION_DATA[1].as_ptr() as u32) });

            // Enable DMA2 Stream 0
            p.DMA2.st[0].cr.modify(|_, w| w.en().enabled());

            /*
               Configure ADC1 to 12-bits resolution in
               single conversion mode and to be triggered externally from TIM2 TRGO
               and read out using DMA
            */

            // Power down ADC1
            p.ADC1.cr2.modify(|_, w| w.adon().clear_bit());

            // Reset ADC1
            let adc1 = p.ADC1;
            <pac::ADC1 as Reset>::reset(&mut rcc.apb2);

            // Setup ADC1 for contonuous conversion mode
            adc1.cr2.modify(|_, w| w.cont().continuous());
            adc1.cr1
                .modify(|_, w| w.scan().clear_bit().discen().clear_bit());

            // Setup ADC1 for external triggering by TIM2 TRGO
            adc1.cr2
                .modify(|_, w| unsafe { w.exten().rising_edge().extsel().bits(0b1011) });

            // Setup ADC1 resolution to 12 bit
            adc1.cr1.modify(|_, w| w.res().bits(0b00));

            // Enable DMA on ADC1
            adc1.cr2.modify(|_, w| w.dma().enabled().dds().continuous());

            // Enable ADC end-of-conversion interrupt
            adc1.cr1
                .modify(|_, w| w.eocie().disabled().ovrie().enabled());

            // Use PA3 as input
            adc1.sqr3.modify(|_, w| unsafe { w.sq1().bits(3) });

            // Power up ADC1
            adc1.cr2.modify(|_, w| w.adon().enabled());

            /*
                Setup TIM2 to trigger ADC1 using TRGO on update event generation
            */
            // Set Master mode trigger on timer enable, which will enable ADC1 as well
            p.TIM2.cr2.modify(|_, w| w.mms().enable());
            // ARR resets to u32::MAX
            p.TIM2.arr.write(|w| w.arr().bits(u32::MAX));

            // Enable TIM2 CC1 capture interrupt as well as update interrupt to detect overflows
            p.TIM2
                .dier
                .modify(|_, w| w.uie().enabled().cc1ie().enabled());

            // Connect PTP to TIM2 ITR1
            p.TIM2.or.write(|w| unsafe { w.itr1_rmp().bits(0b01) });
            // Connect TIM2 ITR1 to ITR
            p.TIM2.smcr.modify(|_, w| w.ts().itr1());

            // Configures TIM2 CC1 to capture the timer value on ITR
            p.TIM2
                .ccmr1_input()
                .modify(|_, w| unsafe { w.cc1s().trc().ic1f().no_filter().ic1psc().bits(0b00) });

            // Enable TIM2
            p.TIM2.cr1.modify(|_, w| w.cen().enabled());

            p.TIM2
                .ccer
                .modify(|_, w| w.cc1p().clear_bit().cc1np().clear_bit());
            p.TIM2.cr2.modify(|_, w| w.ti1s().normal());

            // Enable CC1
            p.TIM2.ccer.modify(|_, w| w.cc1e().set_bit());
        }
        defmt::println!("👻");

        // Setup Ethernet
        let Parts {
            mut dma,
            mac,
            mut ptp,
        } = {
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

        defmt::trace!("Enabling DMA interrupts");
        dma.enable_interrupt();

        // Setup PPS
        ptp.enable_pps(pps);
        ptp.set_pps_freq(4);

        // Setup PHY
        crate::ethernet::setup_phy(mac);

        // Setup smoltcp as our network stack
        let mac_address = generate_mac_address();
        let (interface, mut sockets) =
            crate::ethernet::setup_smoltcp(cx.local.sockets, &mut dma, mac_address);

        // Create sockets
        let [tc_res, g_res] = cx.local.udp_resources;

        let event_socket = crate::ethernet::setup_udp_socket(&mut sockets, tc_res, 319);
        let general_socket = crate::ethernet::setup_udp_socket(&mut sockets, g_res, 320);

        // Setup DHCP
        let dhcp_socket = crate::ethernet::setup_dhcp_socket(&mut sockets);

        let net = NetworkStack {
            dma,
            iface: interface,
            sockets,
        };

        // Setup statime
        let rng = p.RNG.init();
        let (ptp_instance, ptp_port, ptp_clock) = setup_statime(ptp, mac_address, rng);

        // Setup message channels
        type TimerMsg = (TimerName, core::time::Duration);
        let (timer_sender, timer_receiver) = make_channel!(TimerMsg, 4);

        type PacketIdMsg = (statime::TimestampContext, PacketId);
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

            // Poll network interfaces and run DHCP
            poll_smoltcp::spawn().unwrap_or_else(|_| defmt::panic!("Failed to start poll_smoltcp"));
            // dhcp::spawn(dhcp_socket).unwrap_or_else(|_| defmt::panic!("Failed to start dhcp"));
        }

        (
            Shared {
                net,
                ptp_port,
                tx_waker: WakerRegistration::new(),
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
        mut packet_id_receiver: Receiver<'static, (statime::TimestampContext, PacketId), 16>,
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
                match net.lock(|net| net.dma.poll_tx_timestamp(&packet_id)) {
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

    #[task(binds = DMA2_STREAM0, priority = 1)]
    fn on_dma2_stream0(mut cx: on_dma2_stream0::Context) {
        let dma2 = unsafe { &*pac::DMA2::ptr() };
        let lisr = dma2.lisr.read().bits();
        dma2.lifcr.write(|w| unsafe { w.bits(lisr) });

        let cfg = dma2.st[0].cr.read().bits();
        let ndtr = dma2.st[0].ndtr.read().bits();
        // defmt::println!(
        //     "cfg={=u32:032b}\tndtr={=u32}\tlisr={=u32:032b}",
        //     cfg,
        //     ndtr,
        //     lisr
        // );

        let datum = unsafe { &ADC_CONVERSION_DATA[0][..16] };
        let datum2 = unsafe { &ADC_CONVERSION_DATA[1][..16] };
        // defmt::println!("DMA2_STREAM0! {:?}", datum);
        // defmt::println!("DMA2_STREAM0! {:?}", datum2);
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

    /// Run a DHCP client to dynamically acquire an IP address
    #[task(shared = [net], priority = 0)]
    async fn dhcp(mut cx: dhcp::Context, dhcp_handle: SocketHandle) {
        loop {
            core::future::poll_fn(|ctx| {
                cx.shared.net.lock(|net| {
                    let dhcp_socket = net.sockets.get_mut::<dhcpv4::Socket>(dhcp_handle);
                    dhcp_socket.register_waker(ctx.waker());

                    match dhcp_socket.poll() {
                        Some(dhcpv4::Event::Deconfigured) => {
                            defmt::warn!("DHCP got deconfigured");
                            net.iface.update_ip_addrs(|addrs| {
                                let dest = unwrap!(addrs.iter_mut().next());
                                *dest = IpCidr::Ipv4(Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
                            });
                            net.iface.routes_mut().remove_default_ipv4_route();
                            Poll::Pending
                        }
                        Some(dhcpv4::Event::Configured(config)) => {
                            defmt::debug!("DHCP config acquired!");

                            defmt::debug!("IP address:      {}", config.address);
                            net.iface.update_ip_addrs(|addrs| {
                                let dest = unwrap!(addrs.iter_mut().next());
                                *dest = IpCidr::Ipv4(config.address);
                            });
                            if let Some(router) = config.router {
                                defmt::debug!("Default gateway: {}", router);
                                unwrap!(net.iface.routes_mut().add_default_ipv4_route(router));
                            } else {
                                defmt::debug!("Default gateway: None");
                                net.iface.routes_mut().remove_default_ipv4_route();
                            }

                            for (i, s) in config.dns_servers.iter().enumerate() {
                                defmt::debug!("DNS server {}:    {}", i, s);
                            }
                            Poll::Ready(())
                        }
                        None => Poll::Pending,
                    }
                })
            })
            .await;
        }
    }
}
