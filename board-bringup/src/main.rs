#![allow(dead_code)]
use fugit::RateExtU32;
#[cfg(feature = "eth")]
use stm32_eth::dma::{RxRingEntry, TxRingEntry};
use stm32_eth::{mac, EthPins};
use stm32f7xx_hal::gpio::{Alternate, Analog, DynamicPin, GpioExt, Output, Pin, Speed};
use stm32f7xx_hal::otg_fs::USB;
use stm32f7xx_hal::pac::{Peripherals, TIM4, USART3};
use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode, RccExt, PLL48CLK};
use stm32f7xx_hal::serial::Serial;
use stm32f7xx_hal::timer::{PwmChannel, PwmExt};

pub mod mic;

pub struct UserInterface {
    user_btn: Pin<'A', 0>,
    led_blue: PwmChannel<TIM4, 1>,
    led_red: Pin<'B', 14, Output>,
}

pub struct MicInterface<const ENABLED: bool = false> {
    audio_in: Pin<'A', 3, Analog>,
    enable: Pin<'A', 4, Output>,
    gain: DynamicPin<'A', 6>,
    ar: DynamicPin<'E', 2>,
    // ar: Pin<'A', 7>, // Bodged away...
}

pub struct PowerSupply {
    poe: Pin<'A', 8>,
    usb: Pin<'A', 9>,
}

pub struct Timing {
    // pps_out: Pin<'B', 5, Output>, // <-- owned by Ethernet peripheral
    pps_on_tim2: Pin<'A', 15, Alternate<1>>,
    pps_on_adc2: Pin<'B', 0, Analog>,
    sync_in_0: Pin<'B', 10, Alternate<1>>,
    sync_in_1: Pin<'B', 11, Alternate<1>>,
}

pub struct Board {
    pub ui: UserInterface,
    pub mic: MicInterface<true>,
    pub psu: PowerSupply,
    pub timing: Timing,
    pub uart: Serial<USART3, (Pin<'D', 8, Alternate<7>>, Pin<'D', 9, Alternate<7>>)>,
    #[cfg(feature = "usb")]
    pub usb: USB,
    #[cfg(feature = "eth")]
    pub eth: stm32_eth::Parts<
        'static,
        'static,
        mac::EthernetMACWithMii<Pin<'A', 2, Alternate<11>>, Pin<'C', 1, Alternate<11>>>,
    >,
}

pub fn setup_board(p: Peripherals) -> Board {
    let rcc = p.RCC.constrain();
    let clocks = rcc
        .cfgr
        .hse(HSEClock::new(25.MHz(), HSEClockMode::Oscillator))
        .use_pll()
        .use_pll48clk(PLL48CLK::Pllq)
        .sysclk(216.MHz())
        .freeze();

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiod = p.GPIOD.split();
    let gpioe = p.GPIOE.split();
    let _gpiof = p.GPIOF.split();
    let gpiog = p.GPIOG.split();

    let led_blue = {
        let pb7 = gpiob.pb7.into_alternate();
        let mut pwm = p.TIM4.pwm_hz(pb7, 1.kHz(), &clocks).split();

        let max = pwm.get_max_duty();
        pwm.set_duty(max / 2);
        pwm.enable();

        pwm
    };

    let ui = UserInterface {
        user_btn: gpioa.pa0,
        led_blue,
        led_red: gpiob.pb14.into_push_pull_output(),
    };

    let mic = MicInterface {
        audio_in: gpioa.pa3.into_analog(),
        enable: gpioa.pa4.into_push_pull_output(),
        gain: gpioa.pa6.into_dynamic(),
        ar: gpioe.pe2.into_dynamic(),
    };

    let psu = PowerSupply {
        poe: gpioa.pa8,
        usb: gpioa.pa9,
    };

    let timing = Timing {
        pps_on_tim2: gpioa.pa15.into_alternate(),
        pps_on_adc2: gpiob.pb0.into_analog(),
        sync_in_0: gpiob.pb10.into_alternate(),
        sync_in_1: gpiob.pb11.into_alternate(),
    };

    let uart = Serial::new(
        p.USART3,
        (gpiod.pd8.into_alternate(), gpiod.pd9.into_alternate()),
        &clocks,
        Default::default(),
    );

    #[cfg(feature = "usb")]
    let usb = USB::new(
        p.OTG_FS_GLOBAL,
        p.OTG_FS_DEVICE,
        p.OTG_FS_PWRCLK,
        (gpioa.pa11.into_alternate(), gpioa.pa12.into_alternate()),
        &clocks,
    );

    #[cfg(feature = "eth")]
    let eth = {
        let eth_pins = {
            let ref_clk = gpioa.pa1.into_floating_input();
            let crs = gpioa.pa7.into_floating_input(); // NOTE: wrong connected on board, apply BODGE!

            let tx_d1 = gpiob.pb13.into_floating_input();

            let rx_d0 = gpioc.pc4.into_floating_input();
            let rx_d1 = gpioc.pc5.into_floating_input();

            let tx_en = gpiog.pg11.into_floating_input();
            let tx_d0 = gpiog.pg13.into_floating_input();

            EthPins {
                ref_clk,
                crs,
                tx_en,
                tx_d0,
                tx_d1,
                rx_d0,
                rx_d1,
            }
        };

        let mdio = gpioa.pa2.into_alternate::<11>().set_speed(Speed::VeryHigh);
        let mdc = gpioc.pc1.into_alternate::<11>().set_speed(Speed::VeryHigh);

        let mut parts = {
            let ethernet = stm32_eth::PartsIn {
                dma: p.ETHERNET_DMA,
                mac: p.ETHERNET_MAC,
                mmc: p.ETHERNET_MMC,
                ptp: p.ETHERNET_PTP,
            };

            static mut RX_RING: [RxRingEntry; 2] = [RxRingEntry::new(), RxRingEntry::new()];
            static mut TX_RING: [TxRingEntry; 2] = [TxRingEntry::new(), TxRingEntry::new()];

            // SAFETY: Initialization only happens once
            let rx_ring = unsafe { &mut RX_RING };
            let tx_ring = unsafe { &mut TX_RING };

            stm32_eth::new_with_mii(ethernet, rx_ring, tx_ring, clocks, eth_pins, mdio, mdc)
                .unwrap()
        };

        parts.ptp.enable_pps(gpiob.pb5.into_push_pull_output());
        parts.ptp.set_pps_freq(4);

        parts.dma.enable_interrupt();

        parts
    };

    Board {
        ui,
        mic: mic.enable(),
        psu,
        timing,
        uart,
        #[cfg(feature = "usb")]
        usb,
        #[cfg(feature = "eth")]
        eth,
    }
}

pub fn main() {
    let p = Peripherals::take().unwrap();
    let _todo = setup_board(p);
}
