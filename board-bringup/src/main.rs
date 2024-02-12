#![no_std]
#![no_main]
#![allow(dead_code)]

use defmt::Debug2Format;
use fugit::RateExtU32;
use stm32_eth::dma::{RxRingEntry, TxRingEntry};
use stm32_eth::{mac, EthPins};
use stm32f7xx_hal::gpio::{Alternate, Analog, DynamicPin, Output, Pin, Speed};
use stm32f7xx_hal::otg_fs::USB;
use stm32f7xx_hal::pac::{CorePeripherals, Peripherals, TIM4, USART3};
use stm32f7xx_hal::prelude::*;
use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode, PLL48CLK};
use stm32f7xx_hal::serial::Serial;
use stm32f7xx_hal::timer::{PwmChannel, SysDelay};

use defmt_rtt as _;
use panic_probe as _;

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
    pub usb: Option<USB>,
    pub eth: Option<
        stm32_eth::Parts<
            'static,
            'static,
            mac::EthernetMACWithMii<Pin<'A', 2, Alternate<11>>, Pin<'C', 1, Alternate<11>>>,
        >,
    >,
    pub delay: SysDelay,
}

pub fn setup_board(p: Peripherals, cp: CorePeripherals, init_eth: bool, init_usb: bool) -> Board {
    defmt::info!("Constraining RCC");
    let rcc = p.RCC.constrain();

    defmt::info!("Setting up clocks");
    let real_board = false;
    let clocks = if real_board {
        rcc.cfgr
            .hse(HSEClock::new(25.MHz(), HSEClockMode::Oscillator))
            .use_pll()
            .use_pll48clk(PLL48CLK::Pllq)
            .sysclk(216.MHz())
            .freeze()
    } else {
        rcc.cfgr.freeze()
    };
    defmt::info!("Clocks setup: {:?}", Debug2Format(&clocks));

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
        defmt::info!("PWM out activated... blue led now is at 1kHz 50% duty");

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
    }
    .enable();
    defmt::info!("Enabled mic");

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

    let mut uart = Serial::new(
        p.USART3,
        (gpiod.pd8.into_alternate(), gpiod.pd9.into_alternate()),
        &clocks,
        Default::default(),
    );
    defmt::info!("UART ready, sending Moin");
    for &byte in b"Moin\n" {
        loop {
            match uart.write(byte) {
                Ok(_) => break,
                Err(e) => continue,
            }
        }
    }

    let usb = if init_usb {
        defmt::info!("Setting up USB");
        Some(USB::new(
            p.OTG_FS_GLOBAL,
            p.OTG_FS_DEVICE,
            p.OTG_FS_PWRCLK,
            (gpioa.pa11.into_alternate(), gpioa.pa12.into_alternate()),
            &clocks,
        ))
    } else {
        None
    };

    let eth = if init_eth {
        defmt::info!("Setting up eth");
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
        defmt::info!("Enabled PPS output at 16Hz");

        parts.dma.enable_interrupt();

        Some(parts)
    } else {
        None
    };

    let delay = cp.SYST.delay(&clocks);

    defmt::info!("Board setup done!");

    Board {
        ui,
        mic,
        psu,
        timing,
        uart,
        usb,
        eth,
        delay,
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let mut b = setup_board(p, cp, false, false);

    let mut i = 0u32;
    let mut btn_pressed = false;

    loop {
        // Loop slowly...
        b.delay.delay(fugit::MicrosDurationU32::millis(100));

        // Check button...
        let btn_high = b.ui.user_btn.is_high();
        if btn_high != btn_pressed {
            defmt::info!("Button toggled... is_high={}", btn_high);
            btn_pressed = btn_high;
        }

        // Blink LED (1s on/off)
        if i % 10 == 0 {
            b.ui.led_red.toggle();
        }

        // Print power state (every 10s)
        if i % 100 == 0 {
            defmt::info!(
                "Power sources: USB({}), PoE({})",
                b.psu.usb.is_high(),
                b.psu.poe.is_high()
            );

            defmt::info!("User Butt: {}", b.ui.user_btn.is_high());
        }

        i = i.wrapping_add(1);
    }
}
