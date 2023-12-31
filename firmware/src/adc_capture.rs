use core::{
    cmp::Ordering,
    ops::{Range, RangeFrom, RangeTo},
};

use hal::{
    pac,
    rcc::{self, Enable, Reset, APB2},
};
use stm32f7xx_hal as hal;

pub type Dma2Stream = pac::dma2::ST;

type AdcCaptureChunk = [u16; AdcCapture::CONVS_PER_CHUNK];
pub type AdcCaptureBuffer = [u16; AdcCapture::CONVS_PER_CHUNK * AdcCapture::BUF_NUM_CHUNKS];

enum RingRange {
    Empty,
    Normal(Range<usize>),
    Wrapped(RangeFrom<usize>, RangeTo<usize>),
}

struct DoubleBufferedRingBuffer {
    buffer: &'static mut AdcCaptureBuffer,
    app_range: RingRange,
    free_range: RingRange,
}
impl DoubleBufferedRingBuffer {
    fn new(buffer: &'static mut AdcCaptureBuffer) -> DoubleBufferedRingBuffer {
        Self {
            app_range: RingRange::Empty,
            free_range: RingRange::Normal(0..buffer.len()),
            buffer,
        }
    }

    fn next_dma_buffer(&mut self) -> Option<&mut [u16; AdcCapture::CONVS_PER_CHUNK]> {
        let (new_range, dma_range) = match &self.free_range {
            RingRange::Empty => return None,
            RingRange::Normal(range) => {
                let new_start = range.start + AdcCapture::CONVS_PER_CHUNK;
                match new_start.cmp(&range.end) {
                    Ordering::Less => (
                        RingRange::Normal(new_start..range.end),
                        range.start..new_start,
                    ),
                    Ordering::Equal => (RingRange::Empty, range.clone()),
                    Ordering::Greater => return None,
                }
            }
            RingRange::Wrapped(morning, afternoon) => {
                let new_start = morning.start + AdcCapture::CONVS_PER_CHUNK;
                match new_start.cmp(&self.buffer.len()) {
                    Ordering::Less => (RingRange::Wrapped(new_start.., afternoon.clone()), morning.start..new_start),
                    Ordering::Equal =>  (RingRange::Normal(0..afternoon.end), morning.start..self.buffer.len()),
                    Ordering::Greater => unreachable!("morning.start was already bigger the length or the length as not divisible by CONVS_PER_CHUNK"),
                }
            }
        };

        self.free_range = new_range;
        todo!()
        // let dma_buffer = &mut self.buffer[dma_range]
        //     .try_into()
        //     .expect("dma_range is always CONVS_PER_CHUNK long");

        // Some(dma_buffer)
    }

    fn app_release(&mut self, num_bytes: usize) {
        let (new_app_range, released_range) = match &self.app_range {
            RingRange::Empty => panic!("Released bytes while not owning any!"),
            RingRange::Normal(app) => {
                let new_start = app.start + num_bytes;
                if !app.contains(&new_start) {
                    panic!("Released more bytes then you owned");
                }

                (
                    RingRange::Normal(new_start..app.end),
                    RingRange::Normal(app.start..new_start),
                )
            }
            RingRange::Wrapped(morning, afternoon) => {
                let new_start = morning.start + num_bytes;
                match new_start.cmp(&self.buffer.len()) {
                    Ordering::Less => (
                        RingRange::Wrapped(new_start.., afternoon.clone()),
                        RingRange::Normal(morning.start..new_start),
                    ),
                    Ordering::Equal => (
                        RingRange::Normal(0..afternoon.end),
                        RingRange::Normal(morning.start..self.buffer.len()),
                    ),
                    Ordering::Greater => {
                        if num_bytes > self.buffer.len() + morning.start {
                            // TODO somewhere here should also be a new_range EMPTY
                            (
                                RingRange::Normal((new_start % self.buffer.len())..afternoon.end),
                                RingRange::Wrapped(
                                    morning.clone(),
                                    ..(new_start % self.buffer.len()),
                                ),
                            )
                        } else {
                            (
                                RingRange::Normal(0..afternoon.end),
                                RingRange::Normal(morning.start..self.buffer.len()),
                            )
                        }
                    }
                }
            }
        };

        self.app_range = new_app_range;

        // TODO now update the free range eggs
    }
}

pub struct AdcCapture {
    adc1: pac::ADC1,
    tim2: pac::TIM2,
    dma2: pac::DMA2,
    buffer: DoubleBufferedRingBuffer,
}

impl AdcCapture {
    pub const CONVS_PER_CHUNK: usize = 1024 * 8;
    pub const BUF_NUM_CHUNKS: usize = 4;
    pub fn init(
        buffer: &'static mut AdcCaptureBuffer,
        adc1: pac::ADC1,
        tim2: pac::TIM2,
        dma2: pac::DMA2,
        apb1: &mut rcc::APB1,
        apb2: &mut rcc::APB2,
        ahb1: &mut rcc::AHB1,
    ) -> Self {
        let initial_dma_m0a = buffer[..].as_ptr();
        let initial_dma_m1a = buffer[Self::BUF_NUM_CHUNKS..].as_ptr();
        let mut this = Self {
            adc1,
            tim2,
            dma2,
            buffer: DoubleBufferedRingBuffer::new(buffer),
        };

        this.init_dma2(ahb1);
        this.init_adc1(apb2);
        this.init_tim2(apb1);

        this
    }

    /// Configure DMA2 Stream 0 to read 16-bit conversions from ADC1
    /// and write them into ADC_CONVERSION_DATA
    /// in double-buffer mode
    fn init_dma2(&mut self, ahb1: &mut rcc::AHB1) {
        let dma2_stream0 = &self.dma2.st[0];
        <pac::DMA2 as Enable>::enable(ahb1);
        // Disable DMA2 Stream 0
        dma2_stream0.cr.modify(|_, w| w.en().disabled());

        // Choose channel 0 for DMA2 Stream 0
        dma2_stream0.cr.modify(|_, w| {
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
        dma2_stream0
            .ndtr
            .modify(|_, w| w.ndt().bits(Self::CONVS_PER_CHUNK.try_into().unwrap()));

        // Set peripheral address to ADC1 data register
        dma2_stream0
            .par
            .write(|w| unsafe { w.pa().bits(self.adc1.dr.as_ptr() as u32) });

        // Point DMA Memory 0 to first chunk of data buffer
        dma2_stream0.m0ar.write(|w| unsafe {
            w.m0a()
                .bits(self.buffer.next_dma_buffer().unwrap().as_ptr() as u32)
        });
        // Point DMA Memory 1 to second chunk of data buffer
        dma2_stream0.m1ar.write(|w| unsafe {
            w.m1a()
                .bits(self.buffer.next_dma_buffer().unwrap().as_ptr() as u32)
        });

        // Enable DMA2 Stream 0
        dma2_stream0.cr.modify(|_, w| w.en().enabled());
    }

    /// Configure ADC1 to 12-bits resolution in
    /// single conversion mode and to be triggered externally from TIM2 TRGO
    /// and read out using DMA
    fn init_adc1(&mut self, apb2: &mut APB2) {
        let adc1 = &self.adc1;
        <pac::ADC1 as Enable>::enable(apb2);
        // Power down ADC1
        adc1.cr2.modify(|_, w| w.adon().clear_bit());
        <pac::ADC1 as Reset>::reset(apb2);

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
    }

    /// Setup TIM2 to trigger ADC1 using TRGO on update event generation
    fn init_tim2(&mut self, apb1: &mut rcc::APB1) {
        let tim2 = &self.tim2;
        <pac::TIM2 as Enable>::enable(apb1);
        // Set Master mode trigger on timer enable, which will enable ADC1 as well
        tim2.cr2.modify(|_, w| w.mms().enable());
        // ARR resets to u32::MAX
        tim2.arr.write(|w| w.arr().bits(u32::MAX));

        // Enable TIM2 CC1 capture interrupt as well as update interrupt to detect overflows
        tim2.dier.modify(|_, w| w.uie().enabled().cc1ie().enabled());

        // Connect PTP to TIM2 ITR1
        tim2.or.write(|w| unsafe { w.itr1_rmp().bits(0b01) });
        // Connect TIM2 ITR1 to ITR
        tim2.smcr.modify(|_, w| w.ts().itr1());

        // Configures TIM2 CC1 to capture the timer value on ITR
        tim2.ccmr1_input()
            .modify(|_, w| unsafe { w.cc1s().trc().ic1f().no_filter().ic1psc().bits(0b00) });

        // Enable TIM2
        tim2.cr1.modify(|_, w| w.cen().enabled());

        tim2.ccer
            .modify(|_, w| w.cc1p().clear_bit().cc1np().clear_bit());
        tim2.cr2.modify(|_, w| w.ti1s().normal());

        // Enable CC1
        tim2.ccer.modify(|_, w| w.cc1e().set_bit());
    }
}
