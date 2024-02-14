#![cfg_attr(not(test), no_std)]
#![feature(iter_map_windows)]

/// This module contains the logic to capture data from the ADC, via the DMA into a ring buffer
///
/// It sets up ADC1, DMA2 channel 0, and TIM2.
/// See [`AdcCapture::init`](`adc_capture::AdcCapture::init`) for the details of how they are
/// configured. Further it provides an interrupt handler for DMA2 in
/// [`AdcCapture::dma_interrupt_handler`](`adc_capture::AdcCapture::dma_interrupt_handler`). As well
/// as a way to read and release the captured data.
pub mod adc_capture;

/// This module contains the network stack.
///
/// It uses [`stm32_eth`] as a driver for the ethernet interface, [`smoltcp`] as a TCP/IP stack, and
/// [`minimq`] as a MQTT client.
pub mod network;

/// This module wrappes functionality used for [`statime`].
///
/// This part comes directly from the [`statime` STM32 example](https://github.com/pendulum-project/statime/tree/main/statime-stm32).
pub mod statime_wrapper;
