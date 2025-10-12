//! Range measurement anchor node
//!
//! This is an anchor node used for range measurement. Anchors have a known
//! location, and provide the support infrastructure requires by tag nodes to
//! determine their own distance from the available anchors.
//!
//! Currently, distance measurements have a highly inaccurate result. One reason
//! that could account for this is the lack of antenna delay calibration, but
//! it's possible that there are various hidden bugs that contribute to this.

#![allow(dead_code)]
#![no_main]
#![no_std]

use dw1000::{mac, ranging, RxConfig, DW1000};
use dw1000::ranging::Message;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Timer, Instant};
// Print panic message to probe console
use {defmt_rtt as _, panic_probe as _};

// STM32L432KC specific imports
#[cfg(feature = "stm32l432kc")]
use embassy_stm32::{
    qspi::{
        self,
        enums::{AddressSize, ChipSelectHighTime, FIFOThresholdLevel, MemorySize, SampleShifting},
    },
    spi::{self, Spi},
    gpio::{Level, Output, Speed},
    rng::Rng,
};

// STM32F103 specific imports
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    spi::{self, Spi},
};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::mode::Async;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("Launching anchor");

    let p = embassy_stm32::init(Default::default());

    #[cfg(feature = "stm32l432kc")]
    {
        // SPI1 configuration for STM32L432KC
        // Pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
        let mut spi_config = spi::Config::default();
        spi_config.frequency = embassy_stm32::time::Hertz(1_000_000); // 1 MHz

        let spi = Spi::new(
            p.SPI1,
            p.PA5, // SCK
            p.PA7, // MOSI
            p.PA6, // MISO
            p.DMA1_CH3, // TX DMA (correct channel for SPI1)
            p.DMA1_CH2, // RX DMA (correct channel for SPI1)
            spi_config,
        );

        let cs = Output::new(p.PA4, Level::High, Speed::Low);
        let dw1000 = DW1000::new(spi, cs);

        // QSPI configuration for STM32L432KC
        let mut config = qspi::Config::default();
        config.memory_size = MemorySize::_16MiB;
        config.address_size = AddressSize::_24bit;
        config.prescaler = 200;
        config.cs_high_time = ChipSelectHighTime::_1Cycle;
        config.fifo_threshold = FIFOThresholdLevel::_16Bytes;
        config.sample_shifting = SampleShifting::None;

        defmt::info!("STM32L432KC initialized with SPI and QSPI configuration");

        loop {
            Timer::after_millis(1000).await;
        }
    }

    #[cfg(feature = "stm32f103")]
    {
        // SPI1 configuration for STM32F103
        // Pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
        let mut spi_config = spi::Config::default();
        spi_config.frequency = embassy_stm32::time::Hertz(2_000_000); // 2 MHz

        let spi = Spi::new(
            p.SPI1,
            p.PA5, // SCK
            p.PA7, // MOSI
            p.PA6, // MISO
            p.DMA1_CH3, // TX DMA
            p.DMA1_CH2, // RX DMA
            spi_config,
        );

        // GPIO configuration for STM32F103 (using built-in LED on PC13)
        let mut led = Output::new(p.PC13, Level::Low, Speed::Low);

        let cs = Output::new(p.PA4, Level::Low, Speed::Low);
        let mut uwb_irq = ExtiInput::new(p.PB0, p.EXTI0, Pull::None);
        let mut uwb_reset = Output::new(p.PB15, Level::Low, Speed::Low);
        let dw1000 = DW1000::new(spi, cs);

        // Reset DW1000
        uwb_reset.set_low();
        Timer::after(Duration::from_millis(1000)).await;
        uwb_reset.set_high();
        Timer::after(Duration::from_millis(10)).await;

        // Initialize DW1000
        let mut dw1000 = dw1000.init(&mut embassy_time::Delay).unwrap();
        dw1000.enable_rx_interrupts().unwrap();
        dw1000.enable_tx_interrupts().unwrap();

        // These are the hardcoded calibration values from the dwm1001-examples
        // repository. Ideally, the calibration values would be determined using
        // the proper calibration procedure, but hopefully those are good enough for
        // now.
        dw1000.set_antenna_delay(16384, 16384).unwrap();

        // Set network address with random device address
        let address = 1234u16;
        dw1000
            .set_address(
                mac::PanId(0x0d57),                    // hardcoded network id
                mac::ShortAddress(address),        // random device address
            )
            .expect("Failed to set address");

        defmt::info!("STM32F103 initialized with SPI and GPIO configuration, device address: {}", address);

        let mut buf = [0; 128];
        let mut frame_id = 0;
        let mut ping_id = 0;
        let mut last_ping_time = Instant::now();

        // Main anchor loop
        loop {
            /*
            Strategy:
            - Sending a ranging ping every 5 seconds
            - Waiting for a ranging request
            - Responding with a ranging response
            */

            // After receiving for a while, it's time to send out a ping
            if last_ping_time.elapsed() >= Duration::from_millis(100) {
                defmt::info!("Sending ping {}", ping_id);
                ping_id += 1;
                last_ping_time = Instant::now();

                // Blink LED to indicate ping transmission
                led.set_high();
                Timer::after_millis(10).await;
                led.set_low();

                let mut sending = ranging::Ping::new(&mut dw1000)
                    .expect("Failed to initiate ping")
                    .send(dw1000)
                    .expect("Failed to initiate ping transmission");

                // Wait for transmission complete interrupt
                match with_timeout(Duration::from_millis(100), uwb_irq.wait_for_rising_edge()).await {
                    Ok(()) => {
                        sending.wait_transmit().expect("Failed to send ping");
                        dw1000 = sending.finish_sending().expect("Failed to finish sending");
                        defmt::info!("Ping sent successfully");
                    }
                    Err(_) => {
                        defmt::info!("Timeout waiting for ping transmit");
                        dw1000 = sending.finish_sending().expect("Failed to finish sending");
                        continue;
                    }
                }
            }

            defmt::info!("Starting receive. Frame ID: {}", frame_id);
            frame_id += 1;

            // Start receiving
            let mut receiving = dw1000
                .receive(RxConfig::default())
                .expect("Failed to receive message");

            // Wait for receive interrupt with timeout
            let message = match with_timeout(Duration::from_millis(500), uwb_irq.wait_for_rising_edge()).await {
                Ok(()) => {
                    match receiving.wait_receive(&mut buf) {
                        Ok(message) => {
                            dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                            Some(message)
                        }
                        Err(_) => {
                            dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                            defmt::info!("Error receiving message");
                            continue;
                        }
                    }
                }
                Err(_) => {
                    dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                    continue; // Timeout, continue to next iteration
                }
            };

            let message = match message {
                Some(msg) => msg,
                None => continue,
            };

            defmt::info!("Response found");

            // Blink LED to indicate message received
            led.set_high();
            Timer::after_millis(10).await;
            led.set_low();

            // Try to decode as ranging request
            let request = ranging::Request::decode::<Spi<Async>, Output>(&message);

            let request = match request {
                Ok(Some(request)) => request,
                Ok(None) | Err(_) => {
                    defmt::info!("Ignoring message that is not a request");
                    continue;
                }
            };

            // Another LED blink to indicate valid request
            led.set_high();
            Timer::after_millis(10).await;
            led.set_low();

            // Wait for a moment, to give the tag a chance to start listening for
            // the reply.
            Timer::after_millis(10).await;

            // Send ranging response
            let mut sending = ranging::Response::new(&mut dw1000, &request)
                .expect("Failed to initiate response")
                .send(dw1000)
                .expect("Failed to initiate response transmission");

            // Wait for transmission complete interrupt
            match with_timeout(Duration::from_millis(100), uwb_irq.wait_for_rising_edge()).await {
                Ok(()) => {
                    sending.wait_transmit().expect("Failed to send ranging response");
                    dw1000 = sending.finish_sending().expect("Failed to finish sending");

                    // Final LED blink to indicate successful response
                    led.set_high();
                    Timer::after_millis(10).await;
                    led.set_low();

                    defmt::info!("Ranging response sent successfully");
                }
                Err(_) => {
                    defmt::info!("Timeout waiting for response transmit");
                    dw1000 = sending.finish_sending().expect("Failed to finish sending");
                }
            }
        }
    }
}
