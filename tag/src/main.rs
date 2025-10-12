//! Range measurement tag node
//!
//! This is a tag node used for range measurement. Tags use anchor nodes to
//! measure their distance from those anchors.
//!
//! Currently, distance measurements have a highly inaccurate result. One reason
//! that could account for this is the lack of antenna delay calibration, but
//! it's possible that there are various hidden bugs that contribute to this.

#![no_main]
#![no_std]

use dw1000::{mac, ranging, RxConfig, DW1000};
use dw1000::ranging::Message;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Timer};
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
};

// STM32F103 specific imports
#[cfg(feature = "stm32f103")]
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    spi::{self, Spi},
};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::mode::Async;
use embedded_hal_async::digital::Wait;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("Launching tag");

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

        defmt::info!("STM32L432KC Tag initialized with SPI and QSPI configuration");

        loop {
            Timer::after_millis(1000).await;
        }
    }

    #[cfg(feature = "stm32f103")]
    {
        // SPI1 configuration for STM32F103
        // Pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
        let mut spi_config = spi::Config::default();
        spi_config.frequency = embassy_stm32::time::Hertz(1_000_000); // 1 MHz

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

        let cs = Output::new(p.PA4, Level::High, Speed::Low);
        let mut uwb_irq = ExtiInput::new(p.PB0, p.EXTI0, Pull::None);
        let mut uwb_reset = Output::new(p.PB15, Level::Low, Speed::Low);
        let dw1000 = DW1000::new(spi, cs);

        // Reset DW1000
        uwb_reset.set_low();
        Timer::after(Duration::from_millis(100)).await;
        uwb_reset.set_high();

        let mut dw1000 = dw1000.init(&mut embassy_time::Delay).unwrap();
        dw1000.enable_rx_interrupts().unwrap();
        dw1000.enable_tx_interrupts().unwrap();

        // These are the hardcoded calibration values from the dwm1001-examples
        // repository. Ideally, the calibration values would be determined using
        // the proper calibration procedure, but hopefully those are good enough for
        // now.
        dw1000.set_antenna_delay(16384, 16384).unwrap();

        // Generate a random device address (simplified random for demo)
        let device_addr = 0x1234u16; // In real implementation, use proper RNG

        dw1000
            .set_address(
                mac::PanId(0x0d57),                    // hardcoded network id
                mac::ShortAddress(device_addr),        // device address
            )
            .expect("Failed to set address");

        defmt::info!("STM32F103 Tag initialized with SPI and GPIO configuration");

        let mut buf = [0; 128];

        loop {
            defmt::info!("waiting for base station ping");

            // Toggle LED to show activity
            led.toggle();

            let mut receiving = dw1000
                .receive(RxConfig::default())
                .expect("Failed to receive message");

            // Wait for incoming message with timeout
            match with_timeout(Duration::from_millis(500), uwb_irq.wait_for_rising_edge()).await {
                Ok(()) => {}
                Err(_) => {
                    defmt::info!("Timeout waiting for message, retrying...");
                    dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                    continue;
                }
            }

            let message = match receiving.wait_receive(&mut buf) {
                Ok(message) => message,
                Err(_) => {
                    defmt::info!("Failed to receive message");
                    dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                    continue;
                }
            };
            dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");

            defmt::info!("msg from base station: received");

            let ping = ranging::Ping::decode::<Spi<Async>, Output>(&message)
                .expect("Failed to decode ping");

            if let Some(ping) = ping {
                // Received ping from an anchor. Reply with a ranging request.
                defmt::info!("Received ping, sending ranging request");

                // Wait for a moment, to give the anchor a chance to start listening
                Timer::after_millis(10).await;

                let mut sending = ranging::Request::new(&mut dw1000, &ping)
                    .expect("Failed to initiate request")
                    .send(dw1000)
                    .expect("Failed to initiate request transmission");

                // Wait for transmission completion
                match with_timeout(Duration::from_millis(100), uwb_irq.wait_for_rising_edge()).await {
                    Ok(()) => {}
                    Err(_) => {
                        defmt::info!("Timeout waiting for transmit completion");
                        dw1000 = sending.finish_sending().expect("Finishing sending failed");
                        continue;
                    }
                }

                sending.wait_transmit().expect("Waiting for transmit failed");
                dw1000 = sending.finish_sending().expect("Finishing sending failed");

                continue;
            }

            let response = ranging::Response::decode::<Spi<Async>, Output>(&message)
                .expect("Failed to decode response");
            if let Some(response) = response {
                // Received ranging response from anchor. Now we can compute the distance.
                defmt::info!("Received ranging response");

                // If this is not a PAN ID and short address, it doesn't
                // come from a compatible node. Ignore it.
                let (pan_id, addr) = match response.source {
                    Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
                    _ => {
                        defmt::info!("Ignoring response with incompatible address format");
                        continue;
                    }
                };

                // Ranging response received. Compute distance.
                let distance_mm = ranging::compute_distance_mm(&response).unwrap();

                defmt::info!("Distance to anchor (PAN ID: {:04x}, Addr: {:04x}): {} mm", pan_id.0, addr.0, distance_mm);

                continue;
            }

            defmt::info!("Ignored message that was neither ping nor response");
        }
    }
}
