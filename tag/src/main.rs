#![allow(dead_code)]
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

        let cs = Output::new(p.PA4, Level::Low, Speed::Low);
        let mut uwb_irq = ExtiInput::new(p.PB0, p.EXTI0, Pull::None);
        let dw1000 = DW1000::new(spi, cs);

        let mut dw1000 = dw1000.init(&mut embassy_time::Delay).unwrap();
        dw1000.enable_rx_interrupts().unwrap();
        dw1000.enable_tx_interrupts().unwrap();
        dw1000.set_antenna_delay(16456, 16300).unwrap();
        dw1000
            .set_address(
                mac::PanId(0x0d57),             // hardcoded network id
                mac::ShortAddress(12345u16),    // different device address for tag
            )
            .expect("Failed to set address");

        defmt::info!("STM32F103 Tag initialized with SPI and GPIO configuration");

        let mut buf = [0; 128];

        // Tag behavior: initiate ranging requests
        loop {
            // Toggle LED to show activity
            led.toggle();

            let mut receiving = dw1000
                .receive(RxConfig::default())
                .expect("Failed to receive message");
            match with_timeout(Duration::from_millis(500), uwb_irq.wait_for_rising_edge()).await {
                Ok(()) => {}
                Err(_) => {
                    defmt::info!("No incoming message, proceeding to send ranging request");
                    dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                    continue;
                }
            }

            let message = match receiving.wait_receive(&mut buf) {
                Ok(message) => message,
                Err(_) => {
                    dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                    continue;
                }
            };
            dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");

            defmt::info!("msg from base station: received");

            let ping = ranging::Ping::decode::<Spi<Async>, Output>(&message)
                .expect("Failed to decode ping");

            if let Some(ping) = ping {
                Timer::after_millis(10).await;

                let mut sending = ranging::Request::new(&mut dw1000, &ping)
                    .expect("Failed to initiate request")
                    .send(dw1000)
                    .expect("Failed to initiate request transmission");

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
                Timer::after_millis(10).await;

                let (pan_id, addr) = match response.source {
                    Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
                    _ => continue,
                };

                // Ranging response received. Compute distance.
                let distance_mm = ranging::compute_distance_mm(&response).unwrap();

                Timer::after_millis(10).await;

                defmt::info!("Distance to anchor (PAN ID: {}, Addr: {}): {} mm", pan_id.0, addr.0, distance_mm);

                continue;
            }
            defmt::info!("Ignored message that was neither ping nor response\n");
        }
    }
}
