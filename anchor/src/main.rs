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
        let mut uwb_reset = Output::new(p.PA15, Level::Low, Speed::Low);
        let dw1000 = DW1000::new(spi, cs);

        uwb_reset.set_high();
        Timer::after(Duration::from_millis(10)).await;
        uwb_reset.set_low();
        Timer::after(Duration::from_millis(10)).await;

        let mut dw1000 = dw1000.init(&mut embassy_time::Delay).unwrap();
        dw1000.enable_rx_interrupts().unwrap();
        dw1000.enable_tx_interrupts().unwrap();
        dw1000.set_antenna_delay(16456, 16300).unwrap();
        dw1000
            .set_address(
                mac::PanId(0x0d57),             // hardcoded network id
                mac::ShortAddress(12443u16),    // random device address
            )
            .expect("Failed to set address");

        defmt::info!("STM32F103 initialized with SPI and GPIO configuration");

        let mut buf = [0; 128];
        let mut frame_id = 0;

        // Blink LED to show activity
        loop {
            led.toggle();
            Timer::after_millis(10).await;

            let mut sending = ranging::Ping::new(&mut dw1000)
                .expect("Failed to create ranging ping")
                .send(dw1000)
                .expect("Receiving ranging ping failed");
            uwb_irq.wait_for_rising_edge().await;
            sending.wait_transmit().expect("Waiting for transmit failed");
            dw1000 = sending.finish_sending().expect("Finishing sending failed");
            defmt::info!("completed sending");

            defmt::info!("Starting receive. Frame ID: {}", frame_id);
            frame_id += 1;

            let mut receiving = dw1000
                .receive(RxConfig::default())
                .expect("Failed to receive message");

            match with_timeout(Duration::from_millis(500), uwb_irq.wait_for_rising_edge()).await {
                Ok(()) => {}
                Err(_) => {
                    defmt::info!("Timeout waiting for message\n");
                    dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                    continue;
                }
            }
            let message = match receiving.wait_receive(&mut buf) {
                Ok(message) => message,
                Err(_) => {
                    dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
                    defmt::info!("Error receiving message\n");
                    continue;
                }
            };

            dw1000 = receiving
                .finish_receiving()
                .expect("Failed to finish receiving");

            Timer::after_millis(10).await;

            let request = ranging::Request::decode::<Spi<Async>, Output>(&message);
            let request = match request {
                Ok(Some(request)) => request,
                Ok(None) | Err(_) => {
                    defmt::info!("Ignoring message that is not a request\n");
                    continue;
                }
            };

            let mut sending = ranging::Response::new(&mut dw1000, &request)
                .expect("Failed to create response")
                .send(dw1000)
                .expect("Failed to send response");
            match with_timeout(Duration::from_millis(500), uwb_irq.wait_for_rising_edge()).await {
                Ok(()) => {}
                Err(_) => {
                    defmt::info!("Timeout waiting for transmit\n");
                    dw1000 = sending.finish_sending().expect("Finishing sending failed");
                    continue;
                }
            }
            sending.wait_transmit().expect("Waiting for transmit failed");
            dw1000 = sending.finish_sending().expect("Finishing sending failed");
        }
    }
}
