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

use embassy_executor::Spawner;
// Print panic message to probe console
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("Launching tag");

    let p = embassy_stm32::init(Default::default());
    //
    // // SPI1 configuration for STM32F103
    // // Pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
    // let mut spi_config = spi::Config::default();
    // spi_config.frequency = embassy_stm32::time::Hertz(1_000_000); // 1 MHz
    //
    // let cs = Output::new(p.PA4, Level::High, Speed::Low);
    //
    // let spi = Spi::new(
    //     p.SPI1,
    //     p.PA5, // SCK
    //     p.PA7, // MOSI
    //     p.PA6, // MISO
    //     p.DMA1_CH3, // TX DMA
    //     p.DMA1_CH2, // RX DMA
    //     spi_config,
    // );
    // let spi = ExclusiveDevice::new(spi, cs, Delay);
    //
    // // GPIO configuration for STM32F103 (using built-in LED on PC13)
    // let mut led = Output::new(p.PC13, Level::Low, Speed::Low);
    //
    //
    // let mut uwb_irq = ExtiInput::new(p.PB0, p.EXTI0, Pull::None);
    // let mut uwb_reset = Output::new(p.PB15, Level::Low, Speed::Low);
    //
    // let dw1000 = DW1000::new(spi);
    //
    // // Reset DW1000
    // uwb_reset.set_low();
    // Timer::after(Duration::from_millis(100)).await;
    // uwb_reset.set_high();
    //
    // let mut dw1000 = dw1000.init(&mut Delay).unwrap();
    // dw1000.enable_rx_interrupts().unwrap();
    // dw1000.enable_tx_interrupts().unwrap();
    //
    // // These are the hardcoded calibration values from the dwm1001-examples
    // // repository. Ideally, the calibration values would be determined using
    // // the proper calibration procedure, but hopefully those are good enough for
    // // now.
    // // Adjusted antenna delays to correct distance over-estimation
    // // Original values were 16456, 16300 - reduced to compensate for +40-50cm error at 3m
    // dw1000.set_antenna_delay(16456, 16456).unwrap();
    //
    // // Generate a random device address
    //
    // dw1000
    //     .set_address(
    //         mac::PanId(0x0d57),                    // hardcoded network id
    //         mac::ShortAddress(1122u16),        // random device address
    //     )
    //     .expect("Failed to set address");
    //
    // defmt::info!("STM32F103 Tag initialized with SPI and GPIO configuration");
    //
    // let mut buf = [0; 128];
    //
    // loop {
    //     defmt::info!("waiting for base station ping");
    //
    //     let mut receiving = dw1000
    //         .receive(RxConfig::default())
    //         .expect("Failed to receive message");
    //
    //     // Wait for incoming message with timeout
    //     let message = match with_timeout(Duration::from_millis(500), uwb_irq.wait_for_rising_edge()).await {
    //         Ok(()) => {
    //             match receiving.wait_receive(&mut buf) {
    //                 Ok(message) => { Some(message) }
    //                 Err(_) => {
    //                     defmt::info!("Failed to receive message");
    //                     None
    //                 }
    //             }
    //         }
    //         Err(_) => {
    //             defmt::info!("Timeout waiting for message, retrying...");
    //             None
    //         }
    //     };
    //     dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
    //
    //     let message = match message {
    //         Some(msg) => msg,
    //         None => continue,
    //     };
    //
    //     defmt::info!("msg from base station: received");
    //
    //     let ping = ranging::Ping::decode::<ExclusiveDevice<Spi<Async>, Output, Delay>>(&message)
    //         .expect("Failed to decode ping");
    //
    //     if let Some(ping) = ping {
    //         // Received ping from an anchor. Reply with a ranging request.
    //         defmt::info!("Received ping, sending ranging request");
    //
    //         // Flash LED to indicate ping received
    //         led.set_high();
    //         Timer::after_millis(10).await;
    //         led.set_low();
    //
    //         // Wait for a moment, to give the anchor a chance to start listening
    //         Timer::after_millis(10).await;
    //
    //         let mut sending = ranging::Request::new::<ExclusiveDevice<Spi<Async>, Output, Delay>, Output>(&mut dw1000, &ping)
    //             .expect("Failed to initiate request")
    //             .send::<ExclusiveDevice<Spi<Async>, Output, Delay>, Output>(dw1000)
    //             .expect("Failed to initiate request transmission");
    //
    //         // Wait for transmission completion
    //         match with_timeout(Duration::from_millis(500), uwb_irq.wait_for_rising_edge()).await {
    //             Ok(()) => {
    //                 sending.wait_transmit().expect("Waiting for transmit failed");
    //             }
    //             Err(_) => {
    //                 defmt::info!("Timeout waiting for transmit completion");
    //             }
    //         }
    //         dw1000 = sending.finish_sending().expect("Finishing sending failed");
    //         continue;
    //     }
    //
    //     let response = ranging::Response::decode::<ExclusiveDevice<Spi<Async>, Output, Delay>>(&message)
    //         .expect("Failed to decode response");
    //     if let Some(response) = response {
    //         // Received ranging response from anchor. Now we can compute the distance.
    //         defmt::info!("Received ranging response");
    //
    //         // Flash LED to indicate response received
    //         led.set_high();
    //         Timer::after_millis(10).await;
    //         led.set_low();
    //
    //         // If this is not a PAN ID and short address, it doesn't
    //         // come from a compatible node. Ignore it.
    //         let (pan_id, addr) = match response.source {
    //             Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
    //             _ => {
    //                 defmt::info!("Ignoring response with incompatible address format");
    //                 continue;
    //             }
    //         };
    //
    //         // Ranging response received. Compute distance.
    //         let distance_mm = ranging::compute_distance_mm(&response).unwrap();
    //         let distance_cm = correct_ch_5_prf_16(distance_mm / 10);
    //
    //         // Flash LED to indicate successful ranging
    //         led.set_high();
    //         Timer::after_millis(10).await;
    //         led.set_low();
    //
    //         defmt::info!("{:04x}:{:04x} - {} mm", pan_id.0, addr.0, distance_cm);
    //
    //         continue;
    //     }
    //
    //     defmt::info!("Ignored message that was neither ping nor response");
    // }

}

pub fn correct_ch_5_prf_16(dist_cm: u64) -> i64 {
    let dist_cm = i64::try_from(dist_cm).unwrap();
    let correction = if dist_cm <= 25 { 0 }
    else if dist_cm <= 50 { 2 }
    else if dist_cm <= 75 { 3 }
    else if dist_cm <= 100 { 4 }
    else if dist_cm <= 125 { 5 }
    else if dist_cm <= 150 { 6 }
    else if dist_cm <= 175 { 8 }
    else if dist_cm <= 200 { 9 }
    else if dist_cm <= 225 { 10 }
    else if dist_cm <= 275 { 11 }
    else if dist_cm <= 300 { 12 }
    else if dist_cm <= 350 { 13 }
    else if dist_cm <= 375 { 14 }
    else if dist_cm <= 400 { 15 }
    else if dist_cm <= 450 { 16 }
    else if dist_cm <= 500 { 17 }
    else if dist_cm <= 525 { 18 }
    else if dist_cm <= 575 { 19 }
    else if dist_cm <= 625 { 20 }
    else if dist_cm <= 675 { 21 }
    else if dist_cm <= 725 { 22 }
    else if dist_cm <= 775 { 23 }
    else if dist_cm <= 850 { 24 }
    else if dist_cm <= 900 { 25 }
    else if dist_cm <= 950 { 26 }
    else if dist_cm <= 1025 { 27 }
    else if dist_cm <= 1100 { 28 }
    else if dist_cm <= 1200 { 29 }
    else if dist_cm <= 1325 { 30 }
    else if dist_cm <= 1475 { 31 }
    else if dist_cm <= 1700 { 32 }
    else if dist_cm <= 2075 { 33 }
    else if dist_cm <= 3000 { 34 }
    else if dist_cm <= 3700 { 35 }
    else { 36 };
    dist_cm + (-23 + correction)
}
