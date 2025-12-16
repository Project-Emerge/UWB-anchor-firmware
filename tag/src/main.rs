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

use defmt::{info, warn};
use dw1000::{RxConfig, mac, ranging::{self, Message}};
use embassy_time::{Delay, with_timeout};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use uwb_anchor::peripherals;
// Print panic message to probe console
use {defmt_rtt as _, panic_probe as _};

use peripherals::battery::{BatteryMonitor, SingleCellLiIonBatteryMonitor};
use peripherals::bootstrap::{BootstrapDevice, STM6600BootstrapDevice};
use peripherals::led::{IndicatorLed, LtstIndicatorLed};
use embassy_stm32::{adc::Adc, gpio::Flex, mode::Async, spi::Spi};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::peripherals::PA3;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    Config, Peri,
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Launching anchor");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        }); // needed for USB
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV8), // sysclk 80Mhz (16 / 1 * 10 / 2)
        });
        config.rcc.mux.clk48sel = mux::Clk48sel::HSI48;
        config.rcc.mux.adcsel = mux::Adcsel::SYS; // Enable ADC clock from system clock
    }
    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);

    let mut charger_enable = Output::new(p.PA10, Level::Low, Speed::Low);
    let mut charger_en1 = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut charger_en2 = Output::new(p.PB1, Level::Low, Speed::Low);

    // Bootstrap the STM6600 power management IC
    let power_int = ExtiInput::new(p.PB7, p.EXTI7, Pull::Up);
    let ps_hold = Output::new(p.PB5, Level::Low, Speed::Low);
    let mut bootstrap = STM6600BootstrapDevice::new(power_int, ps_hold);
    bootstrap
        .initialize()
        .await
        .expect("Can't initialize bootstrap");

    // Initialize Battery Monitor
    let adc = Adc::new(p.ADC1);
    let battery_read_pin: Peri<PA3> = p.PA3;
    // Initialize Indicator Led
    let orange_pin = Output::new(p.PA8, Level::Low, Speed::Low);
    let green_pin = Output::new(p.PA9, Level::Low, Speed::Low);

    spawner
        .spawn(monitor_battery(adc, battery_read_pin, orange_pin, green_pin))
        .expect("Failed to spawn battery monitor task");

    charger_enable.set_low();

    // Set recharge current to 500mA
    charger_en1.set_low();
    charger_en2.set_low();

    let spi: Spi<'_, embassy_stm32::mode::Async> = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, Default::default());
    let cs = Output::new(p.PA4, Level::High, Speed::High);
    let spi_device = ExclusiveDevice::new(spi, cs, Delay).expect("Unable to get SPI device");
    let mut uwb_irq = ExtiInput::new(p.PA2, p.EXTI2, Pull::None);
    let mut uwb_reset = Flex::new(p.PA1);

    let dw1000 = dw1000::DW1000::new(spi_device);

    // spawner
    //     .spawn(manage_button())
    //     .expect("Failed to spawn button management task");

    // spawner
    //     .spawn(manage_uwb_antenna())
    //     .expect("Failed to spawn UWB antenna management task");

    info!("Anchor initialization complete");

    // Initialize UWB module
    uwb_reset.set_as_output(Speed::Low);
    uwb_reset.set_low();
    Timer::after(Duration::from_millis(100)).await;
    uwb_reset.set_high();
    uwb_reset.set_as_input(Pull::None);
    // Enable interrupts
    let mut dw1000 = dw1000.init(&mut Delay).unwrap();
    dw1000.enable_rx_interrupts().unwrap();
    dw1000.enable_tx_interrupts().unwrap();
    // Set antenna delays (hardcoded calibration values)
    dw1000.set_antenna_delay(16456, 16300).unwrap();
    // Set device address
    dw1000
        .set_address(
            mac::PanId(0x0d57),                 // hardcoded network id
            mac::ShortAddress(3344u16),         // random device address
        )
        .expect("Failed to set address");

    let mut buf = [0; 128];

    let mut frame_id = 0;
    let mut ping_id = 0;
    let mut last_ping_time = embassy_time::Instant::now();

    // Create indicator LEDs (if you have them available - adjust based on your hardware)
    // let mut led_d9 = Output::new(p.PX, Level::Low, Speed::Low);
    // let mut led_d10 = Output::new(p.PX, Level::Low, Speed::Low);
    // let mut led_d11 = Output::new(p.PX, Level::Low, Speed::Low);
    // let mut led_d12 = Output::new(p.PX, Level::Low, Speed::Low);

    loop {
        info!("waiting for base station ping");
        
        let mut receiving = dw1000
            .receive(RxConfig::default())
            .expect("Failed to receive message");

        // Wait for receive interrupt with 500ms timeout
        let result = with_timeout(Duration::from_millis(500), uwb_irq.wait_for_high()).await;

        let message = match result {
            Ok(_) => {
                match receiving.wait_receive(&mut buf) {
                    Ok(msg) => msg,
                    Err(_) => {
                        dw1000 = receiving
                            .finish_receiving()
                            .expect("Failed to finish receiving");
                        continue;
                    }
                }
            }
            Err(_) => {
                info!("Timeout error occured");
                dw1000 = receiving
                    .finish_receiving()
                    .expect("Failed to finish receiving");
                continue;
            }
        };

        dw1000 = receiving
            .finish_receiving()
            .expect("Failed to finish receiving");

        info!("msg from base station: received");

        // Try to decode as Ping
        let ping = ranging::Ping::decode::<ExclusiveDevice<Spi<'_, Async>, Output<'_>, Delay>>(&message)
            .expect("Failed to decode ping");
        
        if let Some(ping) = ping {
            // Received ping from an anchor. Reply with a ranging request.

            // Indicate ping received (LED D10)
            // led_d10.set_high();
            Timer::after_millis(10).await;
            // led_d10.set_low();

            // Wait for a moment, to give the anchor a chance to start listening for the reply.
            Timer::after_millis(10).await;

            let mut sending = ranging::Request::new::<ExclusiveDevice<embassy_stm32::spi::Spi<'_, embassy_stm32::mode::Async>, embassy_stm32::gpio::Output<'_>, Delay>, embassy_stm32::gpio::Output<'_>>(&mut dw1000, &ping)
                .expect("Failed to initiate request")
                .send::<ExclusiveDevice<Spi<Async>, Output, Delay>, Output>(dw1000)
                .expect("Failed to initiate request transmission");

            // Wait for transmission complete interrupt with timeout
            match with_timeout(Duration::from_millis(500), uwb_irq.wait_for_rising_edge()).await {
                Ok(()) => {
                    sending.wait_transmit().expect("Failed to send ranging request");
                }
                Err(_) => {
                    warn!("Timeout waiting for request transmit");
                    dw1000 = sending.finish_sending().expect("Failed to finish sending");
                    continue;
                }
            }

            dw1000 = sending.finish_sending().expect("Failed to finish sending");

            continue;
        }

        // Try to decode as Response
        let response = ranging::Response::decode::<ExclusiveDevice<Spi<'_, Async>, Output<'_>, Delay>>(&message)
            .expect("Failed to decode response");
        
        if let Some(response) = response {
            // Received ranging response from anchor. Now we can compute the distance.

            // Indicate response received (LED D11)
            // led_d11.set_high();
            Timer::after_millis(10).await;
            // led_d11.set_low();

            // If this is not a PAN ID and short address, it doesn't come from a compatible node. Ignore it.
            let (pan_id, addr) = match response.source {
                Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
                _ => continue,
            };

            // Ranging response received. Compute distance.
            let distance_mm = match ranging::compute_distance_mm(&response) {
                Ok(distance) => distance,
                Err(_) => {
                    warn!("Failed to compute distance");
                    continue;
                }
            };

            // Indicate distance computed (LED D9)
            // led_d9.set_high();
            Timer::after_millis(10).await;
            // led_d9.set_low();

            info!("{:04x}:{:04x} - {} mm\n", pan_id.0, addr.0, distance_mm,);

            continue;
        }

        info!("Ignored message that was neither ping nor response\n");
    }
}

#[embassy_executor::task]
async fn monitor_battery(adc: Adc<'static, embassy_stm32::peripherals::ADC1>, peri: Peri<'static, PA3>, orange_pin: Output<'static>, green_pin: Output<'static>) {
    let mut battery_monitor = SingleCellLiIonBatteryMonitor::new(adc, peri);
    let mut indicator_led = LtstIndicatorLed::new(orange_pin, green_pin);
    loop {
        let percentage = battery_monitor.read_percentage()
            .await
            .expect("Can't read Percentage");

        info!("Battery Percentage: {}%", percentage);
        indicator_led.show_battery_percentage(percentage).expect("Can't set LED state");
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn manage_button() {
    // Placeholder for button management logic
    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn manage_uwb_antenna() {
    loop {
        // Placeholder for UWB antenna management logic
        Timer::after(Duration::from_secs(10)).await;
    }
}
