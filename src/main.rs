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

mod peripherals;

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
// Print panic message to probe console
use {defmt_rtt as _, panic_probe as _};

use crate::peripherals::battery::{BatteryMonitor, SingleCellLiIonBatteryMonitor};
use crate::peripherals::bootstrap::{BootstrapDevice, STM6600BootstrapDevice};
use crate::peripherals::led::{IndicatorLed, LtstIndicatorLed};
use embassy_stm32::adc::Adc;
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
    let p = embassy_stm32::init(config);

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
    charger_en1.set_high();
    charger_en2.set_low();

    info!("Anchor initialization complete");

    loop {
        Timer::after(Duration::from_millis(100)).await;
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
