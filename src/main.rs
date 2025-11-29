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

use embassy_executor::Spawner;
use embassy_time::Timer;
// Print panic message to probe console
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::{bind_interrupts, gpio::{Level, Output, Speed}, usb, Config, Peri};
use embassy_stm32::adc::{Adc, Averaging, SampleTime};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::peripherals::PA3;
use crate::peripherals::battery::{BatteryMonitor, SingleCellLiIonBatteryMonitor};
use crate::peripherals::bootstrap::{BootstrapDevice, STM6600BootstrapDevice};
use crate::peripherals::led::{IndicatorLed, LtstIndicatorLed};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("Launching anchor");

    let p = embassy_stm32::init(Config::default());
    let mut charger_enable = Output::new(p.PA10, Level::Low, Speed::Low);
    let mut charger_en1 = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut charger_en2 = Output::new(p.PB1, Level::Low, Speed::Low);

    // Bootstrap the STM6600 power management IC
    let power_int = ExtiInput::new(p.PB7, p.EXTI7, Pull::Up);
    let ps_hold = Output::new(p.PB5, Level::Low, Speed::Low);
    let mut bootstrap = STM6600BootstrapDevice::new(power_int, ps_hold);
    bootstrap.initialize().await.expect("Can't initialize bootstrap");

    // Initialize Battery Monitor
    // let adc = Adc::new(p.ADC1);
    // let battery_read_pin: Peri<PA3> = p.PA3;
    // let mut battery_monitor = SingleCellLiIonBatteryMonitor::new(adc, battery_read_pin);
    // let fist_voltage = battery_monitor.read_voltage_mv().await.expect("Can't read Voltage");
    // let fist_percentage = battery_monitor.read_percentage().await.expect("Can't read Percentage");
    // defmt::info!("Battery monitor initialized with voltage: {} mV -- {}", fist_voltage, fist_percentage);

    // Initialize Indicator Led
    let mut orange_pin = Output::new(p.PA8, Level::Low, Speed::Low);
    let mut green_pin = Output::new(p.PA9, Level::Low, Speed::Low);
    // let mut indicator_led = LtstIndicatorLed::new(orange_pin, green_pin);
    // indicator_led.show_battery_percentage(fist_percentage).expect("Can't set initial LED state");

    orange_pin.set_high();
    green_pin.set_high();

    charger_enable.set_low();

    // Set recharge current to 500mA
    charger_en1.set_high();
    charger_en2.set_low();

    loop {
        Timer::after_millis(1000).await;
    }
}
