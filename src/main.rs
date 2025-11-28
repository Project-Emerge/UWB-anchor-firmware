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

use cortex_m::asm::bootstrap;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
// Print panic message to probe console
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::{
    gpio::{Level, Output, Speed},
};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use crate::peripherals::bootstrap::{BootstrapDevice, STM6600BootstrapDevice};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("Launching anchor");

    let p = embassy_stm32::init(Default::default());
    let power_int = ExtiInput::new(p.PB7, p.EXTI7, Pull::Up);
    let ps_hold = Output::new(p.PB5, Level::Low, Speed::Low);
    let mut charger_enable = Output::new(p.PA10, Level::Low, Speed::Low);
    let mut charger_en1 = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut charger_en2 = Output::new(p.PB1, Level::Low, Speed::Low);
    let mut led_r = Output::new(p.PA9, Level::Low, Speed::Low);

    let mut bootstrap = STM6600BootstrapDevice::new(power_int, ps_hold);
    bootstrap.initialize().await.expect("can't initialize bootstrap");

    charger_enable.set_low();

    // Set recharge current to 500mA
    charger_en1.set_high();
    charger_en2.set_low();

    Timer::after(Duration::from_millis(1000)).await;
    led_r.set_high();

    loop {
        Timer::after_millis(1000).await;
    }
}
