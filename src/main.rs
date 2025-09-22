#![allow(dead_code)]
#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_stm32::qspi::{self, enums::{AddressSize, ChipSelectHighTime, FIFOThresholdLevel, MemorySize, SampleShifting}};
use embassy_time::Timer;
// Print panic message to probe console
use {defmt_rtt as _, panic_probe as _};

// use cortex_m_rt::entry;
// use stm32l4xx_hal::{
//     pac,
//     prelude::*,
// };

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _peripherials = embassy_stm32::init(Default::default());

    let mut config = qspi::Config::default();
    config.memory_size = MemorySize::_16MiB;
    config.address_size = AddressSize::_24bit;
    config.prescaler = 200;
    config.cs_high_time = ChipSelectHighTime::_1Cycle;
    config.fifo_threshold = FIFOThresholdLevel::_16Bytes;
    config.sample_shifting = SampleShifting::None;

    loop {
        Timer::after_millis(1000).await;
    }
}
