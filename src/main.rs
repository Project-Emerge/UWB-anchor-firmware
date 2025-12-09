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
use embassy_futures::join::join;
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
use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    usb, Config, Peri,
};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;

bind_interrupts!(struct Irqs {
    USB => usb::InterruptHandler<embassy_stm32::peripherals::USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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
            divr: Some(PllRDiv::DIV2), // sysclk 80Mhz (16 / 1 * 10 / 2)
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
    // class.write_packet("STM6600 Bootstrap complete\r\n".as_bytes()).await.expect("Can't send message");

    // Initialize Battery Monitor
    let adc = Adc::new(p.ADC1);
    let battery_read_pin: Peri<PA3> = p.PA3;
    let mut battery_monitor = SingleCellLiIonBatteryMonitor::new(adc, battery_read_pin);
    let fist_voltage = battery_monitor
        .read_voltage_mv()
        .await
        .expect("Can't read Voltage");
    let fist_percentage = battery_monitor
        .read_percentage()
        .await
        .expect("Can't read Percentage");
    defmt::info!(
        "Battery monitor initialized with voltage: {} mV -- {}",
        fist_voltage,
        fist_percentage
    );

    // Initialize Indicator Led
    let orange_pin = Output::new(p.PA8, Level::Low, Speed::Low);
    let green_pin = Output::new(p.PA9, Level::Low, Speed::Low);
    let mut indicator_led = LtstIndicatorLed::new(orange_pin, green_pin);
    indicator_led
        .show_battery_percentage(fist_percentage)
        .expect("Can't set initial LED state");

    charger_enable.set_low();

    // Set recharge current to 500mA
    charger_en1.set_high();
    charger_en2.set_low();

    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}
