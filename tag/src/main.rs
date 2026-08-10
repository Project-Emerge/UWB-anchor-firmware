//! Temporary DWM3000 tag used to validate anchor ranging.
//!
//! The tag listens continuously, answers only polls addressed to its static
//! short address, and computes the asymmetric DS-TWR distance on reception of
//! the anchor response. Robot firmware can consume the resulting
//! `(anchor_id, distance_mm)` measurements for trilateration.

#![allow(dead_code)]
#![no_main]
#![no_std]

use defmt::{debug, info, warn};
use dw3000_ng::{hl::SendTime, time::Instant, Config, DW3000};
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::PA3;
use embassy_stm32::spi::Config as SpiConfig;
use embassy_stm32::time::Hertz;
use embassy_stm32::{adc::Adc, gpio::Flex, mode::Async, spi::Spi, Config as StmConfig, Peri};
use embassy_time::{with_timeout, Delay, Duration, Timer};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use smoltcp::wire::{Ieee802154Address, Ieee802154Pan};
use uwb_anchor::peripherals;
use uwb_anchor::uwb::{self, delayed_after, radio_config, Packet, PAN_ID, RX_TIMEOUT_US, TAG_IDS};
use {defmt_rtt as _, panic_probe as _};

use peripherals::battery::{BatteryMonitor, SingleCellLiIonBatteryMonitor};
use peripherals::bootstrap::{BootstrapDevice, STM6600BootstrapDevice};
use peripherals::led::{IndicatorLed, LtstIndicatorLed};

#[cfg(feature = "tag-1")]
const TAG_INDEX: usize = 0;
#[cfg(feature = "tag-2")]
const TAG_INDEX: usize = 1;
#[cfg(feature = "tag-3")]
const TAG_INDEX: usize = 2;
#[cfg(feature = "tag-4")]
const TAG_INDEX: usize = 3;
#[cfg(feature = "tag-5")]
const TAG_INDEX: usize = 4;
#[cfg(feature = "tag-6")]
const TAG_INDEX: usize = 5;
#[cfg(feature = "tag-7")]
const TAG_INDEX: usize = 6;
#[cfg(feature = "tag-8")]
const TAG_INDEX: usize = 7;
#[cfg(feature = "tag-9")]
const TAG_INDEX: usize = 8;
#[cfg(feature = "tag-10")]
const TAG_INDEX: usize = 9;
#[cfg(feature = "tag-11")]
const TAG_INDEX: usize = 10;
#[cfg(feature = "tag-12")]
const TAG_INDEX: usize = 11;

const TAG_ID: u16 = TAG_IDS[TAG_INDEX];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Launching DWM3000 test tag {:04x}", TAG_ID);

    let mut config = StmConfig::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            // 160 MHz VCO / 2 = 80 MHz, the STM32L432's maximum. The ranging
            // rate is limited by how fast a reply can be programmed after a
            // frame arrives, and most of that turnaround is MCU-side work
            // (DMA setup, the driver's busy-wait loops) rather than SPI bits,
            // so the core clock matters as much as the SPI clock. It also
            // lifts PCLK2 to 80 MHz, which is what allows the faster SPI below.
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.mux.clk48sel = mux::Clk48sel::HSI48;
        config.rcc.mux.adcsel = mux::Adcsel::SYS;
    }
    let p = embassy_stm32::init(config);

    let mut charger_enable = Output::new(p.PA10, Level::Low, Speed::Low);
    let mut charger_en1 = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut charger_en2 = Output::new(p.PB1, Level::Low, Speed::Low);

    let power_int = ExtiInput::new(p.PB7, p.EXTI7, Pull::Up);
    // Assert PS_HOLD immediately: on a debugger-triggered boot (no fresh
    // button press) the power_int edge below may never come, and the
    // STM6600 cuts power if it sees PS_HOLD low for too long.
    let ps_hold = Output::new(p.PB5, Level::High, Speed::Low);
    let mut bootstrap = STM6600BootstrapDevice::new(power_int, ps_hold);
    bootstrap
        .initialize()
        .await
        .expect("Can't initialize bootstrap");

    let adc = Adc::new(p.ADC1);
    let battery_read_pin: Peri<PA3> = p.PA3;
    let red_pin = Output::new(p.PA8, Level::Low, Speed::Low);
    let green_pin = Output::new(p.PA9, Level::Low, Speed::Low);
    spawner
        .spawn(monitor_battery(adc, battery_read_pin, green_pin, red_pin))
        .expect("Failed to spawn battery monitor task");

    // Preserve the original tag's USB100 charging mode.
    charger_enable.set_low();
    charger_en1.set_low();
    charger_en2.set_low();

    // embassy defaults to 1 MHz, which makes the driver's 129-byte frame-buffer
    // transfers slow enough to blow every DS-TWR reply deadline. The DW3000
    // only requires SPI below 7 MHz while it is in INIT_RC, so start at 5 MHz
    // and raise it once the clock PLL is locked (see below).
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(5_000_000);
    let spi: Spi<'_, Async> = Spi::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH3,
        p.DMA1_CH2,
        spi_config,
    );
    let cs = Output::new(p.PA4, Level::High, Speed::High);
    let spi_device = ExclusiveDevice::new(spi, cs, Delay).expect("Unable to get SPI device");
    let mut uwb_irq = ExtiInput::new(p.PA2, p.EXTI2, Pull::None);
    let mut uwb_reset = Flex::new(p.PA1);

    uwb_reset.set_as_output(Speed::Low);
    uwb_reset.set_low();
    Timer::after_millis(10).await;
    uwb_reset.set_as_input(Pull::None);
    Timer::after_millis(5).await;

    let phy = radio_config();
    let radio = DW3000::new(spi_device)
        .init()
        .await
        .expect("DWM3000 init failed")
        .config(phy, Delay)
        .await
        .expect("DWM3000 configuration failed");
    let mut radio = radio;

    // init() and config() have now locked the clock PLL, so the INIT_RC limit
    // no longer applies and the DW3000 accepts up to 38 MHz. Everything on the
    // ranging critical path is register and frame-buffer traffic, so this is
    // the single biggest lever on the achievable update rate.
    {
        let bus = radio.ll().spi.bus_mut();
        let mut fast = bus.get_current_config();
        fast.frequency = Hertz(20_000_000);
        bus.set_config(&fast).expect("Unable to raise SPI frequency");
    }

    radio
        .set_address(
            Ieee802154Pan(PAN_ID),
            Ieee802154Address::Short(TAG_ID.to_be_bytes()),
        )
        .await
        .expect("Unable to configure tag address");
    radio
        // Note the driver takes (rx, tx), not (tx, rx).
        .set_antenna_delay(uwb::RX_ANTENNA_DELAY, uwb::TX_ANTENNA_DELAY)
        .await
        .expect("Unable to configure antenna delay");
    radio
        .enable_rx_interrupts()
        .await
        .expect("Unable to enable RX IRQ");
    radio
        .enable_tx_interrupts()
        .await
        .expect("Unable to enable TX IRQ");

    loop {
        let (next_radio, received) = receive_packet(
            radio,
            &mut uwb_irq,
            phy,
            Duration::from_micros(uwb::SUPERFRAME_US.into()),
        )
        .await;
        radio = next_radio;
        let Some((packet, poll_rx)) = received else {
            debug!("No frame received this superframe");
            continue;
        };
        match packet {
            Packet::Sync { sequence } => debug!("RX Sync seq={}", sequence),
            Packet::Poll {
                anchor_id, tag_id, ..
            } => debug!("RX Poll anchor={:04x} tag={:04x}", anchor_id, tag_id),
            Packet::Request { anchor_id, .. } => debug!("RX Request anchor={:04x}", anchor_id),
            Packet::Response { anchor_id, .. } => debug!("RX Response anchor={:04x}", anchor_id),
        }

        let Packet::Poll {
            anchor_id,
            tag_id,
            sequence,
            poll_tx: _,
        } = packet
        else {
            continue;
        };
        if tag_id != TAG_ID {
            continue;
        }

        // The reply deadline is measured from the hardware receive timestamp, so
        // report how much of REPLY_DELAY_US the turnaround actually consumes.
        if let Some(elapsed) = uwb::elapsed_since_us(&mut radio, poll_rx).await {
            debug!(
                "reply setup: {} us of {} us budget",
                elapsed,
                uwb::REPLY_DELAY_US
            );
        }

        let request_tx = delayed_after(poll_rx, uwb::REPLY_DELAY_US);
        let mut buffer = [0; Packet::MAX_LEN];
        let len = Packet::Request {
            anchor_id,
            tag_id: TAG_ID,
            sequence,
            poll_rx: poll_rx.value(),
            // The frame cannot carry a timestamp read after its own
            // transmission, so predict what the radio will report.
            request_tx: uwb::delayed_tx_timestamp(request_tx),
        }
        .encode(&mut buffer);
        let (next_radio, sent) = send_packet(
            radio,
            &mut uwb_irq,
            &buffer[..len],
            SendTime::Delayed(request_tx),
            phy,
            Duration::from_micros(RX_TIMEOUT_US.into()),
        )
        .await;
        radio = next_radio;
        // Range against the timestamp the radio actually reported rather than
        // the instant we programmed; the two differ by the TX antenna delay.
        let Some(actual_request_tx) = sent else {
            warn!("Ranging request TX failed");
            continue;
        };

        let (next_radio, received) = receive_packet(
            radio,
            &mut uwb_irq,
            phy,
            Duration::from_micros(RX_TIMEOUT_US.into()),
        )
        .await;
        radio = next_radio;
        let Some((packet, response_rx)) = received else {
            warn!("No ranging response from anchor {:04x}", anchor_id);
            continue;
        };

        let Packet::Response {
            anchor_id: response_anchor,
            tag_id: response_tag,
            sequence: response_sequence,
            poll_tx: response_poll_tx,
            request_rx,
            response_tx,
        } = packet
        else {
            continue;
        };
        if response_anchor != anchor_id || response_tag != TAG_ID || response_sequence != sequence {
            continue;
        }

        match uwb::distance_mm(
            response_poll_tx,
            poll_rx.value(),
            actual_request_tx.value(),
            request_rx,
            response_tx,
            response_rx.value(),
        ) {
            Some(distance) => info!("anchor {:04x}: {} mm", anchor_id, distance),
            None => warn!("Invalid DS-TWR timestamps from anchor {:04x}", anchor_id),
        }
    }
}

async fn send_packet<SPI>(
    radio: DW3000<SPI, dw3000_ng::Ready>,
    irq: &mut ExtiInput<'_>,
    data: &[u8],
    when: SendTime,
    config: Config,
    timeout: Duration,
) -> (DW3000<SPI, dw3000_ng::Ready>, Option<Instant>)
where
    SPI: SpiDevice<u8>,
{
    let mut sending = radio
        .send(data, when, config)
        .await
        .expect("DWM3000 send setup failed");
    let timestamp = if with_timeout(timeout, irq.wait_for_rising_edge())
        .await
        .is_ok()
    {
        match sending.s_wait().await {
            Ok(instant) => Some(instant),
            Err(_) => {
                warn!("TX rejected by radio after IRQ");
                None
            }
        }
    } else {
        // A delayed send whose instant has already passed never transmits, so
        // this is the symptom of a missed deadline as well as of a dead IRQ.
        debug!("TX IRQ timeout (missed delayed-send deadline?)");
        None
    };
    let radio = sending
        .finish_sending()
        .await
        .expect("DWM3000 send recovery failed");
    (radio, timestamp)
}

async fn receive_packet<SPI>(
    mut radio: DW3000<SPI, dw3000_ng::Ready>,
    irq: &mut ExtiInput<'_>,
    config: Config,
    timeout: Duration,
) -> (DW3000<SPI, dw3000_ng::Ready>, Option<(Packet, Instant)>)
where
    SPI: SpiDevice<u8>,
{
    // Release the IRQ line before arming the receiver, or a flag left over from
    // an earlier failed receive would mean no rising edge ever arrives.
    if let Some(error) = uwb::take_rx_error(&mut radio).await {
        debug!("Cleared stale RX flag before receive: {}", error);
    }

    let mut receiving = radio
        .receive(config)
        .await
        .expect("DWM3000 receive setup failed");
    if with_timeout(timeout, irq.wait_for_rising_edge())
        .await
        .is_err()
    {
        let mut radio = receiving
            .finish_receiving()
            .await
            .expect("DWM3000 receive recovery failed");
        if let Some(error) = uwb::take_rx_error(&mut radio).await {
            debug!("RX aborted: {}", error);
        }
        return (radio, None);
    }
    let mut buffer = [0; 127];
    let received = receiving
        .r_wait(&mut buffer)
        .await
        .ok()
        .and_then(|message| {
            let payload = message.frame.payload()?;
            // Distinguish "nothing on air" from "frame arrived but rejected":
            // folding both into None made a decode bug look like radio silence.
            match Packet::decode(payload) {
                Ok(packet) => Some((packet, message.rx_time)),
                Err(_) => {
                    debug!("Undecodable payload ({} bytes)", payload.len());
                    None
                }
            }
        });
    let mut radio = receiving
        .finish_receiving()
        .await
        .expect("DWM3000 receive recovery failed");
    // Always clear: r_wait only resets the flags when it succeeds, and a
    // leftover flag would also block the *next* transmission's IRQ wait.
    if let Some(error) = uwb::take_rx_error(&mut radio).await {
        if received.is_none() {
            debug!("RX failed: {}", error);
        }
    }
    (radio, received)
}

#[embassy_executor::task]
async fn monitor_battery(
    adc: Adc<'static, embassy_stm32::peripherals::ADC1>,
    peri: Peri<'static, PA3>,
    green_pin: Output<'static>,
    red_pin: Output<'static>,
) {
    let mut battery_monitor = SingleCellLiIonBatteryMonitor::new(adc, peri);
    let mut indicator_led = LtstIndicatorLed::new(green_pin, red_pin);
    loop {
        let percentage = battery_monitor
            .read_percentage()
            .await
            .expect("Can't read Percentage");
        debug!("Battery Percentage: {}%", percentage);
        indicator_led
            .show_battery_percentage(percentage)
            .expect("Can't set LED state");
        Timer::after_secs(1).await;
    }
}
