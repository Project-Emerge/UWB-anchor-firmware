//! DWM3000 indoor-positioning anchor.
//!
//! The first anchor is the TDMA master. It emits a synchronization frame; all
//! anchors use its hardware timestamp to schedule their own poll slots. A tag
//! completes asymmetric double-sided TWR and calculates the distance locally.

#![allow(dead_code)]
#![no_main]
#![no_std]

mod peripherals;

use defmt::{debug, info, warn};
use dw3000_ng::{hl::SendTime, time::Instant, Config, DW3000};
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::PA3;
use embassy_stm32::spi::Config as SpiConfig;
use embassy_stm32::time::Hertz;
use embassy_stm32::{adc::Adc, gpio::Flex, spi::Spi, Config as StmConfig, Peri};
use embassy_time::{with_timeout, Delay, Duration, Timer};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use smoltcp::wire::{Ieee802154Address, Ieee802154Pan};
use uwb_anchor::uwb::{
    self, anchor_slot, delayed_after, radio_config, Packet, ACTIVE_TAG_COUNT, ANCHOR_IDS,
    MASTER_ANCHOR_ID, MASTER_SYNC_WAIT_US, PAN_ID, RX_TIMEOUT_US, SUPERFRAME_US,
};
use {defmt_rtt as _, panic_probe as _};

use peripherals::battery::{BatteryMonitor, SingleCellLiIonBatteryMonitor};
use peripherals::bootstrap::{BootstrapDevice, STM6600BootstrapDevice};
use peripherals::led::{IndicatorLed, LtstIndicatorLed};

#[cfg(feature = "anchor-1")]
const ANCHOR_INDEX: usize = 0;
#[cfg(feature = "anchor-2")]
const ANCHOR_INDEX: usize = 1;
#[cfg(feature = "anchor-3")]
const ANCHOR_INDEX: usize = 2;
#[cfg(feature = "anchor-4")]
const ANCHOR_INDEX: usize = 3;

const ANCHOR_ID: u16 = ANCHOR_IDS[ANCHOR_INDEX];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Launching DWM3000 anchor {:04x}", ANCHOR_ID);

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

    // Preserve the BQ24074 USB500 configuration from the original anchor.
    charger_enable.set_low();
    charger_en1.set_high();
    charger_en2.set_low();

    // embassy defaults to 1 MHz, which makes the driver's 129-byte frame-buffer
    // transfers slow enough to blow every DS-TWR reply deadline. The DW3000
    // only requires SPI below 7 MHz while it is in INIT_RC, so start at 5 MHz
    // and raise it once the clock PLL is locked (see below).
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(5_000_000);
    let spi = Spi::new(
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

    // DWM3000 RSTn is open-drain. Drive only low, then release the pin; never
    // actively drive it high.
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
            Ieee802154Address::Short(ANCHOR_ID.to_be_bytes()),
        )
        .await
        .expect("Unable to configure anchor address");
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

    info!(
        "Anchor initialized; superframe {} us for {} tags and {} anchors",
        SUPERFRAME_US,
        ACTIVE_TAG_COUNT,
        uwb::ACTIVE_ANCHOR_COUNT
    );
    let mut sequence = 0u16;

    loop {
        let sync_base = if ANCHOR_ID == MASTER_ANCHOR_ID {
            let mut buffer = [0; Packet::MAX_LEN];
            let len = Packet::Sync { sequence }.encode(&mut buffer);
            let (next_radio, tx_time) = send_packet(
                radio,
                &mut uwb_irq,
                &buffer[..len],
                SendTime::Now,
                phy,
                Duration::from_micros(RX_TIMEOUT_US.into()),
            )
            .await;
            radio = next_radio;
            let Some(tx_time) = tx_time else {
                warn!("TDMA sync TX failed; retrying next superframe");
                continue;
            };
            tx_time
        } else {
            loop {
                let (next_radio, received) = receive_packet(
                    radio,
                    &mut uwb_irq,
                    phy,
                    Duration::from_micros(SUPERFRAME_US.into()),
                )
                .await;
                radio = next_radio;
                let Some((packet, rx_time)) = received else {
                    debug!("No Sync received this superframe");
                    continue;
                };
                // Adopt whatever sequence the master actually broadcast
                // rather than requiring it to match our stale local counter:
                // a single missed Sync (RF loss, or this anchor booting
                // after the master already advanced) would otherwise wait
                // forever for a sequence number the master never resends.
                if let Packet::Sync {
                    sequence: received_sequence,
                } = packet
                {
                    debug!("RX Sync seq={}", received_sequence);
                    sequence = received_sequence;
                    break rx_time;
                } else {
                    debug!("RX non-Sync frame while waiting for Sync");
                }
            }
        };

        for tag_index in 0..ACTIVE_TAG_COUNT {
            let tag_id = uwb::TAG_IDS[tag_index];
            let slot_delay_us = anchor_slot(ANCHOR_INDEX, tag_index);
            let poll_tx = delayed_after(sync_base, slot_delay_us);
            let mut buffer = [0; Packet::MAX_LEN];
            let len = Packet::Poll {
                anchor_id: ANCHOR_ID,
                tag_id,
                sequence,
                poll_tx: poll_tx.value(),
            }
            .encode(&mut buffer);

            // The DW3000 doesn't actually fire this TX until slot_delay_us
            // after sync_base (that's the whole point of a delayed send), so
            // the wait must cover that full delay plus the usual TX-done
            // margin - RX_TIMEOUT_US alone is only long enough for an
            // immediate send and made every delayed Poll TX time out before
            // it ever fired.
            let (next_radio, actual_poll_tx) = send_packet(
                radio,
                &mut uwb_irq,
                &buffer[..len],
                SendTime::Delayed(poll_tx),
                phy,
                Duration::from_micros((slot_delay_us + RX_TIMEOUT_US).into()),
            )
            .await;
            radio = next_radio;
            let Some(actual_poll_tx) = actual_poll_tx else {
                warn!("Poll TX failed for tag {:04x}", tag_id);
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
            let Some((packet, request_rx)) = received else {
                debug!("No request from tag {:04x}", tag_id);
                continue;
            };

            let Packet::Request {
                anchor_id,
                tag_id: request_tag,
                sequence: request_sequence,
                ..
            } = packet
            else {
                continue;
            };
            if anchor_id != ANCHOR_ID || request_tag != tag_id || request_sequence != sequence {
                continue;
            }

            // Same deadline as the tag's reply, measured from the hardware
            // receive timestamp rather than from MCU time.
            if let Some(elapsed) = uwb::elapsed_since_us(&mut radio, request_rx).await {
                debug!(
                    "response setup: {} us of {} us budget",
                    elapsed,
                    uwb::REPLY_DELAY_US
                );
            }

            let response_tx = delayed_after(request_rx, uwb::REPLY_DELAY_US);
            let len = Packet::Response {
                anchor_id: ANCHOR_ID,
                tag_id,
                sequence,
                poll_tx: actual_poll_tx.value(),
                request_rx: request_rx.value(),
                // The frame cannot carry a timestamp read after its own
                // transmission, so predict what the radio will report.
                response_tx: uwb::delayed_tx_timestamp(response_tx),
            }
            .encode(&mut buffer);
            let (next_radio, sent) = send_packet(
                radio,
                &mut uwb_irq,
                &buffer[..len],
                SendTime::Delayed(response_tx),
                phy,
                Duration::from_micros(RX_TIMEOUT_US.into()),
            )
            .await;
            radio = next_radio;
            if sent.is_none() {
                warn!("Response TX failed for tag {:04x}", tag_id);
            }
        }

        sequence = sequence.wrapping_add(1);
        if ANCHOR_ID == MASTER_ANCHOR_ID {
            // Wait through the remaining slave slots and the guard before
            // issuing the next sync; this is derived from the anchor/tag
            // table so it can't drift out of sync with SUPERFRAME_US.
            Timer::after_micros(MASTER_SYNC_WAIT_US.into()).await;
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
