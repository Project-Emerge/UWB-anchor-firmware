use crate::peripherals::led::IndicatorLedError::LedOperationFailed;
use embedded_hal::digital::OutputPin;

#[derive(Debug)]
pub enum IndicatorLedError<Err> {
    LedOperationFailed(Err),
}

pub trait IndicatorLed {
    type Error;
    fn show_battery_percentage(
        &mut self,
        percentage: u8,
    ) -> Result<(), IndicatorLedError<Self::Error>>;
}

pub struct LtstIndicatorLed<GreenPin: OutputPin, OrangePin: OutputPin> {
    green_led: GreenPin,
    orange_led: OrangePin,
}

impl<GreenPin: OutputPin, OrangePin: OutputPin> LtstIndicatorLed<GreenPin, OrangePin> {
    pub fn new(green_led: GreenPin, orange_led: OrangePin) -> Self {
        Self {
            green_led,
            orange_led,
        }
    }
}

// Error type to handle both pin errors
#[derive(Debug)]
pub enum PinError<E1, E2> {
    GreenPin(E1),
    OrangePin(E2),
}

impl<GreenPin: OutputPin, OrangePin: OutputPin> IndicatorLed
    for LtstIndicatorLed<GreenPin, OrangePin>
where
    GreenPin::Error: core::fmt::Debug,
    OrangePin::Error: core::fmt::Debug,
{
    type Error = PinError<GreenPin::Error, OrangePin::Error>;

    fn show_battery_percentage(
        &mut self,
        percentage: u8,
    ) -> Result<(), IndicatorLedError<Self::Error>> {
        if percentage > 20 {
            self.green_led
                .set_high()
                .map_err(|e| LedOperationFailed(PinError::GreenPin(e)))?;
            self.orange_led
                .set_low()
                .map_err(|e| LedOperationFailed(PinError::OrangePin(e)))?;
        } else {
            self.green_led
                .set_low()
                .map_err(|e| LedOperationFailed(PinError::GreenPin(e)))?;
            self.orange_led
                .set_high()
                .map_err(|e| LedOperationFailed(PinError::OrangePin(e)))?;
        }
        Ok(())
    }
}
