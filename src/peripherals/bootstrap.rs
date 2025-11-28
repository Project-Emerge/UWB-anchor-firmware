use core::convert::Infallible;
use embassy_stm32::exti::ExtiInput;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin};

#[derive(Debug)]
pub enum BootstrapError<Err> {
    InitializationFailed(Err),
}

pub trait BootstrapDevice {
    type Error;
    async fn initialize(&mut self) -> Result<(), BootstrapError<Self::Error>>;
}

pub struct STM6600BootstrapDevice<'a, PsHold>
where
    PsHold: OutputPin,
{
    power_int: ExtiInput<'a>,
    ps_hold: PsHold,
}
impl <'a, PsHold> STM6600BootstrapDevice<'a, PsHold>
where
    PsHold: OutputPin,
{
    pub fn new(power_int: ExtiInput<'a>, ps_hold: PsHold) -> Self {
        Self { power_int, ps_hold }
    }
}

impl <'a, PsHold> BootstrapDevice for STM6600BootstrapDevice<'a, PsHold>
where
    PsHold: OutputPin,
{
    type Error = <PsHold as ErrorType>::Error;
    async fn initialize(&mut self) -> Result<(), BootstrapError<Self::Error>> {
        // Initialization code specific to STM6600
        defmt::info!("Initializing STM6600 Bootstrap Device");
        self.power_int.wait_for_high().await;
        self.ps_hold.set_high().map_err(BootstrapError::InitializationFailed)
    }
}
