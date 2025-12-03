use core::convert::Infallible;
use embassy_stm32::adc::{Adc, AdcChannel, Instance, SampleTime};

pub trait BatteryMonitor {
    type Error;
    /// Reads the current battery voltage in millivolts.
    async fn read_voltage_mv(&mut self) -> Result<u16, Self::Error>;
    /// Reads the current battery percentage (0-100%).
    async fn read_percentage(&mut self) -> Result<u8, Self::Error>;
}

pub struct SingleCellLiIonBatteryMonitor<'a, T: Instance, P: AdcChannel<T>> {
    adc: Adc<'a, T>,
    channel: P,
    /// Moving average filter buffer to reduce ADC noise
    filter_buffer: [u16; 8],
    filter_index: usize,
    samples_collected: usize,
}

impl<'a, T: Instance, P: AdcChannel<T>> SingleCellLiIonBatteryMonitor<'a, T, P> {
    pub fn new(adc: Adc<'a, T>, channel: P) -> Self {
        Self {
            adc,
            channel,
            filter_buffer: [0; 8],
            filter_index: 0,
            samples_collected: 0,
        }
    }

    /// Read raw ADC value with moving average filtering
    async fn read_filtered_adc(&mut self) -> u16 {
        // Set sample time for accurate reading
        self.adc.set_sample_time(SampleTime::CYCLES12_5);

        // Read raw ADC value
        let raw_value: u16 = self.adc.blocking_read(&mut self.channel);

        // Update circular buffer
        self.filter_buffer[self.filter_index] = raw_value;
        self.filter_index = (self.filter_index + 1) % self.filter_buffer.len();

        if self.samples_collected < self.filter_buffer.len() {
            self.samples_collected += 1;
        }

        // Calculate moving average
        let sum: u32 = self.filter_buffer[..self.samples_collected]
            .iter()
            .map(|&x| x as u32)
            .sum();

        (sum / self.samples_collected as u32) as u16
    }
}

impl<'a, T: Instance, P: AdcChannel<T>> BatteryMonitor for SingleCellLiIonBatteryMonitor<'a, T, P> {
    type Error = Infallible;

    async fn read_voltage_mv(&mut self) -> Result<u16, Self::Error> {
        // Read filtered ADC value (moving average)
        let filtered_value = self.read_filtered_adc().await;

        // Convert raw ADC value to millivolts (assuming 3.3V reference and 12-bit ADC)
        let measured_mv = (filtered_value as u32 * 3300 / 4095) as u16;

        // Account for voltage divider: Vbat -- 10k -- Pin -- 20k -- GND
        // Voltage divider ratio: Vpin = Vbat * (20k / (10k + 20k)) = Vbat * (2/3)
        // Therefore: Vbat = Vpin * (3/2) = Vpin * 1.5
        let battery_mv = (measured_mv as u32 * 3 / 2) as u16;

        Ok(battery_mv)
    }

    async fn read_percentage(&mut self) -> Result<u8, Self::Error> {
        let voltage_mv = self.read_voltage_mv().await?;
        // Simple linear approximation for Li-Ion battery percentage
        // Typical single-cell Li-Ion: 4.2V (full) to 3.0V (empty)
        let percentage = if voltage_mv >= 4200 {
            100
        } else if voltage_mv <= 3000 {
            0
        } else {
            ((voltage_mv - 3000) * 100 / (4200 - 3000)) as u8
        };
        Ok(percentage)
    }
}