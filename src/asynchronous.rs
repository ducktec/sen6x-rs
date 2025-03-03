//! # Async API
//!
//! This module contains the async API for the SEN6X sensor modules.
//! It is based on the `embedded-hal-async` traits and is intended to be used
//! with asynchronous code.
//!
//! The methods usually return `Result` with the error type being `Sen6xError`.
//!
//! ## Examples
//!
//! ### Create a driver instance
//!
//! ```rust
//! use sen6x::r#async::Sen6X;
//! use embedded_hal_async::i2c::I2c;
//! use embedded_hal_async::delay::DelayNs;
//!
//! async fn example() {
//!     // These would be your actual implementations
//!     let i2c = MyI2C::new();
//!     let delay = MyDelay::new();
//!     
//!     // Create the Sen6X driver instance
//!     let mut sensor = Sen6X::new(delay, i2c);
//! }
//! ```
//!
//! ### Read sample
//!
//! ```rust
//! use sen6x::r#async::Sen6X;
//! use embedded_hal_async::i2c::I2c;
//! use embedded_hal_async::delay::DelayNs;
//!
//! async fn example() {
//!     // Initialize peripherals
//!     let i2c = MyI2C::new();
//!     let delay = MyDelay::new();
//!     
//!     // Create the Sen6X driver instance
//!     let mut sensor = Sen6X::new(delay, i2c);
//!     
//!     // Start continuous measurement (1 measurement per second)
//!     sensor.start_continuous_measurement().await.unwrap();
//!     
//!     // Example loop
//!     loop {
//!         // Check if new data is ready
//!         if sensor.get_is_data_ready().await.unwrap() {
//!             // Read the measured values
//!             let sample = sensor.get_sample().await.unwrap();
//!             
//!             // Print the values
//!             println!("PM1.0: {} μg/m³", sample.pm1p0);
//!             println!("PM2.5: {} μg/m³", sample.pm2p5);
//!             println!("PM4.0: {} μg/m³", sample.pm4p0);
//!             println!("PM10: {} μg/m³", sample.pm10p0);
//!             println!("Temperature: {} °C", sample.temperature);
//!             println!("Humidity: {} %RH", sample.humidity);
//!             println!("VOC index: {}", sample.voc_index);
//!             println!("NOx index: {}", sample.nox_index);
//!             
//!             // Exit the loop for this example
//!             break;
//!         }
//!         
//!         // Wait a bit before checking again
//!         // In a real application, you might use a more sophisticated approach
//!         sensor.delay.delay_ms(100).await;
//!     }
//!     
//!     // Stop measurement when done
//!     sensor.stop_measurement().await.unwrap();
//! }
//! ```

use crate::{
    AlgorithmTuningParameters, CommandId, DeviceStatus, MAX_RX_BYTES, MAX_TX_BYTES, MODULE_ADDR,
    MeasuredSample, ModuleState, RawConcentrationSample, RawMeasuredSample, Result, Sen6xError,
    TempAccelPars, TempOffsetPars, crc_internal, get_execution_time,
};

/// Represents an I2C-connected SEN6X sensor module with async operations.
pub struct Sen6X<I2C, D> {
    /// Delay provider for async delay operations.
    pub delay: D,

    /// I2C Interface for communicating with the module asynchronously.
    i2c: I2C,

    /// The current measurement state of the module
    state: ModuleState,
}

impl<I2C, D> Sen6X<I2C, D>
where
    D: embedded_hal_async::delay::DelayNs,
    I2C: embedded_hal_async::i2c::I2c,
{
    /// Creates SEN6X instance representation for async operation.
    pub fn new(delay: D, i2c: I2C) -> Self {
        Self {
            delay,
            i2c,
            state: ModuleState::Idle,
        }
    }

    /// Convenience function to raise error if we're measuring
    /// for commands that cannot be executed during measuring
    fn check_not_measuring(&mut self) -> Result<()> {
        if self.state == ModuleState::Measuring {
            return Err(Sen6xError::InvalidState);
        }
        Ok(())
    }

    /// Start continuous measurement (1/second)
    pub async fn start_continuous_measurement(&mut self) -> Result<()> {
        self.check_not_measuring()?;
        // Set state regardless of the result of the command (conservative)
        self.state = ModuleState::Measuring;
        self.send_wait(CommandId::StartContinuousMeasurement).await
    }

    /// Stops continuous measuring mode
    pub async fn stop_measurement(&mut self) -> Result<()> {
        let result = self.send_wait(CommandId::StopMeasurement).await;
        if result.is_ok() {
            self.state = ModuleState::Idle;
        }
        result
    }

    /// Check if new data can be retrieved from the sensor module
    pub async fn get_is_data_ready(&mut self) -> Result<bool> {
        let mut data = [0 as u16; 1];
        self.send_wait_read(CommandId::GetDataReady, &mut data)
            .await?;
        Ok(data[0] == 0x1)
    }

    /// Read the last measured values from the sensor module
    pub async fn get_sample(&mut self) -> Result<MeasuredSample> {
        self.check_not_measuring()?;

        #[cfg(any(feature = "sen66", feature = "sen68"))]
        let mut data = [0 as u16; 9];
        #[cfg(feature = "sen65")]
        let mut data = [0 as u16; 8];
        #[cfg(feature = "sen63c")]
        let mut data = [0 as u16; 7];
        self.send_wait_read(CommandId::ReadMeasuredValues, &mut data)
            .await?;

        Ok(MeasuredSample::from(data))
    }

    /// Read the last measured raw values from the sensor module
    ///
    /// This excludes the concentration raw values, which can be
    /// read with `get_raw_concentration_sample`.
    ///
    /// This is for advanced use only, normally you would use `get_sample`.
    pub async fn get_raw_sample(&mut self) -> Result<RawMeasuredSample> {
        self.check_not_measuring()?;

        #[cfg(any(feature = "sen65", feature = "sen68"))]
        let mut data = [0 as u16; 4];
        #[cfg(feature = "sen66")]
        let mut data = [0 as u16; 5];
        #[cfg(feature = "sen63c")]
        let mut data = [0 as u16; 2];
        self.send_wait_read(CommandId::ReadMeasuredRawValues, &mut data)
            .await?;

        Ok(RawMeasuredSample::from(data))
    }

    /// Read the last measured concentration values from the sensor module
    ///
    /// This only includes the concentration (PM) values, all other raw values
    /// can be read with `get_raw_sample`.
    ///
    /// This is for advanced use only, normally you would use `get_sample`.
    pub async fn get_raw_concentration_sample(&mut self) -> Result<RawConcentrationSample> {
        self.check_not_measuring()?;

        let mut data = [0 as u16; 4];
        self.send_wait_read(CommandId::ReadNumberConcentrationValues, &mut data)
            .await?;

        Ok(RawConcentrationSample::from(data))
    }

    /// Set the temperature offset parameters
    pub async fn set_temp_offset_pars(&mut self, temp_offset_pars: TempOffsetPars) -> Result<()> {
        let data: [u16; 4] = <[u16; 4]>::from(temp_offset_pars);
        self.send_write_wait(CommandId::SetTempOffsetPars, &data)
            .await
    }

    /// Set the temperature acceleration parameters
    pub async fn set_temp_accel_pars(&mut self, temp_accel_pars: TempAccelPars) -> Result<()> {
        let data: [u16; 4] = <[u16; 4]>::from(temp_accel_pars);
        self.send_write_wait(CommandId::SetTempAccelPars, &data)
            .await
    }

    /// Get the product name of the sensor module
    /// (to provide the string slice back, a 32-byte u32 buffer must be provided
    /// by the caller)
    pub async fn get_product_name<'a>(&mut self, buffer: &'a mut [u8; 32]) -> Result<&'a str> {
        let mut data = [0 as u16; 16];
        self.send_wait_read(CommandId::GetProductName, &mut data)
            .await?;

        let mut len = 0;
        for i in 0..data.len() {
            let bytes = data[i].to_be_bytes();

            // Check first byte
            if bytes[0] == 0 {
                break;
            }
            buffer[len] = bytes[0];
            len += 1;

            // Check second byte
            if bytes[1] == 0 {
                break;
            }
            buffer[len] = bytes[1];
            len += 1;
        }

        match core::str::from_utf8(&buffer[..len]) {
            Ok(s) => Ok(s),
            Err(_) => Err(Sen6xError::InvalidData),
        }
    }

    /// Return the module serial number as a string slice
    /// (to provide the string slice back, a 32-byte u32 buffer must be provided
    /// by the caller)
    pub async fn get_serial_number<'a>(&mut self, buffer: &'a mut [u8; 32]) -> Result<&'a str> {
        let mut data = [0 as u16; 16];
        self.send_wait_read(CommandId::GetSerialNumber, &mut data)
            .await?;

        let mut len = 0;
        for i in 0..data.len() {
            let bytes = data[i].to_be_bytes();

            // Check first byte
            if bytes[0] == 0 {
                break;
            }
            buffer[len] = bytes[0];
            len += 1;

            // Check second byte
            if bytes[1] == 0 {
                break;
            }
            buffer[len] = bytes[1];
            len += 1;
        }

        match core::str::from_utf8(&buffer[..len]) {
            Ok(s) => Ok(s),
            Err(_) => Err(Sen6xError::InvalidData),
        }
    }

    /// Read the device status (no clearing of flags!)
    pub async fn read_device_status(&mut self) -> Result<DeviceStatus> {
        let mut data = [0 as u16; 2];
        self.send_wait_read(CommandId::ReadDeviceStatus, &mut data)
            .await?;

        Ok(DeviceStatus::from(data))
    }

    /// Read the device status (no clearing of flags!)
    pub async fn read_and_clear_device_status(&mut self) -> Result<DeviceStatus> {
        let mut data = [0 as u16; 2];
        self.send_wait_read(CommandId::ReadAndClearDeviceStatus, &mut data)
            .await?;

        Ok(DeviceStatus::from(data))
    }

    /// Reset the sensor module
    pub async fn reset(&mut self) -> Result<()> {
        self.check_not_measuring()?;
        self.send_wait(CommandId::DeviceReset).await
    }

    /// Start the fan cleaning process
    pub async fn start_fan_cleaning(&mut self) -> Result<()> {
        self.check_not_measuring()?;
        self.send_wait(CommandId::StartFanCleaning).await
    }

    /// Activate the SHT heater
    pub async fn activate_sht_heater(&mut self) -> Result<()> {
        self.check_not_measuring()?;
        self.send_wait(CommandId::ActivateShtHeater).await
    }

    /// Get the VOC algorithm tuning parameters

    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub async fn get_voc_algo_tuning_parameters(&mut self) -> Result<AlgorithmTuningParameters> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 6];
        self.send_wait_read(CommandId::VocAlgoTuningPars, &mut data)
            .await?;

        Ok(AlgorithmTuningParameters::from(data))
    }

    /// Set the VOC algorithm tuning parameters
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub async fn set_voc_algo_tuning_parameters(
        &mut self,
        tuning_pars: AlgorithmTuningParameters,
    ) -> Result<()> {
        self.check_not_measuring()?;
        let data: [u16; 6] = <[u16; 6]>::from(tuning_pars);
        self.send_write_wait(CommandId::VocAlgoTuningPars, &data)
            .await
    }

    /// Set the VOC algorithm state
    ///
    /// Due to the long initialization phase of the algorithm and no sensor-internal
    /// persistent storing of it's state, the algorithm state ideally should be preserved
    /// between restarts.
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub async fn get_voc_algo_state(&mut self) -> Result<[u16; 4]> {
        let mut data = [0 as u16; 4];
        self.send_wait_read(CommandId::VocAlgoState, &mut data)
            .await?;

        Ok(data)
    }

    /// Set the VOC algorithm state
    ///
    /// Due to the long initialization phase of the algorithm and no sensor-internal
    /// persistent storing of it's state, the algorithm state ideally should be preserved
    /// between restarts.
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub async fn set_voc_algo_state(&mut self, state: [u16; 4]) -> Result<()> {
        self.check_not_measuring()?;
        self.send_write_wait(CommandId::VocAlgoState, &state).await
    }

    /// Get the NOx algorithm tuning parameters
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub async fn get_nox_algo_tuning_parameters(&mut self) -> Result<AlgorithmTuningParameters> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 6];
        self.send_wait_read(CommandId::NoxAlgoTuningPars, &mut data)
            .await?;

        Ok(AlgorithmTuningParameters::from(data))
    }

    /// Set the NOx algorithm tuning parameters
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub async fn set_nox_algo_tuning_parameters(
        &mut self,
        tuning_pars: AlgorithmTuningParameters,
    ) -> Result<()> {
        self.check_not_measuring()?;
        let data: [u16; 6] = <[u16; 6]>::from(tuning_pars);
        self.send_write_wait(CommandId::NoxAlgoTuningPars, &data)
            .await
    }

    /// Perform a forced CO2 recalibration
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    pub async fn perform_forced_co2_recalibration(
        &mut self,
        target_concentration: u16,
    ) -> Result<u16> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 1];
        self.send_write_wait_read(
            CommandId::PerformForcedCo2Recalibration,
            &[target_concentration],
            &mut data,
        )
        .await?;

        Ok(data[0])
    }

    /// Get the automatic self-calibration state of the CO2 sensor
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    pub async fn get_is_co2_auto_self_calibrated(&mut self) -> Result<bool> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 1];
        self.send_wait_read(CommandId::Co2SensorAutoCalibrationState, &mut data)
            .await?;

        Ok(data[0] == 0x1)
    }

    /// Enable or disable the automatic self-calibration of the CO2 sensor
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    pub async fn set_co2_auto_self_calibration(&mut self, enabled: bool) -> Result<()> {
        self.check_not_measuring()?;
        let data: [u16; 1] = [if enabled { 0x01 } else { 0x00 }];
        self.send_write_wait(CommandId::Co2SensorAutoCalibrationState, &data)
            .await
    }

    /// Get the ambient pressure in hPa that is assumed by the sensor
    /// (used by the sensor for pressure compensation)
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    pub async fn get_ambient_pressure(&mut self) -> Result<u16> {
        let mut data = [0 as u16; 1];
        self.send_wait_read(CommandId::AmbientPressure, &mut data)
            .await?;

        Ok(data[0])
    }

    /// Set the ambient pressure in hPa that is assumed by the sensor
    /// (used by the sensor for pressure compensation)
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    pub async fn set_ambient_pressure(&mut self, pressure: u16) -> Result<()> {
        self.send_write_wait(CommandId::AmbientPressure, &[pressure])
            .await
    }

    /// Get the altitude in meters that is assumed by the sensor
    /// (used by the sensor for pressure compensation)
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    pub async fn get_altitude(&mut self) -> Result<u16> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 1];
        self.send_wait_read(CommandId::SensorAltitude, &mut data)
            .await?;

        Ok(data[0])
    }

    /// Set the sensor altitude in meters that is assumed by the sensor
    /// (used by the sensor for pressure compensation)
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    pub async fn set_altitude(&mut self, altitude: u16) -> Result<()> {
        self.check_not_measuring()?;
        self.send_write_wait(CommandId::SensorAltitude, &[altitude])
            .await
    }

    /// Send a command to the sensor module and wait for the execution time associated to the command
    async fn send_wait(&mut self, command: CommandId) -> Result<()> {
        self.i2c
            .write(MODULE_ADDR, &(command as u16).to_be_bytes())
            .await
            .map_err(|_| Sen6xError::WriteI2CError)?;
        self.delay.delay_ms(get_execution_time(command)).await;
        Ok(())
    }

    /// Send a command to the sensor module, wait for the execution time associated to the command
    /// and read the data from the sensor module
    async fn send_wait_read(&mut self, command: CommandId, data: &mut [u16]) -> Result<()> {
        self.send_wait(command).await?;
        self.read(data).await
    }

    /// Read data from the sensor module (no command, just data, command must be sent separately)
    async fn read(&mut self, data: &mut [u16]) -> Result<()> {
        let mut raw_data = [0 as u8; MAX_RX_BYTES];
        let internal_length = data.len() * 3;

        self.i2c
            .read(MODULE_ADDR, &mut raw_data[0..internal_length])
            .await
            .map_err(|_| Sen6xError::ReadI2CError)?;

        // Validate return values
        match crc_internal::validate_and_extract_data(&raw_data[..internal_length], data) {
            Ok(_) => Ok(()),
            Err(e) => Err(Sen6xError::from(e)),
        }
    }

    /// Send command, write data to the sensor module and wait for the execution time associated to the command
    async fn send_write_wait(&mut self, command: CommandId, data: &[u16]) -> Result<()> {
        let mut raw_data = [0 as u8; MAX_TX_BYTES + 2];
        let internal_length = data.len() * 3 + 2;

        if internal_length > MAX_TX_BYTES + 2 {
            return Err(Sen6xError::TooMuchData);
        }

        // Encode command ID
        let command_bytes = (command as u16).to_be_bytes();
        raw_data[0] = command_bytes[0];
        raw_data[1] = command_bytes[1];

        // Fill raw data structure with data and CRCs
        for (i, &d) in data.iter().enumerate() {
            let crc = crc_internal::generate_crc(&d.to_be_bytes());
            let data_bytes = d.to_be_bytes();
            raw_data[i * 3 + 2] = data_bytes[0];
            raw_data[i * 3 + 3] = data_bytes[1];
            raw_data[i * 3 + 4] = crc;
        }

        self.i2c
            .write(MODULE_ADDR, &raw_data[0..internal_length])
            .await
            .map_err(|_| Sen6xError::WriteI2CError)?;
        self.delay.delay_ms(get_execution_time(command)).await;
        Ok(())
    }

    /// Send command, write some data in one transaction, then wait for the execution time
    /// associated to the command and finally the data from the sensor module without
    /// any command sent again
    async fn send_write_wait_read(
        &mut self,
        command: CommandId,
        data_in: &[u16],
        data_out: &mut [u16],
    ) -> Result<()> {
        self.send_write_wait(command, data_in).await?;
        self.read(data_out).await
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::{
        delay::NoopDelay as DelayMock,
        i2c::{Mock as I2cMock, Transaction as I2cTransaction},
    };
    use futures::executor::block_on;

    // Same helpers as before
    const CRC_ALGO: crc::Crc<u8> = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);

    // Helper macro to generate data bytes with correct CRC
    macro_rules! bytes_with_crc {
        ($msb:expr, $lsb:expr) => {{
            let data = [$msb, $lsb];
            let crc = CRC_ALGO.checksum(&data);
            vec![$msb, $lsb, crc]
        }};
    }

    // Helper to combine multiple bytes with CRC
    macro_rules! combine_bytes_with_crc {
        ($( [$msb:expr, $lsb:expr] ),*) => {{
            let mut result = Vec::new();
            $(
                let mut bytes = bytes_with_crc!($msb, $lsb);
                result.append(&mut bytes);
            )*
            result
        }};
    }

    #[test]
    fn test_new_async() {
        let i2c = I2cMock::new(&[]);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        // Verify the initial state
        assert_eq!(sensor.state, ModuleState::Idle);

        sensor.i2c.done();
    }

    #[test]
    fn test_start_continuous_measurement_async() {
        let expectations = [I2cTransaction::write(MODULE_ADDR, vec![0x00, 0x21])];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.start_continuous_measurement()).is_ok());
        assert_eq!(sensor.state, ModuleState::Measuring);

        sensor.i2c.done();
    }

    #[test]
    fn test_stop_measurement_async() {
        let expectations = [I2cTransaction::write(MODULE_ADDR, vec![0x01, 0x04])];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);
        sensor.state = ModuleState::Measuring;

        assert!(block_on(sensor.stop_measurement()).is_ok());
        assert_eq!(sensor.state, ModuleState::Idle);

        sensor.i2c.done();
    }

    #[test]
    fn test_get_is_data_ready_true_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x02, 0x02]),
            I2cTransaction::read(MODULE_ADDR, bytes_with_crc!(0x00, 0x01)),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let result = block_on(sensor.get_is_data_ready()).unwrap();
        assert_eq!(result, true);

        sensor.i2c.done();
    }

    #[test]
    fn test_get_is_data_ready_false_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x02, 0x02]),
            I2cTransaction::read(MODULE_ADDR, bytes_with_crc!(0x00, 0x00)),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let result = block_on(sensor.get_is_data_ready()).unwrap();
        assert_eq!(result, false);

        sensor.i2c.done();
    }
    #[cfg(feature = "sen63c")]
    #[test]
    fn test_get_sample_sen63c_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x03, 0x00]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x00, 0x0A], // PM1.0 = 1.0 μg/m³
                    [0x00, 0x0F], // PM2.5 = 1.5 μg/m³
                    [0x00, 0x14], // PM4.0 = 2.0 μg/m³
                    [0x00, 0x19], // PM10 = 2.5 μg/m³
                    [0x13, 0x88], // Humidity = 50.00 %RH (5000)
                    [0x09, 0xC4], // Temperature = 25.00 °C (2500)
                    [0x01, 0x90]  // CO2 = 400 ppm
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let sample = block_on(sensor.get_sample()).unwrap();

        assert_eq!(sample.pm1, 1.0);
        assert_eq!(sample.pm2_5, 1.5);
        assert_eq!(sample.pm4, 2.0);
        assert_eq!(sample.pm10, 2.5);
        assert_eq!(sample.humidity, 50.0);
        assert_eq!(sample.temperature, 25.0);
        assert_eq!(sample.co2, 400);

        sensor.i2c.done();
    }

    #[cfg(feature = "sen65")]
    #[test]
    fn test_get_sample__sen65_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x03, 0x00]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x00, 0x0A], // PM1.0 = 1.0 μg/m³
                    [0x00, 0x0F], // PM2.5 = 1.5 μg/m³
                    [0x00, 0x14], // PM4.0 = 2.0 μg/m³
                    [0x00, 0x19], // PM10 = 2.5 μg/m³
                    [0x13, 0x88], // Humidity = 50.00 %RH (5000)
                    [0x09, 0xC4], // Temperature = 25.00 °C (2500)
                    [0x00, 0x64], // VOC = 10.0
                    [0x00, 0x01], // NOX = 0.1
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let sample = block_on(sensor.get_sample()).unwrap();

        assert_eq!(sample.pm1, 1.0);
        assert_eq!(sample.pm2_5, 1.5);
        assert_eq!(sample.pm4, 2.0);
        assert_eq!(sample.pm10, 2.5);
        assert_eq!(sample.humidity, 50.0);
        assert_eq!(sample.temperature, 25.0);
        assert_eq!(sample.voc, 10.0);
        assert_eq!(sample.nox, 0.1);

        sensor.i2c.done();
    }

    #[cfg(feature = "sen66")]
    #[test]
    fn test_get_sample_sen66_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x03, 0x00]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x00, 0x0A], // PM1.0 = 1.0 μg/m³
                    [0x00, 0x0F], // PM2.5 = 1.5 μg/m³
                    [0x00, 0x14], // PM4.0 = 2.0 μg/m³
                    [0x00, 0x19], // PM10 = 2.5 μg/m³
                    [0x13, 0x88], // Humidity = 50.00 %RH (5000)
                    [0x09, 0xC4], // Temperature = 25.00 °C (2500)
                    [0x00, 0x64], // VOC = 10.0
                    [0x00, 0x01], // NOX = 0.1
                    [0x01, 0x90]  // CO2 = 400 ppm
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let sample = block_on(sensor.get_sample()).unwrap();

        assert_eq!(sample.pm1, 1.0);
        assert_eq!(sample.pm2_5, 1.5);
        assert_eq!(sample.pm4, 2.0);
        assert_eq!(sample.pm10, 2.5);
        assert_eq!(sample.humidity, 50.0);
        assert_eq!(sample.temperature, 25.0);
        assert_eq!(sample.co2, 400);
        assert_eq!(sample.voc, 10.0);
        assert_eq!(sample.nox, 0.1);

        sensor.i2c.done();
    }

    #[cfg(feature = "sen68")]
    #[test]
    fn test_get_sample_sen68_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x03, 0x00]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x00, 0x0A], // PM1.0 = 1.0 μg/m³
                    [0x00, 0x0F], // PM2.5 = 1.5 μg/m³
                    [0x00, 0x14], // PM4.0 = 2.0 μg/m³
                    [0x00, 0x19], // PM10 = 2.5 μg/m³
                    [0x13, 0x88], // Humidity = 50.00 %RH (5000)
                    [0x09, 0xC4], // Temperature = 25.00 °C (2500)
                    [0x00, 0x64], // VOC = 10.0
                    [0x00, 0x01], // NOX = 0.1
                    [0x01, 0x90]  // HCHO = 40.0 ppb
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let sample = block_on(sensor.get_sample()).unwrap();

        assert_eq!(sample.pm1, 1.0);
        assert_eq!(sample.pm2_5, 1.5);
        assert_eq!(sample.pm4, 2.0);
        assert_eq!(sample.pm10, 2.5);
        assert_eq!(sample.humidity, 50.0);
        assert_eq!(sample.temperature, 25.0);
        assert_eq!(sample.hcho, 40.0);

        sensor.i2c.done();
    }

    #[cfg(feature = "sen63c")]
    #[test]
    fn test_get_raw_sample_sen63c_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x04, 0x05]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x30, 0x39], // Raw humidity = 12345
                    [0xFF, 0x85], // Raw temperature = -123
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let raw_sample = block_on(sensor.get_raw_sample()).unwrap();

        assert_eq!(raw_sample.raw_humidity, 12345);
        assert_eq!(raw_sample.raw_temperature, -123);

        sensor.i2c.done();
    }

    #[cfg(feature = "sen66")]
    #[test]
    fn test_get_raw_sample_sen66_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x04, 0x05]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x30, 0x39], // Raw humidity = 12345
                    [0xFF, 0x85], // Raw temperature = -123
                    [0x02, 0x37], // Raw VOC = 567
                    [0x00, 0x59], // Raw NOx = 89
                    [0x03, 0x15]  // Raw CO2 = 789
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let raw_sample = block_on(sensor.get_raw_sample()).unwrap();

        assert_eq!(raw_sample.raw_humidity, 12345);
        assert_eq!(raw_sample.raw_temperature, -123);
        assert_eq!(raw_sample.raw_voc, 567);
        assert_eq!(raw_sample.raw_nox, 89);
        assert_eq!(raw_sample.raw_co2, 789);

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen65", feature = "sen68"))]
    #[test]
    fn test_get_raw_sample_sen65_sen68_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x04, 0x05]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x30, 0x39], // Raw humidity = 12345
                    [0xFF, 0x85], // Raw temperature = -123
                    [0x02, 0x37], // Raw VOC = 567
                    [0x00, 0x59], // Raw NOx = 89
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let raw_sample = block_on(sensor.get_raw_sample()).unwrap();

        assert_eq!(raw_sample.raw_humidity, 12345);
        assert_eq!(raw_sample.raw_temperature, -123);
        assert_eq!(raw_sample.raw_voc, 567);
        assert_eq!(raw_sample.raw_nox, 89);

        sensor.i2c.done();
    }

    #[test]
    fn test_get_raw_concentration_sample_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x03, 0x16]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x00, 0x0A], // PM1.0 = 10
                    [0x00, 0x0F], // PM2.5 = 15
                    [0x00, 0x14], // PM4.0 = 20
                    [0x00, 0x19]  // PM10 = 25
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let raw_conc = block_on(sensor.get_raw_concentration_sample()).unwrap();

        assert_eq!(raw_conc.pm1, 10);
        assert_eq!(raw_conc.pm2_5, 15);
        assert_eq!(raw_conc.pm4, 20);
        assert_eq!(raw_conc.pm10, 25);

        sensor.i2c.done();
    }

    #[test]
    fn test_set_temp_offset_pars_async() {
        let temp_offset_pars = TempOffsetPars {
            offset: 100,
            slope: 200,
            time_constant: 300,
            slot: 1,
        };

        let expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x60, 0xB2],            // Command
                bytes_with_crc!(0x00, 0x64), // offset = 100
                bytes_with_crc!(0x00, 0xC8), // slope = 200
                bytes_with_crc!(0x01, 0x2C), // time_constant = 300
                bytes_with_crc!(0x00, 0x01), // slot = 1
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_temp_offset_pars(temp_offset_pars)).is_ok());

        sensor.i2c.done();
    }

    #[test]
    fn test_set_temp_accel_pars_async() {
        let temp_accel_pars = TempAccelPars {
            k: 100,
            p: 200,
            t1: 300,
            t2: 400,
        };

        let expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x61, 0x00],            // Command
                bytes_with_crc!(0x00, 0x64), // k = 100
                bytes_with_crc!(0x00, 0xC8), // p = 200
                bytes_with_crc!(0x01, 0x2C), // t1 = 300
                bytes_with_crc!(0x01, 0x90), // t2 = 400
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_temp_accel_pars(temp_accel_pars)).is_ok());

        sensor.i2c.done();
    }

    #[test]
    fn test_get_product_name_async() {
        // For text data, we need to create character bytes + CRC sequences
        let product_name_bytes = [
            // "SEN6X" + null + padding
            bytes_with_crc!(0x53, 0x45), // "SE"
            bytes_with_crc!(0x4E, 0x36), // "N6"
            bytes_with_crc!(0x58, 0x00), // "X" + null terminator
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
        ]
        .concat();

        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0xD0, 0x14]),
            I2cTransaction::read(MODULE_ADDR, product_name_bytes),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let mut buffer = [0u8; 32];
        let product_name = block_on(sensor.get_product_name(&mut buffer)).unwrap();

        assert_eq!(product_name, "SEN6X");

        sensor.i2c.done();
    }

    #[test]
    fn test_get_serial_number_async() {
        // For text data, we need to create character bytes + CRC sequences
        let serial_number_bytes = [
            // "1234567890" + null + padding
            bytes_with_crc!(0x31, 0x32), // "12"
            bytes_with_crc!(0x33, 0x34), // "34"
            bytes_with_crc!(0x35, 0x36), // "56"
            bytes_with_crc!(0x37, 0x38), // "78"
            bytes_with_crc!(0x39, 0x30), // "90"
            bytes_with_crc!(0x00, 0x00), // null terminator
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
            bytes_with_crc!(0x00, 0x00), // padding
        ]
        .concat();

        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0xD0, 0x33]),
            I2cTransaction::read(MODULE_ADDR, serial_number_bytes),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let mut buffer = [0u8; 32];
        let serial_number = block_on(sensor.get_serial_number(&mut buffer)).unwrap();

        assert_eq!(serial_number, "1234567890");

        sensor.i2c.done();
    }

    #[test]
    fn test_read_device_status_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0xD2, 0x06]),
            I2cTransaction::read(
                MODULE_ADDR,
                [
                    bytes_with_crc!(0b00000000, 0b00100000), // Status word 2 - (fan speed warning)
                    bytes_with_crc!(0b00000000, 0b10010000), // Status word 1 - (fan error and gas error)
                ]
                .concat(),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let status = block_on(sensor.read_device_status()).unwrap();

        assert_eq!(status.fan_speed_warning, true);
        assert_eq!(status.co2_error, false);
        assert_eq!(status.pm_error, false);
        assert_eq!(status.gas_error, true);
        assert_eq!(status.rh_t_error, false);
        assert_eq!(status.fan_error, true);

        sensor.i2c.done();
    }

    #[test]
    fn test_reset_async() {
        let expectations = [I2cTransaction::write(MODULE_ADDR, vec![0xD3, 0x04])];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.reset()).is_ok());

        sensor.i2c.done();
    }

    #[test]
    fn test_start_fan_cleaning_async() {
        let expectations = [I2cTransaction::write(MODULE_ADDR, vec![0x56, 0x07])];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.start_fan_cleaning()).is_ok());

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    #[test]
    fn test_set_voc_algo_tuning_parameters_async() {
        let tuning_pars = AlgorithmTuningParameters {
            index_offset: 100,
            learning_time_offset_hours: 12,
            learning_time_gain_hours: 12,
            gating_max_duration_minutes: 180,
            std_initial: 50,
            gain_factor: 230,
        };

        let expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x60, 0xD0],            // Command
                bytes_with_crc!(0x00, 0x64), // index_offset = 100
                bytes_with_crc!(0x00, 0x0C), // learning_time_offset_hours = 12
                bytes_with_crc!(0x00, 0x0C), // learning_time_gain_hours = 12
                bytes_with_crc!(0x00, 0xB4), // gating_max_duration_minutes = 180
                bytes_with_crc!(0x00, 0x32), // std_initial = 50
                bytes_with_crc!(0x00, 0xE6), // gain_factor = 230
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_voc_algo_tuning_parameters(tuning_pars)).is_ok());

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    #[test]
    fn test_get_voc_algo_tuning_parameters_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x60, 0xD0]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x00, 0x64], // index_offset = 100
                    [0x00, 0x0C], // learning_time_offset_hours = 12
                    [0x00, 0x0C], // learning_time_gain_hours = 12
                    [0x00, 0xB4], // gating_max_duration_minutes = 180
                    [0x00, 0x32], // std_initial = 50
                    [0x00, 0xE6]  // gain_factor = 230
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let tuning_pars = block_on(sensor.get_voc_algo_tuning_parameters()).unwrap();

        assert_eq!(tuning_pars.index_offset, 100);
        assert_eq!(tuning_pars.learning_time_offset_hours, 12);
        assert_eq!(tuning_pars.learning_time_gain_hours, 12);
        assert_eq!(tuning_pars.gating_max_duration_minutes, 180);
        assert_eq!(tuning_pars.std_initial, 50);
        assert_eq!(tuning_pars.gain_factor, 230);

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    #[test]
    fn test_set_and_get_ambient_pressure_async() {
        // Test setting ambient pressure
        let pressure = 1013;
        let set_expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x67, 0x20],            // Command
                bytes_with_crc!(0x03, 0xF5), // pressure = 1013
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&set_expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_ambient_pressure(pressure)).is_ok());
        sensor.i2c.done();

        // Test getting ambient pressure
        let get_expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x67, 0x20]),
            I2cTransaction::read(MODULE_ADDR, bytes_with_crc!(0x03, 0xF5)),
        ];

        let i2c = I2cMock::new(&get_expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let read_pressure = block_on(sensor.get_ambient_pressure()).unwrap();
        assert_eq!(read_pressure, 1013);

        sensor.i2c.done();
    }

    #[test]
    fn test_error_handling_async() {
        // Test I2C write error
        let expectations = [I2cTransaction::write(MODULE_ADDR, vec![0x00, 0x21])
            .with_error(embedded_hal::i2c::ErrorKind::Bus)];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let result = block_on(sensor.start_continuous_measurement());
        assert!(result.is_err());
        assert!(matches!(result, Err(Sen6xError::WriteI2CError)));

        sensor.i2c.done();

        // Test I2C read error
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x02, 0x02]),
            I2cTransaction::read(MODULE_ADDR, vec![0; 3])
                .with_error(embedded_hal::i2c::ErrorKind::Bus),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let result = block_on(sensor.get_is_data_ready());
        assert!(result.is_err());
        assert!(matches!(result, Err(Sen6xError::ReadI2CError)));

        sensor.i2c.done();

        // Test invalid state error
        let expectations = []; // No I2C transactions expected

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);
        sensor.state = ModuleState::Measuring;

        let result = block_on(sensor.reset());
        assert!(result.is_err());
        assert!(matches!(result, Err(Sen6xError::InvalidState)));

        sensor.i2c.done();

        // Test CRC error
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x02, 0x02]),
            I2cTransaction::read(MODULE_ADDR, vec![0x00, 0x01, 0xFF]), // Invalid CRC
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let result = block_on(sensor.get_is_data_ready());
        assert!(result.is_err());

        sensor.i2c.done();
    }

    #[test]
    fn test_activate_sht_heater_async() {
        let expectations = [I2cTransaction::write(MODULE_ADDR, vec![0x67, 0x65])];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.activate_sht_heater()).is_ok());

        sensor.i2c.done();
    }

    #[test]
    fn test_read_and_clear_device_status_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0xD2, 0x10]),
            I2cTransaction::read(
                MODULE_ADDR,
                [
                    bytes_with_crc!(0b00000000, 0b00100000), // Status word 2 - (fan speed warning)
                    bytes_with_crc!(0b00000000, 0b10010000), // Status word 1 - (fan error and gas error)
                ]
                .concat(),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let status = block_on(sensor.read_and_clear_device_status()).unwrap();

        assert_eq!(status.fan_speed_warning, true);
        assert_eq!(status.fan_error, true);
        assert_eq!(status.gas_error, true);
        assert_eq!(status.rh_t_error, false);
        assert_eq!(status.pm_error, false);
        assert_eq!(status.co2_error, false);

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    #[test]
    fn test_set_altitude_async() {
        let expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x67, 0x36],            // Command
                bytes_with_crc!(0x01, 0xF4), // altitude = 500
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_altitude(500)).is_ok());

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    #[test]
    fn test_get_altitude_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x67, 0x36]),
            I2cTransaction::read(MODULE_ADDR, bytes_with_crc!(0x01, 0xF4)), // 500
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let altitude = block_on(sensor.get_altitude()).unwrap();
        assert_eq!(altitude, 500);

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    #[test]
    fn test_get_voc_algo_state_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x61, 0x81]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x12, 0x34], // state[0] = 0x1234
                    [0x56, 0x78], // state[1] = 0x5678
                    [0x9A, 0xBC], // state[2] = 0x9ABC
                    [0xDE, 0xF0]  // state[3] = 0xDEF0
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let state = block_on(sensor.get_voc_algo_state()).unwrap();
        assert_eq!(state, [0x1234, 0x5678, 0x9ABC, 0xDEF0]);

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    #[test]
    fn test_set_voc_algo_state_async() {
        let state = [0x1234, 0x5678, 0x9ABC, 0xDEF0];
        let expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x61, 0x81],            // Command
                bytes_with_crc!(0x12, 0x34), // state[0] = 0x1234
                bytes_with_crc!(0x56, 0x78), // state[1] = 0x5678
                bytes_with_crc!(0x9A, 0xBC), // state[2] = 0x9ABC
                bytes_with_crc!(0xDE, 0xF0), // state[3] = 0xDEF0
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_voc_algo_state(state)).is_ok());

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    #[test]
    fn test_get_nox_algo_tuning_parameters_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x60, 0xE1]),
            I2cTransaction::read(
                MODULE_ADDR,
                combine_bytes_with_crc!(
                    [0x00, 0x64], // index_offset = 100
                    [0x00, 0x0C], // learning_time_offset_hours = 12
                    [0x00, 0x0C], // learning_time_gain_hours = 12
                    [0x00, 0xB4], // gating_max_duration_minutes = 180
                    [0x00, 0x32], // std_initial = 50
                    [0x00, 0xE6]  // gain_factor = 230
                ),
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let tuning_pars = block_on(sensor.get_nox_algo_tuning_parameters()).unwrap();

        assert_eq!(tuning_pars.index_offset, 100);
        assert_eq!(tuning_pars.learning_time_offset_hours, 12);
        assert_eq!(tuning_pars.learning_time_gain_hours, 12);
        assert_eq!(tuning_pars.gating_max_duration_minutes, 180);
        assert_eq!(tuning_pars.std_initial, 50);
        assert_eq!(tuning_pars.gain_factor, 230);

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    #[test]
    fn test_set_nox_algo_tuning_parameters_async() {
        let tuning_pars = AlgorithmTuningParameters {
            index_offset: 100,
            learning_time_offset_hours: 12,
            learning_time_gain_hours: 12,
            gating_max_duration_minutes: 180,
            std_initial: 50,
            gain_factor: 230,
        };

        let expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x60, 0xE1],            // Command
                bytes_with_crc!(0x00, 0x64), // index_offset = 100
                bytes_with_crc!(0x00, 0x0C), // learning_time_offset_hours = 12
                bytes_with_crc!(0x00, 0x0C), // learning_time_gain_hours = 12
                bytes_with_crc!(0x00, 0xB4), // gating_max_duration_minutes = 180
                bytes_with_crc!(0x00, 0x32), // std_initial = 50
                bytes_with_crc!(0x00, 0xE6), // gain_factor = 230
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_nox_algo_tuning_parameters(tuning_pars)).is_ok());

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    #[test]
    fn test_perform_forced_co2_recalibration_async() {
        let target_concentration = 400;

        let expectations = [
            I2cTransaction::write(
                MODULE_ADDR,
                [
                    vec![0x67, 0x07],            // Command
                    bytes_with_crc!(0x01, 0x90), // target_concentration = 400
                ]
                .concat(),
            ),
            I2cTransaction::read(MODULE_ADDR, bytes_with_crc!(0x01, 0x90)), // correction value = 400
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let correction =
            block_on(sensor.perform_forced_co2_recalibration(target_concentration)).unwrap();
        assert_eq!(correction, 400);

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    #[test]
    fn test_get_is_co2_auto_self_calibrated_async() {
        let expectations = [
            I2cTransaction::write(MODULE_ADDR, vec![0x67, 0x11]),
            I2cTransaction::read(MODULE_ADDR, bytes_with_crc!(0x00, 0x01)), // enabled
        ];

        let i2c = I2cMock::new(&expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        let is_enabled = block_on(sensor.get_is_co2_auto_self_calibrated()).unwrap();
        assert_eq!(is_enabled, true);

        sensor.i2c.done();
    }

    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    #[test]
    fn test_set_co2_auto_self_calibration_async() {
        // Test enabling
        let enable_expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x67, 0x11],            // Command
                bytes_with_crc!(0x00, 0x01), // enable = true
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&enable_expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_co2_auto_self_calibration(true)).is_ok());
        sensor.i2c.done();

        // Test disabling
        let disable_expectations = [I2cTransaction::write(
            MODULE_ADDR,
            [
                vec![0x67, 0x11],            // Command
                bytes_with_crc!(0x00, 0x00), // enable = false
            ]
            .concat(),
        )];

        let i2c = I2cMock::new(&disable_expectations);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);

        assert!(block_on(sensor.set_co2_auto_self_calibration(false)).is_ok());
        sensor.i2c.done();
    }

    #[test]
    fn test_invalid_state_errors_async() {
        // Test functions that should fail when in measuring state
        let i2c = I2cMock::new(&[]);
        let delay = DelayMock::new();
        let mut sensor = Sen6X::new(delay, i2c);
        sensor.state = ModuleState::Measuring;

        // Try operations that should fail in measuring state
        assert!(matches!(
            block_on(sensor.start_continuous_measurement()),
            Err(Sen6xError::InvalidState)
        ));
        assert!(matches!(
            block_on(sensor.get_sample()),
            Err(Sen6xError::InvalidState)
        ));
        assert!(matches!(
            block_on(sensor.get_raw_sample()),
            Err(Sen6xError::InvalidState)
        ));
        assert!(matches!(
            block_on(sensor.get_raw_concentration_sample()),
            Err(Sen6xError::InvalidState)
        ));
        assert!(matches!(
            block_on(sensor.reset()),
            Err(Sen6xError::InvalidState)
        ));
        assert!(matches!(
            block_on(sensor.start_fan_cleaning()),
            Err(Sen6xError::InvalidState)
        ));
        assert!(matches!(
            block_on(sensor.activate_sht_heater()),
            Err(Sen6xError::InvalidState)
        ));
        assert!(matches!(
            block_on(sensor.set_altitude(500)),
            Err(Sen6xError::InvalidState)
        ));

        sensor.i2c.done();
    }
}
