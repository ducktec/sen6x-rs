//! # Blocking API
//!
//! This module contains the blocking API for the SEN6X sensor modules.
//! It is based on the `embedded-hal` traits and is intended to be used
//! with synchronous blocking code.
//!
//! The methods usually return `Result` with the error type being `Sen6xError`.
//!
//! ## Examples
//!
//! ### Create a driver instance
//!
//! ```rust
//! use sen6x::blocking::Sen6X;
//! use embedded_hal::i2c::I2c;
//! use embedded_hal::delay::DelayNs;
//!
//! fn main() {
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
//! use sen6x::blocking::Sen6X;
//! use embedded_hal::i2c::I2c;
//! use embedded_hal::delay::DelayNs;
//!
//! fn main() {
//!     // Initialize peripherals
//!     let i2c = MyI2C::new();
//!     let delay = MyDelay::new();
//!     let delay2 = MyDelay::new(); // delay is consumed by the driver, delay2 for the loop
//!     
//!     // Create the Sen6X driver instance
//!     let mut sensor = Sen6X::new(delay, i2c);
//!     
//!     // Start continuous measurement (1 measurement per second)
//!     sensor.start_continuous_measurement().unwrap();
//!     
//!     // Main loop
//!     loop {
//!         // Check if new data is ready
//!         if sensor.get_is_data_ready().unwrap() {
//!             // Read the measured values
//!             let sample = sensor.get_sample().unwrap();
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
//!         }
//!         
//!         // Wait a bit before checking again
//!         // In a real application, you would use a timer or other mechanism
//!         delay2.delay_ms(100u32);
//!     }
//!     
//!     // When done (not reached in this example)
//!     // sensor.stop_measurement().unwrap();
//! }
//! ```

use crate::{
    AlgorithmTuningParameters, CommandId, DeviceStatus, MAX_RX_BYTES, MAX_TX_BYTES, MODULE_ADDR,
    MeasuredSample, ModuleState, RawConcentrationSample, RawMeasuredSample, Result, Sen6xError,
    TempAccelPars, TempOffsetPars, crc, get_execution_time,
};

/// Represents an I2C-connected SEN6X sensor module.
#[derive(Copy, Clone, Debug)]
pub struct Sen6X<I2C, D> {
    /// Marker to satisfy the compiler.
    delay: D,

    /// I2C Interface for communicating with the module.
    i2c: I2C,

    /// The current measurement state of the module
    state: ModuleState,
}

impl<I2C, D> Sen6X<I2C, D>
where
    D: embedded_hal::delay::DelayNs,
    I2C: embedded_hal::i2c::I2c,
{
    /// Creates SEN6X instance representation.
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
    pub fn start_continuous_measurement(&mut self) -> Result<()> {
        self.check_not_measuring()?;
        // Set state regardless of the result of the command (conservative)
        self.state = ModuleState::Measuring;
        self.send_wait(CommandId::StartContinuousMeasurement)
    }

    /// Stops continuous measuring mode
    pub fn stop_measurement(&mut self) -> Result<()> {
        let result = self.send_wait(CommandId::StopMeasurement);
        if result.is_ok() {
            self.state = ModuleState::Idle;
        }
        result
    }

    /// Check if new data can be retrieved from the sensor module
    pub fn get_is_data_ready(&mut self) -> Result<bool> {
        let mut data = [0 as u16; 1];
        self.send_wait_read(CommandId::GetDataReady, &mut data)?;
        Ok(data[0] == 0x1)
    }

    /// Read the last measured values from the sensor module
    pub fn get_sample(&mut self) -> Result<MeasuredSample> {
        self.check_not_measuring()?;

        let mut data = [0 as u16; 9];
        self.send_wait_read(CommandId::ReadMeasuredValues, &mut data)?;

        Ok(MeasuredSample::from(data))
    }

    /// Read the last measured raw values from the sensor module
    ///
    /// This excludes the concentration raw values, which can be
    /// read with `get_raw_concentration_sample`.
    ///
    /// This is for advanced use only, normally you would use `get_sample`.
    pub fn get_raw_sample(&mut self) -> Result<RawMeasuredSample> {
        self.check_not_measuring()?;

        let mut data = [0 as u16; 5];
        self.send_wait_read(CommandId::ReadMeasuredRawValues, &mut data)?;

        Ok(RawMeasuredSample::from(data))
    }

    /// Read the last measured concentration values from the sensor module
    ///
    /// This only includes the concentration (PM) values, all other raw values
    /// can be read with `get_raw_sample`.
    ///
    /// This is for advanced use only, normally you would use `get_sample`.
    pub fn get_raw_concentration_sample(&mut self) -> Result<RawConcentrationSample> {
        self.check_not_measuring()?;

        let mut data = [0 as u16; 4];
        self.send_wait_read(CommandId::ReadNumberConcentrationValues, &mut data)?;

        Ok(RawConcentrationSample::from(data))
    }

    /// Set the temperature offset parameters
    pub fn set_temp_offset_pars(&mut self, temp_offset_pars: TempOffsetPars) -> Result<()> {
        let data: [u16; 4] = <[u16; 4]>::from(temp_offset_pars);
        self.send_write_wait(CommandId::SetTempOffsetPars, &data)
    }

    /// Set the temperature acceleration parameters
    pub fn set_temp_accel_pars(&mut self, temp_accel_pars: TempAccelPars) -> Result<()> {
        let data: [u16; 4] = <[u16; 4]>::from(temp_accel_pars);
        self.send_write_wait(CommandId::SetTempAccelPars, &data)
    }

    /// Get the product name of the sensor module
    /// (to provide the string slice back, a 32-byte u32 buffer must be provided
    /// by the caller)
    pub fn get_product_name<'a>(&mut self, buffer: &'a mut [u8; 32]) -> Result<&'a str> {
        let mut data = [0 as u16; 32];
        self.send_wait_read(CommandId::GetProductName, &mut data)?;

        let mut len = 0;
        for i in 0..buffer.len() {
            if data[i] == 0 {
                break;
            }
            buffer[i] = data[i] as u8;
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
    pub fn get_serial_number<'a>(&mut self, buffer: &'a mut [u8; 32]) -> Result<&'a str> {
        let mut data = [0 as u16; 32];
        self.send_wait_read(CommandId::GetSerialNumber, &mut data)?;

        let mut len = 0;
        for i in 0..buffer.len() {
            if data[i] == 0 {
                break;
            }
            buffer[i] = data[i] as u8;
            len += 1;
        }

        match core::str::from_utf8(&buffer[..len]) {
            Ok(s) => Ok(s),
            Err(_) => Err(Sen6xError::InvalidData),
        }
    }

    /// Read the device status (no clearing of flags!)
    pub fn read_device_status(&mut self) -> Result<DeviceStatus> {
        let mut data = [0 as u16; 2];
        self.send_wait_read(CommandId::ReadDeviceStatus, &mut data)?;

        Ok(DeviceStatus::from(data))
    }

    /// Read the device status (no clearing of flags!)
    pub fn read_and_clear_device_status(&mut self) -> Result<DeviceStatus> {
        let mut data = [0 as u16; 2];
        self.send_wait_read(CommandId::ReadAndClearDeviceStatus, &mut data)?;

        Ok(DeviceStatus::from(data))
    }

    /// Reset the sensor module
    pub fn reset(&mut self) -> Result<()> {
        self.check_not_measuring()?;
        self.send_wait(CommandId::DeviceReset)
    }

    /// Start the fan cleaning process
    pub fn start_fan_cleaning(&mut self) -> Result<()> {
        self.check_not_measuring()?;
        self.send_wait(CommandId::StartFanCleaning)
    }

    /// Activate the SHT heater
    pub fn activate_sht_heater(&mut self) -> Result<()> {
        self.check_not_measuring()?;
        self.send_wait(CommandId::ActivateShtHeater)
    }

    /// Get the VOC algorithm tuning parameters
    pub fn get_voc_algo_tuning_parameters(&mut self) -> Result<AlgorithmTuningParameters> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 6];
        self.send_wait_read(CommandId::VocAlgoTuningPars, &mut data)?;

        Ok(AlgorithmTuningParameters::from(data))
    }

    /// Set the VOC algorithm tuning parameters
    pub fn set_voc_algo_tuning_parameters(
        &mut self,
        tuning_pars: AlgorithmTuningParameters,
    ) -> Result<()> {
        self.check_not_measuring()?;
        let data: [u16; 6] = <[u16; 6]>::from(tuning_pars);
        self.send_write_wait(CommandId::VocAlgoTuningPars, &data)
    }

    /// Set the VOC algorithm state
    ///
    /// Due to the long initialization phase of the algorithm and no sensor-internal
    /// persistent storing of it's state, the algorithm state ideally should be preserved
    /// between restarts.
    pub fn get_voc_algo_state(&mut self) -> Result<[u16; 4]> {
        let mut data = [0 as u16; 4];
        self.send_wait_read(CommandId::VocAlgoState, &mut data)?;

        Ok(data)
    }

    /// Set the VOC algorithm state
    ///
    /// Due to the long initialization phase of the algorithm and no sensor-internal
    /// persistent storing of it's state, the algorithm state ideally should be preserved
    /// between restarts.
    pub fn set_voc_algo_state(&mut self, state: [u16; 4]) -> Result<()> {
        self.check_not_measuring()?;
        self.send_write_wait(CommandId::VocAlgoState, &state)
    }

    /// Get the NOx algorithm tuning parameters
    pub fn get_nox_algo_tuning_parameters(&mut self) -> Result<AlgorithmTuningParameters> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 6];
        self.send_wait_read(CommandId::NoxAlgoTuningPars, &mut data)?;

        Ok(AlgorithmTuningParameters::from(data))
    }

    /// Set the NOx algorithm tuning parameters
    pub fn set_nox_algo_tuning_parameters(
        &mut self,
        tuning_pars: AlgorithmTuningParameters,
    ) -> Result<()> {
        self.check_not_measuring()?;
        let data: [u16; 6] = <[u16; 6]>::from(tuning_pars);
        self.send_write_wait(CommandId::NoxAlgoTuningPars, &data)
    }

    /// Perform a forced CO2 recalibration
    pub fn perform_forced_co2_recalibration(&mut self, target_concentration: u16) -> Result<u16> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 1];
        self.send_write_wait_read(
            CommandId::PerformForcedCo2Recalibration,
            &[target_concentration],
            &mut data,
        )?;

        Ok(data[0])
    }

    /// Get the automatic self-calibration state of the CO2 sensor
    pub fn get_is_co2_auto_self_calibrated(&mut self) -> Result<bool> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 1];
        self.send_wait_read(CommandId::Co2SensorAutoCalibrationState, &mut data)?;

        Ok(data[1] == 0x1)
    }

    /// Enable or disable the automatic self-calibration of the CO2 sensor
    pub fn set_co2_auto_self_calibration(&mut self, enabled: bool) -> Result<()> {
        self.check_not_measuring()?;
        let data: [u16; 2] = [0x00, if enabled { 0x01 } else { 0x00 }];
        self.send_write_wait(CommandId::Co2SensorAutoCalibrationState, &data)
    }

    /// Get the ambient pressure in hPa that is assumed by the sensor
    /// (used by the sensor for pressure compensation)
    pub fn get_ambient_pressure(&mut self) -> Result<u16> {
        let mut data = [0 as u16; 1];
        self.send_wait_read(CommandId::AmbientPressure, &mut data)?;

        Ok(data[0])
    }

    /// Set the ambient pressure in hPa that is assumed by the sensor
    /// (used by the sensor for pressure compensation)
    pub fn set_ambient_pressure(&mut self, pressure: u16) -> Result<()> {
        self.send_write_wait(CommandId::AmbientPressure, &[pressure])
    }

    /// Get the altitude in meters that is assumed by the sensor
    /// (used by the sensor for pressure compensation)
    pub fn get_altitude(&mut self) -> Result<u16> {
        self.check_not_measuring()?;
        let mut data = [0 as u16; 1];
        self.send_wait_read(CommandId::SensorAltitude, &mut data)?;

        Ok(data[0])
    }

    /// Set the sensor altitude in meters that is assumed by the sensor
    /// (used by the sensor for pressure compensation)
    pub fn set_altitude(&mut self, altitude: u16) -> Result<()> {
        self.check_not_measuring()?;
        self.send_write_wait(CommandId::SensorAltitude, &[altitude])
    }

    /// Send a command to the sensor module and wait for the execution time associated to the command
    fn send_wait(&mut self, command: CommandId) -> Result<()> {
        self.i2c
            .write(MODULE_ADDR, &(command as u16).to_be_bytes())
            .map_err(|_| Sen6xError::WriteI2CError)?;
        self.delay.delay_ms(get_execution_time(command));
        Ok(())
    }

    /// Send a command to the sensor module, wait for the execution time associated to the command
    /// and read the data from the sensor module
    fn send_wait_read(&mut self, command: CommandId, data: &mut [u16]) -> Result<()> {
        self.send_wait(command)?;
        self.read(data)
    }

    /// Read data from the sensor module (no command, just data, command must be sent separately)
    fn read(&mut self, data: &mut [u16]) -> Result<()> {
        let mut raw_data = [0 as u8; MAX_RX_BYTES];
        let internal_length = data.len() * 3;

        self.i2c
            .read(MODULE_ADDR, &mut raw_data[0..internal_length])
            .map_err(|_| Sen6xError::ReadI2CError)?;

        // Validate return values
        match crc::validate_and_extract_data(&raw_data[..internal_length], data) {
            Ok(_) => Ok(()),
            Err(e) => Err(Sen6xError::from(e)),
        }
    }

    /// Send command, write data to the sensor module and wait for the execution time associated to the command
    fn send_write_wait(&mut self, command: CommandId, data: &[u16]) -> Result<()> {
        let mut raw_data = [0 as u8; MAX_TX_BYTES + 2];
        let internal_length = data.len() * 3 + 2;

        if internal_length > MAX_TX_BYTES + 1 {
            return Err(Sen6xError::TooMuchData);
        }

        // Encode command ID
        let command_bytes = (command as u16).to_be_bytes();
        raw_data[0] = command_bytes[0];
        raw_data[1] = command_bytes[1];

        // Fill raw data structure with data and CRCs
        for (i, &d) in data.iter().enumerate() {
            let crc = crc::generate_crc(&d.to_be_bytes());
            let data_bytes = d.to_be_bytes();
            raw_data[i * 3 + 2] = data_bytes[0];
            raw_data[i * 3 + 3] = data_bytes[1];
            raw_data[i * 3 + 4] = crc;
        }

        self.i2c
            .write(MODULE_ADDR, &raw_data[0..internal_length])
            .map_err(|_| Sen6xError::WriteI2CError)?;
        self.delay.delay_ms(get_execution_time(command));
        Ok(())
    }

    /// Send command, write some data in one transaction, then wait for the execution time
    /// associated to the command and finally the data from the sensor module without
    /// any command sent again
    fn send_write_wait_read(
        &mut self,
        command: CommandId,
        data_in: &[u16],
        data_out: &mut [u16],
    ) -> Result<()> {
        self.send_write_wait(command, data_in)?;
        self.read(data_out)
    }
}
