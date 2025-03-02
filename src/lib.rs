//! This crate provides a platform agnostic no_std driver for the SEN6X sensor modules.
//! The driver is compatible with the [`embedded-hal`](https://crates.io/crates/embedded-hal) traits.
//!
//! The data sheet of the sensor can be found [here](https://sensirion.com/resource/datasheet/SEN6x).
//!
//! ## Supported features
//! * Both blocking and async support
//! * Start/stop continuous measurement mode
//! * Check for availability of new measurement data
//! * Get new measurement data (`struct`` containing characteristics per module variant)
//! * Get new raw measurement data
//! * Configure device (advanced use):
//!     * set temperature offset,
//!     * set temperature acceleration,
//!     * get/set sensor altitude
//!     * get/set ambient pressure
//!     * get/set VOC/NOX algorithm tuning parameters
//!     * get/set VOC algorithm state
//!     * get product name (aka, variant)
//!     * get serial number
//!     * get/set co2 sensor auto calibration state
//!     * get/clear device status
//! * Control device (advanced use):
//!     * reset device
//!     * start fan cleaning
//!     * activate SHT heater
//!     * perform forced co2 recalibration
//!
//! ## Usage
//!
//! By default, the driver is in blocking mode. To use the driver in async mode, enable the `async` feature.
//!
//! The documentation and examples of both modes can be found in the respective modules `blocking` and `async`.
//!
//! ## Unit tests
//! The unit tests are only working with a specific feature flag enabled. To run the tests, use the following command:
//! ```sh
//! cargo test --features unittesting
//! ```
//! The reason for the feature flag is that the mock hal implementation is not `no_std` and thus cannot be part of a build for
//! a `no_std` target.

#![cfg_attr(not(test), no_std)]

use crc_internal::CrcError;

#[cfg(feature = "async")]
pub mod r#async;

pub mod blocking;

pub mod crc_internal;

/// I2C address for the sensor module.
#[cfg(not(feature = "sen60"))]
pub const MODULE_ADDR: u8 = 0x6B;
#[cfg(feature = "sen60")]
pub const MODULE_ADDR: u8 = 0x6C;

/// Command ID enum.
#[repr(u16)]
#[derive(Copy, Clone, Debug)]
pub enum CommandId {
    StartContinuousMeasurement = 0x0021,
    StopMeasurement = 0x0104,
    GetDataReady = 0x0202,
    ReadMeasuredValues = 0x0300,
    ReadMeasuredRawValues = 0x0405,
    ReadNumberConcentrationValues = 0x0316,
    SetTempOffsetPars = 0x60B2,
    SetTempAccelPars = 0x6100,
    GetProductName = 0xD014,
    GetSerialNumber = 0xD033,
    ReadDeviceStatus = 0xD206,
    ReadAndClearDeviceStatus = 0xD210,
    DeviceReset = 0xD304,
    StartFanCleaning = 0x5607,
    ActivateShtHeater = 0x6765,
    VocAlgoTuningPars = 0x60D0,
    VocAlgoState = 0x6181,
    NoxAlgoTuningPars = 0x60E1,
    PerformForcedCo2Recalibration = 0x6707,
    Co2SensorAutoCalibrationState = 0x6711,
    AmbientPressure = 0x6720,
    SensorAltitude = 0x6736,
}

/// Get execution time per command id
pub fn get_execution_time(command: CommandId) -> u32 {
    match command {
        CommandId::StartContinuousMeasurement => 50,
        CommandId::StopMeasurement => 1000,
        CommandId::ActivateShtHeater => 1300,
        CommandId::PerformForcedCo2Recalibration => 500,
        _ => 20,
    }
}

/// Representing sensor measurement state
#[derive(Copy, Clone, Debug, PartialEq)]
enum ModuleState {
    Idle,
    Measuring,
}

/// The maximum number of bytes that the driver has to read for any command
const MAX_RX_BYTES: usize = 48;

/// The maximum number of bytes that the driver has to write for any command
const MAX_TX_BYTES: usize = 18;

/// Shorthand for all functions returning an error in this module.
type Result<T> = core::result::Result<T, Sen6xError>;

/// Represents any error that may happen during communication.
#[derive(Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum Sen6xError {
    /// An error occurred while reading from the module.
    ReadI2CError,
    /// An error occurred while writing to the module.
    WriteI2CError,
    /// The sensor module is in a state that does not permit this command
    InvalidState,
    /// The sensor module returned data which could not be parsed
    InvalidData,
    /// Too much data was provided to the driver implementation
    TooMuchData,
    /// CRC related error
    CrcError(CrcError),
}

impl From<CrcError> for Sen6xError {
    fn from(e: CrcError) -> Self {
        Sen6xError::CrcError(e)
    }
}

/// Represents a measured sample from the sensor module.
#[derive(Debug)]
pub struct MeasuredSample {
    /// PM1 concentration in µg/m³
    pub pm1: f32,
    /// PM2.5 concentration in µg/m³
    pub pm2_5: f32,
    /// PM4 concentration in µg/m³
    pub pm4: f32,
    /// PM10 concentration in µg/m³
    pub pm10: f32,
    /// Relative humidity in percent
    pub humidity: f32,
    /// Temperature in degrees Celsius
    pub temperature: f32,
    /// CO2 concentration in ppm
    pub co2: f32,
    /// VOC concentration in ppb
    pub voc: f32,
    /// NOX concentration in ppb
    pub nox: f32,
}

impl From<[u16; 9]> for MeasuredSample {
    fn from(data: [u16; 9]) -> Self {
        Self {
            pm1: data[0] as f32,
            pm2_5: data[1] as f32,
            pm4: data[2] as f32,
            pm10: data[3] as f32,
            humidity: data[4] as f32 / 100.0,
            temperature: data[5] as f32 / 100.0,
            co2: data[6] as f32,
            voc: data[7] as f32,
            nox: data[8] as f32,
        }
    }
}

/// Represents a raw measured sample from the sensor module.
#[derive(Debug)]
pub struct RawMeasuredSample {
    /// Raw Humidity
    pub raw_humidity: i16,
    /// Raw Temperature
    pub raw_temperature: i16,
    /// Raw VOC ticks
    pub raw_voc: u16,
    /// Raw NOx ticks
    pub raw_nox: u16,
    /// Raw (non-interpolated) CO2 value
    pub raw_co2: u16,
}

impl From<[u16; 5]> for RawMeasuredSample {
    fn from(data: [u16; 5]) -> Self {
        Self {
            raw_humidity: data[0] as i16,
            raw_temperature: data[1] as i16,
            raw_voc: data[2],
            raw_nox: data[3],
            raw_co2: data[4],
        }
    }
}

/// Represents a concentration sample from the sensor module.
/// (these are the same values as in the MeasuredSample struct,
/// just not scaled to the correct values)
#[derive(Debug)]
pub struct RawConcentrationSample {
    /// PM1 concentration
    pub pm1: u16,
    /// PM2.5 concentration
    pub pm2_5: u16,
    /// PM4 concentration
    pub pm4: u16,
    /// PM10 concentration
    pub pm10: u16,
}

impl From<[u16; 4]> for RawConcentrationSample {
    fn from(data: [u16; 4]) -> Self {
        Self {
            pm1: data[0],
            pm2_5: data[1],
            pm4: data[2],
            pm10: data[3],
        }
    }
}

pub struct DeviceStatus {
    /// Fan speed is too high or too low
    pub fan_speed_warning: bool,
    /// CO2 sensor error
    pub co2_error: bool,
    /// PM sensor error
    pub pm_error: bool,
    /// VOC/NOx sensor error
    pub gas_error: bool,
    /// Humidity/temperature sensor error
    pub rh_t_error: bool,
    /// Fan error
    pub fan_error: bool,
}

impl From<[u16; 2]> for DeviceStatus {
    fn from(data: [u16; 2]) -> Self {
        Self {
            fan_speed_warning: (data[0] & (1 << 5)) != 0,
            co2_error: (data[1] & (1 << 9)) != 0,
            pm_error: (data[1] & (1 << 11)) != 0,
            gas_error: (data[1] & (1 << 7)) != 0,
            rh_t_error: (data[1] & (1 << 6)) != 0,
            fan_error: (data[1] & (1 << 4)) != 0,
        }
    }
}

/// (Volatile) internal temperature offset parameters
#[derive(Debug)]
pub struct TempOffsetPars {
    /// Offset (raw value, not scaled)
    pub offset: i16,
    /// Slope (raw value, not scaled)
    pub slope: i16,
    /// Time constant
    pub time_constant: u16,
    /// Slot (in range 0..4)
    pub slot: u16,
}

impl From<TempOffsetPars> for [u16; 4] {
    fn from(data: TempOffsetPars) -> [u16; 4] {
        [
            data.offset as u16,
            data.slope as u16,
            data.time_constant,
            data.slot,
        ]
    }
}

/// (Volatile) internal temperature acceleration parameters
#[derive(Debug)]
pub struct TempAccelPars {
    /// Filter constant K (already scaled)
    pub k: u16,
    /// Filter constant P (already scaled)
    pub p: u16,
    /// Time constant T1 (already scaled)
    pub t1: u16,
    /// Time constant T2 (already scaled)
    pub t2: u16,
}

impl From<TempAccelPars> for [u16; 4] {
    fn from(data: TempAccelPars) -> [u16; 4] {
        [data.k, data.p, data.t1, data.t2]
    }
}

/// VOC/NOx algorithm tuning parameters
#[derive(Debug)]
pub struct AlgorithmTuningParameters {
    /// Index offset (range 1..250, default 100)
    pub index_offset: i16,
    /// Learning time offset hours (range 1..1000, default 12)
    pub learning_time_offset_hours: i16,
    /// Learning time gain hours (range 1.1000, default 12)
    pub learning_time_gain_hours: i16,
    /// Gating max duration minutes (range 0..3000, default 180)
    pub gating_max_duration_minutes: i16,
    /// Std initial (range 10..5000, default 50)
    pub std_initial: i16,
    /// Gain factor (range 1..1000, default 230)
    pub gain_factor: i16,
}

impl From<AlgorithmTuningParameters> for [u16; 6] {
    fn from(data: AlgorithmTuningParameters) -> [u16; 6] {
        [
            data.index_offset as u16,
            data.learning_time_offset_hours as u16,
            data.learning_time_gain_hours as u16,
            data.gating_max_duration_minutes as u16,
            data.std_initial as u16,
            data.gain_factor as u16,
        ]
    }
}

impl From<[u16; 6]> for AlgorithmTuningParameters {
    fn from(data: [u16; 6]) -> Self {
        Self {
            index_offset: data[0] as i16,
            learning_time_offset_hours: data[1] as i16,
            learning_time_gain_hours: data[2] as i16,
            gating_max_duration_minutes: data[3] as i16,
            std_initial: data[4] as i16,
            gain_factor: data[5] as i16,
        }
    }
}
