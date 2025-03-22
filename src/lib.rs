//! This crate provides a platform agnostic no_std driver for the SEN6X sensor modules.
//! The driver is compatible with the [`embedded-hal`](https://crates.io/crates/embedded-hal) traits.
//!
//! The data sheet of the sensor modules can be found [here](https://sensirion.com/resource/datasheet/SEN6x).
//!
//! ## Supported Features by Sensor Variant (Blocking & Async)
//!
//! Both blocking and async modes are implemented for the following functionality.
//!
//! | Functionality                        | SEN63C | SEN65 | SEN66 | SEN68 |
//! |--------------------------------------|--------|-------|-------|-------|
//! | Start/stop measurement               | ✓      | ✓     | ✓     | ✓     |
//! | Get data ready status                | ✓      | ✓     | ✓     | ✓     |
//! | Read measured values                 | ✓      | ✓     | ✓     | ✓     |
//! | Read raw measured values             | ✓      | ✓     | ✓     | ✓     |
//! | Read number concentration values     | ✓      | ✓     | ✓     | ✓     |
//! | Set temperature offset parameters    | ✓      | ✓     | ✓     | ✓     |
//! | Set temperature acceleration         | ✓      | ✓     | ✓     | ✓     |
//! | Get product name                     | ✓      | ✓     | ✓     | ✓     |
//! | Get serial number                    | ✓      | ✓     | ✓     | ✓     |
//! | Read/clear device status             | ✓      | ✓     | ✓     | ✓     |
//! | Device reset                         | ✓      | ✓     | ✓     | ✓     |
//! | Fan cleaning                         | ✓      | ✓     | ✓     | ✓     |
//! | Activate SHT heater                  | ✓      | ✓     | ✓     | ✓     |
//! | VOC algorithm tuning parameters      | ✗      | ✓     | ✓     | ✓     |
//! | VOC algorithm state                  | ✗      | ✓     | ✓     | ✓     |
//! | NOx algorithm tuning parameters      | ✗      | ✓     | ✓     | ✓     |
//! | Forced CO2 recalibration             | ✓      | ✗     | ✓     | ✗     |
//! | CO2 sensor auto calibration          | ✓      | ✗     | ✓     | ✗     |
//! | Set/get ambient pressure             | ✓      | ✗     | ✓     | ✗     |
//! | Set/get sensor altitude              | ✓      | ✗     | ✓     | ✗     |
//!
//! ## Sensor Measurements by Variant
//!
//! | Measurement                          | SEN63C | SEN65 | SEN66 | SEN68 |
//! |--------------------------------------|--------|-------|-------|-------|
//! | PM1.0 (µg/m³)                       | ✓      | ✓     | ✓     | ✓     |
//! | PM2.5 (µg/m³)                       | ✓      | ✓     | ✓     | ✓     |
//! | PM4.0 (µg/m³)                       | ✓      | ✓     | ✓     | ✓     |
//! | PM10 (µg/m³)                        | ✓      | ✓     | ✓     | ✓     |
//! | Humidity (%)                         | ✓      | ✓     | ✓     | ✓     |
//! | Temperature (°C)                     | ✓      | ✓     | ✓     | ✓     |
//! | CO2 (ppm)                           | ✓      | ✗     | ✓     | ✗     |
//! | VOC (index points)                  | ✗      | ✓     | ✓     | ✓     |
//! | NOx (index points)                  | ✗      | ✓     | ✓     | ✓     |
//! | HCHO (ppb)                          | ✗      | ✗     | ✗     | ✓     |
//!
//! ## Raw Measurements by Variant
//!
//! | Raw Measurement                      | SEN63C | SEN65 | SEN66 | SEN68 |
//! |--------------------------------------|--------|-------|-------|-------|
//! | Raw humidity                         | ✓      | ✓     | ✓     | ✓     |
//! | Raw temperature                      | ✓      | ✓     | ✓     | ✓     |
//! | Raw VOC ticks                        | ✗      | ✓     | ✓     | ✓     |
//! | Raw NOx ticks                        | ✗      | ✓     | ✓     | ✓     |
//! | Raw CO2                              | ✗      | ✗     | ✓     | ✗     |
//!
//! ## Supported sensor variants
//!
//! <div class="warning">This generated documentation covers the SEN66 variant (default feature). For any other
//! variant, please consult the source code. The same functions should be available (if the variant supports that functionality), but the
//! sample data returned will contain different characteristics.</div>
//!
//! * SEN63C (crate feature `sen63c`)
//! * SEN65 (crate feature `sen65`)
//! * SEN66 (crate feature `sen66`, default)
//! * SEN68 (crate feature `sen68`)
//!
//! The sensor variants are mutually exclusive. Only one sensor variant feature can be enabled at a time.
//! To activate a sensor variant other than the default SEN66, disable the default feature and enable the desired sensor variant feature.
//!
//!
//! ## Currently unsupported sensor variants
//! * SEN60 (substantially different command set)
//!
//! ## Usage
//!
//! By default, the driver is in blocking mode. To access the async mode variant, enable the `async` feature.
//!
//! The documentation and examples of both modes can be found in the respective modules `blocking` and `async`.
//!
//! ## Unit tests
//! To run the extensive unit tests, use the following command:
//! ```bash
//! cargo xtask test-features
//! ```
#![cfg_attr(all(not(test), not(feature = "std")), no_std)]

// Ensure only one SEN6X sensor variant feature is enabled at a time
#[cfg(feature = "sen60")]
mod feature_check_sen60 {
    #[cfg(any(
        feature = "sen63c",
        feature = "sen65",
        feature = "sen66",
        feature = "sen68"
    ))]
    compile_error!("Only one sensor variant feature can be enabled at a time");
}

#[cfg(feature = "sen63c")]
mod feature_check_sen63c {
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    compile_error!("Only one sensor variant feature can be enabled at a time");
}

#[cfg(feature = "sen65")]
mod feature_check_sen65 {
    #[cfg(any(feature = "sen66", feature = "sen68"))]
    compile_error!("Only one sensor variant feature can be enabled at a time");
}

#[cfg(feature = "sen66")]
mod feature_check_sen66 {
    #[cfg(feature = "sen68")]
    compile_error!("Only one sensor variant feature can be enabled at a time");
}

use crc_internal::CrcError;

#[cfg(feature = "async")]
pub mod asynchronous;

pub mod blocking;

mod crc_internal;

/// I2C address for the sensor module.
const MODULE_ADDR: u8 = 0x6B;

/// Command ID enum.
#[cfg(not(feature = "sen60"))]
#[repr(u16)]
#[derive(Copy, Clone, Debug)]
enum CommandId {
    StartContinuousMeasurement = 0x0021,
    StopMeasurement = 0x0104,
    GetDataReady = 0x0202,

    #[cfg(feature = "sen63c")]
    ReadMeasuredValues = 0x0471,
    #[cfg(feature = "sen65")]
    ReadMeasuredValues = 0x0446,
    #[cfg(feature = "sen66")]
    ReadMeasuredValues = 0x0300,
    #[cfg(feature = "sen68")]
    ReadMeasuredValues = 0x0467,

    #[cfg(feature = "sen63c")]
    ReadMeasuredRawValues = 0x0492,
    #[cfg(any(feature = "sen65", feature = "sen68"))]
    ReadMeasuredRawValues = 0x0455,
    #[cfg(feature = "sen66")]
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

    #[cfg(any(feature = "sen65", feature = "sen68", feature = "sen66"))]
    VocAlgoTuningPars = 0x60D0,
    #[cfg(any(feature = "sen65", feature = "sen68", feature = "sen66"))]
    VocAlgoState = 0x6181,
    #[cfg(any(feature = "sen65", feature = "sen68", feature = "sen66"))]
    NoxAlgoTuningPars = 0x60E1,

    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    PerformForcedCo2Recalibration = 0x6707,
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    Co2SensorAutoCalibrationState = 0x6711,
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    AmbientPressure = 0x6720,
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    SensorAltitude = 0x6736,
}

/// Get execution time per command id
fn get_execution_time(command: CommandId) -> u32 {
    match command {
        CommandId::StartContinuousMeasurement => 50,
        CommandId::StopMeasurement => 1000,
        CommandId::ActivateShtHeater => 1300,
        #[cfg(any(feature = "sen63c", feature = "sen66"))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
pub enum Sen6xError {
    /// An error occurred while reading from the module.
    #[cfg_attr(
        feature = "std",
        error("An error occurred while reading from the SEN6x module")
    )]
    ReadI2CError,
    /// An error occurred while writing to the module.
    #[cfg_attr(
        feature = "std",
        error("An error occurred while writing to the SEN6x module")
    )]
    WriteI2CError,
    /// The sensor module is in a state that does not permit this command
    #[cfg_attr(
        feature = "std",
        error("The SEN6x module is in a state that does not permit this command")
    )]
    InvalidState,
    /// The sensor module returned data which could not be parsed
    #[cfg_attr(
        feature = "std",
        error("The SEN6x module returned data which could not be parsed")
    )]
    InvalidData,
    /// Too much data was provided to the driver implementation
    #[cfg_attr(
        feature = "std",
        error("The SEN6x module provided toto much data to the driver implementation")
    )]
    TooMuchData,
    /// CRC related error
    #[cfg_attr(feature = "std", error("CRC failure on SEN6x data"))]
    CrcError(CrcError),
}

impl From<CrcError> for Sen6xError {
    fn from(e: CrcError) -> Self {
        Sen6xError::CrcError(e)
    }
}

/// Represents a measured sample from the sensor module.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    #[cfg(any(feature = "sen63c", feature = "sen66"))]
    pub co2: u16,
    /// VOC concentration in index points
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub voc: f32,
    /// NOX concentration in index points
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub nox: f32,
    /// Formaldehyde (HCHO) concentration in ppb
    #[cfg(feature = "sen68")]
    pub hcho: f32,
}

#[cfg(feature = "sen66")]
impl From<[u16; 9]> for MeasuredSample {
    fn from(data: [u16; 9]) -> Self {
        Self {
            pm1: data[0] as f32 / 10.0,
            pm2_5: data[1] as f32 / 10.0,
            pm4: data[2] as f32 / 10.0,
            pm10: data[3] as f32 / 10.0,
            humidity: data[4] as f32 / 100.0,
            temperature: data[5] as f32 / 200.0,
            voc: data[6] as f32 / 10.0,
            nox: data[7] as f32 / 10.0,
            co2: data[8],
        }
    }
}

#[cfg(feature = "sen63c")]
impl From<[u16; 7]> for MeasuredSample {
    fn from(data: [u16; 7]) -> Self {
        Self {
            pm1: data[0] as f32 / 10.0,
            pm2_5: data[1] as f32 / 10.0,
            pm4: data[2] as f32 / 10.0,
            pm10: data[3] as f32 / 10.0,
            humidity: data[4] as f32 / 100.0,
            temperature: data[5] as f32 / 200.0,
            co2: data[6],
        }
    }
}

#[cfg(feature = "sen65")]
impl From<[u16; 8]> for MeasuredSample {
    fn from(data: [u16; 8]) -> Self {
        Self {
            pm1: data[0] as f32 / 10.0,
            pm2_5: data[1] as f32 / 10.0,
            pm4: data[2] as f32 / 10.0,
            pm10: data[3] as f32 / 10.0,
            humidity: data[4] as f32 / 100.0,
            temperature: data[5] as f32 / 200.0,
            voc: data[6] as f32 / 10.0,
            nox: data[7] as f32 / 10.0,
        }
    }
}

#[cfg(feature = "sen68")]
impl From<[u16; 9]> for MeasuredSample {
    fn from(data: [u16; 9]) -> Self {
        Self {
            pm1: data[0] as f32 / 10.0,
            pm2_5: data[1] as f32 / 10.0,
            pm4: data[2] as f32 / 10.0,
            pm10: data[3] as f32 / 10.0,
            humidity: data[4] as f32 / 100.0,
            temperature: data[5] as f32 / 200.0,
            voc: data[6] as f32 / 10.0,
            nox: data[7] as f32 / 10.0,
            hcho: data[8] as f32 / 10.0,
        }
    }
}

/// Represents a raw measured sample from the sensor module.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawMeasuredSample {
    /// Raw Humidity
    pub raw_humidity: i16,
    /// Raw Temperature
    pub raw_temperature: i16,
    /// Raw VOC ticks
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub raw_voc: u16,
    /// Raw NOx ticks
    #[cfg(any(feature = "sen65", feature = "sen66", feature = "sen68"))]
    pub raw_nox: u16,
    /// Raw (non-interpolated) CO2 value
    #[cfg(feature = "sen66")]
    pub raw_co2: u16,
}

#[cfg(feature = "sen63c")]
impl From<[u16; 2]> for RawMeasuredSample {
    fn from(data: [u16; 2]) -> Self {
        Self {
            raw_humidity: data[0] as i16,
            raw_temperature: data[1] as i16,
        }
        // It is surprising here that, according to the datasheet v0.9,
        // the SEN63C does not provide raw CO2 values.
    }
}

#[cfg(any(feature = "sen65", feature = "sen68"))]
impl From<[u16; 4]> for RawMeasuredSample {
    fn from(data: [u16; 4]) -> Self {
        Self {
            raw_humidity: data[0] as i16,
            raw_temperature: data[1] as i16,
            raw_voc: data[2],
            raw_nox: data[3],
        }
        // It is surprising here that, according to the datasheet v0.9,
        // the SEN68 does not provide raw HCHO values.
    }
}

#[cfg(any(feature = "sen66"))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// HCHO error
    pub hcho_error: bool,
}

impl From<[u16; 2]> for DeviceStatus {
    fn from(data: [u16; 2]) -> Self {
        // For now, we're not feature-gating the error flags
        // per variant beyond the co2-switch. Non-existing flags
        // in a variant can simply be ignored.
        Self {
            fan_speed_warning: (data[0] & (1 << 5)) != 0,
            #[cfg(any(feature = "sen66", feature = "sen68", feature = "sen65"))]
            co2_error: (data[1] & (1 << 9)) != 0,
            #[cfg(feature = "sen63c")]
            co2_error: (data[1] & (1 << 12)) != 0,
            pm_error: (data[1] & (1 << 11)) != 0,
            gas_error: (data[1] & (1 << 7)) != 0,
            rh_t_error: (data[1] & (1 << 6)) != 0,
            fan_error: (data[1] & (1 << 4)) != 0,
            hcho_error: (data[1] & (1 << 10)) != 0,
        }
    }
}

/// (Volatile) internal temperature offset parameters
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
