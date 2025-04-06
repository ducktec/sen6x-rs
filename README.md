# SEN6x Rust Driver

A Rust `no-std`/`std` driver for the SEN6x environmental sensor nodes for air quality applications.

[![Crates.io](https://img.shields.io/crates/v/sen6x.svg)](https://crates.io/crates/sen6x)
[![Docs.rs](https://docs.rs/sen6x/badge.svg)](https://docs.rs/sen6x)

## Description

This library provides a Rust interface for the SEN6x family of environment sensor nodes (SEN-63C, SEN-65, SEN-66, SEN68). It is designed to be used in no-std environments, making it suitable for use in embedded systems.

## Supported features

All described features are supported both in blocking as well as async mode.

* Start/stop measurement
* Read sensor samples (both processed and raw)
* Read device name, ID, and status
* Manage sensor configuration (if applicable for module variant)
    * Temperature offset
    * VOC algorithm tuning
    * NOX algorithm tuning
    * Ambient pressure/altitude
* Reset module
* Forced CO2 recalibration
* Fan Cleaning
* Activate SHT heater

Please see the doc.rs documentation for a feature matrix of the supported features for each module variant.

## Unsupported features

* The SEN-60 variant is not yet supported (it has a substantially different command set), but may be added in the future

## Rust Features
* `async` *(default)*: Enables async capabilities of the driver
* `defmt`: Enables the `defmt::Format` trait for the `defmt` logging framework (adds `defmt` crate as dependency)
* `serde`: Enables `serde` support for the driver (adds `serde` crate as dependency, with default features disabled for `no-std` compatibility)
* `std`: Enables `std` functionality of the driver (unlocks `thiserror` error macros)
* `sen60`/`sen63`/`sen65`/`sen66` *(default)* /`sen68`: Enables support for the respective sensor variant. Only one of these features can be enabled at a time. The variant enabled by default is `sen66`.

## Dependencies

This driver depends on the `embedded-hal` and `embedded-hal-async` crate for hardware abstraction. If activated by feature `defmt`, it also implements the `defmt::Format` trait for the `defmt` logging framework (and thus depends on `defmt`) for relevant `structs`.

For development, it uses the `embedded-hal-mock` crate for mocking the hardware abstraction layer. For the example, it uses `linux-embedded-hal`, to be tentatively run on an RPI (not tested yet!).

## Usage

> [!WARNING]  
> The sensor variant features are mutually exclusive. Only one sensor variant feature can be enabled at a time.
> To activate a sensor variant other than the default SEN66, disable the default feature and enable the desired
> sensor variant feature. data returned will contain different characteristics.

To use this driver with the SEN66 variant (default), add the following to your `Cargo.toml` file:

```toml
[dependencies]
sen6x = "0.1.0"
```

Then, you can use it in your Rust code like this:

```rust
// Create peripherals here!

// Create instance of the driver
let mut sen6x = Sen6x::new(&mut timer, i2c);

// Start measurement
match sen6x.start_continuous_measurement() {
    Ok(_) => info!("Measurement started"),
    Err(e) => error!("Error starting measurement: {:?}", e),
}

// Read measurement
match sen6x.get_sample() {
    Ok(values) => {
        info!("Sample: {:?}\n", values);
    }
    Err(e) => error!("Error reading values: {:?}", e),
}

// Stop measurement
match sen6x.stop_measurement() {
    Ok(_) => info!("Measurement stopped"),
    Err(e) => error!("Error stopping measurement: {:?}", e),
}
```

## Testing

The library provides extensive unit tests for all features. To run the tests for all variant (and blocking/async), use the following command:

```bash
cargo xtask test-features
```

This library was tested so far (using the examples) on a Raspberry Pi Pico 2W and an directly connected engineering sample of the SEN-66 module.

## Examples
Basic example reading for sensor values are available in the [`examples-rp2350`](https://github.com/ducktec/sen6x-rs/tree/main/examples-rp2350) directory of the source code repository. Follow the instructions in the README.md in that directory to run the examples on a Raspberry Pi Pico 2/2W.

## Links
Module Datasheet: [SEN6x Datasheet](https://sensirion.com/resource/datasheet/SEN6x) (developed on version `0.9`)

## License
This library is licensed under either of
* MIT license (LICENSE-MIT or http://opensource.org/licenses/MIT)
* Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)
at your option.

## Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

# Changelog

v0.1.0 - Initial release

v0.1.1 - Fix keywords issue in `Cargo.toml`

v0.1.2 - Improve documentation, fix doc.rs build issue, reduce `pub` visibility

v0.1.3 - Fix unit error for VOC and NOx (should be index points instead of ppb)

v0.1.4 - Serde support (for samples), derive `Clone` for sample, add `std` feature to allow for `thiserror` convenience (all thanks to @sbruton)