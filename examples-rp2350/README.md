# Raspberry Pi Pico 2W Examples for SEN6X Sensor Library (sen6x-rs)

This directory contains examples of using the `sen6x-rs` library with the Raspberry Pi Pico 2/2W (RP2350) microcontroller board.

## Prerequisites

- Raspberry Pi Pico 2/2W board
- Sensirion SEN66 sensor
- Rust installed with `thumbv8m.main-none-eabihf` target
- `probe-rs` installed ([probe.rs](https://probe.rs/))
- A debug probe (e.g. [Raspberry Pi Debug Probe](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html)) connected to the SWD header of the Pico 2/2W.
- Appropriate connections between the Pico 2/2W and sensor

## Hardware Setup

Connect the SEN66 sensor to the Raspberry Pi Pico 2/2W using I2C:

| Pico W Pin | SEN66 Pin |
|------------|-----------|
| GP26       | SDA       |
| GP27       | SCL       |
| 3.3V       | VDD       |
| GND        | GND       |

## Building and Running

```bash
cargo run --bin sen66-read-blocking # or --bin sen66-read-async
```

This will build and download the example to the board. The example will read the sensor data and print it to the `defmt` log which is opened immediately after flashing the program.