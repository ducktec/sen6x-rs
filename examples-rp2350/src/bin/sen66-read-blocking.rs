//! Example of reading sensor values in blocking mode.
//!
//! This example uses rp-hal and the blocking variant of the sen6x crate to
//! read sensor values for 5 minutes.

#![no_std]
#![no_main]

use defmt::*;
use embedded_hal::delay::DelayNs;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use rp235x_hal as hal;
use {defmt_rtt as _, panic_probe as _};

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

use sen6x::blocking::Sen6x;

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();
    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio26.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio27.reconfigure();

    // For loop delays
    let mut delay = hal::Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    // For driver to consume
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let mut sen6x = Sen6x::new(&mut timer, i2c);

    // Read device information
    let mut product_name_buf = [0u8; 32];
    match sen6x.get_product_name(&mut product_name_buf) {
        Ok(name) => info!("Product name: {}", name),
        Err(e) => error!("Error reading product name: {:?}", e),
    }

    let mut serial_number_buf = [0u8; 32];
    match sen6x.get_serial_number(&mut serial_number_buf) {
        Ok(serial) => info!("Serial number: {}", serial),
        Err(e) => error!("Error reading serial number: {:?}", e),
    }

    // Start measurement
    match sen6x.start_continuous_measurement() {
        Ok(_) => info!("Measurement started"),
        Err(e) => error!("Error starting measurement: {:?}", e),
    }

    // Read measurements for 5 minutes
    for _ in 0..300 {
        delay.delay_ms(1000);
        match sen6x.get_sample() {
            Ok(values) => {
                info!("Sample: {:?}\n", values);
            }
            Err(e) => error!("Error reading values: {:?}", e),
        }
    }

    // Stop measurement
    match sen6x.stop_measurement() {
        Ok(_) => info!("Measurement stopped"),
        Err(e) => error!("Error stopping measurement: {:?}", e),
    }

    // Wait forever
    loop {
        hal::arch::wfi();
    }
}
