//! Example of reading sensor values asynchronously.
//!
//! This example uses embassy and the async variant of the sen6x create to
//! read sensor values asynchronously for 5 minutes.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::i2c::InterruptHandler;
use embassy_time::{Duration, Timer};
use sen6x::asynchronous::Sen6x;
use {defmt_rtt as _, panic_probe as _};

embassy_rp::bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_task_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_26;
    let scl = p.PIN_27;
    let config = embassy_rp::i2c::Config::default();
    let mut bus = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, config);
    let mut delay = embassy_time::Delay;

    let mut sen6x = Sen6x::new(&mut delay, &mut bus);

    // Read device information
    let mut product_name_buf = [0u8; 32];
    match sen6x.get_product_name(&mut product_name_buf).await {
        Ok(name) => info!("Product name: {}", name),
        Err(e) => error!("Error reading product name: {:?}", e),
    }

    let mut serial_number_buf = [0u8; 32];
    match sen6x.get_serial_number(&mut serial_number_buf).await {
        Ok(serial) => info!("Serial number: {}", serial),
        Err(e) => error!("Error reading serial number: {:?}", e),
    }

    // Start measurement
    match sen6x.start_continuous_measurement().await {
        Ok(_) => info!("Measurement started"),
        Err(e) => error!("Error starting measurement: {:?}", e),
    }

    // Read measurements for 5 minutes
    for _ in 0..300 {
        Timer::after(Duration::from_millis(1000)).await;
        match sen6x.get_sample().await {
            Ok(values) => {
                info!("Sample: {:?}\n", values);
            }
            Err(e) => error!("Error reading values: {:?}", e),
        }
    }

    // Stop measurement
    match sen6x.stop_measurement().await {
        Ok(_) => info!("Measurement stopped"),
        Err(e) => error!("Error stopping measurement: {:?}", e),
    }

    // Just some loop so we never return
    info!("Entering infinite loop");
    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
