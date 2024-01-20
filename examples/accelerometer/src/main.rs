#![no_std]
#![no_main]

use defmt::{info, Debug2Format};
use embassy_executor::Spawner;
use embassy_time::Timer;
use microbit_bsp::{
    accelerometer::Accelerometer,
    embassy_nrf::{bind_interrupts, peripherals::TWISPI0, twim::InterruptHandler},
    Microbit,
};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let board = Microbit::default();
    defmt::info!("Application started!");

    // Bind interrupt to the TWI/SPI peripheral.
    bind_interrupts!(
        struct InterruptRequests {
            SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => InterruptHandler<TWISPI0>;
        }
    );

    let irqs = InterruptRequests {};
    let mut acc = Accelerometer::new(board.twispi0, irqs, board.p23, board.p22).unwrap();

    loop {
        let status = acc.accel_status().unwrap();
        let data = acc.accel_data().unwrap();
        let (x, y, z) = data.xyz_mg();
        info!("status: {:?}, x: {}, y: {}, z: {}", Debug2Format(&status), x, y, z);
        Timer::after_millis(100).await;
    }
}
