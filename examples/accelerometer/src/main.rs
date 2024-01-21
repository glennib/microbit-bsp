#![no_std]
#![no_main]

use defmt::{info, Debug2Format};
use embassy_executor::Spawner;
use embassy_time::Duration;
use microbit_bsp::{
    accelerometer::Accelerometer,
    display::{fonts::CROSS_MARK, Brightness},
    embassy_nrf::{bind_interrupts, peripherals::TWISPI0, twim::InterruptHandler},
    lsm303agr::Acceleration,
    Microbit,
};
use micromath::{
    vector::{Vector, Vector3d},
    F32Ext,
};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let board = Microbit::default();
    defmt::info!("Application started!");

    let mut display = board.display;
    display.set_brightness(Brightness::MAX);

    // Bind interrupt to the TWI/SPI peripheral.
    bind_interrupts!(
        struct InterruptRequests {
            SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => InterruptHandler<TWISPI0>;
        }
    );

    let irqs = InterruptRequests {};
    let mut acc = Accelerometer::new(board.twispi0, irqs, board.p23, board.p22).unwrap();

    loop {
        // let status = acc.accel_status().unwrap();
        // info!("status: {:?}", Debug2Format(&status));

        let (x, y) = sines_from_acceleration(acc.accel_data().unwrap());

        let x = sin_to_level(x);
        let y = sin_to_level(y);

        // info!("{}, {}", x, y);

        if x.abs() == 3 || y.abs() == 3 {
            display.apply(CROSS_MARK);
        } else {
            display.on(usize::try_from(2_i8 - x).unwrap(), usize::try_from(2_i8 - y).unwrap());
        };
        display.display_current(Duration::from_millis(100)).await;
    }
}

fn sin_to_level(a: f32) -> i8 {
    const WORST: f32 = 0.12247903839280565;
    const OK: f32 = 0.06987000497506388;
    const GOOD: f32 = 0.034913677698806274;
    const L_WORST: i8 = L_OK;
    const L_OK: i8 = 2;
    const L_GOOD: i8 = 1;
    const L_GREAT: i8 = 0;
    if a <= -WORST {
        return -L_WORST;
    }
    if a <= -OK {
        return -L_OK;
    }
    if a <= -GOOD {
        return -L_GOOD;
    }
    if a <= GOOD {
        return L_GREAT;
    }
    if a <= OK {
        return L_GOOD;
    }
    if a <= WORST {
        return L_OK;
    }
    L_WORST
}

type V = Vector3d<f32>;

fn sines_from_acceleration(acceleration: Acceleration) -> (f32, f32) {
    let reference = V::from((0., 0., -1000.));
    let measured = V::from({
        let (x, y, z) = acceleration.xyz_mg();
        #[allow(clippy::cast_precision_loss)]
        (x as f32, y as f32, z as f32)
    });
    let measured_xz = V {
        x: measured.x,
        y: 0.,
        z: measured.z,
    };
    let measured_yz = V {
        x: 0.,
        y: measured.y,
        z: measured.z,
    };



    let (x, y) = (sin_between(measured_xz, reference), -sin_between(reference, measured_yz));
    (x, y)
}

// fn cos_between(a: V, b: V) -> f32 {
//     (a.dot(b) / (a.magnitude() * b.magnitude())).acos()
// }

// sin θ = |a × b| / (|a| |b|)
fn sin_between(a: V, b: V) -> f32 {
    let axb = a * b;
    let den = a.magnitude() * b.magnitude();
    let signless = axb.magnitude() / den;
    signless.copysign(axb.z)
}
