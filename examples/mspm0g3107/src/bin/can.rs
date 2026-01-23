#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_mspm0::Config;
use embassy_mspm0::can;
use embassy_time::{Instant, Timer};
use embedded_can::{Id, StandardId};
use {defmt_rtt as _, panic_probe as _};

// To run this example, connect a CAN transiever to the TX and RX pins.
// Connect a CAN adapter, configure for 100kbit/s bitrate, and send a frame.
// The example will send a response frame with ID 0x0ab and the same data payload.
//
// This example also demonstrates bus-off recovery, which can be triggered by repeatedly sending a CAN frame
// with the wrong bitrate configured.

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[derive(defmt::Format)]
pub enum CanError {
    ClockNotComingUp,
    CANNotRespondingReset,
    CANNotRespondingMem,
    CANNotResponding
}

fn setup_hfext() -> Result<(), CanError> {
    // Hack: Set up HFXT with the 25MHz oscillator
    // We'll use this for clocking the CAN peripheral
    // PA6 - PINCM11. Pin function 6 is HFCLK_IN.
    embassy_mspm0::pac::IOMUX.pincm(10).modify(|w| {
        w.set_pf(6);
        w.set_inena(true);
        w.set_pc(true);
    });
    embassy_mspm0::pac::SYSCTL.hsclken().modify(|w| {
        w.set_useexthfclk(true);
    });

    let mut cnt = 0;
    while !embassy_mspm0::pac::SYSCTL.clkstatus().read().hfclkgood() {
        if cnt > 1000 {
            return Err(CanError::ClockNotComingUp);
        }
        cnt += 1;
        cortex_m::asm::delay(1000);
    }

    Ok(())
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Hello from the CAN example!");
    let p = embassy_mspm0::init(Config::default());

    setup_hfext().unwrap();

    info!("init completed");

    // Configure CANFD for a 100kbit/s bitrate.
    let mut candriver = can::Can::new_blocking(p.CANFD0, p.PA27, p.PA26, can::Config::default()).unwrap();

    info!("is here2");

    let mut last_stat_dump  = Instant::now();

    info!("is here");

    loop {

        if candriver.has_frame() {
            let mut frame = candriver.get_frame_blocking().unwrap();
            info!("Received frame: {}", frame);
            frame.set_id(Id::Standard(StandardId::new(0x0ab).unwrap()));
            info!("Sending reply... {}", frame);
            candriver.enqueue_frame_blocking(&frame).unwrap();
        };

        if Instant::now().duration_since(last_stat_dump).as_millis() > 1000 {
            last_stat_dump = Instant::now();

            info!("CAN error counters: {:?}", candriver.get_error_counters());
        }

        match candriver.status() {
            Some(can::BusError::BusOff) => {
                info!("Starting bus-off recovery");
                candriver.recover().unwrap();
                while matches!(candriver.status(), Some(can::BusError::BusOff)) {
                    info!("Waiting for bus-off recovery...");
                    Timer::after_millis(500).await;
                }
                info!("Bus-off recovery completed.");
            }
            _ => {}
        }

    }
}
