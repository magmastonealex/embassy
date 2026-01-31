#![no_std]
#![no_main]

use core::num;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_mspm0::gpio::Output;
use embassy_mspm0::{Config, bind_interrupts, can};
use embassy_mspm0::peripherals::CANFD0;
use embassy_time::{Duration, Instant, Timer};
use embedded_can::{Frame, Id, StandardId};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    CANFD0 => can::InterruptHandler<CANFD0>;
});

// To run this example, connect a CAN transiever to the TX and RX pins.
// Then, connect a USB CAN adapter, configure for 100kbit/s bitrate, and send a frame.
// The example will send a response frame with ID 0x0ab and the same data payload.
//
// It will also send a frame every 5 seconds.
//
// This example also demonstrates bus-off recovery, which can be triggered by shorting the CAN+ and CAN- lines together.

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Hello from the CAN example!");
    let p = embassy_mspm0::init(Config::default());

    // Note: You may need to set the STANDBY pin or similar on your CAN transciver.
    // Uncomment and change these lines as needed.
    let mut _canstb = Output::new(p.PA0, embassy_mspm0::gpio::Level::Low);

    // Configure CANFD for a 100kbit/s bitrate.
    let mut candriver = can::Can::new_async(p.CANFD0, p.PA27, p.PA26, Irqs, can::Config::default()).unwrap();

    let mut last_stat_dump = Instant::now();
    let mut last_sent = Instant::now();

    loop {
        match select(candriver.get_frame(), Timer::after(Duration::from_secs(2))).await {
            Either::First(res) => {
                let mut frame = res.expect("no buserror");
                info!("Received frame: {}", frame);
                frame.set_id(Id::Standard(StandardId::new(0x0ab).unwrap()));
                info!("Sending reply... {}", frame);
                if candriver.enqueue_frame(&frame).await.is_err() {
                    warn!("Frame queue is full!");
                }
            },
            Either::Second(_) => {
                info!("timed out");
            }
        }

        Timer::after(Duration::from_secs(2)).await;

        if Instant::now().duration_since(last_stat_dump).as_millis() > 1000 {
            last_stat_dump = Instant::now();

            let errors = candriver.get_error_counters();

            info!("CAN error counters: {:?}", errors);

            if errors.bus_off {
                info!("Starting bus-off recovery");
                match candriver.recover() {
                    Ok(_) => {
                        info!("Bus-off recovery completed.");
                    }
                    Err(e) => {
                        warn!("Bus-off recovery failed: {}", e);
                    }
                }
            }
        }

        // every 5 seconds, send an unsolicited message.
        if Instant::now().duration_since(last_sent).as_millis() > 5000 {

            for number in 1u8..50 {
                let frame =
                    can::frame::MCanFrame::new(Id::Standard(StandardId::new(0x123).unwrap()), &[0x12u8, 0x34, number])
                        .unwrap();
                if candriver.enqueue_frame(&frame).await.is_err() {
                    warn!("Frame queue is full!");
                } else {
                    info!("Sent hello frame!");
                }
            }
            

            last_sent = Instant::now();
        }
    }
}
