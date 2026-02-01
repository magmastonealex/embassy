#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_mspm0::can::{CanRx, CanTx};
use embassy_mspm0::can::frame::MCanFrame;
use embassy_mspm0::gpio::Output;
use embassy_mspm0::mode::Async;
use embassy_mspm0::{Config, bind_interrupts, can};
use embassy_mspm0::peripherals::CANFD0;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, DynamicReceiver, DynamicSender};
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

static OUTBOUND: Channel<ThreadModeRawMutex, MCanFrame, 10> = Channel::new();

#[embassy_executor::task]
async fn periodic_hello(outgoing: DynamicSender<'static, MCanFrame>) {
    loop {
        Timer::after_secs(5).await;

        for number in 1u8..10 {
            let frame =
                can::frame::MCanFrame::new(Id::Standard(StandardId::new(0x123).unwrap()), &[0x12u8, 0x34, number])
                    .unwrap();
            outgoing.send(frame).await;
            info!("Sent hello frame!");
        }
    }
}

#[embassy_executor::task]
async fn transmitter_mux(mut tx: CanTx<Async>, incoming: DynamicReceiver<'static, MCanFrame>) {
    loop {
        let frame = incoming.receive().await;
        tx.enqueue_frame(&frame).await.expect("no buserror possible");
    }
}

#[embassy_executor::task]
async fn receiver(mut rx: CanRx<Async>, outgoing: DynamicSender<'static, MCanFrame>) {
    loop {
        let mut frame = rx.get_frame().await.expect("no buserror possible right now");
        info!("Received frame: {}", frame);
        frame.set_id(Id::Standard(StandardId::new(0x0ab).unwrap()));
        info!("Sending reply... {}", frame);
        outgoing.send(frame).await;    
    }
}


#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    info!("Hello from the CAN example!");
    let p = embassy_mspm0::init(Config::default());

    // Note: You may need to set the STANDBY pin or similar on your CAN transciver.
    // Uncomment and change these lines as needed.
    let mut _canstb = Output::new(p.PA0, embassy_mspm0::gpio::Level::Low);

    // Configure CANFD for a 100kbit/s bitrate.
    let candriver = can::Can::new_async(p.CANFD0, p.PA27, p.PA26, Irqs, can::Config::default()).unwrap();
    let (tx, rx, mut status) = candriver.split();

    spawner.spawn(receiver(rx, OUTBOUND.dyn_sender()).unwrap());
    spawner.spawn(transmitter_mux(tx, OUTBOUND.dyn_receiver()).unwrap());
    spawner.spawn(periodic_hello(OUTBOUND.dyn_sender()).unwrap());

    loop {
        Timer::after_secs(3).await;

        let errors = status.get_error_counters();

        info!("CAN error counters: {:?}", errors);

        if errors.bus_off {
            info!("Starting bus-off recovery");
            match status.recover() {
                Ok(_) => {
                    info!("Bus-off recovery completed.");
                }
                Err(e) => {
                    warn!("Bus-off recovery failed: {}", e);
                }
            }
        }
    }
}
