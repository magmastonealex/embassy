use core::marker::PhantomData;

use embassy_hal_internal::PeripheralType;

use crate::Peri;
use crate::gpio::AnyPin;
use crate::interrupt::Interrupt;
use crate::mode::{Blocking, Mode};
use crate::pac::canfd::{Canfd as Regs, vals as CanVals};
use embassy_sync::waitqueue::AtomicWaker;

pub(crate) struct Info { // metadata/details about the specific instance of the peripheral in use.
    pub(crate) regs: Regs, // the registers for this specific instance
    pub(crate) interrupt: Interrupt, // which interrupt applies to this peripheral
}

pub(crate) struct State {
    // waker for when interesting things happen, I guess.
    pub(crate) waker: AtomicWaker,
}

// prevent external callers from creating instances of this.
pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

// provide all of the details we need to use the CANFD0 peripheral.
// We would need another one of these for CANFD1, etc. which is why macros are usually used.
impl SealedInstance for crate::peripherals::CANFD0 {
    fn info() -> &'static Info {
        // too many types named Interrupt. There's an impl ... somewhere? in generated code impl Interrupt for CANFD0 which provides the IRQ constant enum value,
        // which we then use to reference the actual interrupt channel in other methods.
        // The type usage here is quite confusing to me.
        use crate::interrupt::typelevel::Interrupt; 

        const INFO: Info = Info {
            regs: crate::pac::CANFD0,
            interrupt: crate::interrupt::typelevel::CANFD0::IRQ,
        };

        &INFO
    }

    fn state() -> &'static State {
        static STATE: State = State {
            waker: AtomicWaker::new()
        };

        &STATE
    }
}

impl Instance for crate::peripherals::CANFD0 {
    type Interrupt = crate::interrupt::typelevel::CANFD0; // I'm still unclear why this type is needed _here_ too.
}


#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockDiv {
    // Do not divide clock source.
    DivBy1,
    // Divide clock source by 2.
    DivBy2,
    // Divide clock source by 4.
    DivBy4,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Structure to encode CAN timing parameter information.
/// Note that the hardware adds '1' to each of the values placed in the registers of the peripheral.
/// This crate handles this for you, so the values in this struct should be the actual values you wish to use.
pub struct CanTimings {
    brs: u16, /// Bitrate prescaler, valid values 1-512. 
    sjw: u8, /// Sync Jump Width - valid values 1-128, though must also be <= ntseg2.
    ntseg1: u16, // Segment 1 time. Valid values are 2-256
    ntseg2: u8, // Segment 2 time. Valid values are 2-128.
}

impl CanTimings {
    pub const fn from_values(brs: u16, sjw: u8, ntseg1: u16, ntseg2: u8) -> Option<CanTimings> {
        Some(CanTimings { brs, sjw, ntseg1, ntseg2 })
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// CAN Configuration
pub struct Config {
    /// Input clock rate
    /// (Temporary - currently this crate doesn't support more complex clock configurations - CAN clock will always be sourced from HFXT/HFEXT_IN, and we require the clock rate here)
    pub functional_clock_rate: u32,

    /// Input clock divider
    pub clock_div: ClockDiv,

    /// CAN timings to use for standard CAN. (CAN-FD support to come later.)
    pub timing: CanTimings
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Config Error
pub enum InitializationError {
    /// Clock source not enabled.
    ///
    /// The clock soure is not enabled is SYSCTL.
    ClockSourceNotEnabled,

    /// Peripheral timed out.
    ///
    /// Failed to handshake with the CAN peripheral in a reasonable time - often indicates the peripheral has locked up
    /// and the device will need to be reset before it will function again.
    PeripheralTimedOut,
}

pub struct Can<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    rx: Option<Peri<'d, AnyPin>>,
    tx: Option<Peri<'d, AnyPin>>,
    _phantom: PhantomData<M>,
}

impl<'d> Can<'d, Blocking> {
    pub fn new_blocking<T: Instance> (
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl RxPin<T>>,
        config: Config
    ) -> Result<Self, InitializationError> {

        Self::new_inner(peri, rx, tx, config)
    }
}

impl<'d, M: Mode> Can<'d, M> {
    fn new_inner<T: Instance> (
        _peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl RxPin<T>>,
        mut config: Config
    ) -> Result<Self, InitializationError> {

        // Note: use new_pin! when in tree.
        
        Err(InitializationError::ClockSourceNotEnabled)
    }
}

// RX and TX pin traits - normally constructed via a macro, we'll do it manually to demonstrate functionality.
// These are effectively sealed because pf_num isn't public so can't be implemented by anyone else.
// we'll implement the correct combinations so the type system enforces you can't pass invalid pins in for each use case.
// pf_num is metadata we need anyways.
pub trait RxPin<T: Instance>: crate::gpio::Pin {
    fn pf_num(&self) -> u8;
}

pub trait TxPin<T: Instance>: crate::gpio::Pin {
    fn pf_num(&self) -> u8;
}

impl RxPin<crate::peripherals::CANFD0> for crate::peripherals::PA27 {
    fn pf_num(&self) -> u8 {
        6u8
    }
}

impl TxPin<crate::peripherals::CANFD0> for crate::peripherals::PA26 {
    fn pf_num(&self) -> u8 {
        6u8
    }
}