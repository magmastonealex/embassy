use core::marker::PhantomData;

use embassy_hal_internal::PeripheralType;

use crate::Peri;
use crate::gpio::{AnyPin, PfType};
use crate::interrupt::Interrupt;
use crate::mode::{Blocking, Mode};
use crate::pac::canfd::{Canfd as Regs, vals as CanVals};
use crate::pac::{self};

use embassy_sync::waitqueue::AtomicWaker;

mod msgram;

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
    brp: u16, /// Bitrate prescaler, valid values 1-512. 
    sjw: u8, /// Sync Jump Width - valid values 1-128, though must also be <= ntseg2.
    ntseg1: u16, // Segment 1 time. Valid values are 2-256
    ntseg2: u8, // Segment 2 time. Valid values are 2-128.
}

impl CanTimings {
    pub const fn from_values(brp: u16, sjw: u8, ntseg1: u16, ntseg2: u8) -> Option<CanTimings> {
        Some(CanTimings { brp, sjw, ntseg1, ntseg2 })
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
    pub timing: CanTimings,

    pub accept_remote_frames: bool,

    pub accept_extended_ids: bool
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
        tx: Peri<'d, impl TxPin<T>>,
        config: Config
    ) -> Result<Self, InitializationError> {
        Self::new_inner(peri, rx, tx, config)
    }
}

impl<'d, M: Mode> Can<'d, M> {
    fn reset_poweron<T: Instance>(_peri: &Peri<'d, T>, config: &Config) -> Result<(), InitializationError> {
        // See e2e: https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1605241/mspm0g3107-mcan-peripheral-does-not-complete-initialization-after-power-on-reset
        // The initialization instructions in the TRM are not accurate at this time, which I had to figure out the hard way.
        // Suggested "restart" / reset approach is to do a reset, then disable power, then re-enable power.
        // If you do not wait >= 50us before accessing peripheral registers for the first time (or trying to enable clock) after enabling power,
        // the peripheral will lock up and only ever return zeros until reset via sysrst.
        let can = T::info().regs;

        let mut hdr = msgram::RxHeader0(10);
        
        

        can.rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(CanVals::ResetKey::KEY);
        });
        cortex_m::asm::delay(16);

        can.pwren().write(|w| {
            w.set_enable(false);
            w.set_key(CanVals::PwrenKey::KEY);
        });
        cortex_m::asm::delay(32);

        can.pwren().write(|w| {
            w.set_enable(true);
            w.set_key(CanVals::PwrenKey::KEY);
        });
        cortex_m::asm::delay(4000); // TODO: this should be calculated from MCLK at some point as 50us.

        // again, not in reference manual and not required for other peripherals, but you need to now turn on the clock request signal.
        can.ti_wrapper(0).msp(0).subsys_clken().write(|w| {
            w.set_clk_reqen(true);
        });

        // Set a functional clock source - for now we only support HFCLK (HFXT or HFCLKIN).
        can.ti_wrapper(0).msp(0).subsys_clkdiv().write(|w| {
            w.set_ratio(match config.clock_div {
                ClockDiv::DivBy1 => CanVals::Ratio::DIV_BY_1_,
                ClockDiv::DivBy2 => CanVals::Ratio::DIV_BY_2_,
                ClockDiv::DivBy4 => CanVals::Ratio::DIV_BY_4_
            });
        });

        pac::SYSCTL.genclkcfg().modify(|w| {
            w.set_canclksrc(pac::sysctl::vals::Canclksrc::HFCLK);
        });

        // Wait for async reset to be complete.
        let mut iter = 0;
        while can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_stat().read().reset() {
            if iter > 1000 {
                return Err(InitializationError::PeripheralTimedOut);
            }
            iter += 1;
            cortex_m::asm::delay(1000);
        }

        // Wait for "memory initialization" to be complete. I think this is zeroing the internal message RAM.
        iter = 0;
        while can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_stat().read().mem_init_done() {
            if iter > 1000 {
                return Err(InitializationError::PeripheralTimedOut);
            }
            iter += 1;
            cortex_m::asm::delay(1000);
        }

        // Sanity check the peripheral came up correctly by reading the release version register.
        let crel = can.mcan(0).crel().read();
        if crel.0 == 0x00 {
            return Err(InitializationError::PeripheralTimedOut);
        }
        debug!("MCAN version: {}.{}.{} - {}{}{}", crel.rel(), crel.step(), crel.substep(), crel.year(), crel.mon(), crel.day());

        Ok(())
    }

    /// Helper function to access write-protected peripheral registers.
    /// Note that while these registers are writable, the peripheral is disconnected from the bus,
    /// and won't send or receive frames, acks, or errors.
    /// If the closure returns with an error, the peripheral will _not_ be placed back into "Normal" mode
    /// as it may be in an inconsistent state.
    fn guarded_config<T: Instance> (_peri: &Peri<'d, T>, f: impl FnOnce(&Regs) -> Result<(), InitializationError> ) -> Result<(), InitializationError> {
        let can = T::info().regs;

        // Put the peripheral into "initialization" mode as a first step to allow register changes.
        can.mcan(0).cccr().modify(|w| {
            w.set_init(true);
        });

        // This goes through clock domain crossings, so make sure it's actually in initialization mode.
        let mut iter = 0;
        while !can.mcan(0).cccr().read().init() {
            if iter > 10000 {
                return Err(InitializationError::PeripheralTimedOut);
            }
            iter += 1;
            cortex_m::asm::delay(10);
        }

        // Now we can set the configuration change enabled bit to unlock registers.
        can.mcan(0).cccr().modify(|w| {
            w.set_cce(true);
        });

        if let Err(e) = f(&can) {
            Err(e)
        } else {
            // re-enter normal state - disable changes, then disable init mode.
            can.mcan(0).cccr().modify(|w| {
                w.set_cce(false);
            });
            can.mcan(0).cccr().modify(|w| {
                w.set_init(false);
            });

            iter = 0;
            while can.mcan(0).cccr().read().init() {
                if iter > 10000 {
                    return Err(InitializationError::PeripheralTimedOut);
                }
                iter += 1;
                cortex_m::asm::delay(10);
            }

            Ok(())
        }

        

    }

    fn new_inner<T: Instance> (
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config
    ) -> Result<Self, InitializationError> {

        // Note: use new_pin! when in tree.
        let rx_inner = new_pin!(rx, PfType::input(crate::gpio::Pull::None, false));
        let tx_inner = new_pin!(tx, PfType::output(crate::gpio::Pull::None, false));

        // Reset and power on the CAN peripheral. Note this _is_ a falliable operation.
        Self::reset_poweron(&peri, &config)?;
    
        Self::guarded_config(&peri, |can| -> Result<(), InitializationError> {
            can.mcan(0).cccr().modify(|w| {
                w.set_fdoe(false); // classic CAN, no FD.
            });

            // Nominal bit-timing (no CAN-FD support yet, so we do not configure data bit timing)
            can.mcan(0).nbtp().write(|w| {
                // Docs state that the hardware will actually use 1 greater than the value set in the register, so subtract one here.
                w.set_nbrp(config.timing.brp - 1);

                w.set_ntseg1(config.timing.ntseg1 as u8 - 1); 
                w.set_ntseg2(config.timing.ntseg2 - 1);

                w.set_nsjw(config.timing.sjw - 1);
            });

            // Global filter configuration
            // Detailed filtering configuration will be follow-up work.
            // For now, we will accept all frames into RX FIFO 0.
            can.mcan(0).gfc().write(|w| {
                w.set_anfs(0b00); // Accept non-matching 11-bit id frames into RX FIFO 0.
                if config.accept_remote_frames {
                    w.set_anfe(0b00); // Accept extended frames.
                } else {
                    w.set_anfe(0b10); // Reject extended frames.
                }
                w.set_rrfs(!config.accept_remote_frames ); // Reject remote frames with 11-bit IDs?
                w.set_rrfe(!(config.accept_remote_frames && config.accept_extended_ids)); // reject remote frames with extended IDs?
            });

            // Message RAM sizing configuration goes here :)

            Ok(())
        })?;

        Ok(Can {
            info: T::info(),
            state: T::state(),
            rx: rx_inner,
            tx: tx_inner,
            _phantom: PhantomData
        })
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