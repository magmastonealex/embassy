use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_hal_internal::PeripheralType;

use crate::Peri;
use crate::can::frame::MCanFrame;
use crate::can::msgram::{MessageRAMAccess, McanMessageRAM};
use crate::gpio::{AnyPin, PfType};
use crate::interrupt::Interrupt;
use crate::mode::{Blocking, Mode};
use crate::pac::canfd::{Canfd as Regs, vals as CanVals};
use crate::pac::{self};

use embassy_sync::waitqueue::AtomicWaker;

mod msgram;

pub mod frame;

// Major TODOs still:
// 1. Concurrency here _feels_ sketchy and needs a good review. Remember the PAC and msgram purposefully disable borrow checker protections against simultaneous access.
// 2. Rip out marker # and TX event details for now - not needed right now.
// 3. Actually implement the trait instead of using the half-implementations we have now :)
//    -> Note the trait actually has no provision for confirming frames were actually sent, so the TX event FIFO is not required.
//    -> The trait also does not implement true async! We may want to provide those anyways?
// 4. do _something_ to handle bus-off and other protocol errors. It's not clear to me yet what the right interface is for that. Probably involves a config to decide how to handle bus-off / error-passive?
//    -> Config has been added, and functions for manually polling and recoverying have been added.
//    -> can simulate by setting invalid bitrate, maybe?
// 5. Write a little test jig to ping things back and forth in various situations to prove things are working
// X. Add tests to msgram and frame to confirm correct construction (done?)
// 5. Docs!
// 6. Bit rate calculations & accompanying tests.
// 7. Pin / peripheral macros.
// At least 

pub(crate) struct Info { // metadata/details about the specific instance of the peripheral in use.
    pub(crate) regs: Regs, // the registers for this specific instance
    pub(crate) interrupt: Interrupt, // which interrupt applies to this peripheral
    mem: MessageRAMAccess
}

pub(crate) struct State {
    // waker for when interesting things happen, I guess.
    pub(crate) waker: AtomicWaker,
    current_marker: AtomicU8
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
            // mild voodoo - message RAM lives at the beginning of the address space of the MCAN peripheral, in a gap in the
            // SVD between the base address and first documented register.
            // Re-use the same register base address.
            mem: unsafe { MessageRAMAccess::from_ptr( crate::pac::CANFD0.as_ptr() )}
        };

        &INFO
    }

    fn state() -> &'static State {
        static STATE: State = State {
            waker: AtomicWaker::new(),
            current_marker: AtomicU8::new(0)
        };

        &STATE
    }
}

impl Instance for crate::peripherals::CANFD0 {
    type Interrupt = crate::interrupt::typelevel::CANFD0; // I'm still unclear why this type is needed _here_ too.
}


/// Functional clock divider - consider this as an additional few bits on top of the bitrate prescaler if needed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockDiv {
    //. Do not divide clock source.
    DivBy1,
    /// Divide clock source by 2.
    DivBy2,
    /// Divide clock source by 4.
    DivBy4,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Structure to encode CAN timing parameter information.
/// Note that the hardware adds '1' to each of the values placed in the registers of the peripheral.
/// This crate handles this for you, so the values in this struct should be the actual values you wish to use.
/// Strongly suggest using the from_bitrate function to determine values here.
pub struct CanTimings {
    pub brp: u16, /// Bitrate prescaler, valid values 1-512. 
    pub sjw: u8, /// Sync Jump Width - valid values 1-128, though must also be <= ntseg2.
    pub ntseg1: u16, // Segment 1 time. Valid values are 2-256
    pub ntseg2: u8, // Segment 2 time. Valid values are 2-128.
}

impl CanTimings {
    pub const fn from_values(brp: u16, sjw: u8, ntseg1: u16, ntseg2: u8) -> Option<CanTimings> {
        Some(CanTimings { brp, sjw, ntseg1, ntseg2 })
    }
}

/// Error handling behaviour - 
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusOffHandling {
    /// Auto Re-Init - when a bus-off condition is encountered, the peripheral will be restarted immediately.
    AutoReInit,

    // Manual Re-Init - The peripheral will be left in the bus-off state indefinitely.
    // It is up to the consumer to regularly poll for status and call recover().
    ManualReInit
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

    pub bus_off_handling: BusOffHandling,

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


#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error status of the CAN peripheral
pub enum BusError {
    /// The peripheral has encountered enough receive errors that it has entered a passive state and will no longer send error frames.
    ErrorPassive,

    /// The transmit or receive error counters have reached a level that suggests something is wrong with the bus, but messages are still
    /// being sent and received.
    ErrorWarning,

    /// The peripheral has disconnected from the bus as too many transmit errors were encountered.
    BusOff,

    /// More than 5 equal bits in a sequence have occurred in a part of a received message where this is not allowed.
    Stuff,
    ///A fixed format part of a received frame has the wrong format.
    Form,
    /// The message transmitted by the peripheral was not acknowledged by another node.
    Acknowledge,
    ///During the transmission of a message (with the exception of the arbitration field), the device wanted to send a recessive level (bit of logical value '1'), but the monitored bus value was dominant.
    BitRecessive,
    /// During the transmission of a message (or acknowledge bit, or active error flag, or overload flag), the device wanted to send a dominant level (data or identifier bit logical value '0'), but the monitored bus value was recessive.
    /// This is also set during bus-off recovery for each sequence of 11 recessive bits and can be used to monitor recovery progress.
    BitDominant,
    ///The CRC check sum of a received message was incorrect. The CRC of an incoming message does not match with the CRC calculated from the received data.
    Crc,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error status of the CAN peripheral
pub enum RecoveryFailure {
    /// The peripheral doesn't need to be recovered manually.
    WasAutomatic,
    /// The peripheral was not in bus-off state.
    WasNotBusOff,
}

pub struct Can<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    config: Config,
    _rx: Option<Peri<'d, AnyPin>>,
    _tx: Option<Peri<'d, AnyPin>>,
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

    // this is fundamentally mutable - after all, we're changing something within the peripheral!
    // As such, we shouldn
    pub fn get_frame(&mut self) -> MCanFrame {
        let fifo_status = self.info.regs.mcan(0).rxf0s();

        // wait until an element becomes available.
        let read_index = loop {
            let cur_status = fifo_status.read();
            if cur_status.f0gi() != cur_status.f0pi() || cur_status.f0f() {
                // there is at least one item to read!
                break cur_status.f0gi();
            }
            cortex_m::asm::delay(10);
        };

        // actually read the element.
        let element = self.info.mem.get_rx_fifo_element(read_index as usize).expect("invalid read index - bad peripheral config?");

        // mark the element as acknowledged.
        self.info.regs.mcan(0).rxf0a().write(|w| {
            w.set_f0ai(read_index);
        });

        element.into()
    }

    pub fn send_frame(&mut self, frame: MCanFrame) {
        let fifo_status = self.info.regs.mcan(0).txfqs();

        // TODO: how does concurrency control work here? Confirm two tasks can't execute this at the same time.
        let write_index = loop {
            let cur_status = fifo_status.read();
            // If TX fifo put index == 
            if !cur_status.tfqf() {
                // TX queue is full already.
                break cur_status.tfqp();
            }
            cortex_m::asm::delay(10);
            continue;
        };

        // convert our frame.
        // Note: this is unsafe and should be replaced with a mutex or similar to track frame #s. I don't care at the moment
        // and just want to get this to work.
        //let new_marker = self.state.current_marker.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        let new_marker = self.state.current_marker.load(Ordering::Relaxed);
        self.state.current_marker.store(new_marker.wrapping_add(1), Ordering::Relaxed);
        
        let txbuf = frame.into_tx_buffer(Some(new_marker));

        self.info.mem.set_tx_element(write_index as usize, txbuf).expect("invalid write index - bad periph config?");

        // tell the peripheral we've written a new entry into the TX FIFO.
        self.info.regs.mcan(0).txbar().write(|w| { 
            w.0 = 1 << write_index;
        });

        // This is sketchy and will stop working in any async scenario, but for now, spin on the TX Event FIFO until we have some evidence our frame was sent.
        // I think this also will block forever if we get a bus-off or other failure. We need another way to track frames which we've enqueued but never got sent off.

        let fifo_status = self.info.regs.mcan(0).txefs();
        let read_index = loop {
            let cur_status = fifo_status.read();
            if cur_status.efgi() != cur_status.efpi() || cur_status.eff() {
                // there is at least one item to read!
                break cur_status.efgi();
            }
            cortex_m::asm::delay(10);
        };

        // actually read the element.
        let element = self.info.mem.get_tx_event(read_index as usize).expect("invalid read index - bad peripheral config?");

        // mark the element as acknowledged.
        self.info.regs.mcan(0).txefa().write(|w| {
            w.set_efai(read_index);
        });

        assert!(element.event.mm() == new_marker);
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
            // Filtering configuration will be follow-up work.
            // For now, we will accept all frames into RX FIFO 0.
            can.mcan(0).gfc().write(|w| {
                w.set_anfs(0b00); // Accept non-matching 11-bit id frames into RX FIFO 0.
                if config.accept_extended_ids {
                    w.set_anfe(0b00); // Accept extended frames.
                } else {
                    w.set_anfe(0b10); // Reject extended frames.
                }
                w.set_rrfs(!config.accept_remote_frames ); // Reject remote frames with 11-bit IDs?
                w.set_rrfe(!(config.accept_remote_frames && config.accept_extended_ids)); // reject remote frames with extended IDs?
            });

            // Sizing for message RAM.

            // Standard filters
            can.mcan(0).sidfc().write(|w| {
                w.set_lss(McanMessageRAM::SIZES.filters as u8);
                w.set_flssa(McanMessageRAM::OFFSETS.filters as u16);
            });

            // 29 bit filters
            can.mcan(0).xidfc().write(|w| {
                w.set_lse(McanMessageRAM::SIZES.extended_filters as u8);
                w.set_flesa(McanMessageRAM::OFFSETS.extended_filters as u16);
            });

            // RX FIFO 0
            can.mcan(0).rxf0c().write(|w| {
                w.set_f0om(false); // blocking mode - don't overwrite messages.
                w.set_f0wm(0); // no watermark.
                w.set_f0s(McanMessageRAM::SIZES.rxfifo0 as u8);
                w.set_f0sa(McanMessageRAM::OFFSETS.rxfifo0 as u16);
            });

            // RX FIFO 1
            can.mcan(0).rxf1c().write(|w| {
                w.set_f1om(false); // blocking mode - don't overwrite.
                w.set_f1wm(0); // no watermark.
                w.set_f1s(McanMessageRAM::SIZES.rxfifo1 as u8);
                w.set_f1sa(McanMessageRAM::OFFSETS.rxfifo1 as u16);
            });

            // RX Buffers
            can.mcan(0).rxbc().write(|w| {
                w.set_rbsa(McanMessageRAM::OFFSETS.rxbuffers as u16);
            });
            // Sizes for various RX elements.
            can.mcan(0).rxesc().write(|w| {
                w.set_rbds(0); // 8 byte max data in RX buffers.
                w.set_f1ds(0); // 8 byte max data in RX fifo 1.
                w.set_f0ds(0); // 8 byte max data in RX fifo 0
            });

            // TX Event FIFO
            can.mcan(0).txefc().write(|w| {
                w.set_efsa(McanMessageRAM::OFFSETS.txevents as u16);
                w.set_efs(McanMessageRAM::SIZES.txevents as u8);
                w.set_efwm(0); // no watermark.
            });
            // TX Buffers
            can.mcan(0).txbc().write(|w| {
                w.set_tfqm(false); // FIFO operation mode, not priority queue.
                w.set_ndtb(0); // No dedicated transmit buffers (not supported [yet?])
                w.set_tfqs(McanMessageRAM::SIZES.txfifo as u8);
                w.set_tbsa(McanMessageRAM::OFFSETS.txfifo as u16);
            });
            can.mcan(0).txesc().write(|w| {
                w.set_tbds(0); // max 8 byte data payloads in TX elements.
            });

            Ok(())
        })?;

        Ok(Can {
            info: T::info(),
            state: T::state(),
            config,
            _rx: rx_inner,
            _tx: tx_inner,
            _phantom: PhantomData
        })
    }

    pub fn has_frame(&self) -> bool {
        let cur_status = self.info.regs.mcan(0).rxf0s().read();
        cur_status.f0gi() != cur_status.f0pi() || cur_status.f0f()
    }


    fn reg_to_error(value: u8) -> Option<BusError> {
        match value {
            1 => Some(BusError::Stuff),
            2 => Some(BusError::Form),
            3 => Some(BusError::Acknowledge),
            4 => Some(BusError::BitRecessive),
            5 => Some(BusError::BitDominant),
            6 => Some(BusError::Crc),
            _ => None,
        }
    }

    pub fn status(&self) -> Option<BusError> {
        let status = self.info.regs.mcan(0).psr().read();
        if status.bo() {
            return Some(BusError::BusOff);
        } else if status.ep() {
            return Some(BusError::ErrorPassive);
        } else if status.ew() {
            return Some(BusError::ErrorWarning);
        } else {
            return Can::<M>::reg_to_error(status.lec())
        }
    }

    /// Attempt to recover from a bus-off condition.
    pub fn recover(&mut self) -> Result<(), RecoveryFailure> {
        // Confirm we are in manual recovery mode (otherwise, ISR will handle this.)
        if self.config.bus_off_handling != BusOffHandling::ManualReInit {
            return Err(RecoveryFailure::WasAutomatic);
        }
        let mcan = self.info.regs.mcan(0);
        // Confirm we're in bus-off state.
        if !mcan.psr().read().bo() {
            return Err(RecoveryFailure::WasNotBusOff);
        }
        // Set CCR.INIT = 0 to start recovery.
        mcan.cccr().modify(|w| {
            w.set_init(false);
        });

        Ok(())
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