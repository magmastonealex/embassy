#![macro_use]

use core::marker::PhantomData;

use embassy_hal_internal::PeripheralType;

use crate::Peri;
use crate::can::frame::MCanFrame;
use crate::can::msgram::{McanMessageRAM, MessageRAMAccess};
use crate::gpio::{AnyPin, PfType};
use crate::mode::{Blocking, Mode};
use crate::pac::canfd::{Canfd as Regs, vals as CanVals};
use crate::pac::{self};

pub(crate) mod msgram;

pub mod frame;

pub(crate) struct Info {
    // metadata/details about the specific instance of the peripheral in use.
    pub(crate) regs: Regs, // the registers for this specific instance
    pub(crate) mem: MessageRAMAccess,
}

// prevent external callers from creating instances of this.
pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

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
    pub brp: u16,
    /// Bitrate prescaler, valid values 1-512.
    pub sjw: u8,
    /// Sync Jump Width - valid values 1-128, though must also be <= ntseg2.
    pub ntseg1: u16, // Segment 1 time. Valid values are 2-256
    pub ntseg2: u8, // Segment 2 time. Valid values are 2-128.
}

impl CanTimings {
    pub const fn from_values(brp: u16, sjw: u8, ntseg1: u16, ntseg2: u8) -> Option<CanTimings> {
        Some(CanTimings {
            brp,
            sjw,
            ntseg1,
            ntseg2,
        })
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// CAN Configuration
pub struct Config {
    /// Input clock divider
    pub clock_div: ClockDiv,

    /// CAN timings to use for standard CAN. (CAN-FD support to come later.)
    pub timing: CanTimings,

    pub accept_remote_frames: bool,

    pub accept_extended_ids: bool,
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

    /// A non-blocking method was called and the request would require blocking to complete.
    WouldBlock,
}

impl embedded_can::Error for BusError {
    fn kind(&self) -> embedded_can::ErrorKind {
        match self {
            Self::Stuff => embedded_can::ErrorKind::Stuff,
            Self::Form => embedded_can::ErrorKind::Form,
            Self::Acknowledge => embedded_can::ErrorKind::Acknowledge,
            Self::BitRecessive => embedded_can::ErrorKind::Bit,
            Self::BitDominant => embedded_can::ErrorKind::Bit,
            Self::Crc => embedded_can::ErrorKind::Crc,
            _ => embedded_can::ErrorKind::Other,
        }
    }
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error status of the CAN peripheral
pub enum RecoveryFailure {
    /// The peripheral was not in bus-off state.
    WasNotBusOff,
}

pub struct Can<'d, M: Mode> {
    info: &'static Info,
    _rx: Option<Peri<'d, AnyPin>>,
    _tx: Option<Peri<'d, AnyPin>>,
    _phantom: PhantomData<M>,
}

impl<'d> Can<'d, Blocking> {
    /// The "Blocking" CAN instance actually implements both the blocking and non-blocking embedded-can traits.
    /// the nb traits either work or do not and aren't actually async.
    /// The Async version of this driver will offer options to properly handle asynchronous work.
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
    ) -> Result<Self, InitializationError> {
        Self::new_inner(peri, rx, tx, config)
    }

    pub fn get_frame(&mut self) -> Option<MCanFrame> {
        let cur_status = self.info.regs.mcan(0).rxf0s().read();
        if cur_status.f0gi() == cur_status.f0pi() && !cur_status.f0f() {
            return None;
        }
        let read_index = cur_status.f0gi();

        let element = self
            .info
            .mem
            .get_rx_fifo_element(read_index as usize)
            .expect("invalid read index - bad peripheral config?");

        // mark the element as acknowledged.
        self.info.regs.mcan(0).rxf0a().write(|w| {
            w.set_f0ai(read_index);
        });

        Some(element.into())
    }

    /// Retrieve a frame from the peripheral in a blocking fashion.
    /// Will return a BusError if the peripheral enters bus-off state.
    pub fn get_frame_blocking(&mut self) -> Result<MCanFrame, BusError> {
        loop {
            if let Some(frame) = self.get_frame() {
                return Ok(frame);
            }

            // BusOff will never recover by itself - bail so the caller can deal with this.
            match self.status() {
                Some(BusError::BusOff) => return Err(BusError::BusOff),
                _ => {}
            }

            cortex_m::asm::delay(100);
        }
    }

    pub fn enqueue_frame(&mut self, frame: &MCanFrame) -> Option<()> {
        let cur_status = self.info.regs.mcan(0).txfqs().read();
        if cur_status.tfqf() {
            // TX queue is full already.
            // TODO: Consider trying to replace a lower-priority item in the future.
            return None;
        }

        let txbuf = frame.to_tx_buffer(None); // note we do not support TX events yet.
        let write_index = cur_status.tfqp();
        self.info
            .mem
            .set_tx_element(write_index as usize, txbuf)
            .expect("invalid write index - bad periph config?");

        // tell the peripheral we've written a new entry into the TX FIFO.
        self.info.regs.mcan(0).txbar().write(|w| {
            w.0 = 1 << write_index;
        });

        Some(())
    }

    pub fn enqueue_frame_blocking(&mut self, frame: &MCanFrame) -> Result<(), BusError> {
        loop {
            if let Some(_) = self.enqueue_frame(frame) {
                return Ok(());
            }

            // BusOff will never recover by itself - bail so the caller can deal with this.
            match self.status() {
                Some(BusError::BusOff) => return Err(BusError::BusOff),
                _ => {}
            }

            cortex_m::asm::delay(100);
        }
    }
}

// Implementations for the embedded_can traits.
// I am really not sure that these are good abstractions for real applications.
// They do not support "confirmable" mesage sending (i.e the docs specifically state that
// transmit() only enqueues frames, there is no feedback mechanisim to confirm if/when a frame
// was actually sent)
// They also do not support monitoring the peripheral's status or recovering from bus-off.
// I will implement them regardless to play nice with the overall ecosystem,
// but similar to embassy-stm32, I'm going to offer a HAL-specific API which provides
// more useful functionality, and add an async variant at some point.

impl<'d> embedded_can::blocking::Can for Can<'d, Blocking> {
    type Error = BusError;
    type Frame = MCanFrame;
    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        self.get_frame_blocking()
    }
    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        self.enqueue_frame_blocking(frame)
    }
}

impl<'d> embedded_can::nb::Can for Can<'d, Blocking> {
    type Error = BusError;
    type Frame = MCanFrame;
    fn receive(&mut self) -> embedded_hal_nb::nb::Result<Self::Frame, Self::Error> {
        match self.get_frame() {
            Some(frame) => Ok(frame),
            None => Err(embedded_hal_nb::nb::Error::WouldBlock),
        }
    }

    fn transmit(&mut self, frame: &Self::Frame) -> embedded_hal_nb::nb::Result<Option<Self::Frame>, Self::Error> {
        if let None = self.enqueue_frame(frame) {
            // TODO: The trait suggests re-ordering the TX queue at this point to replace a lower-priority frame.
            // I am not convinced that is a sound operation in this case as we don't know which frame MCAN is currently
            // transmitting.
            return Err(embedded_hal_nb::nb::Error::WouldBlock);
        }

        return Ok(None);
    }
}

impl<'d, M: Mode> Can<'d, M> {
    fn reset_poweron<T: Instance>(_peri: &Peri<'d, T>, config: &Config) -> Result<(), InitializationError> {
        // See e2e: https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1605241/mspm0g3107-mcan-peripheral-does-not-complete-initialization-after-power-on-reset
        // The initialization instructions in the TRM are not accurate at this time.
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
                ClockDiv::DivBy4 => CanVals::Ratio::DIV_BY_4_,
            });
        });

        pac::SYSCTL.genclkcfg().modify(|w| {
            w.set_canclksrc(pac::sysctl::vals::Canclksrc::HFCLK);
        });

        // Wait for async reset to be complete.
        let mut iter = 0;
        while can
            .ti_wrapper(0)
            .processors(0)
            .subsys_regs(0)
            .subsys_stat()
            .read()
            .reset()
        {
            if iter > 1000 {
                return Err(InitializationError::PeripheralTimedOut);
            }
            iter += 1;
            cortex_m::asm::delay(1000);
        }

        // Wait for "memory initialization" to be complete. I think this is zeroing the internal message RAM.
        iter = 0;
        while can
            .ti_wrapper(0)
            .processors(0)
            .subsys_regs(0)
            .subsys_stat()
            .read()
            .mem_init_done()
        {
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
        debug!(
            "MCAN version: {}.{}.{} - {}{}{}",
            crel.rel(),
            crel.step(),
            crel.substep(),
            crel.year(),
            crel.mon(),
            crel.day()
        );

        Ok(())
    }

    /// Helper function to access write-protected peripheral registers.
    /// Note that while these registers are writable, the peripheral is disconnected from the bus,
    /// and won't send or receive frames, acks, or errors.
    /// If the closure returns with an error, the peripheral will _not_ be placed back into "Normal" mode
    /// as it may be in an inconsistent state.
    fn guarded_config<T: Instance>(
        _peri: &Peri<'d, T>,
        f: impl FnOnce(&Regs) -> Result<(), InitializationError>,
    ) -> Result<(), InitializationError> {
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

    fn new_inner<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
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
                w.set_rrfs(!config.accept_remote_frames); // Reject remote frames with 11-bit IDs?
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
            _rx: rx_inner,
            _tx: tx_inner,
            _phantom: PhantomData,
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
            return Can::<M>::reg_to_error(status.lec());
        }
    }

    /// Attempt to recover from a bus-off condition.
    pub fn recover(&mut self) -> Result<(), RecoveryFailure> {
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

pub trait RxPin<T: Instance>: crate::gpio::Pin {
    fn pf_num(&self) -> u8;
}

pub trait TxPin<T: Instance>: crate::gpio::Pin {
    fn pf_num(&self) -> u8;
}

macro_rules! impl_can_rx_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::can::RxPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}
macro_rules! impl_can_tx_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::can::TxPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

macro_rules! impl_can_instance {
    ($instance: ident) => {
        impl crate::can::SealedInstance for crate::peripherals::$instance {
            fn info() -> &'static crate::can::Info {
                use crate::can::Info;

                const INFO: Info = Info {
                    regs: crate::pac::$instance,
                    // mild voodoo - message RAM lives at the beginning of the address space of the MCAN peripheral, in a gap in the
                    // SVD between the base address and first documented register.
                    // Re-use the same register base address.
                    mem: unsafe { crate::can::msgram::MessageRAMAccess::from_ptr(crate::pac::$instance.as_ptr()) },
                };

                &INFO
            }
        }

        impl crate::can::Instance for crate::peripherals::$instance {
            // TODO: This will be expanded when async support is added to include the specific interrupt used for this peripheral.
        }
    };
}
