use embedded_can::{ExtendedId, Frame, Id, StandardId};

use crate::can::msgram::{MsgHeader, RxBufferElement, TxBufferElement, TxHeader};


pub struct MCanFrame {
    id: Id,
    dlc: usize,
    is_remote: bool,
    data: [u8; 8] // TODO: CAN-FD will require larger data. This also affects peripheral setup and msgram configuration.
}

#[cfg(feature = "defmt")]
impl defmt::Format for MCanFrame {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        match self.id() {
            embedded_can::Id::Standard(id) => {
                defmt::write!(fmt, "CAN frame: Standard ID={:x} len={}, data: {=[u8]:x}", id.as_raw(), self.dlc, &self.data[0..self.dlc])
            }
            embedded_can::Id::Extended(id) => {
                defmt::write!(fmt, "CAN frame: Extended ID={:x} len={}, data: {=[u8]:x}", id.as_raw(), self.dlc, &self.data[0..self.dlc])
            }
        }
    }
}


impl Frame for MCanFrame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        None
    }
    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        None
    }
    fn is_extended(&self) -> bool {
        matches!(self.id(), Id::Extended(_))
    }
    
    fn id(&self) -> embedded_can::Id {
        self.id
    }

    fn dlc(&self) -> usize {
        self.dlc
    }
    
    fn data(&self) -> &[u8] {
        &self.data
    }

    fn is_remote_frame(&self) -> bool {
        self.is_remote
    }
}


impl Into<MCanFrame> for RxBufferElement {
    fn into(self) -> MCanFrame {
        let id = if self.hdr.xtd() {
            // safety - we only read 29 bits of ID, there's no way this can be out of range.
            Id::Extended(unsafe{ExtendedId::new_unchecked(self.hdr.id())})
        } else {
            let id_shifted = (self.hdr.id() >> 18) as u16;
            // Safety - we only read 29 bits of ID, and we just shifted away 18 of them,
            // leaving only 11 possible non-zero bits.
            Id::Standard(unsafe{StandardId::new_unchecked(id_shifted)})
        };

        // should always be true given how the peripheral is configured, but you never know.
        assert!(self.rxhdr.dlc() <= 8);

        MCanFrame {
            id: id,
            dlc: self.rxhdr.dlc() as usize,
            is_remote: self.hdr.rtr(),
            data: self.data
        }
    }
}

impl MCanFrame {
    pub fn set_id(&mut self, id: impl Into<embedded_can::Id>) {
        self.id = id.into();
    }

    /// Convert this MCanFrame into a TXBufferElement ready for transmission.
    /// If msgid is None, then the EFC field will not be set and no confirmation will
    /// be sent to the event FIFO.
    pub(in crate::can) fn into_tx_buffer(self, marker: Option<u8>) -> TxBufferElement {
        let mut glblheader = MsgHeader(0);
        
        glblheader.set_rtr(self.is_remote);
        match self.id {
            Id::Extended(extid) => {
                glblheader.set_xtd(true);
                glblheader.set_id(extid.as_raw());
            },
            Id::Standard(stdid) => {
                glblheader.set_xtd(false);
                glblheader.set_id((stdid.as_raw() as u32) << 18);
            }
        }

        let mut txhdr = TxHeader(0);
        txhdr.set_dlc(self.dlc as u8); // cast safety - checked on all ingest to ensure it's <= 8.

        if let Some(mm) = marker {
            txhdr.set_mm(mm);
            txhdr.set_efc(true);
        }

        TxBufferElement {
            hdr: glblheader,
            txhdr,
            data: self.data
        }
    }
}
