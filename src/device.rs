
use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive)]
#[repr(u8)]
pub enum Register {
    TrxStatus       = 0x01,
    TrxCmd          = 0x02,
    TrxCtrl1        = 0x04,
    IrqMask         = 0x0E,
    PartNum         = 0x1C,
    VersionNum      = 0x1D,
    ManufacturerId  = 0x1E
}

bitflags::bitflags!{
    /// SPI command flags
    pub struct CommandFlags: u8 {
        /// Register command
        const REG =  0b1000_0000;
        /// Frame buffer command
        const BUFF = 0b0010_0000;
        /// SRAM command
        const SRAM = 0b0000_0000;

        /// Read command
        const RD =  0b0000_0000;
        /// Write command
        const WR =  0b0100_0000;

        /// Register read
        const REG_RD = Self::REG.bits | Self::RD.bits;
        /// Register write
        const REG_WR = Self::REG.bits | Self::WR.bits;

        /// Buffer read
        const BUFF_RD = Self::BUFF.bits | Self::RD.bits;
        /// Buffer write
        const BUFF_WR = Self::BUFF.bits | Self::WR.bits;

        /// SRAM read
        const SRAM_RD = Self::SRAM.bits | Self::RD.bits;
        /// SRAM write
        const SRAM_WR = Self::SRAM.bits | Self::WR.bits;
    }
}

bitflags::bitflags!{
    /// IRQ flags
    pub struct Irqs: u8 {
        /// Supply voltage below programmed threshold
        const BATT_LOW      = 0b1000_0000;
        /// Frame buffer access violation
        const TRX_UR        = 0b0100_0000;
        /// Address match
        const AMI           = 0b0010_0000;
        /// Awake end or CCA or ED measurement complete
        const CCA_ED_DONE   = 0b0001_0000;
        /// Completion of frame rx/tx
        const TRX_END       = 0b0000_1000;
        /// Start of PDSU reception
        const RX_START      = 0b0000_0100;
        /// PLL unlock
        const PLL_UNLOCK    = 0b0000_0010;
        /// PLL lock
        const PLL_LOCK      = 0b0000_0001;
    }
}

#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive)]
#[repr(u8)]
pub enum TrxStatus {
    POn = 0x00,
    BusyRx = 0x01,
    BusyTx = 0x02,
    RxOn = 0x06,
    TrxOff = 0x08,
    PllOn = 0x09,
    Sleep = 0x0F,
    PrepDeepSleep = 0x10,
    BusyRxAack = 0x11,
    BusyTxAret = 0x12,
    RxAackOn = 0x16,
    TxAretOn = 0x19,
    StateTransition = 0x1F,
}

#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive)]
#[repr(u8)]
pub enum TrxCmd {
    Nop = 0x00,
    TxStart = 0x02,
    ForceTrxOff = 0x03,
    ForcePllOn = 0x04,
    RxOn = 0x06,
    TrxOff = 0x08,
    PllOn = 0x09,
    PrepDeepSleep = 0x10,
    RxAackOn = 0x16,
    TxAretOn = 0x19,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_command_flags() {
        let commands = [
            (CommandFlags::REG_RD,  0b1000_0000),
            (CommandFlags::REG_WR,  0b1100_0000),
            (CommandFlags::BUFF_RD, 0b0010_0000),
            (CommandFlags::BUFF_WR, 0b0110_0000),
            (CommandFlags::SRAM_RD, 0b0000_0000),
            (CommandFlags::SRAM_WR, 0b0100_0000),
        ];

        for (c, r) in commands {
            assert_eq!(c.bits(), r, "Invalid command flags: {:?}", c);
        }
    }


}