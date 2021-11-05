use core::fmt::Debug;
use log::{debug, warn};
use std::marker::PhantomData;

use radio::{Registers as _, State as _};

pub mod base;
pub mod device;
pub mod prelude;

use base::Base;
use device::*;

/// AT86RF23x driver object
pub struct At86Rf23x<B, SpiErr: Debug, PinErr: Debug, DelayErr: Debug> {
    hal: B,

    _err: PhantomData<Error<SpiErr, PinErr, DelayErr>>,
}

/// Error type for AT86RF23x
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "thiserror", derive(thiserror::Error))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<SpiErr: Debug, PinErr: Debug, DelayErr: Debug> {
    #[error("SPI error: {0}")]
    /// Communications (SPI or UART) error
    Spi(SpiErr),

    #[error("Pin error: {0}")]
    /// Pin control error
    Pin(PinErr),

    #[error("Delay error: {0}")]
    /// Delay error
    Delay(DelayErr),

    #[error("Unsupported state command")]
    /// Unsupported state command
    Unsupported,

    #[error("No response from device")]
    /// No response from device
    NoResponse,
}

#[derive(Copy, Clone, PartialEq, Debug, strum_macros::Display)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum State {
    Init,
    Idle,
    PllOn,
    BusyTx,
    RxOn,
    BusyRx,
    Sleep,
    Busy,
}

/// Part information
#[derive(Debug, Clone, PartialEq)]
pub struct Info {
    pub part: u8,
    pub ver: u8,
    pub mfr: u16,
}

impl<B, SpiErr, PinErr, DelayErr> At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    pub fn new(hal: B) -> Result<Self, Error<SpiErr, PinErr, DelayErr>> {
        let mut s = Self {
            hal,
            _err: PhantomData,
        };

        debug!("Connecting to device");

        // Deassert sleep
        s.hal.sleep(false)?;

        // Perform reset
        s.hal.reset()?;

        // TODO: enable AWAKE_END IRQ to detect move to TRX_OFF

        // Write TRX_OFF to enter ready state
        s.reg_write(Register::TrxCmd, TrxCmd::TrxOff as u8)?;

        // Wait for TRX_OFF

        // Read info
        let i = s.info()?;
        if i.part == 0 && i.ver == 0 && i.mfr == 0 {
            warn!("Init failed, communication error");
            return Err(Error::NoResponse);
        }

        debug!("Device info: {:02x?}", i);

        Ok(s)
    }

    pub fn info(&mut self) -> Result<Info, Error<SpiErr, PinErr, DelayErr>> {
        let i = Info {
            part: self.reg_read(Register::PartNum)?,
            ver: self.reg_read(Register::VersionNum)?,
            mfr: u16::from_le_bytes([
                self.reg_read(Register::ManId0)?,
                self.reg_read(Register::ManId1)?,
            ]),
        };
        Ok(i)
    }

    /// Write to the device buffer
    fn buff_write(&mut self, data: &[u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_write(CommandFlags::BUFF_WR.bits(), data)
    }

    /// Read from the device buffer
    fn buff_read(&mut self, data: &mut [u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_read(CommandFlags::BUFF_RD.bits(), data)
    }

    /// Write to the device SRAM
    fn sram_write(&mut self, data: &[u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_write(CommandFlags::SRAM_WR.bits(), data)
    }

    /// Read from the device SRAM
    fn sram_read(&mut self, data: &mut [u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_read(CommandFlags::SRAM_RD.bits(), data)
    }

    /// Read a device register
    fn reg_read2<R: Reg>(&mut self) -> Result<R, Error<SpiErr, PinErr, DelayErr>> {
        let mut d = [0u8];
        self.hal
            .spi_read(R::ADDRESS as u8 | CommandFlags::REG_RD.bits(), &mut d)?;
        Ok(R::from(d[0]))
    }

    /// Write a device register
    fn reg_write2<R: Reg>(&mut self, v: R) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_write(R::ADDRESS as u8 | CommandFlags::REG_WR.bits(), &[v.into()])
    }
}

impl<B, SpiErr, PinErr, DelayErr> radio::Registers<Register>
    for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    /// Write a value to a device register
    fn reg_write(&mut self, reg: Register, v: u8) -> Result<(), Self::Error> {
        self.hal
            .spi_write(reg as u8 | CommandFlags::REG_WR.bits(), &[v])
    }

    /// Read a value from a device register
    fn reg_read(&mut self, reg: Register) -> Result<u8, Self::Error> {
        let mut d = [0u8];
        self.hal
            .spi_read(reg as u8 | CommandFlags::REG_RD.bits(), &mut d)?;
        Ok(d[0])
    }
}

impl<B, SpiErr, PinErr, DelayErr> radio::State for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type State = State;
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn set_state(&mut self, s: State) -> Result<(), Self::Error> {
        // Convert requested state to commands
        let v = match s {
            State::Idle => TrxCmd::ForceTrxOff,
            State::PllOn => TrxCmd::PllOn,
            State::RxOn => TrxCmd::RxOn,
            State::Sleep => todo!("Set slp_tr high to sleep"),
            //State::DeepSleep => todo!("Call prep deep sleep, set slp_tr"),
            _ => return Err(Error::Unsupported),
        };

        // TODO: check state is not StateTransitionInProgress before applying

        // Write command
        self.reg_write(Register::TrxCmd, v as u8)
    }

    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        use TrxStatus::*;

        // Read status register
        let raw = self.reg_read(Register::TrxStatus)?;
        debug!("TRX status: 0x{:02x}", raw);

        let trx_status = match TrxStatus::try_from(raw) {
            Ok(v) => v,
            Err(e) => {
                warn!("unrecognised TrxStatus 0x{:02x}", raw);
                return Err(Error::Unsupported);
            }
        };

        // Convert to state enum
        let s = match trx_status {
            POn => State::Init,
            BusyRx | BusyRxAack => State::BusyRx,
            BusyTx | BusyTxAret => State::BusyTx,
            RxOn => State::RxOn,
            TrxOff => State::Idle,
            PllOn => State::PllOn,
            Sleep | PrepDeepSleep => State::Sleep,
            RxAackOn => State::BusyRx,
            TxAretOn => State::BusyTx,
            StateTransition => State::Busy,
        };

        Ok(s)
    }
}

impl<B, SpiErr, PinErr, DelayErr> radio::Interrupts for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Irq = Irqs;
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn get_interrupts(&mut self, _: bool) -> Result<Self::Irq, Self::Error> {
        self.reg_read2::<Irqs>()
    }
}

impl<B, SpiErr, PinErr, DelayErr> radio::Channel for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Channel = ();

    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn set_channel(&mut self, channel: &Self::Channel) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<B, SpiErr, PinErr, DelayErr> radio::Transmit for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        // TODO: Set to PLL_ON state

        // TODO: load data into FIFO

        todo!()
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        // TODO: check TRX_END for completion
        todo!()
    }

    
}

impl<B, SpiErr, PinErr, DelayErr> radio::Receive for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    type Info = ();

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        // TODO: revert to PLL_ON if required (not already in PLL_ON or RX)

        // TODO: Set RX_ON

        todo!()
    }

    fn check_receive(&mut self, restart: bool) -> Result<bool, Self::Error> {
        // Check for RX_START TRX_END RX_CRC_VALID AMI (if extended mode enabled) IRQs, BUSY_RX state
        let irqs = self.reg_read2::<Irqs>()?;

        // TRX_END signals receive completion
        if irqs.contains(Irqs::TRX_END) {
            Ok(true)

        // TRX_UR signifies underflow
        } else if irqs.contains(Irqs::TRX_UR) {
            todo!()

        // Receiving in progress
        } else if irqs.contains(Irqs::RX_START) {
            Ok(true)

        // Nothing happening
        } else {
            Ok(false)
        }
    }

    fn get_received(&mut self, info: &mut Self::Info, buff: &mut [u8]) -> Result<usize, Self::Error> {
        // TODO: read RSSI

        // TODO: read data from FIFO
        todo!()
    }

    
}
#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
