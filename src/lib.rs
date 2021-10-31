
use core::fmt::Debug;
use log::{debug};

use radio::{Registers as _, State as _};


pub mod prelude;
pub mod base;
pub mod device;

use base::Base;
use device::*;


/// AT86RF23x driver object
pub struct At86Rf23x<B> {
    hal: B
}


/// Error type for AT86RF23x
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "thiserror", derive(thiserror::Error))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<SpiErr, PinErr, DelayErr> {
    /// Communications (SPI or UART) error
    Spi(SpiErr),
    /// Pin control error
    Pin(PinErr),
    /// Delay error
    Delay(DelayErr),
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum State {
    PllOn,
    TxAretOn,
    BusyRxAack,
    TrxOff,
    PrepDeepSleep,
    Sleep,
    DeepSleep,
}



/// Part information
#[derive(Debug, Clone, PartialEq)]
pub struct Info {
    pub part: u8,
    pub ver: u8,
    pub mfr: u8,
}

impl <B> At86Rf23x<B>
where
    B: Base,
{
    pub fn new(hal: B) -> Result<Self, B::E> {
        let mut s = Self { hal };

        debug!("Connecting to device");

        // Perform reset
        s.hal.reset()?;

        // Read info
        let i = s.info()?;
        debug!("Device info: {:02x?}", i);

        Ok(s)
    }

    pub fn info(&mut self) -> Result<Info, B::E> {
        let i = Info {
            part: self.reg_read(Register::PartNum)?,
            ver: self.reg_read(Register::VersionNum)?,
            mfr: self.reg_read(Register::ManufacturerId)?,
        };
        Ok(i)
    }

    /// Write to the device buffer
    fn buff_write(&mut self, data: &[u8]) -> Result<(), B::E> {
        self.hal.spi_write(CommandFlags::BUFF_WR.bits(), data)
    }

    /// Read from the device buffer
    fn buff_read(&mut self, data: &mut [u8]) -> Result<(), B::E> {
        self.hal.spi_read(CommandFlags::BUFF_RD.bits(), data)
    }

    /// Write to the device SRAM
    fn sram_write(&mut self, data: &[u8]) -> Result<(), B::E> {
        self.hal.spi_write(CommandFlags::SRAM_WR.bits(), data)
    }

    /// Read from the device SRAM
    fn sram_read(&mut self, data: &mut [u8]) -> Result<(), B::E> {
        self.hal.spi_read(CommandFlags::SRAM_RD.bits(), data)
    }
}

impl <B> radio::Registers<Register> for At86Rf23x<B>
where
    B: Base,
{
    type Error = B::E;

    /// Write a value to a device register
    fn reg_write(&mut self, reg: Register, v: u8) -> Result<(), B::E> {
        self.hal.spi_write(reg as u8 | CommandFlags::REG_WR.bits(), &[v])
    }

    /// Read a value from a device register
    fn reg_read(&mut self, reg: Register) -> Result<u8, B::E> {
        let mut d = [0u8];
        self.hal.spi_read(reg as u8 | CommandFlags::REG_RD.bits(), &mut d)?;
        Ok(d[0])
    }
}

impl <B> radio::State for At86Rf23x<B>
where
    B: Base,
{
    type State = State;
    type Error = B::E;
    
    fn set_state(&mut self, _: Self::State) -> Result<(), Self::Error> {
        let v = 0;
        
        // TODO: set actual state

        self.reg_write(Register::TrxCmd, v)
    }
    
    fn get_state(&mut self) -> Result<Self::State, Self::Error> { 
        let r = self.reg_read(Register::TrxStatus)?;

        // TODO: parse out state

        todo!()
    }
}

impl <B> radio::Interrupts for At86Rf23x<B>
where
    B: Base,
{
    type Irq = Irqs;
    type Error = B::E;

    fn get_interrupts(&mut self, _: bool) -> Result<Self::Irq, Self::Error> { 
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
