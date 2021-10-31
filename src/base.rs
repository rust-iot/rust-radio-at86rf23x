
use embedded_hal::{
    spi::blocking::*,
    digital::blocking::*,
    delay::blocking::*,
};

use crate::{Error, device::Register, device::CommandFlags};

/// IO container object to centralise IO traits and bounds
pub struct Io<Spi, Cs, Rst, SlpTr, Irq, Delay> {
    pub spi: Spi,
    pub cs: Cs,
    pub rst: Rst,
    pub slp_tr: SlpTr,
    pub irq: Irq,
    pub delay: Delay
}

/// Base trait provides methods for underlying device interaction
pub trait Base {
    /// Error type for base impl
    type E;

    /// SPI write command
    fn spi_write(&mut self, cmd: u8, data: &[u8]) -> Result<(), Self::E>;

    /// SPI read command
    fn spi_read(&mut self, cmd: u8, data: &mut [u8]) -> Result<(), Self::E>;

    /// Reset the device
    fn reset(&mut self) -> Result<(), Self::E>;
}

/// Base trait implementation for Io objects
impl <Spi, SpiErr, Cs, Rst, SlpTr, Irq, PinErr, Delay, DelayErr> Base for Io<Spi, Cs, Rst, SlpTr, Irq, Delay>
where
    Spi: Transactional<u8, Error=SpiErr>,
    Cs: OutputPin<Error=PinErr>,
    Rst: OutputPin<Error=PinErr>,
    SlpTr: OutputPin<Error=PinErr>,
    Irq: InputPin<Error=PinErr>,
    Delay: DelayMs<u32, Error=DelayErr> + DelayUs<u32, Error=DelayErr>,
{
    type E = Error<SpiErr, PinErr, DelayErr>;

    fn spi_write(&mut self, cmd: u8, data: &[u8]) -> Result<(), Self::E> {
        let cmd = [cmd];
        let mut t = [
            Operation::Write(&cmd),
            Operation::Write(data),
        ];

        self.cs.set_low().map_err(Error::Pin)?;

        let r = self.spi.exec(&mut t).map_err(Error::Spi);

        self.cs.set_high().map_err(Error::Pin)?;

        r
    }

    fn spi_read(&mut self, cmd: u8, data: &mut [u8]) -> Result<(), Self::E> {
        let cmd = [cmd];
        let mut t = [
            Operation::Write(&cmd),
            Operation::Transfer(data),
        ];

        self.cs.set_low().map_err(Error::Pin)?;

        let r = self.spi.exec(&mut t).map_err(Error::Spi);

        self.cs.set_high().map_err(Error::Pin)?;

        r
    }

    fn reset(&mut self) -> Result<(), Self::E> {
        self.rst.set_low().map_err(Error::Pin)?;

        // TODO: how long should this be?
        let r = self.delay.delay_ms(10).map_err(Error::Delay);

        self.rst.set_high().map_err(Error::Pin)?;

        r
    }
    
}
