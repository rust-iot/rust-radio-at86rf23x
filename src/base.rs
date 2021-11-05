use core::fmt::Debug;

use embedded_hal::{delay::blocking::*, digital::blocking::*, spi::blocking::*};

use crate::{device::CommandFlags, device::Register, Error};

/// IO container object to centralise IO traits and bounds
pub struct Io<Spi, Cs, Rst, SlpTr, Irq, Delay> {
    pub spi: Spi,
    pub cs: Cs,
    pub rst: Rst,
    pub slp_tr: SlpTr,
    pub irq: Irq,
    pub delay: Delay,
}

/// Base trait provides methods for underlying device interaction
pub trait Base<SpiErr: Debug, PinErr: Debug, DelayErr: Debug> {
    /// SPI write command
    fn spi_write(&mut self, cmd: u8, data: &[u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>>;

    /// SPI read command
    fn spi_read(&mut self, cmd: u8, data: &mut [u8])
        -> Result<(), Error<SpiErr, PinErr, DelayErr>>;

    /// Assert slp_tr
    fn sleep(&mut self, sleep: bool) -> Result<(), Error<SpiErr, PinErr, DelayErr>>;

    /// Reset the device
    fn reset(&mut self) -> Result<(), Error<SpiErr, PinErr, DelayErr>>;
}

/// Base trait implementation for Io objects
impl<Spi, SpiErr, Cs, Rst, SlpTr, Irq, PinErr, Delay, DelayErr> Base<SpiErr, PinErr, DelayErr>
    for Io<Spi, Cs, Rst, SlpTr, Irq, Delay>
where
    Spi: Transactional<u8, Error = SpiErr>,
    Cs: OutputPin<Error = PinErr>,
    Rst: OutputPin<Error = PinErr>,
    SlpTr: OutputPin<Error = PinErr>,
    Irq: InputPin<Error = PinErr>,
    Delay: DelayMs<u32, Error = DelayErr> + DelayUs<u32, Error = DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    fn spi_write(&mut self, cmd: u8, data: &[u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        let cmd = [cmd];
        let mut t = [Operation::Write(&cmd), Operation::Write(data)];

        self.cs.set_low().map_err(Error::Pin)?;

        let r = self.spi.exec(&mut t).map_err(Error::Spi);

        self.cs.set_high().map_err(Error::Pin)?;

        r
    }

    fn spi_read(
        &mut self,
        cmd: u8,
        data: &mut [u8],
    ) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        let cmd = [cmd];
        let mut t = [Operation::Write(&cmd), Operation::Transfer(data)];

        self.cs.set_low().map_err(Error::Pin)?;

        let r = self.spi.exec(&mut t).map_err(Error::Spi);

        self.cs.set_high().map_err(Error::Pin)?;

        r
    }

    fn reset(&mut self) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        // Deassert CS pin (active high)
        self.cs.set_high().map_err(Error::Pin)?;

        // Deassert sleep pin (active high)
        self.slp_tr.set_low().map_err(Error::Pin)?;

        // Assert reset pin (active low)
        self.rst.set_low().map_err(Error::Pin)?;

        // Wait a moment for reset
        // TODO: how long should this be?
        let r = self.delay.delay_ms(10).map_err(Error::Delay);

        // Deassert reset (active low)
        self.rst.set_high().map_err(Error::Pin)?;

        // Wait a moment for init
        // TODO: how long should this be?
        self.delay.delay_ms(10).map_err(Error::Delay)?;

        r
    }

    fn sleep(&mut self, sleep: bool) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        match sleep {
            true => self.slp_tr.set_high().map_err(Error::Pin),
            false => self.slp_tr.set_low().map_err(Error::Pin),
        }
    }
}
