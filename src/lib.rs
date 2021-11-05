use core::fmt::Debug;
use embedded_hal::delay::blocking::DelayUs;
use log::{debug, warn};
use std::marker::PhantomData;

use radio::{BasicInfo, Registers as _, State as _};

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

impl radio::RadioState for State {
    fn idle() -> Self {
        State::Idle
    }

    fn sleep() -> Self {
        State::Sleep
    }
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
        s.hal.slp_tr(false)?;

        // Perform reset
        s.hal.reset()?;

        // TODO: enable AWAKE_END IRQ to detect move to TRX_OFF

        // Write TRX_OFF to enter ready state
        s.reg_write(Register::TrxCmd, TrxCmd::TrxOff as u8)?;

        // TODO: Wait for TRX_OFF state (PLL lock)

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

    /// Write to the device FIFO
    pub fn fifo_write(&mut self, data: &[u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_write(CommandFlags::BUFF_WR.bits(), data)
    }

    /// Read from the device FIFO
    pub fn fifo_read(&mut self, data: &mut [u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_read(CommandFlags::BUFF_RD.bits(), data)
    }

    /// Write to the device SRAM
    pub fn sram_write(&mut self, data: &[u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_write(CommandFlags::SRAM_WR.bits(), data)
    }

    /// Read from the device SRAM
    pub fn sram_read(&mut self, data: &mut [u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
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

    pub fn reg_update<R: Reg, F: Fn(&mut R)>(&mut self, f: F) -> Result<R, Error<SpiErr, PinErr, DelayErr>> {
        let mut v = self.reg_read2::<R>()?;
        f(&mut v);
        self.reg_write2(v)?;
        Ok(v)
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

        debug!("Set state cmd: {:?} (requested: {:?})", v, s);

        // TODO: check state is not StateTransitionInProgress before applying

        // Write command
        self.reg_write(Register::TrxCmd, v as u8)
    }

    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        use TrxStatus::*;

        // Read status register
        let raw = self.reg_read(Register::TrxStatus)?;

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

        debug!("TRX status: 0x{:02x} state: {:?}", raw, s);


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
        let irqs = self.reg_read2::<Irqs>()?;

        if !irqs.is_empty() {
            debug!("IRQs: {:?}", irqs);
        }

        Ok(irqs)
    }
}

/// AT86RF23x Channel Object
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ch {
    /// Channel bitrate
    pub bitrate: OqpskDataRate,
    /// Channel frequency
    pub channel: Channel,
}

impl<B, SpiErr, PinErr, DelayErr> radio::Channel for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Channel = Ch;

    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn set_channel(&mut self, ch: &Self::Channel) -> Result<(), Self::Error> {
        // Ensure we're idle
        self.set_state(State::PllOn)?;

        // TODO: support alternate channel configs?

        // Load new channel
        self.reg_update::<PhyCcCca, _>(|r| r.set_channel(ch.channel) )?;

        // Load new bitrate
        self.reg_update::<TrxCtrl2, _>(|r| r.set_oqpsk_data_rate(ch.bitrate) )?;

        Ok(())
    }
}

impl<B, SpiErr, PinErr, DelayErr> DelayUs<u32> for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        self.hal.delay_us(us)
    }
}

impl<B, SpiErr, PinErr, DelayErr> radio::Power for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn set_power(&mut self, _power: i8) -> Result<(), Self::Error> {
        // TODO: convert power from int to nearest power value
        let p = Power::P4dBm;

        // Update power register
        self.reg_update::<PhyTxPwr, _>(|r| r.set_tx_pwr(p) )?;

        Ok(())
    }

}

impl<B, SpiErr, PinErr, DelayErr> radio::Rssi for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn poll_rssi(&mut self) -> Result<i16, Self::Error> {
        let r = self.reg_read2::<PhyRssi>()?;
        Ok(-94 + r.rssi() as i16 * 3)
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
        // Ensure we're idle and the PLL is on
        self.set_state(State::PllOn)?;

        // Load data into FIFO
        // TODO: check length is valid

        debug!("TX data: {:02x?}", data);

        // First length
        self.fifo_write(&[data.len() as u8])?;

        // Then data
        self.fifo_write(data)?;

        // Setup IRQs
        let irqs = Irqs::TRX_END | Irqs::TRX_UR | Irqs::AMI;
        self.reg_write(Register::IrqMask, irqs.bits())?;

        debug!("Entering TX state");

        // Set to TX state
        self.reg_write(Register::TrxCmd, TrxCmd::TxStart as u8)?;

        Ok(())
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        // Poll on IRQ pin if available
        // TODO: feature gate this like other impls?
        if !self.hal.irq()? {
            return Ok(false)
        }

        // Check for RX_START TRX_END RX_CRC_VALID AMI (if extended mode enabled) IRQs, BUSY_RX state
        let irqs = self.reg_read2::<Irqs>()?;

        if !irqs.is_empty() {
            debug!("TX IRQs: {:?}", irqs);
        }

        // TRX_END signals receive completion
        if irqs.contains(Irqs::TRX_END) {
            debug!("TX complete");
            Ok(true)

        } else {
            Ok(false)
        }
        
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

    type Info = BasicInfo;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        // Ensure we're idle and the PLL is on
        // TODO: skip this if not required? (ie. already in PLL_ON or RX_ON)
        self.set_state(State::PllOn)?;

        // Setup IRQs
        let irqs = Irqs::RX_START | Irqs::TRX_END | Irqs::TRX_UR | Irqs::AMI;
        self.reg_write(Register::IrqMask, irqs.bits())?;

        debug!("Entering RX state");

        // Set to receive state
        self.reg_write(Register::TrxCmd, TrxCmd::RxOn as u8)?;

        Ok(())
    }

    /// Poll to check for receive completion
    fn check_receive(&mut self, _restart: bool) -> Result<bool, Self::Error> {
        // Poll on IRQ pin if available
        // TODO: feature gate this like other impls?
        if !self.hal.irq()? {
            return Ok(false)
        }

        // Check for RX_START TRX_END RX_CRC_VALID AMI (if extended mode enabled) IRQs, BUSY_RX state
        let irqs = self.reg_read2::<Irqs>()?;

        if !irqs.is_empty() {
            debug!("RX IRQs: {:?}", irqs);
        }

        // TRX_END signals receive completion
        if irqs.contains(Irqs::TRX_END) {
            debug!("RX complete");
            Ok(true)

        // TODO: RX_CRC_VALID, AMI IRQs for extended mode

        // TRX_UR signifies FIFO underflow
        } else if irqs.contains(Irqs::TRX_UR) {
            todo!()

        // Receiving in progress
        } else if irqs.contains(Irqs::RX_START) {
            debug!("RX start");
            Ok(true)

        // Nothing happening
        } else {
            Ok(false)
        }
    }

    fn get_received(&mut self, buff: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        // TODO: read RSSI

        // Read first byte PHR to discern length
        self.fifo_read(&mut buff[..1])?;
        let n = buff[0] as usize;

        // Read following data
        self.fifo_read(&mut buff[1..])?;

        // Return packet length
        Ok((n, BasicInfo::default()))
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
