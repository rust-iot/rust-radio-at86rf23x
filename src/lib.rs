use core::fmt::Debug;
use embedded_hal::delay::blocking::DelayUs;
use log::{debug, warn};
use std::marker::PhantomData;

use radio::{BasicInfo, Channel as _, Interrupts, Registers as _, State as _};

pub mod base;
pub mod device;
pub mod prelude;

use base::Base;
use device::*;

pub use device::CHANNELS;

/// AT86RF23x driver object
pub struct At86Rf23x<B, SpiErr: Debug, PinErr: Debug, DelayErr: Debug> {
    hal: B,
    auto_crc: bool,
    _err: PhantomData<Error<SpiErr, PinErr, DelayErr>>,
}

/// AT86RF23x device configuration
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub xtal_mode: XtalMode,
    pub channel: Ch,
    pub auto_crc: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            xtal_mode: XtalMode::InternalOscillator,
            channel: Ch {
                bitrate: OqpskDataRate::D250kbps,
                channel: CHANNELS[0],
            },
            auto_crc: true,
        }
    }
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

    #[error("Unexpected register value (reg: 0x{0:02x} val: 0x:{1:02x}")]
    /// No response from device
    UnexpectedValue(u8, u8),
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
    pub part: Part,
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
    pub fn new(hal: B, config: Config) -> Result<Self, Error<SpiErr, PinErr, DelayErr>> {
        let mut s = Self {
            hal,
            auto_crc: config.auto_crc,
            _err: PhantomData,
        };

        debug!("Connecting to device");

        // Deassert sleep
        s.hal.slp_tr(false)?;

        // Perform reset
        s.hal.reset()?;

        // TODO: enable AWAKE_END IRQ to detect move to TRX_OFF

        // Write TRX_OFF to enter ready state
        s.write_register::<TrxCmd>(TrxCmd::TrxOff)?;

        // TODO: Wait for TRX_OFF state (PLL lock)

        // Read info
        let i = s.info()?;
        if i.part == Part::None && i.ver == 0 && i.mfr == 0 {
            warn!("Init failed, communication error");
            return Err(Error::NoResponse);
        }

        // TODO: check voltages are okay

        // TODO: configure xtal

        // Set channel
        s.set_channel(&config.channel)?;

        // TODO: set CCA mode

        // Enable promiscuous mode
        s.update_register::<XahCtrl1, _>(|r| r.with_aack_prom_mode(true))?;

        if s.auto_crc {
            // Enable auto-crc in TX
            s.update_register::<TrxCtrl1, _>(|r| r.with_tx_auto_crc_on(true))?;
        }

        // Enable dynamic frame buffer protection
        s.update_register::<TrxCtrl2, _>(|r| r.with_rx_safe_mode(true))?;

        debug!("Device info: {:02x?}", i);

        Ok(s)
    }

    pub fn info(&mut self) -> Result<Info, Error<SpiErr, PinErr, DelayErr>> {
        let i = Info {
            part: self.read_register::<PartNum>().map(|p| p.part())?,
            ver: self.read_register::<VersionNum>()?.into(),
            mfr: u16::from_le_bytes([
                self.read_register::<ManId0>()?.into(),
                self.read_register::<ManId1>()?.into(),
            ]),
        };
        Ok(i)
    }

    /// Write to the device FIFO
    pub fn fifo_write(&mut self, data: &[u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_write(&[CommandFlags::BUFF_WR.bits()], data)
    }

    /// Read from the device FIFO
    pub fn fifo_read(&mut self, data: &mut [u8]) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal.spi_read(&[CommandFlags::BUFF_RD.bits()], data)
    }

    /// Write to the device SRAM
    pub fn sram_write(
        &mut self,
        offset: u8,
        data: &[u8],
    ) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal
            .spi_write(&[CommandFlags::SRAM_WR.bits(), offset], data)
    }

    /// Read from the device SRAM
    pub fn sram_read(
        &mut self,
        offset: u8,
        data: &mut [u8],
    ) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.hal
            .spi_read(&[CommandFlags::SRAM_RD.bits(), offset], data)
    }
}

impl<B, SpiErr, PinErr, DelayErr> radio::Registers<u8> for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    /// Read a register value
    fn read_register<R: radio::Reg<u8>>(&mut self) -> Result<R, Self::Error> {
        let mut d = [0u8];
        self.hal
            .spi_read(&[R::ADDRESS as u8 | CommandFlags::REG_RD.bits()], &mut d)?;

        R::try_from(d[0]).map_err(|_e| Error::UnexpectedValue(R::ADDRESS, d[0]))
    }

    /// Write a register value
    fn write_register<R: radio::Reg<u8>>(&mut self, value: R) -> Result<(), Self::Error> {
        self.hal.spi_write(
            &[R::ADDRESS as u8 | CommandFlags::REG_WR.bits()],
            &[value.into()],
        )
    }
}

impl<B, SpiErr, PinErr, DelayErr> radio::Registers<u16> for At86Rf23x<B, SpiErr, PinErr, DelayErr>
where
    B: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    /// Read a register value
    fn read_register<R: radio::Reg<u16>>(&mut self) -> Result<R, Self::Error> {
        let mut r = [0u8, 0u8];
        self.hal
            .spi_read(&[R::ADDRESS as u8 | CommandFlags::REG_RD.bits()], &mut r)?;

        let d = u16::from_le_bytes(r);

        R::try_from(d).map_err(|_e| Error::UnexpectedValue(R::ADDRESS, r[0]))
    }

    /// Write a register value
    fn write_register<R: radio::Reg<u16>>(&mut self, value: R) -> Result<(), Self::Error> {
        let v = u16::to_le_bytes(value.into());

        self.hal
            .spi_write(&[R::ADDRESS as u8 | CommandFlags::REG_WR.bits()], &v)
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
        self.write_register::<TrxCmd>(v)?;

        Ok(())
    }

    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        use TrxStatus::*;

        // Read status register
        let trx_status = self.read_register::<TrxStatus>()?;

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

        debug!("TRX status: {:?} state: {:?}", trx_status, s);

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
        let irqs = self.read_register::<Irqs>()?;

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
        self.update_register::<PhyCcCca, _>(|r| r.with_channel(ch.channel))?;

        // Load new bitrate
        self.update_register::<TrxCtrl2, _>(|r| r.with_oqpsk_data_rate(ch.bitrate))?;

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
        self.update_register::<PhyTxPwr, _>(|r| r.with_tx_pwr(p))?;

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
        let r = self.read_register::<PhyRssi>()?;
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

        // Calculate length
        let mut len = data.len() as u8;
        if self.auto_crc {
            len += 2;
        }

        // First length
        self.sram_write(0, &[len])?;

        // Then data
        self.sram_write(1, data)?;

        // Add CRC padding if required
        if self.auto_crc {
            let crc = [0xFFu8; 2];
            self.sram_write(1 + data.len() as u8, &crc)?;
        }

        // Setup IRQs
        let irqs = Irqs::TRX_END | Irqs::TRX_UR | Irqs::AMI;
        self.write_register::<IrqMask>(irqs.bits().into())?;
        let _ = self.get_interrupts(true)?;

        debug!("Entering TX state");

        // Set to TX state
        self.write_register::<TrxCmd>(TrxCmd::TxStart)?;

        Ok(())
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        // Poll on IRQ pin if available
        // TODO: feature gate this like other impls?
        if !self.hal.irq()? {
            return Ok(false);
        }

        // Check for RX_START TRX_END RX_CRC_VALID AMI (if extended mode enabled) IRQs, BUSY_RX state
        let irqs = self.read_register::<Irqs>()?;

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
        self.write_register::<IrqMask>(irqs.bits().into())?;
        let _ = self.get_interrupts(true)?;

        debug!("Entering RX state");

        // Set to receive state
        self.write_register::<TrxCmd>(TrxCmd::RxOn)?;

        Ok(())
    }

    /// Poll to check for receive completion
    fn check_receive(&mut self, _restart: bool) -> Result<bool, Self::Error> {
        // Poll on IRQ pin if available
        // TODO: feature gate this like other impls?
        if !self.hal.irq()? {
            return Ok(false);
        }

        // Check for RX_START TRX_END RX_CRC_VALID AMI (if extended mode enabled) IRQs, BUSY_RX state
        let irqs = self.read_register::<Irqs>()?;

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
        // Read RSSI (using energy detect per datasheet recommendation)
        let ed = self.read_register::<PhyEdLevel>()?;
        let info = if ed.ed_level() < 0x54 {
            BasicInfo::new(-94 + ed.ed_level() as i16, u16::MIN)
        } else {
            BasicInfo::default()
        };

        // TODO: apparently different 231 and 233 devices operate differently here...
        // https://github.com/msolters/arduino-at86rf233/blob/master/at86rf2xx.cpp#L253

        // Read first byte PHR to discern length
        self.sram_read(0, &mut buff[..1])?;
        let n = buff[0] as usize;

        // Read following data
        self.sram_read(1, &mut buff[1..][..n])?;

        // TODO: if we have auto CRC enabled should we check here / remove from returned array?

        // Return packet length
        Ok((n, info))
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
