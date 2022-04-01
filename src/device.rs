use core::convert::Infallible;

use modular_bitfield::prelude::*;
use num_enum::{IntoPrimitive, TryFromPrimitive, TryFromPrimitiveError};

/// Register address listing
#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive)]
#[repr(u8)]
pub enum Register {
    TrxStatus = 0x01,
    TrxCmd = 0x02,
    TrxCtrl0 = 0x03,
    TrxCtrl1 = 0x04,
    PhyTxPwr = 0x05,  // reserved reserved reserved TX_PWR 116
    PhyRssi = 0x06,   // RX_CRC_VALID RND_VALUE RSSI 98, 100, 151
    PhyEdLevel = 0x07,// ED_LEVEL 103
    PhyCcCca = 0x08,  // CCA_REQUEST CCA_MODE CHANNEL 107, 134
    CcaThres = 0x09,  // reserved CCA_ED_THRES 108
    RxCtrl = 0x0A,    // PEL_SHIFT_VALUE reserved reserved PDT_THRES 161, 179
    SfdValue = 0x0B,  // SFD_VALUE 171
    TrxCtrl2 = 0x0C,  // RX_SAFE_MODE reserved OQPSK_SCRAM_EN reserved OQPSK_DATA_RATE 156, 170
    AntDiv = 0x0D,    // ANT_SEL reserved ANT_DIV_EN ANT_EXT_SW_EN ANT_CTRL 161
    IrqMask = 0x0E,   // IRQ_MASK 33
    IrqStatus = 0x0F, // IRQ_7_BAT_LOW IRQ_6_TRX_UR IRQ_5_AMI IRQ_4_CCA_ED_DONE IRQ_3_TRX_END IRQ_2_RX_START  IRQ_1_PLL_UNLOCK IRQ_0_PLL_LOCK 33
    VregCtrl = 0x10,  // AVREG_EXT AVDD_OK reserved DVREG_EXT DVDD_OK reserved 122
    Batmon = 0x11,    // reserved reserved BATMON_OK BATMON_HR BATMON_VTH 125
    XoscCtrl = 0x12,  // XTAL_MODE XTAL_TRIM 130
    CcCtrl0 = 0x13,   // CC_NUMBER 135
    CcCtrl1 = 0x14,   // reserved CC_BAND 136
    RxSyn = 0x15,     // RX_PDT_DIS reserved RX_PDT_LEVEL 112, 157
    TrxRpc = 0x16,    // RX_RPC_CTRL RX_RPC_EN PDT_RPC_EN PLL_RPC_EN XAH_TX_RPC_EN IPAN_RPC_EN reserved 175
    XahCtrl1 = 0x17,  // ARET_TX_TS_EN reserved AACK_FLTR_RES_FT AACK_UPLD_RES_FT reserved AACK_ACK_TIME  AACK_PROM_MODE AACK_SPC_EN
    FtnCtrl = 0x18,   // FTN_START reserved FTNV 139
    XahCtrl2 = 0x19,  // ARET_FRAME_RETRIES ARET_CSMA_RETRIES reserved 74
    PllCf = 0x1A,     // PLL_CF_START reserved reserved PLL_CF 137
    PllDcu = 0x1B,    // PLL_DCU_START reserved reserved 137
    PartNum = 0x1C,   // PART_NUM 27
    VersionNum = 0x1D, // VERSION_NUM 27
    ManId0 = 0x1E,    // MAN_ID_0 28
    ManId1 = 0x1F,    // MAN_ID_1 28
    ShortAddr0 = 0x20, // SHORT_ADDR_0 91
    ShortAddr1 = 0x21, // SHORT_ADDR_1 91
    PanId0 = 0x22,    // PAN_ID_0 92
    PanId1 = 0x23,    // PAN_ID_1 92
    IeeeAddr0 = 0x24, // IEEE_ADDR_0 92
    IeeeAddr1 = 0x25, // IEEE_ADDR_1 93
    IeeeAddr2 = 0x26, // IEEE_ADDR_2 93
    IeeeAddr3 = 0x27, // IEEE_ADDR_3 93
    IeeeAddr4 = 0x28, // IEEE_ADDR_4 94
    IeeeAddr5 = 0x29, // IEEE_ADDR_5 94
    IeeeAddr6 = 0x2A, // IEEE_ADDR_6 94
    IeeeAddr7 = 0x2B, // IEEE_ADDR_7 94
    XahCtrl0 = 0x2C,
    CsmaSeed0 = 0x2D,
    CsmaSeed1 = 0x2E,
    CsmaBe = 0x2F,
    TstCtrlDigi = 0x36,
    TstAgc = 0x3C,
    TstSdm = 0x3D,
}

/// TRX_STATUS register
#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive, IntoPrimitive)]
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

impl radio::Register for TrxStatus {
    const ADDRESS: u8 = Register::TrxStatus as u8;
    type Word = u8;
    type Error = TryFromPrimitiveError<Self>;
}

/// TRX_CMD register
#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive, IntoPrimitive)]
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
impl radio::Register for TrxCmd {
    const ADDRESS: u8 = Register::TrxCmd as u8;
    type Word = u8;
    type Error = TryFromPrimitiveError<Self>;
}

/// TRX_CTRL0 register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TrxCtrl0 {
    pub tom_en: bool,
    pub reserved: bool,
    pub pmu_en: bool,
    pub pmu_if_inverse: bool,
    pub clkm_sha_sel: bool,
    pub clkm_ctrl: B3,
}
impl radio::Register for TrxCtrl0 {
    const ADDRESS: u8 = Register::TrxCtrl0 as u8;
    type Word = u8;
    type Error = Infallible;
}

/// TRX_CTRL1 register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TrxCtrl1 {
    pub pa_ext_en: bool,
    pub irq_2_ext_en: bool,
    pub tx_auto_crc_on: bool,
    pub rx_bl_ctrl: bool,
    pub spi_cmd_mode: B2,
    pub irq_mask_mode: bool,
    pub irq_polarity: bool,
}

impl radio::Register for TrxCtrl1 {
    const ADDRESS: u8 = Register::TrxCtrl1 as u8;
    type Word = u8;
    type Error = Infallible;
}

/// PHY_TX_PWR register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PhyTxPwr {
    pub reserved: B4,
    pub tx_pwr: Power,
}

impl radio::Register for PhyTxPwr {
    const ADDRESS: u8 = Register::PhyTxPwr as u8;
    type Word = u8;
    type Error = Infallible;
}

/// RF output power enumeration. Used in PHY_TX_PWR
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 4]
pub enum Power {
    P4dBm = 0x0,
    P3_7dBm = 0x1,
    P3_4dBm = 0x2,
    P3dBm = 0x3,
    P2_5dBm = 0x4,
    P2dBm = 0x5,
    P1dBm = 0x6,
    P0dBm = 0x7,
    Pn1dBm = 0x8,
    Pn2dBm = 0x9,
    Pn3dBm = 0xA,
    Pn4dBm = 0xB,
    Pn6dBm = 0xC,
    Pn8dBm = 0xD,
    Pn12dBm = 0xE,
    Pn17dBm = 0xF,
}

/// PHY_RSSI register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PhyRssi {
    pub rx_crc_valid: bool,
    pub rnd_value: B2,
    pub rssi: B5,
}

impl radio::Register for PhyRssi {
    const ADDRESS: u8 = Register::PhyRssi as u8;
    type Word = u8;
    type Error = Infallible;
}

/// PHY_ED_LEVEL register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PhyEdLevel {
    pub ed_level: u8,
}

impl radio::Register for PhyEdLevel {
    const ADDRESS: u8 = Register::PhyEdLevel as u8;
    type Word = u8;
    type Error = Infallible;
}

/// PHY_CC_CCA register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PhyCcCca {
    pub cca_request: bool,
    pub cca_mode: B2,
    pub channel: Channel,
}

impl radio::Register for PhyCcCca {
    const ADDRESS: u8 = Register::PhyCcCca as u8;
    type Word = u8;
    type Error = Infallible;
}

/// IEEE802.15.4 channel enumeration (for default channel mode)
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 5]
pub enum Channel {
    C2405MHz = 0x0B,
    C2410MHz = 0x0C,
    C2415MHz = 0x0D,
    C2420MHz = 0x0E,
    C2425MHz = 0x0F,
    C2430MHz = 0x10,
    C2435MHz = 0x11,
    C2440MHz = 0x12,
    C2445MHz = 0x13,
    C2450MHz = 0x14,
    C2455MHz = 0x15,
    C2460MHz = 0x16,
    C2465MHz = 0x17,
    C2470MHz = 0x18,
    C2475MHz = 0x19,
    C2480MHz = 0x1A,
}

/// Channel list for access by IEEE802.15.4 channel index
pub const CHANNELS: &[Channel] = &[
    Channel::C2405MHz,
    Channel::C2410MHz,
    Channel::C2415MHz,
    Channel::C2420MHz,
    Channel::C2425MHz,
    Channel::C2430MHz,
    Channel::C2435MHz,
    Channel::C2440MHz,
    Channel::C2445MHz,
    Channel::C2450MHz,
    Channel::C2455MHz,
    Channel::C2460MHz,
    Channel::C2465MHz,
    Channel::C2470MHz,
    Channel::C2475MHz,
    Channel::C2480MHz,
];

/// Attempt to convert frequency in khz to a channel index
impl TryFrom<u32> for Channel {
    type Error = ();

    fn try_from(freq_khz: u32) -> Result<Self, Self::Error> {
        let ch = match freq_khz {
            2405 => Channel::C2405MHz,
            2410 => Channel::C2410MHz,
            2415 => Channel::C2415MHz,
            2420 => Channel::C2420MHz,
            2425 => Channel::C2425MHz,
            2430 => Channel::C2430MHz,
            2435 => Channel::C2435MHz,
            2440 => Channel::C2440MHz,
            2445 => Channel::C2445MHz,
            2450 => Channel::C2450MHz,
            2455 => Channel::C2455MHz,
            2460 => Channel::C2460MHz,
            2465 => Channel::C2465MHz,
            2470 => Channel::C2470MHz,
            2475 => Channel::C2475MHz,
            2480 => Channel::C2480MHz,
            _ => return Err(()),
        };
        Ok(ch)
    }
}

/// CCA_THRES register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CcaThres {
    pub reserved: B4,
    pub cca_ed_thres: B4,
}
impl radio::Register for CcaThres {
    const ADDRESS: u8 = Register::CcaThres as u8;
    type Word = u8;
    type Error = Infallible;
}

/// RX_CTRL register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxCtrl {
    pub pel_shift_falue: B2,
    #[skip]
    pub __: B2,
    pub pdt_thresh: B4,
}
impl radio::Register for RxCtrl {
    const ADDRESS: u8 = Register::RxCtrl as u8;
    type Word = u8;
    type Error = Infallible;
}

/// SFD_VALUE
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct SfdValue {
    pub sfd_value: u8,
}
impl radio::Register for SfdValue {
    const ADDRESS: u8 = Register::SfdValue as u8;
    type Word = u8;
    type Error = Infallible;
}

/// TRX_CTRL2
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TrxCtrl2 {
    pub rx_safe_mode: bool,
    #[skip]
    pub __1: bool,
    pub oqpsk_scram_en: bool,
    #[skip]
    pub __1: B2,
    pub oqpsk_data_rate: OqpskDataRate,
}
impl radio::Register for TrxCtrl2 {
    const ADDRESS: u8 = Register::TrxCtrl2 as u8;
    type Word = u8;
    type Error = Infallible;
}

/// OQPSK data rate enumeration, used in [TrxCtrl2]
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum OqpskDataRate {
    D250kbps = 0,
    D500kbps = 1,
    D1000kbps = 2,
    D2000kbps = 3,
}

impl TryFrom<u32> for OqpskDataRate {
    type Error = ();

    /// Attempt to convert from data rate in bps
    fn try_from(bps: u32) -> Result<Self, Self::Error> {
        match bps {
            250_000 => Ok(Self::D250kbps),
            500_000 => Ok(Self::D500kbps),
            1000_000 => Ok(Self::D1000kbps),
            2000_000 => Ok(Self::D2000kbps),
            _ => Err(()),
        }
    }
}

/// ANT_DIV register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct AntDiv {
    pub ant_sel: B3,
    #[skip]
    pub __: bool,
    pub ant_div_en: bool,
    pub ant_ext_sw_en: bool,
    pub ant_ctrl: B2,
}
impl radio::Register for AntDiv {
    const ADDRESS: u8 = Register::AntDiv as u8;
    type Word = u8;
    type Error = Infallible;
}

/// IRQ_MASK register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask {
    pub irq_mask: u8,
}
impl radio::Register for IrqMask {
    const ADDRESS: u8 = Register::IrqMask as u8;
    type Word = u8;
    type Error = Infallible;
}

/// IRQ_STATUS register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus {
    /// Supply voltage below programmed threshold
    pub batt_low: bool,
    /// Frame buffer access violation
    pub trx_ur: bool,
    /// Address match
    pub ami: bool,
    /// Awake end or CCA or ED measurement complete
    pub cca_ed_done: bool,
    /// Completion of frame rx/tx
    pub trx_end: bool,
    /// Start of PDSU reception
    pub rx_start: bool,
    /// PLL unlock
    pub pll_unlock: bool,
    /// PLL lock
    pub pll_lock: bool,
}
impl radio::Register for IrqStatus {
    const ADDRESS: u8 = Register::IrqStatus as u8;
    type Word = u8;
    type Error = Infallible;
}

/// VREG_CTRL register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VregCtrl {
    pub avreg_ext: bool,
    pub avdd_ok: bool,
    #[skip]
    pub __1: B2,
    pub dvreg_ext: bool,
    pub dvdd_ok: bool,
    #[skip]
    pub __2: B2,
}
impl radio::Register for VregCtrl {
    const ADDRESS: u8 = Register::VregCtrl as u8;
    type Word = u8;
    type Error = Infallible;
}

/// BATMON register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Batmon {
    #[skip]
    pub __: B2,
    pub batmon_ok: bool,
    pub batmon_hr: bool,
    pub batmon_vth: B4,
}
impl radio::Register for Batmon {
    const ADDRESS: u8 = Register::Batmon as u8;
    type Word = u8;
    type Error = Infallible;
}

/// XOSC_CTRL register
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XoscCtrl {
    pub xtal_mode: XtalMode,
    pub xtal_trim: B4,
}
impl radio::Register for XoscCtrl {
    const ADDRESS: u8 = Register::XoscCtrl as u8;
    type Word = u8;
    type Error = Infallible;
}

#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 4]
pub enum XtalMode {
    ExternalReference = 0x5,
    InternalOscillator = 0xF,
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CcCtrl0 {
    pub cc_number: u8,
}
impl radio::Register for CcCtrl0 {
    const ADDRESS: u8 = Register::CcCtrl0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CcCtrl1 {
    #[skip]
    pub __: B4,
    pub cc_band: B4,
}
impl radio::Register for CcCtrl1 {
    const ADDRESS: u8 = Register::CcCtrl1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxSyn {
    pub rx_pdt_dis: bool,
    #[skip]
    pub __: B3,
    pub rx_pdt_level: B4,
}
impl radio::Register for RxSyn {
    const ADDRESS: u8 = Register::RxSyn as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TrxRpc {
    pub rx_rpc_ctrl: B2,
    pub rx_rpc_en: bool,
    pub pdt_rpc_en: bool,
    pub pll_rpc_en: bool,
    pub xah_tx_rpc_en: bool,
    pub ipan_rpc_en: bool,
    #[skip]
    pub __: bool,
}
impl radio::Register for TrxRpc {
    const ADDRESS: u8 = Register::TrxRpc as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XahCtrl1 {
    pub aret_tx_ts_en: bool,
    #[skip]
    pub __1: bool,
    pub aack_fltr_res_ft: bool,
    pub aack_upld_res_ft: bool,
    #[skip]
    pub __2: bool,
    pub aack_ack_time: bool,
    pub aack_prom_mode: bool,
    pub aack_spc_en: bool,
}
impl radio::Register for XahCtrl1 {
    const ADDRESS: u8 = Register::XahCtrl1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FtnCtrl {
    pub ftn_start: bool,
    #[skip]
    __: bool,
    pub ftnv: B6,
}

impl radio::Register for FtnCtrl {
    const ADDRESS: u8 = Register::FtnCtrl as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XahCtrl2 {
    pub aret_frame_retries: B4,
    pub aret_csma_retries: B3,
    #[skip]
    pub __: bool,
}

impl radio::Register for XahCtrl2 {
    const ADDRESS: u8 = Register::XahCtrl2 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PllCf {
    pub pll_cf_start: bool,
    #[skip]
    __: B3,
    pub pll_cf: B4,
}

impl radio::Register for PllCf {
    const ADDRESS: u8 = Register::PllCf as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PartNum {
    pub part: Part,
}

impl radio::Register for PartNum {
    const ADDRESS: u8 = Register::PartNum as u8;
    type Word = u8;
    type Error = Infallible;
}


#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 8]
pub enum Part {
    None = 0x00,
    At86RF231 = 0x03,
    At86Rf233 = 0x0b,
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VersionNum {
    pub version: u8,
}

impl radio::Register for VersionNum {
    const ADDRESS: u8 = Register::VersionNum as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ManId0 {
    pub man_id_0: u8,
}

impl radio::Register for ManId0 {
    const ADDRESS: u8 = Register::ManId0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ManId1 {
    pub man_id_1: u8,
}

impl radio::Register for ManId1 {
    const ADDRESS: u8 = Register::ManId1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PllDcu {
    pub pll_dcu_start: bool,
    pub reserved: B7,
}

impl radio::Register for PllDcu {
    const ADDRESS: u8 = Register::PllDcu as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ShortAddr0 {
    pub short_addr0: u8,
}
impl radio::Register for ShortAddr0 {
    const ADDRESS: u8 = Register::ShortAddr0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ShortAddr1 {
    pub short_addr1: u8,
}
impl radio::Register for ShortAddr1 {
    const ADDRESS: u8 = Register::ShortAddr1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PanId0 {
    pub pan_id0: u8,
}
impl radio::Register for PanId0 {
    const ADDRESS: u8 = Register::PanId0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PanId1 {
    pub pan_id1: u8,
}
impl radio::Register for PanId1 {
    const ADDRESS: u8 = Register::PanId1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IeeeAddr0 {
    pub ieee_addr0: u8,
}
impl radio::Register for IeeeAddr0 {
    const ADDRESS: u8 = Register::IeeeAddr0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IeeeAddr1 {
    pub ieee_addr1: u8,
}
impl radio::Register for IeeeAddr1 {
    const ADDRESS: u8 = Register::IeeeAddr1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IeeeAddr2 {
    pub ieee_addr2: u8,
}
impl radio::Register for IeeeAddr2 {
    const ADDRESS: u8 = Register::IeeeAddr2 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IeeeAddr3 {
    pub ieee_addr3: u8,
}
impl radio::Register for IeeeAddr3 {
    const ADDRESS: u8 = Register::IeeeAddr3 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IeeeAddr4 {
    pub ieee_addr4: u8,
}
impl radio::Register for IeeeAddr4 {
    const ADDRESS: u8 = Register::IeeeAddr4 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IeeeAddr5 {
    pub ieee_addr5: u8,
}
impl radio::Register for IeeeAddr5 {
    const ADDRESS: u8 = Register::IeeeAddr5 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IeeeAddr6 {
    pub ieee_addr6: u8,
}
impl radio::Register for IeeeAddr6 {
    const ADDRESS: u8 = Register::IeeeAddr6 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IeeeAddr7 {
    pub ieee_addr7: u8,
}
impl radio::Register for IeeeAddr7 {
    const ADDRESS: u8 = Register::IeeeAddr7 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XahCtrl0 {
    pub xah_ctrl0: u8,
}
impl radio::Register for XahCtrl0 {
    const ADDRESS: u8 = Register::XahCtrl0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaSeed0 {
    pub csma_seed0: u8,
}
impl radio::Register for CsmaSeed0 {
    const ADDRESS: u8 = Register::CsmaSeed0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaSeed1 {
    pub csma_seed1: u8,
}
impl radio::Register for CsmaSeed1 {
    const ADDRESS: u8 = Register::CsmaSeed1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaBe {
    pub csma_be: u8,
}
impl radio::Register for CsmaBe {
    const ADDRESS: u8 = Register::CsmaBe as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TstCtrlDigi {
    pub tst_ctrl_digi: u8,
}
impl radio::Register for TstCtrlDigi {
    const ADDRESS: u8 = Register::TstCtrlDigi as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TstAgc {
    pub tst_agc: u8,
}
impl radio::Register for TstAgc {
    const ADDRESS: u8 = Register::TstAgc as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TstSdm {
   pub  tst_sdm: u8,
}
impl radio::Register for TstSdm {
    const ADDRESS: u8 = Register::TstSdm as u8;
    type Word = u8;
    type Error = Infallible;
}


bitflags::bitflags! {
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

bitflags::bitflags! {
    /// IRQ flags (alternate bitflag version)
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
impl From<u8> for Irqs {
    fn from(v: u8) -> Self {
        Self::from_bits_truncate(v)
    }
}
impl From<Irqs> for u8 {
    fn from(v: Irqs) -> u8 {
        v.bits()
    }
}
impl radio::Register for Irqs {
    const ADDRESS: u8 = Register::IrqStatus as u8;
    type Word = u8;
    type Error = Infallible;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_command_flags() {
        let commands = [
            (CommandFlags::REG_RD, 0b1000_0000),
            (CommandFlags::REG_WR, 0b1100_0000),
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
