use num_enum::TryFromPrimitive;

use modular_bitfield::prelude::*;

#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive)]
#[repr(u8)]
pub enum Register {
    TrxStatus = 0x01,
    TrxCmd = 0x02,
    TrxCtrl0 = 0x03,
    TrxCtrl1 = 0x04,
    PhyTxPwr = 0x05,   // reserved reserved reserved TX_PWR 116
    PhyRssi = 0x06,    // RX_CRC_VALID RND_VALUE RSSI 98, 100, 151
    PhyEdLevel = 0x07, // ED_LEVEL 103
    PhyCcCca = 0x08,   // CCA_REQUEST CCA_MODE CHANNEL 107, 134
    CcaThres = 0x09,   // reserved CCA_ED_THRES 108
    RxCtrl = 0x0A,     // PEL_SHIFT_VALUE reserved reserved PDT_THRES 161, 179
    SfdValue = 0x0B,   // SFD_VALUE 171
    TrxCtrl2 = 0x0C,   // RX_SAFE_MODE reserved OQPSK_SCRAM_EN reserved OQPSK_DATA_RATE 156, 170
    AntDiv = 0x0D,     // ANT_SEL reserved ANT_DIV_EN ANT_EXT_SW_EN ANT_CTRL 161
    IrqMask = 0x0E,    // IRQ_MASK 33
    IrqStatus = 0x0F, // IRQ_7_BAT_LOW IRQ_6_TRX_UR IRQ_5_AMI IRQ_4_CCA_ED_DONE IRQ_3_TRX_END IRQ_2_RX_START  IRQ_1_PLL_UNLOCK IRQ_0_PLL_LOCK 33
    VregCtrl = 0x10,  // AVREG_EXT AVDD_OK reserved DVREG_EXT DVDD_OK reserved 122
    Batmon = 0x11,    // reserved reserved BATMON_OK BATMON_HR BATMON_VTH 125
    XoscCtrl = 0x12,  // XTAL_MODE XTAL_TRIM 130
    CcCtrl0 = 0x13,   // CC_NUMBER 135
    CcCtrl1 = 0x14,   // reserved CC_BAND 136
    RxSyn = 0x15,     // RX_PDT_DIS reserved RX_PDT_LEVEL 112, 157
    TrxRpc = 0x16, // RX_RPC_CTRL RX_RPC_EN PDT_RPC_EN PLL_RPC_EN XAH_TX_RPC_EN IPAN_RPC_EN reserved 175
    XahCtrl1 = 0x17, // ARET_TX_TS_EN reserved AACK_FLTR_RES_FT AACK_UPLD_RES_FT reserved AACK_ACK_TIME  AACK_PROM_MODE AACK_SPC_EN
    FtnCtrl = 0x18,  // FTN_START reserved FTNV 139
    XahCtrl2 = 0x19, // ARET_FRAME_RETRIES ARET_CSMA_RETRIES reserved 74
    PllCf = 0x1A,    // PLL_CF_START reserved reserved PLL_CF 137
    PllDcu = 0x1B,   // PLL_DCU_START reserved reserved 137
    PartNum = 0x1C,  // PART_NUM 27
    VersionNum = 0x1D, // VERSION_NUM 27
    ManId0 = 0x1E,   // MAN_ID_0 28
    ManId1 = 0x1F,   // MAN_ID_1 28
    ShortAddr0 = 0x20, // SHORT_ADDR_0 91
    ShortAddr1 = 0x21, // SHORT_ADDR_1 91
    PanId0 = 0x22,   // PAN_ID_0 92
    PanId1 = 0x23,   // PAN_ID_1 92
    IeeeAddr0 = 0x24, // IEEE_ADDR_0 92
    IeeeAddr1 = 0x25, // IEEE_ADDR_1 93
    IeeeAddr2 = 0x26, // IEEE_ADDR_2 93
    IeeeAddr3 = 0x27, // IEEE_ADDR_3 93
    IeeeAddr4 = 0x28, // IEEE_ADDR_4 94
    IeeeAddr5 = 0x29, // IEEE_ADDR_5 94
    IeeeAddr6 = 0x2A, // IEEE_ADDR_6 94
}

/// Register trait provides address for reading / writing as
/// well as conversions to/from u8 values
pub trait Reg: From<u8> + Into<u8> {
    const ADDRESS: Register;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TrxCtrl0 {
    pub tom_en: bool,
    pub reserved: bool,
    pub pmu_en: bool,
    pub pmu_if_inverse: bool,
    pub clkm_sha_sel: bool,
    pub clkm_ctrl: B3,
}

impl From<u8> for TrxCtrl0 {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}
impl From<TrxCtrl0> for u8 {
    fn from(v: TrxCtrl0) -> u8 {
        v.into_bytes()[0]
    }
}
impl Reg for TrxCtrl0 {
    const ADDRESS: Register = Register::TrxCtrl0;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TrxCtrl1 {
    pub pa_ext_en: bool,
    pub irq_2_ext_en: bool,
    pub tx_auto_crc_on: bool,
    pub rx_bl_ctrl: bool,
    pub spi_cmd_mode: B2,
    pub irq_mask_mode: bool,
    pub irq_polarity: bool,
}
impl From<u8> for TrxCtrl1 {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}
impl From<TrxCtrl1> for u8 {
    fn from(v: TrxCtrl1) -> u8 {
        v.into_bytes()[0]
    }
}
impl Reg for TrxCtrl1 {
    const ADDRESS: Register = Register::TrxCtrl1;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PhyTxPwr {
    pub reserved: B4,
    pub tx_pwr: B4,
}
impl From<u8> for PhyTxPwr {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}
impl From<PhyTxPwr> for u8 {
    fn from(v: PhyTxPwr) -> u8 {
        v.into_bytes()[0]
    }
}
impl Reg for PhyTxPwr {
    const ADDRESS: Register = Register::PhyTxPwr;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PhyRssi {
    pub rx_crc_valid: bool,
    pub rnd_value: B2,
    pub rssi: B5,
}
impl From<u8> for PhyRssi {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}
impl From<PhyRssi> for u8 {
    fn from(v: PhyRssi) -> u8 {
        v.into_bytes()[0]
    }
}
impl Reg for PhyRssi {
    const ADDRESS: Register = Register::PhyRssi;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PhyEdLevel {
    pub ed_level: u8,
}
impl From<u8> for PhyEdLevel {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<PhyEdLevel> for u8 {
    fn from(v: PhyEdLevel) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for PhyEdLevel {
    const ADDRESS: Register = Register::PhyEdLevel;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PhyCcCca {
    pub cca_request: bool,
    pub cca_mode: B2,
    pub channel: B5,
}

impl From<u8> for PhyCcCca {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<PhyCcCca> for u8 {
    fn from(v: PhyCcCca) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for PhyCcCca {
    const ADDRESS: Register = Register::PhyCcCca;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CcaThres {
    pub reserved: B4,
    pub cca_ed_thres: B4,
}
impl From<u8> for CcaThres {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<CcaThres> for u8 {
    fn from(v: CcaThres) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for CcaThres {
    const ADDRESS: Register = Register::CcaThres;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxCtrl {
    pub pel_shift_falue: B2,
    #[skip]
    pub __: B2,
    pub pdt_thresh: B4,
}
impl From<u8> for RxCtrl {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<RxCtrl> for u8 {
    fn from(v: RxCtrl) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for RxCtrl {
    const ADDRESS: Register = Register::RxCtrl;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SfdValue {
    pub sfd_value: u8,
}
impl From<u8> for SfdValue {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<SfdValue> for u8 {
    fn from(v: SfdValue) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for SfdValue {
    const ADDRESS: Register = Register::SfdValue;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TrxCtrl2 {
    pub rx_safe_mode: bool,
    #[skip]
    pub __1: bool,
    pub oqpsk_scram_en: bool,
    #[skip]
    pub __1: B2,
    pub oqpsk_data_rate: B3,
}
impl From<u8> for TrxCtrl2 {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<TrxCtrl2> for u8 {
    fn from(v: TrxCtrl2) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for TrxCtrl2 {
    const ADDRESS: Register = Register::TrxCtrl2;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AntDiv {
    pub ant_sel: B3,
    #[skip]
    pub __: bool,
    pub ant_div_en: bool,
    pub ant_ext_sw_en: bool,
    pub ant_ctrl: B2,
}
impl From<u8> for AntDiv {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<AntDiv> for u8 {
    fn from(v: AntDiv) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for AntDiv {
    const ADDRESS: Register = Register::AntDiv;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IrqMask {
    pub irq_mask: u8,
}
impl From<u8> for IrqMask {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<IrqMask> for u8 {
    fn from(v: IrqMask) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for IrqMask {
    const ADDRESS: Register = Register::IrqMask;
}
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
impl From<u8> for IrqStatus {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<IrqStatus> for u8 {
    fn from(v: IrqStatus) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for IrqStatus {
    const ADDRESS: Register = Register::IrqStatus;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
impl From<u8> for VregCtrl {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<VregCtrl> for u8 {
    fn from(v: VregCtrl) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for VregCtrl {
    const ADDRESS: Register = Register::VregCtrl;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Batmon {
    #[skip]
    pub __: B2,
    pub batmon_ok: bool,
    pub batmon_hr: bool,
    pub batmon_vth: B4,
}
impl From<u8> for Batmon {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<Batmon> for u8 {
    fn from(v: Batmon) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for Batmon {
    const ADDRESS: Register = Register::Batmon;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct XoscCtrl {
    pub xtal_mode: B4,
    pub xtal_trim: B4,
}
impl From<u8> for XoscCtrl {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<XoscCtrl> for u8 {
    fn from(v: XoscCtrl) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for XoscCtrl {
    const ADDRESS: Register = Register::XoscCtrl;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CcCtrl0 {
    pub cc_number: u8,
}
impl From<u8> for CcCtrl0 {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<CcCtrl0> for u8 {
    fn from(v: CcCtrl0) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for CcCtrl0 {
    const ADDRESS: Register = Register::CcCtrl0;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CcCtrl1 {
    #[skip]
    pub __: B4,
    pub cc_band: B4,
}
impl From<u8> for CcCtrl1 {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<CcCtrl1> for u8 {
    fn from(v: CcCtrl1) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for CcCtrl1 {
    const ADDRESS: Register = Register::CcCtrl1;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxSyn {
    pub rx_pdt_dis: bool,
    #[skip]
    pub __: B3,
    pub rx_pdt_level: B4,
}
impl From<u8> for RxSyn {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<RxSyn> for u8 {
    fn from(v: RxSyn) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for RxSyn {
    const ADDRESS: Register = Register::RxSyn;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TrxRpc {
    pub rx_rpc_ctrl: B2,
    pub rx_rpc_en: bool,
    pub pdt_rpc_en: bool,
    pub pll_rpc_en: bool,
    pub xah_tx_rpc_en: bool,
    pub ipan_rpc_en: bool,
    pub __: bool,
}
impl From<u8> for TrxRpc {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<TrxRpc> for u8 {
    fn from(v: TrxRpc) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for TrxRpc {
    const ADDRESS: Register = Register::TrxRpc;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
impl From<u8> for XahCtrl1 {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}

impl From<XahCtrl1> for u8 {
    fn from(v: XahCtrl1) -> u8 {
        v.into_bytes()[0]
    }
}

impl Reg for XahCtrl1 {
    const ADDRESS: Register = Register::XahCtrl1;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FtnCtrl {
    pub ftn_start: bool,
    #[skip]
    __: bool,
    pub ftnv: B6,
}
impl From<u8> for FtnCtrl {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}
impl From<FtnCtrl> for u8 {
    fn from(v: FtnCtrl) -> u8 {
        v.into_bytes()[0]
    }
}
impl Reg for FtnCtrl {
    const ADDRESS: Register = Register::FtnCtrl;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct XahCtrl2 {
    pub aret_frame_retries: B4,
    pub aret_csma_retries: B3,
    #[skip]
    pub __: bool,
}
impl From<u8> for XahCtrl2 {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}
impl From<XahCtrl2> for u8 {
    fn from(v: XahCtrl2) -> u8 {
        v.into_bytes()[0]
    }
}
impl Reg for XahCtrl2 {
    const ADDRESS: Register = Register::XahCtrl2;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PllCf {
    pub pll_cf_start: bool,
    #[skip]
    __: B3,
    pub pll_cf: B4,
}
impl From<u8> for PllCf {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}
impl From<PllCf> for u8 {
    fn from(v: PllCf) -> u8 {
        v.into_bytes()[0]
    }
}
impl Reg for PllCf {
    const ADDRESS: Register = Register::PllCf;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PllDcu {
    pub pll_dcu_start: bool,
    pub reserved: B7,
}
impl From<u8> for PllDcu {
    fn from(v: u8) -> Self {
        Self::from_bytes([v])
    }
}
impl From<PllDcu> for u8 {
    fn from(v: PllDcu) -> u8 {
        v.into_bytes()[0]
    }
}
impl Reg for PllDcu {
    const ADDRESS: Register = Register::PllDcu;
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
    /// IRQ flags (alternate version)
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
impl Reg for Irqs {
    const ADDRESS: Register = Register::IrqStatus;
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
