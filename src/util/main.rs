

use log::{debug, trace, info, error};
use structopt::StructOpt;

use tracing_subscriber::FmtSubscriber;
use tracing_subscriber::filter::{LevelFilter, EnvFilter};

use driver_pal::hal::{HalInst, HalDelay, DeviceConfig};

use radio::*;
use radio_at86rf23x::prelude::*;

#[derive(Debug, PartialEq, Clone, StructOpt)]
pub enum Command {
    Info,
} 

#[derive(Debug, StructOpt)]
pub struct Options {
    #[structopt(subcommand)]
    pub command: Command,

    #[structopt(flatten)]
    pub spi_config: DeviceConfig,

    #[structopt(long, default_value = "info")]
    /// Configure radio log level
    pub log_level: LevelFilter,
}


fn main() -> anyhow::Result<()> {
    // Load options
    let opts = Options::from_args();

    // Initialise logging
    let filter = EnvFilter::from_default_env()
        .add_directive(opts.log_level.into());
    let _ = FmtSubscriber::builder()
        .with_env_filter(filter)
        .without_time()
        .try_init();

    debug!("Connecting to platform SPI");

    let HalInst{base: _, spi, pins} = match HalInst::load(&opts.spi_config) {
        Ok(v) => v,
        Err(e) => {
            return Err(anyhow::anyhow!("Error connecting to platform HAL: {:?}", e));
        }
    };

    debug!("Setting up radio device");

    // Build IO object
    let io = Io{
        spi,
        cs: pins.cs,
        rst: pins.reset,
        // TODO: add appropriate pins to driver-pal or work out
        // how to better genericise / instantiate this
        slp_tr: pins.led0,
        irq: pins.ready,
        //clkm: Option::<()>::None,
        //dig2: Option::<()>::None,
        delay: HalDelay{},
    };

    // Instantiate radio
    let mut _radio = match At86Rf23x::new(io) {
        Ok(r) => r,
        Err(e) => {
            return Err(anyhow::anyhow!("Failed to initialise radio: {:?}", e));
        }
    };

    todo!()
}