use argh::FromArgs;

fn default_spi_speed_mhz() -> u32 {
    12
}

#[derive(FromArgs)]
/// Agent for `tc2-firmware` to output thermal camera frames to either a local unix domain socket,
/// or a remote viewer application via wifi.
pub struct ModeConfig {
    /// output frames over wifi to `tc2-frames` desktop application
    #[argh(switch)]
    pub(crate) use_wifi: bool,

    /// raspberry pi SPI speed in Mhz, defaults to 12
    #[argh(option, default = "default_spi_speed_mhz()")]
    pub(crate) spi_speed_mhz: u32,
}
