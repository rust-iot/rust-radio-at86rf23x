# Rust AT86RF23x radio driver

A rust driver and CLI for the [Microchip AT86RF23x]() family of 2.4GHz ISM band radio ICs.

## Status

WIP. Only tested on AT86RF233. Basic TX and RX working, pan/addr filtering and extended mode not yet implemented.

[![GitHub tag](https://img.shields.io/github/tag/rust-iot/rust-radio-at86rf23x.svg)](https://github.com/rust-iot/rust-radio-at86rf23x)
![Build Status](https://github.com/rust-iot/rust-radio-at86rf23x/workflows/Rust/badge.svg)
[![Crates.io](https://img.shields.io/crates/v/radio-at86rf23x.svg)](https://crates.io/crates/radio-at86rf23x)
[![Docs.rs](https://docs.rs/radio-at86rf23x/badge.svg)](https://docs.rs/radio-at86rf23x)
[![Snap Status](https://build.snapcraft.io/badge/rust-iot/rust-radio-at86rf23x.svg)](https://build.snapcraft.io/user/rust-iot/rust-radio-at86rf23x)

[Open Issues](https://github.com/rust-iot/rust-radio-at86rf23x/issues)

## Usage

Add to your project with `cargo add radio-at86rf23x`

Install the utility via one of the following methods:

- `cargo install radio-at86rf23x` to install from source
- `cargo binstall radio-at86rf23x` to install a pre-compiled binary via [cargo-binstall](https://github.com/ryankurte/cargo-binstall)
- Manually fetch the latest [release](https://github.com/rust-iot/rust-radio-at86rf23x/releases/)

## Useful Resources

- [AT86RF233 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8351-MCU_Wireless-AT86RF233_Datasheet.pdf)
- [AT86RF231 Datasheet](http://ww1.microchip.com/downloads/en/devicedoc/doc8111.pdf)
