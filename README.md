# BH1750 driver
[![Crates.io](https://img.shields.io/crates/d/bh1750.svg)](https://crates.io/crates/bh1750) [![Crates.io](https://img.shields.io/crates/v/bh1750.svg)](https://crates.io/crates/bh1750) [![Released API docs](https://docs.rs/bh1750/badge.svg)](https://docs.rs/bh1750)

A platform-agnostic, no_std compatible Rust driver for the BH1750 ambient light sensor.

The driver supports both blocking and async operation via the embedded-hal and
embedded-hal-async traits.

The I²C instruction set follows the BH1750 datasheet:
https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf

Raw sensor values are converted to lux, taking into account the selected resolution
mode and the measurement time register.

Features
--------

- blocking (default): blocking driver using embedded-hal
- async: async driver using embedded-hal-async

Usage
-----

Blocking example:

use bh1750::{BH1750, Resolution};

fn demo<I2C, D>(i2c: I2C, delay: D) -> Result<(), bh1750::BH1750Error<I2C::Error>>
where
    I2C: embedded_hal::i2c::I2c,
    D: embedded_hal::delay::DelayNs,
{
    let mut dev = BH1750::new(i2c, delay, false);
    let lux = dev.get_one_time_measurement(Resolution::High)?;
    let _ = lux;
    Ok(())
}

Async example:

use bh1750::{BH1750Async, Resolution};

async fn demo<I2C, D>(i2c: I2C, delay: D) -> Result<(), bh1750::BH1750Error<I2C::Error>>
where
    I2C: embedded_hal_async::i2c::I2c,
    D: embedded_hal_async::delay::DelayNs,
{
    let mut dev = BH1750Async::new(i2c, delay, false);
    let lux = dev.get_one_time_measurement(Resolution::High).await?;
    let _ = lux;
    Ok(())
}

Measurement modes
-----------------

- One-time measurement:
  get_one_time_measurement(...) triggers a one-shot measurement and waits for the
  typical conversion time before reading the result.

- Continuous measurement:
  start_continuous_measurement(...) starts continuous mode.
  get_current_measurement(...) reads the most recent register value and does
  not wait for a new conversion.

License
-------

Licensed under either of:
- Apache License, Version 2.0
- MIT license
