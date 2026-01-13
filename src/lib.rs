//! BH1750 ambient light sensor driver.
//!
//! Platform-agnostic and `no_std` compatible. Supports both blocking and async operation.
//!
//! ## Features
//! - `blocking` (default): blocking driver using `embedded-hal`
//! - `async`: async driver using `embedded-hal-async`
//!
//! ## Blocking example
//! ```no_run
//! use bh1750::{BH1750, Resolution};
//!
//! # fn demo<I2C, D>(i2c: I2C, delay: D) -> Result<(), bh1750::BH1750Error<I2C::Error>>
//! # where
//! #   I2C: embedded_hal::i2c::I2c,
//! #   D: embedded_hal::delay::DelayNs,
//! # {
//! let mut dev = BH1750::new(i2c, delay, false);
//! let lux = dev.get_one_time_measurement(Resolution::High)?;
//! let _ = lux;
//! # Ok(())
//! # }
//! ```
//!
//! ## Async example
//! ```no_run
//! use bh1750::{BH1750Async, Resolution};
//!
//! # async fn demo<I2C, D>(i2c: I2C, delay: D) -> Result<(), bh1750::BH1750Error<I2C::Error>>
//! # where
//! #   I2C: embedded_hal_async::i2c::I2c,
//! #   D: embedded_hal_async::delay::DelayNs,
//! # {
//! let mut dev = BH1750Async::new(i2c, delay, false);
//! let lux = dev.get_one_time_measurement(Resolution::High).await?;
//! let _ = lux;
//! # Ok(())
//! # }
//! ```
//!
//! ## Measurement modes
//! - `get_one_time_measurement(...)`: triggers a one-shot measurement and waits for the typical conversion time.
//! - `start_continuous_measurement(...)` + `get_current_measurement(...)`: starts continuous mode and reads the most recent register value
//!   (does not wait for a new conversion).
//!
#![no_std]

mod common;
pub use common::{BH1750Error, Resolution};

#[cfg(feature = "blocking")]
pub mod blocking;

#[cfg(feature = "async")]
pub mod asynch;

#[cfg(feature = "blocking")]
pub use blocking::*;

#[cfg(feature = "async")]
pub use asynch::*;
