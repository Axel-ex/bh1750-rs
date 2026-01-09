#![no_std]

mod common;

#[cfg(feature = "blocking")]
pub mod blocking;

#[cfg(feature = "async")]
pub mod asynch;

#[cfg(feature = "blocking")]
pub use blocking::*;

#[cfg(feature = "async")]
pub use asynch::*;
