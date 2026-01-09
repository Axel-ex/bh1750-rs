#![no_std]

pub mod common;

#[cfg(feature = "blocking")]
pub mod blocking;

#[cfg(feature = "async")]
pub mod asynch;

#[cfg(feature = "blocking")]
pub use blocking::*;

#[cfg(feature = "async")]
pub use asynch::*;
