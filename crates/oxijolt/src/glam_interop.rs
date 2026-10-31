//! `From` conversions between the math types and [glam](https://docs.rs/glam)'s, one module per
//! glam version (features `glam032` and `glam033`) over the same code in `conversions.rs`.
//!
//! Each conversion copies the fields, so a value converted there and back has the same bits. It
//! neither normalizes nor checks: the calls that take a value check it as usual.

// `conversions.rs` is loaded once per glam version on purpose, each time against another `glam`.
#![allow(clippy::duplicate_mod)]

#[cfg(feature = "glam032")]
mod glam032;
#[cfg(feature = "glam033")]
mod glam033;
