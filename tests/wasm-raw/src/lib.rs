//! A raw `oxijolt-sys` module for wasm32-unknown-unknown, driven from Node by `run.mjs`.
//!
//! It checks the runtime contract of the raw layer on this target across export calls: a world
//! created in one call keeps stepping in the next ones, `JPH_Init` runs the C++ static
//! constructors once, the module imports nothing, the WASI definitions behave as documented, and
//! Rust's allocator and wasi-libc's `malloc` grow linear memory side by side. A single export
//! call would hide a constructor bug: wasm-ld reruns the constructors at the start of every call
//! unless the module calls them itself, so state only goes missing in the next call.
//!
//! Every export returns 0 on success and a nonzero code naming the failed check; the codes are
//! listed on each function. On other targets the crate is empty.

#![cfg(all(target_arch = "wasm32", target_os = "unknown"))]

mod probes;
mod world;
