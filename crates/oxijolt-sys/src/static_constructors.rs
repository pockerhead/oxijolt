//! The once-only run of the module's C++ static constructors on wasm32-unknown-unknown.

use std::sync::Once;

extern "C" {
    /// Runs the module's C++ dynamic initialisers; wasm-ld synthesises it.
    fn __wasm_call_ctors();
}

/// joltc's `JPH_Init`, after the module's C++ static constructors have run once.
///
/// wasm-ld runs the constructors at the start of every export call unless some code references
/// `__wasm_call_ctors`; this function is that reference, so Jolt's dynamically initialised
/// statics, such as its fixed-to-world body, persist between export calls. Call it before any
/// other joltc function except `JPH_SetAssertFailureHandler`, `JPH_SetTraceHandler` and the
/// `*_SetProcs` functions. Nothing else in the module may call `__wasm_call_ctors`.
///
/// # Safety
/// As for joltc's `JPH_Init`: no other thread calls into joltc meanwhile.
#[allow(non_snake_case)]
pub unsafe extern "C" fn JPH_Init() -> bool {
    static CONSTRUCTORS: Once = Once::new();
    // SAFETY: wasm-ld defines `__wasm_call_ctors` in every module. `Once` makes this the
    // module's only call, and the crate documents that no other code calls it.
    CONSTRUCTORS.call_once(|| unsafe { __wasm_call_ctors() });
    // SAFETY: the caller upholds joltc's contract for `JPH_Init`.
    unsafe { crate::generated::JPH_Init() }
}
