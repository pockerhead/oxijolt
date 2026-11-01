//! Definitions of the WASI functions wasi-libc imports, so the module needs no WASI host.
//!
//! wasi-libc, linked for Jolt's C++ runtime, imports these seven `wasi_snapshot_preview1`
//! functions under the symbols `__imported_wasi_snapshot_preview1_<name>`. Defining the symbols
//! here replaces the imports. The behaviour of each function is listed in the crate docs. The
//! signatures lower to the import's wasm types: `i32` everywhere except the `i64` offset and
//! precision.

use core::arch::wasm32::unreachable;
use core::mem::{align_of, offset_of, size_of};

/// `__WASI_ERRNO_BADF` of `wasi/wasip1.h`: bad file descriptor.
const ERRNO_BADF: i32 = 8;

/// `__WASI_ERRNO_INVAL` of `wasi/wasip1.h`: invalid argument.
const ERRNO_INVAL: i32 = 28;

/// `__WASI_ERRNO_SPIPE` of `wasi/wasip1.h`: invalid seek.
const ERRNO_SPIPE: i32 = 70;

/// File descriptor of standard output.
const STDOUT: i32 = 1;

/// File descriptor of standard error.
const STDERR: i32 = 2;

/// WASI's `__wasi_ciovec_t`: one buffer of a gathered write.
#[repr(C)]
struct Ciovec {
    /// Start of the buffer. Never read: written bytes are discarded.
    _buf: *const u8,
    /// Length of the buffer in bytes.
    buf_len: usize,
}

// `wasi/wasip1.h` asserts the same layout for `__wasi_ciovec_t`.
const _: () = {
    assert!(size_of::<Ciovec>() == 8);
    assert!(align_of::<Ciovec>() == 4);
    assert!(offset_of!(Ciovec, _buf) == 0);
    assert!(offset_of!(Ciovec, buf_len) == 4);
};

/// Discards what is written to standard output and standard error and reports it written.
///
/// Returns `ERRNO_INVAL` without writing `nwritten` when the lengths overflow `usize`, and
/// `ERRNO_BADF` without writing it for any other file descriptor.
///
/// # Safety
/// For fd 1 and 2, `iovs` points to `iovs_len` valid, aligned `Ciovec`s (unless `iovs_len` is
/// 0) and `nwritten` is valid for an aligned write, as wasi-libc passes them.
#[no_mangle]
unsafe extern "C" fn __imported_wasi_snapshot_preview1_fd_write(
    fd: i32,
    iovs: *const Ciovec,
    iovs_len: usize,
    nwritten: *mut usize,
) -> i32 {
    if fd != STDOUT && fd != STDERR {
        return ERRNO_BADF;
    }
    let iovs = if iovs_len == 0 {
        &[][..]
    } else {
        // SAFETY: the caller passes `iovs_len` valid, aligned entries at `iovs`.
        unsafe { core::slice::from_raw_parts(iovs, iovs_len) }
    };
    let Some(total) = iovs
        .iter()
        .try_fold(0usize, |total, iov| total.checked_add(iov.buf_len))
    else {
        return ERRNO_INVAL;
    };
    // SAFETY: the caller passes a `nwritten` valid for an aligned write.
    unsafe { nwritten.write(total) };
    0
}

/// Returns `ERRNO_BADF`: no file is ever opened.
#[no_mangle]
extern "C" fn __imported_wasi_snapshot_preview1_fd_close(_fd: i32) -> i32 {
    ERRNO_BADF
}

/// Returns `ERRNO_BADF` without writing `stat`, so libc treats standard output as not a terminal.
#[no_mangle]
extern "C" fn __imported_wasi_snapshot_preview1_fd_fdstat_get(_fd: i32, _stat: *mut u8) -> i32 {
    ERRNO_BADF
}

/// Returns `ERRNO_SPIPE` without writing `new_offset`: no descriptor can seek.
#[no_mangle]
extern "C" fn __imported_wasi_snapshot_preview1_fd_seek(
    _fd: i32,
    _offset: i64,
    _whence: i32,
    _new_offset: *mut u64,
) -> i32 {
    ERRNO_SPIPE
}

/// Returns 0: a single-threaded module has nothing to yield to.
#[no_mangle]
extern "C" fn __imported_wasi_snapshot_preview1_sched_yield() -> i32 {
    0
}

/// Traps: the simulation must not read a clock.
#[no_mangle]
extern "C" fn __imported_wasi_snapshot_preview1_clock_time_get(
    _id: i32,
    _precision: i64,
    _time: *mut u64,
) -> i32 {
    unreachable()
}

/// Traps: only sleeps reach it, and nothing can wake a single-threaded module.
#[no_mangle]
extern "C" fn __imported_wasi_snapshot_preview1_poll_oneoff(
    _subscriptions: *const u8,
    _events: *mut u8,
    _count: usize,
    _done: *mut usize,
) -> i32 {
    unreachable()
}
