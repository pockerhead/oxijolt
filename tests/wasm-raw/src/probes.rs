//! Probes of the module's runtime: a scratch buffer for the WASI checks, libc's stdout, and the
//! two allocators growing linear memory side by side.

use std::cell::{Cell, RefCell};
use std::ffi::{c_char, c_int, c_void};
use std::ptr::null_mut;

use core::arch::wasm32::memory_size;
use oxijolt_sys::{JPH_TempAllocator, JPH_TempAllocator_Create, JPH_TempAllocator_Destroy};

use crate::world;

/// Size of the scratch buffer in bytes.
const SCRATCH_BYTES: usize = 256;

/// Size of one Rust heap chunk in bytes.
const RUST_CHUNK_BYTES: usize = 1024 * 1024;

/// Most Rust chunks one `grow_rust_heap` call allocates.
const MAX_RUST_CHUNKS: usize = 64;

/// Size of one native temp allocator in bytes.
const NATIVE_ALLOCATOR_BYTES: u32 = 4 * 1024 * 1024;

/// Most native temp allocators one `grow_native_heap` call creates.
const MAX_NATIVE_ALLOCATORS: usize = 16;

thread_local! {
    static SCRATCH: Cell<*mut u8> = const { Cell::new(null_mut()) };
    static RUST_CHUNKS: RefCell<Vec<Vec<u8>>> = const { RefCell::new(Vec::new()) };
    static NATIVE_ALLOCATORS: RefCell<Vec<*mut JPH_TempAllocator>> = const { RefCell::new(Vec::new()) };
}

extern "C" {
    fn puts(s: *const c_char) -> c_int;
    fn fflush(stream: *mut c_void) -> c_int;
}

/// A zeroed 256-byte buffer, allocated on the first call and kept for the module's lifetime, in
/// which Node places WASI arguments and canaries.
#[no_mangle]
pub extern "C" fn scratch_buffer() -> *mut u8 {
    SCRATCH.with(|scratch| {
        if scratch.get().is_null() {
            scratch.set(Box::leak(Box::new([0u8; SCRATCH_BYTES])).as_mut_ptr());
        }
        scratch.get()
    })
}

/// Writes a line through wasi-libc's stdout and flushes it, which reaches `fd_fdstat_get` and
/// `fd_write`: 0, 1 if `puts` or `fflush` failed.
#[no_mangle]
pub extern "C" fn libc_stdout() -> i32 {
    // SAFETY: the string is nul-terminated and static; a null stream flushes every stream.
    let ok = unsafe { puts(c"oxijolt-sys wasm".as_ptr()) >= 0 && fflush(null_mut()) == 0 };
    if ok {
        0
    } else {
        1
    }
}

/// The byte at `byte` of Rust chunk `chunk`.
fn pattern_byte(chunk: usize, byte: usize) -> u8 {
    (chunk * 31 + byte) as u8
}

/// Allocates 1 MiB Rust chunks with a known pattern until linear memory grew: 0, 2 if 64 chunks
/// did not grow it.
#[no_mangle]
pub extern "C" fn grow_rust_heap() -> i32 {
    let pages_before = memory_size(0);
    RUST_CHUNKS.with_borrow_mut(|chunks| {
        for _ in 0..MAX_RUST_CHUNKS {
            let chunk = chunks.len();
            chunks.push(
                (0..RUST_CHUNK_BYTES)
                    .map(|byte| pattern_byte(chunk, byte))
                    .collect(),
            );
            if memory_size(0) > pages_before {
                return 0;
            }
        }
        2
    })
}

/// Creates 4 MiB joltc temp allocators (wasi-libc's `malloc`) until linear memory grew: 0, 2 if
/// 16 allocators did not grow it, 3 if joltc returned null. Call it after `create_world`, whose
/// `JPH_Init` installs Jolt's allocator hooks.
#[no_mangle]
pub extern "C" fn grow_native_heap() -> i32 {
    let pages_before = memory_size(0);
    NATIVE_ALLOCATORS.with_borrow_mut(|allocators| {
        for _ in 0..MAX_NATIVE_ALLOCATORS {
            // SAFETY: Jolt allocates the temp allocator through the hooks `JPH_Init` installed;
            // the runner calls `create_world` (which runs `JPH_Init`) before this export.
            let allocator = unsafe { JPH_TempAllocator_Create(NATIVE_ALLOCATOR_BYTES) };
            if allocator.is_null() {
                return 3;
            }
            allocators.push(allocator);
            if memory_size(0) > pages_before {
                return 0;
            }
        }
        2
    })
}

/// Steps the live world once with every retained temp allocator, then checks every Rust chunk's
/// pattern: 0, 1 without a world, 4 on an update error, 5 if a Rust chunk changed.
#[no_mangle]
pub extern "C" fn check_heaps() -> i32 {
    let updated = NATIVE_ALLOCATORS.with_borrow(|allocators| {
        allocators.iter().try_for_each(|&allocator| {
            // SAFETY: the allocator came from `JPH_TempAllocator_Create` and stays in the list
            // until `free_heaps` destroys it.
            unsafe { world::update_with(allocator) }
        })
    });
    if let Err(code) = updated {
        return code;
    }
    let intact = RUST_CHUNKS.with_borrow(|chunks| {
        chunks.iter().enumerate().all(|(chunk, bytes)| {
            bytes
                .iter()
                .enumerate()
                .all(|(byte, &value)| value == pattern_byte(chunk, byte))
        })
    });
    if intact {
        0
    } else {
        5
    }
}

/// Destroys every retained temp allocator and drops the Rust chunks: 0.
#[no_mangle]
pub extern "C" fn free_heaps() -> i32 {
    NATIVE_ALLOCATORS.with_borrow_mut(|allocators| {
        for allocator in allocators.drain(..) {
            // SAFETY: each allocator came from `JPH_TempAllocator_Create` and leaves the list
            // here, so it is destroyed once.
            unsafe { JPH_TempAllocator_Destroy(allocator) };
        }
    });
    RUST_CHUNKS.with_borrow_mut(Vec::clear);
    0
}
