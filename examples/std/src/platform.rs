//! Host platform glue for OpenThread's `heap-ext-ot` feature.
//!
//! With `heap-ext-ot`, OpenThread routes *its own* heap (and the MbedTLS
//! allocator it manages) through `otPlatCAlloc` / `otPlatFree` instead of its
//! internal fixed-size block allocator. On a 64-bit host that matters for
//! correctness, not just capacity: OpenThread's internal heap only aligns to
//! `sizeof(void*)` (8 bytes), but the `mbedtls-rs-sys` digest/cipher contexts
//! require 16-byte alignment. libc `calloc` returns 16-byte-aligned memory (as
//! the C standard's `malloc`/`calloc` guarantee for any standard type), so
//! forwarding to it satisfies MbedTLS.
//!
//! Included into each example bin via `#[path = "../platform.rs"] mod platform;`.

/// Zero-initialized allocation, forwarded to libc `calloc`.
///
/// # Safety
/// Called by OpenThread's C with a valid `(count, size)`; returns memory owned
/// by the caller until passed back to [`otPlatFree`].
#[no_mangle]
pub unsafe extern "C" fn otPlatCAlloc(count: usize, size: usize) -> *mut core::ffi::c_void {
    libc::calloc(count, size)
}

/// Free memory previously returned by [`otPlatCAlloc`], forwarded to libc `free`.
///
/// # Safety
/// `ptr` must be null or a pointer previously returned by [`otPlatCAlloc`].
#[no_mangle]
pub unsafe extern "C" fn otPlatFree(ptr: *mut core::ffi::c_void) {
    libc::free(ptr)
}
