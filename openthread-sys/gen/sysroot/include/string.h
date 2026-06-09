// See: <https://en.cppreference.com/w/c/header/string.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Since <string.h> has to define `NULL` and `size_t`, let's re-export the
// contents of <stddef.h> here.
#include <stddef.h>

#define strcpy __builtin_strcpy
#define strncpy __builtin_strncpy

#define strlen __builtin_strlen
#define strcmp __builtin_strcmp
#define strncmp __builtin_strncmp
#define strchr __builtin_strchr
#define strrchr __builtin_strrchr
#define strstr __builtin_strstr

#define memcmp __builtin_memcmp

// `memset` is *declared* but intentionally NOT defined here (and is NOT a
// `#define` to `__builtin_memset`). OpenThread's bundled MbedTLS takes its
// address (`platform_util.c`: `memset_func = memset`); a `#define memset
// __builtin_memset` would make that `&__builtin_memset`, which Clang rejects
// under `-fbuiltin` ("builtin functions must be directly called").
//
// A bare declaration sidesteps that: `&memset` references an ordinary external
// symbol (resolved at link time — by libc on std targets, by the `tinyrlibc`
// polyfill on `core`-only targets), while *direct* `memset(...)` calls are still
// lowered to the `__builtin_memset` fast path by `-fbuiltin`. Providing a body
// here would be a trap: under `-fbuiltin`, `__builtin_memset` with a runtime
// length lowers back to a *call to `memset`*, so any in-header definition risks
// recursing into itself (a `jmp memset` spin in `mbedtls_platform_zeroize`).
// Leaving it undefined avoids that class of bug entirely. The return type must
// be `void *` to match the real `memset` (and MbedTLS's `memset_func` pointer).
void *memset(void *s, int c, size_t n);

#define memcpy __builtin_memcpy
#define memmove __builtin_memmove

#ifdef __cplusplus
} // extern "C"
#endif
