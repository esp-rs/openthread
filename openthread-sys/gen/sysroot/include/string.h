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

// HACK:
// We need memset to be a function rather than a macro because mbedtls
// assigns it to a static variable. Clang doesn't allow built-in functions
// to be called through a function pointer, so we have to wrap it.
//
// This can be changed to the macro version once we compile against
// mbedtls-rs-sys instead of the vendored version included in OpenThread.
// See: <https://github.com/esp-rs/openthread/issues/61>
inline void* memset(void* s, int c, size_t n) {
  return __builtin_memset(s, c, n);
}

#define memcpy __builtin_memcpy
#define memmove __builtin_memmove

#ifdef __cplusplus
} // extern "C"
#endif
