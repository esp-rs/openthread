// See: <https://en.cppreference.com/w/c/header/stdio.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stddef.h>

// mbedtls uses `FILE` for one of its function declarations.
// To ensure we're not actually using `FILE` at runtime we define it as an
// opaque struct.
typedef struct __forbidden_FILE FILE;

// The following two functions are defined by our `snprintf.c` file in `gen/support`.

int snprintf(char* s, size_t n, const char* format, ...);

int vsnprintf(char* s, size_t n, const char* format, va_list arg);

#ifdef __cplusplus
} // extern "C"
#endif
