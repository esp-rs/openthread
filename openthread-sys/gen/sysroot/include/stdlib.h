// See: <https://en.cppreference.com/w/c/header/stdlib.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stddef.h>

// Called by both mbedtls and OpenThread in case of assertion failures.
_Noreturn void exit(int status);

#ifdef __cplusplus
} // extern "C"
#endif
