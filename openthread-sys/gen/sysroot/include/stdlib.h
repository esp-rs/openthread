// See: <https://en.cppreference.com/w/c/header/stdlib.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stddef.h>

// Called by both mbedtls and OpenThread in case of assertion failures.
_Noreturn void exit(int status);

// Called by the spinel library's factory-diag command parsing
// (`OT_DIAGNOSTIC` builds). Hosted targets get it from their libc;
// freestanding consumers link one in (e.g. `tinyrlibc`'s).
unsigned long int strtoul(const char *str, char **endptr, int base);

#ifdef __cplusplus
} // extern "C"
#endif
