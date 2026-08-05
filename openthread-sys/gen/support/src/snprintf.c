/**
 * `snprintf` / `vsnprintf` for OpenThread, backed by nanoprintf.
 *
 * OpenThread's own string formatting (and the CLI shim next door) funnel
 * through these two symbols, which the top-level `add_definitions` remaps to
 * `_support_snprintf` / `_support_vsnprintf` so hosted targets do not collide
 * with their libc.
 *
 * nanoprintf (`nanoprintf.h`, vendored at tag v0.7.0 from
 * https://github.com/charlesnicholson/nanoprintf, dual-licensed 0BSD /
 * Unlicense) is a single-header, malloc-free, conformance-tested printf
 * implementation. Configuration below: everything OpenThread's formatting
 * uses (field widths, precision, `hh`..`ll`/`z` length modifiers), no
 * floating point or binary conversions (OpenThread formats neither, and
 * dropping them keeps the code small for MCU targets).
 *
 * Both functions return standard C semantics: the length the fully-formatted
 * string would have, which may exceed `size` on truncation (callers clamp -
 * see e.g. `cli_shim.c`).
 */

#define NANOPRINTF_IMPLEMENTATION
#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_SMALL_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS 0

#include "nanoprintf.h"

int vsnprintf(char *restrict str, size_t size, const char *restrict fmt, va_list ap)
{
    return npf_vsnprintf(str, size, fmt, ap);
}

int snprintf(char *restrict str, size_t size, const char *restrict fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int result = npf_vsnprintf(str, size, fmt, ap);
    va_end(ap);
    return result;
}
