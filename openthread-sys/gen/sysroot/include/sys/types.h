// See: <https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// This file only needs to exist for tcplp.

#include <stdint.h>

#if defined(__has_include) && __has_include(<sys/_types/_off_t.h>)
// HACK: MbedTLS ends up including <sys/socket.h> whenever it's available.
//       That header ends up pulling in _off_t.h, which conflicts with our own
//       definition. Let's just use the system one if we can.
#include <sys/_types/_off_t.h>
#else
// Newlib defines `off_t` as `__SLONGWORD_TYPE`.
typedef long int off_t;
#endif

#ifdef __cplusplus
} // extern "C"
#endif
