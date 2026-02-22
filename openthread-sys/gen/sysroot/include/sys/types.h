// See: <https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// This file only needs to exist for tcplp.

#include <stdint.h>

// Newlib defines `off_t` as `__SLONGWORD_TYPE`.
typedef long int off_t;

#ifdef __cplusplus
} // extern "C"
#endif
