// See: <https://en.cppreference.com/w/c/header/time.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Newlib defines `time_t` as `__SLONGWORD_TYPE`.
typedef long int time_t;

#ifdef __cplusplus
} // extern "C"
#endif
