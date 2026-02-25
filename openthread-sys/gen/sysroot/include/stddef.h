// See: <https://en.cppreference.com/w/c/header/stddef.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef __SIZE_TYPE__ size_t;

#ifdef __cplusplus
#define NULL __null
#else
#define NULL ((void*)0)
#endif

#define offsetof(t, d) __builtin_offsetof(t, d)

#ifdef __cplusplus
} // extern "C"
#endif
