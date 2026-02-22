// See: <https://en.cppreference.com/w/c/header/stdarg.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define va_start(ap, param) __builtin_va_start(ap, param)
#define va_end(ap) __builtin_va_end(ap)
#define va_arg(ap, type) __builtin_va_arg(ap, type)

#define va_copy(dest, src) __builtin_va_copy(dest, src)

typedef __builtin_va_list va_list;

#ifdef __cplusplus
} // extern "C"
#endif
