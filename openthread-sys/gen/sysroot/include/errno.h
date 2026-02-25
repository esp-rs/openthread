// See: <https://en.cppreference.com/w/c/header/errno.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// HACK: By defining this we tell OpenThread's spinel.c that we don't provide
//       a `errno` macro. This makes our life slightly easier as long as we
//       don't actually use `errno` at runtime.
#define SPINEL_PLATFORM_DOESNT_IMPLEMENT_ERRNO_VAR 1

#define EPERM 1
#define ENOMEM 12
#define EINVAL 22
#define EPIPE 32
#define ERANGE 34
#define ENOBUFS 64
#define EOVERFLOW 75
#define EMSGSIZE 90
#define EAFNOSUPPORT 97
#define ENETDOWN 100
#define ENETUNREACH 101
#define ECONNABORTED 103
#define ECONNRESET 104
#define EISCONN 106
#define ENOTCONN 107
#define ETIMEDOUT 110
#define ECONNREFUSED 111
#define EHOSTDOWN 112
#define EHOSTUNREACH 113

#ifdef __cplusplus
} // extern "C"
#endif
