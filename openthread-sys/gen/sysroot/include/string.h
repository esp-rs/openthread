// See: <https://en.cppreference.com/w/c/header/string.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Since <string.h> has to define `NULL` and `size_t`, let's re-export the
// contents of <stddef.h> here.
#include <stddef.h>

#define strcpy __builtin_strcpy
#define strncpy __builtin_strncpy

#define strlen __builtin_strlen
#define strcmp __builtin_strcmp
#define strncmp __builtin_strncmp
#define strchr __builtin_strchr
#define strrchr __builtin_strrchr
#define strstr __builtin_strstr

#define memcmp __builtin_memcmp
#define memset __builtin_memset
#define memcpy __builtin_memcpy
#define memmove __builtin_memmove

#ifdef __cplusplus
} // extern "C"
#endif
