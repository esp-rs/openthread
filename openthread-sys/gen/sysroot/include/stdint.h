#ifndef __STDINT_H__
#define __STDINT_H__

/*
 * Minimal freestanding stdint.h for this sysroot.
 *
 * Why we need this: the other headers here replace standard headers
 * with `__builtin_*` / `__INT*_TYPE__` stubs so the build does not
 * depend on the host libc. For stdint specifically, clang's own
 * resource-dir `stdint.h` would normally provide everything in
 * freestanding mode --- but only if libclang's resource dir is on
 * the include search path. On some CI runners (e.g. rs-matter's
 * integration suite) libclang fails to auto-discover its resource
 * dir, and bindgen errors out with "unknown type name 'uint32_t'"
 * etc. while parsing openthread's public headers.
 *
 * To keep things robust, we define the C99 surface openthread actually
 * uses ourselves, in terms of the `__INT*_TYPE__` / `__INT*_MAX__`
 * macros that clang predefines on every platform it targets. This
 * makes the build independent of clang's resource-dir lookup.
 */

typedef __INT8_TYPE__   int8_t;
typedef __INT16_TYPE__  int16_t;
typedef __INT32_TYPE__  int32_t;
typedef __INT64_TYPE__  int64_t;

typedef __UINT8_TYPE__  uint8_t;
typedef __UINT16_TYPE__ uint16_t;
typedef __UINT32_TYPE__ uint32_t;
typedef __UINT64_TYPE__ uint64_t;

typedef __INTPTR_TYPE__  intptr_t;
typedef __UINTPTR_TYPE__ uintptr_t;

#define INT8_MAX    __INT8_MAX__
#define INT16_MAX   __INT16_MAX__
#define INT32_MAX   __INT32_MAX__
#define INT64_MAX   __INT64_MAX__

#define INT8_MIN    (-INT8_MAX - 1)
#define INT16_MIN   (-INT16_MAX - 1)
#define INT32_MIN   (-INT32_MAX - 1)
#define INT64_MIN   (-INT64_MAX - 1)

#define UINT8_MAX   __UINT8_MAX__
#define UINT16_MAX  __UINT16_MAX__
#define UINT32_MAX  __UINT32_MAX__
#define UINT64_MAX  __UINT64_MAX__

#define UINTPTR_MAX __UINTPTR_MAX__
#define SIZE_MAX    __SIZE_MAX__

#endif
