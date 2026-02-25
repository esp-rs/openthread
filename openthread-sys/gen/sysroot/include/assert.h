// See: <https://en.cppreference.com/w/c/header/assert.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// This is inspired by Clang's implementation of <assert.h>.
#ifdef NDEBUG
# define assert(...) ((void)0)
#else
// We just delegate to the `otPlatAssertFail` function, which will be provided
// by our platform implementation.
// TODO: Actually set OPENTHREAD_CONFIG_PLATFORM_ASSERT_MANAGEMENT to 1 so our
//       platform can handle this!
_Noreturn void otPlatAssertFail(const char *aFilename, int aLineNumber);
# define assert(...) ((__VA_ARGS__) ? ((void)0) : otPlatAssertFail(__FILE__, __LINE__))
#endif

#ifdef __cplusplus
} // extern "C"
#endif
