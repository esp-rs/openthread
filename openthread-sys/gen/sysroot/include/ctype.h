// See: <https://en.cppreference.com/w/c/header/ctype.html>
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int iscntrl(int c);
int isprint(int c);
int isupper(int c);

#ifdef __cplusplus
} // extern "C"
#endif
