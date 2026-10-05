#pragma once
#include <stddef.h>
#include <string.h>
static inline size_t test_strlcpy(char *dst, const char *src, size_t size)
{
    const size_t length = strlen(src);
    if (size != 0U) {
        const size_t copy = length < size - 1U ? length : size - 1U;
        memcpy(dst, src, copy);
        dst[copy] = '\0';
    }
    return length;
}
#ifdef strlcpy
#undef strlcpy
#endif
#define strlcpy test_strlcpy
