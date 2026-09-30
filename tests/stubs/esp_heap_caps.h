#pragma once
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM 0x1U
#define MALLOC_CAP_8BIT 0x2U
static inline void *heap_caps_calloc(size_t count, size_t size, unsigned caps)
{
    (void)caps;
    return calloc(count, size);
}
