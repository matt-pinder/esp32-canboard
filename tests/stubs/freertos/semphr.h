#pragma once
#include "FreeRTOS.h"
typedef void *SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t ticks)
{
    (void)mutex; (void)ticks; return pdTRUE;
}
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex)
{
    (void)mutex; return pdTRUE;
}
