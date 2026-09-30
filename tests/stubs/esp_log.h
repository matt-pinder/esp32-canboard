#pragma once

#include <stdarg.h>

static inline void test_esp_log(const char *tag, const char *fmt, ...)
{
    (void)tag;
    (void)fmt;
}

#define ESP_LOGI(...) test_esp_log(__VA_ARGS__)
#define ESP_LOGW(...) test_esp_log(__VA_ARGS__)
#define ESP_LOGE(...) test_esp_log(__VA_ARGS__)
