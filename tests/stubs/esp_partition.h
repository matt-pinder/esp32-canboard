#pragma once
#include <stddef.h>
#include <stdint.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_PARTITION_TYPE_DATA 1
#define ESP_PARTITION_SUBTYPE_ANY 0xFF

typedef struct {
    size_t size;
} esp_partition_t;

const esp_partition_t *esp_partition_find_first(int type, int subtype, const char *label);
esp_err_t esp_partition_read(const esp_partition_t *partition, size_t offset, void *dst, size_t size);
esp_err_t esp_partition_write(const esp_partition_t *partition, size_t offset, const void *src, size_t size);
esp_err_t esp_partition_erase_range(const esp_partition_t *partition, size_t offset, size_t size);

void test_partition_reset(void);
uint8_t *test_partition_data(void);
size_t test_partition_size(void);
