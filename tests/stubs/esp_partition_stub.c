#include "esp_partition.h"
#include <string.h>

#define TEST_PARTITION_SIZE 0x10000U
static uint8_t storage[TEST_PARTITION_SIZE];
static const esp_partition_t partition = {.size = TEST_PARTITION_SIZE};

void test_partition_reset(void) { memset(storage, 0xFF, sizeof(storage)); }
uint8_t *test_partition_data(void) { return storage; }
size_t test_partition_size(void) { return sizeof(storage); }

const esp_partition_t *esp_partition_find_first(int type, int subtype, const char *label)
{
    (void)type; (void)subtype;
    return label != NULL && strcmp(label, "rules") == 0 ? &partition : NULL;
}

esp_err_t esp_partition_read(const esp_partition_t *p, size_t offset, void *dst, size_t size)
{
    if (p != &partition || dst == NULL || offset + size > sizeof(storage)) return ESP_FAIL;
    memcpy(dst, storage + offset, size);
    return ESP_OK;
}

esp_err_t esp_partition_write(const esp_partition_t *p, size_t offset, const void *src, size_t size)
{
    if (p != &partition || src == NULL || offset + size > sizeof(storage)) return ESP_FAIL;
    memcpy(storage + offset, src, size);
    return ESP_OK;
}

esp_err_t esp_partition_erase_range(const esp_partition_t *p, size_t offset, size_t size)
{
    if (p != &partition || offset + size > sizeof(storage)) return ESP_FAIL;
    memset(storage + offset, 0xFF, size);
    return ESP_OK;
}
