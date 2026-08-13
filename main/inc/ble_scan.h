#ifndef BLE_SCAN_H
#define BLE_SCAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_now.h"

#define BLE_SCAN_MAX_RESULTS 12
#define BLE_SCAN_NAME_LEN 32

typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint8_t addr_type;
    char name[BLE_SCAN_NAME_LEN];
    int8_t rssi;
    bool has_dragy_name;
    bool has_fd00_service;
    bool potential_dragy;
} ble_scan_result_t;

typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint8_t addr_type;
} ble_scan_target_t;

esp_err_t ble_scan_init(void);
esp_err_t ble_scan_dragy(uint32_t duration_ms,
                         ble_scan_result_t *results,
                         size_t max_results,
                         size_t *result_count);
esp_err_t ble_scan_find_target(const uint8_t mac[ESP_NOW_ETH_ALEN],
                               uint32_t duration_ms,
                               ble_scan_target_t *target);

#endif // BLE_SCAN_H
