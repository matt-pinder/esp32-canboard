#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"

#define USB_PRESENCE_DETECT GPIO_NUM_38

#include "inc/inputs.h"
#include "inc/can.h"
#include "inc/ble_scan.h"
#include "inc/dragy_gps.h"
#include "inc/espnow_transport.h"

#include "inc/wifi_config.h"
#include "inc/config.h"
#include "esp_spiffs.h"

// Global board configuration (shared with other modules)
board_config_t board_cfg;

void app_main(void)
{
    static const char *log_tag = "APP";
    
    // Mount the legacy/web SPIFFS without formatting. config_load() prefers the
    // dedicated NVS partition and imports /spiffs/config.bin only when NVS is empty.
    esp_err_t spiffs_ret = esp_vfs_spiffs_register(&(esp_vfs_spiffs_conf_t){
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    });
    if (spiffs_ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(log_tag, "SPIFFS already mounted");
    } else if (spiffs_ret != ESP_OK) {
        ESP_LOGW(log_tag, "Failed to mount SPIFFS: %s", esp_err_to_name(spiffs_ret));
    } else {
        ESP_LOGI(log_tag, "SPIFFS mounted for web assets and legacy config import");
    }
    if (!config_load(&board_cfg)) {
        ESP_LOGW(log_tag, "Config CRC invalid or not found, using defaults");
        config_set_defaults(&board_cfg);
        if (!config_save(&board_cfg)) {
            ESP_LOGE(log_tag, "Failed to save default config!");
        }
    } else {
        ESP_LOGI(log_tag, "Config loaded and CRC valid");
    }

    // Start WiFi config mode (AP + HTTP server with timeout)
    wifi_config_mode_start();

    if (ble_scan_init() != ESP_OK) {
        ESP_LOGW(log_tag, "BLE scan support is not active");
    }
    
    // Initialize CPU temperature sensor
    esp_err_t err = initCpuTempSensor();
    if (err != ESP_OK) {
        ESP_LOGW(log_tag, "Failed to initialize CPU temp sensor: %s", esp_err_to_name(err));
    } else {
        int8_t cpu_temp = getCpuTemperature();
        ESP_LOGI(log_tag, "Current CPU Temperature: %d°C", cpu_temp);
    }

    // Initialize ADC channels
    initAdcChannels();

    // Initialize enabled transports with dynamic settings from config.
    if (can_init() != ESP_OK) {
        ESP_LOGE(log_tag, "Failed to initialize CAN driver!");
        abort();
    }
    if (espnow_transport_apply_config() != ESP_OK) {
        ESP_LOGW(log_tag, "ESP-NOW transport is not active");
    }
    if (dragy_gps_start() != ESP_OK) {
        ESP_LOGW(log_tag, "Dragy GPS integration is not active");
    }

    // Create mutex for protecting filtered voltage array
    filtered_voltages_mutex = xSemaphoreCreateMutex();
    if (filtered_voltages_mutex == NULL) {
        ESP_LOGE(log_tag, "Failed to create voltage mutex! Aborting...");
        abort();
    }

    // Process ADCs on Core 1
    BaseType_t task_result = xTaskCreatePinnedToCore(
        adcProcess, "adcProcess", 8192, NULL, 5, NULL, 1);
    if (task_result != pdPASS) {
        ESP_LOGE(log_tag, "Failed to create ADC processing task!");
        abort();
    }

    // Transmit CAN on Core 0
    task_result = xTaskCreatePinnedToCore(
        canTransmit, "canTransmit", 8192, NULL, 10, NULL, 0);
    if (task_result != pdPASS) {
        ESP_LOGE(log_tag, "Failed to create CAN transmit task!");
        abort();
    }

    // Relay external CAN traffic below the priority of sensor and GPS publishing.
    task_result = xTaskCreatePinnedToCore(
        canRelayEspNow, "canRelayEspNow", 4096, NULL, 4, NULL, 0);
    if (task_result != pdPASS) {
        ESP_LOGE(log_tag, "Failed to create CAN relay task!");
        abort();
    }
    
    ESP_LOGI(log_tag, "System initialization complete");
}
