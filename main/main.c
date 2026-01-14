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
#include "driver/twai.h"

#define USB_PRESENCE_DETECT GPIO_NUM_38

#include "inc/inputs.h"
#include "inc/can.h"

#include "inc/wifi_config.h"
#include "inc/config.h"

// Firmware revision
const uint8_t FIRMWARE_REVISION = 2;

// Global board configuration (shared with other modules)
board_config_t board_cfg;

void app_main(void)
{
    static const char *log_tag = "APP";
    
    // Load board config from SPIFFS (with CRC check)
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

    // Set CAN timing config based on board config
    twai_timing_config_t t_can_config = TWAI_TIMING_CONFIG_500KBITS();
    
    if (board_cfg.can_speed_kbps == 125) {
        // Reinitialize with the correct timing config
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_125KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(log_tag, "CAN speed set to 125 kbps");
    } else if (board_cfg.can_speed_kbps == 250) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_250KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(log_tag, "CAN speed set to 250 kbps");
    } else if (board_cfg.can_speed_kbps == 1000) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_1MBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(log_tag, "CAN speed set to 1000 kbps");
    } else if (board_cfg.can_speed_kbps == 500) {
        ESP_LOGI(log_tag, "CAN speed set to 500 kbps");
    } else {
        ESP_LOGW(log_tag, "Invalid CAN speed %lu, defaulting to 500 kbps", board_cfg.can_speed_kbps);
    }

    // Install and start TWAI (CAN) driver
    if (twai_driver_install_v2(&can_config, &t_can_config, &f_config, &twai_can) == ESP_OK) {
        ESP_LOGI(log_tag, "TWAI Driver Installed");
        if (twai_start_v2(twai_can) == ESP_OK) {
            ESP_LOGI(log_tag, "TWAI Driver Started!");
        } else {
            ESP_LOGE(log_tag, "Failed to Start TWAI Driver!");
            abort();
        }
    } else {
        ESP_LOGE(log_tag, "Failed to Install TWAI Driver!");
        abort();
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
    
    ESP_LOGI(log_tag, "System initialization complete");
}
