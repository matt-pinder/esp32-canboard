
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "inc/config.h"
#include "inc/can.h"
#include "inc/inputs.h"

extern board_config_t board_cfg;

twai_handle_t twai_can;
twai_timing_config_t t_can_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = { .acceptance_code = 0xFFFFFFFF, .acceptance_mask = 0x00000000, .single_filter = true };
twai_general_config_t can_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM, TWAI_MODE_NORMAL);

/**
 * @brief Initialize and start TWAI/CAN driver with dynamic speed configuration
 * Reads CAN speed from board_config_t (125/250/500/1000 kbps) and applies appropriate timing.
 * Must be called before creating canTransmit task.
 * @return ESP_OK on success, ESP_FAIL on driver initialization error
 */
esp_err_t can_init(void) {
    // Select timing config based on board configuration
    if (board_cfg.can_speed_kbps == 125) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_125KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 125 kbps");
    } else if (board_cfg.can_speed_kbps == 250) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_250KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 250 kbps");
    } else if (board_cfg.can_speed_kbps == 1000) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_1MBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 1000 kbps");
    } else if (board_cfg.can_speed_kbps == 500) {
        // Default to 500 kbps (already configured above)
        ESP_LOGI(can_log, "CAN speed set to 500 kbps");
    } else {
        ESP_LOGW(can_log, "Invalid CAN speed %lu, defaulting to 500 kbps", board_cfg.can_speed_kbps);
        // t_can_config already defaults to 500 kbps
    }

    // Install TWAI driver
    esp_err_t err = twai_driver_install_v2(&can_config, &t_can_config, &f_config, &twai_can);
    if (err != ESP_OK) {
        ESP_LOGE(can_log, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    ESP_LOGI(can_log, "TWAI driver installed");

    // Start TWAI driver
    err = twai_start_v2(twai_can);
    if (err != ESP_OK) {
        ESP_LOGE(can_log, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    ESP_LOGI(can_log, "TWAI driver started");

    return ESP_OK;
}

/**
 * @brief Initialize and start TWAI/CAN driver with dynamic speed configuration
 * Reads CAN speed from board_config_t (125/250/500/1000 kbps) and applies appropriate timing.
 * Must be called before creating canTransmit task.
 * @return ESP_OK on success, ESP_FAIL on driver initialization error
 */
esp_err_t can_init(void) {
    // Select timing config based on board configuration
    if (board_cfg.can_speed_kbps == 125) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_125KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 125 kbps");
    } else if (board_cfg.can_speed_kbps == 250) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_250KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 250 kbps");
    } else if (board_cfg.can_speed_kbps == 1000) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_1MBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 1000 kbps");
    } else if (board_cfg.can_speed_kbps == 500) {
        // Default to 500 kbps
        ESP_LOGI(can_log, "CAN speed set to 500 kbps");
    } else {
        ESP_LOGW(can_log, "Invalid CAN speed %lu, defaulting to 500 kbps", board_cfg.can_speed_kbps);
        // t_can_config already defaults to 500 kbps
    }

    // Install TWAI driver
    esp_err_t err = twai_driver_install_v2(&can_config, &t_can_config, &f_config, &twai_can);
    if (err != ESP_OK) {
        ESP_LOGE(can_log, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    ESP_LOGI(can_log, "TWAI driver installed");

    // Start TWAI driver
    err = twai_start_v2(twai_can);
    if (err != ESP_OK) {
        ESP_LOGE(can_log, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    ESP_LOGI(can_log, "TWAI driver started");

    return ESP_OK;
}

/**
 * @brief FreeRTOS task that transmits sensor data over CAN bus.
 *
 * Continuously reads filtered voltage data from all ADC channels and transmits
 * them as 3 multiplexed CAN messages. Each message contains:
 * - Message 1 (can_start_id): CPU temperature, firmware revision, channels 0-2
 * - Message 2 (can_start_id+1): Channels 3-6
 * - Message 3 (can_start_id+2): Channels 7-9
 *
 * Voltages are encoded as uint16_t little-endian with 0.001V scale factor
 * (i.e., 2500 mV = 2.5V). CPU Temperature is signed int8_t in degrees Celsius.
 *
 * @param arg Unused (FreeRTOS task parameter)
 *
 * @note Runs in infinite loop until task is deleted.
 *       Protected access to filtered_voltages array via mutex.
 *       All CAN transmit failures are logged as warnings (non-fatal).
 *       Message timing: ~1-2ms per message, 20ms inter-cycle delay (total ~25ms = 40Hz capability)
 *       Actual transmission frequency: 20Hz (50ms cycle time)
 *       Firmware revision is sent in Message 1, Byte 1 for version tracking.
 *
 * @see filtered_voltages_mutex, getCpuTemperature(), FIRMWARE_REVISION
 */
void canTransmit(void *arg)
{
    ESP_LOGI(can_log, "CAN Transmit Task Started");
    extern const uint8_t FIRMWARE_REVISION;
    
    while(1) {
        uint16_t voltages_copy[NUM_ADC_CHANNELS];
        int8_t cpu_temp = 0;
        
        // Safely copy voltage data
        if (xSemaphoreTake(filtered_voltages_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(voltages_copy, filtered_voltages, sizeof(voltages_copy));
            xSemaphoreGive(filtered_voltages_mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        // Get CPU temperature
        cpu_temp = getCpuTemperature();
        
        // Message 1: analogVoltage_1
        twai_message_t msg1 = init_twai_message(board_cfg.can_start_id);
        msg1.data[0] = (uint8_t)cpu_temp;        // CPU Temperature 
        msg1.data[1] = FIRMWARE_REVISION;        // Firmware Revision 
        msg1.data[2] = voltages_copy[0] & 0xFF;  // Channel 0 
        msg1.data[3] = (voltages_copy[0] >> 8) & 0xFF;  
        msg1.data[4] = voltages_copy[1] & 0xFF;  // Channel 1 
        msg1.data[5] = (voltages_copy[1] >> 8) & 0xFF;  
        msg1.data[6] = voltages_copy[2] & 0xFF;  // Channel 2 
        msg1.data[7] = (voltages_copy[2] >> 8) & 0xFF;  
        
        esp_err_t err = twai_transmit(&msg1, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGW(can_log, "Failed to transmit analogVoltage_1: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // Message 2: analogVoltage_2
        twai_message_t msg2 = init_twai_message(board_cfg.can_start_id + 1);
        msg2.data[0] = voltages_copy[3] & 0xFF; // Channel 3
        msg2.data[1] = (voltages_copy[3] >> 8) & 0xFF;
        msg2.data[2] = voltages_copy[4] & 0xFF; // Channel 4
        msg2.data[3] = (voltages_copy[4] >> 8) & 0xFF;
        msg2.data[4] = voltages_copy[5] & 0xFF; // Channel 5
        msg2.data[5] = (voltages_copy[5] >> 8) & 0xFF;
        msg2.data[6] = voltages_copy[6] & 0xFF; // Channel 6
        msg2.data[7] = (voltages_copy[6] >> 8) & 0xFF;
        
        err = twai_transmit(&msg2, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGW(can_log, "Failed to transmit analogVoltage_2: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // Message 3: analogVoltage_3
        twai_message_t msg3 = init_twai_message(board_cfg.can_start_id + 2);
        msg3.data[0] = voltages_copy[7] & 0xFF; // Channel 7
        msg3.data[1] = (voltages_copy[7] >> 8) & 0xFF;
        msg3.data[2] = voltages_copy[8] & 0xFF; // Channel 8
        msg3.data[3] = (voltages_copy[8] >> 8) & 0xFF;
        msg3.data[4] = voltages_copy[9] & 0xFF; // Channel 9
        msg3.data[5] = (voltages_copy[9] >> 8) & 0xFF;
        msg3.data[6] = 0x00;  // Unused
        msg3.data[7] = 0x00;  // Unused
        
        err = twai_transmit(&msg3, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGW(can_log, "Failed to transmit analogVoltage_3: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}
