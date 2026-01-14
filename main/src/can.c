
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
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
twai_general_config_t can_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM, TWAI_MODE_NORMAL);

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
 *       Message timing: ~15ms per message, 80ms inter-cycle delay (total ~130ms)
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
        vTaskDelay(pdMS_TO_TICKS(5));
        
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
        vTaskDelay(pdMS_TO_TICKS(5));
        
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
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    vTaskDelete(NULL);
}
