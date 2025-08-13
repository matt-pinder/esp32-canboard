#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "inc/can.h"
#include "inc/inputs.h"

twai_handle_t twai_can;

/**
 * @brief Initializes a TWAI message with the given identifier and 8 data bytes.
 *        All data bytes are initially set to 0.
 *
 * @param id The identifier of the message.
 *
 * @return The initialized TWAI message.
 */
twai_message_t init_twai_message(uint32_t id) {
    twai_message_t msg = { .identifier = id, .data_length_code = 8 };
    memset(msg.data, 0, sizeof(msg.data));
    return msg;
}

/**
 * @brief The CAN transmit task.
 *
 * This task is responsible for transmitting CAN messages to the bus. It
 * periodically reads the filtered and scaled ADC values and constructs
 * the CAN messages accordingly. The messages are then transmitted using
 * the TWAI driver.
 */
void canTransmit(void *arg)
{
    ESP_LOGI(can_log, "CAN Transmit Task Started");
    // Setup CAN Packets
    twai_message_t tx_msg[5];
    // for (size_t i = 0; i <= 1; ++i) {
    //     tx_msg[i] = init_twai_message(CAN_BASEID + i);
    // }
    tx_msg[0] = init_twai_message(CAN_BASEID);
    while(1) {
        
        uint16_t voltages_copy[NUM_ADC_CHANNELS];
        if (xSemaphoreTake(filtered_voltages_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(voltages_copy, filtered_voltages, sizeof(voltages_copy));
            xSemaphoreGive(filtered_voltages_mutex);
        }

        uint16_t pressures_copy[4];
        if (xSemaphoreTake(scaled_pressures_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(pressures_copy, scaled_pressures, sizeof(pressures_copy));
            xSemaphoreGive(scaled_pressures_mutex);
        }

        // Base Message
        tx_msg[0].data[0] =  voltages_copy[0] & 0xFF;
        tx_msg[0].data[1] = (voltages_copy[0] >> 8) & 0xFF;
        tx_msg[0].data[2] = 0;
        tx_msg[0].data[3] = 0;
        tx_msg[0].data[4] = 0;
        tx_msg[0].data[5] = 0;
        tx_msg[0].data[6] = 0;
        tx_msg[0].data[7] = 0;
        twai_transmit(&tx_msg[0], pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}
