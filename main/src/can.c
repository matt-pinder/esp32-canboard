#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "inc/can.h"
#include "inc/inputs.h"

twai_handle_t twai_can;

twai_timing_config_t t_can_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

twai_general_config_t can_config = { .controller_id = 0, .mode = TWAI_MODE_NORMAL, .tx_io = DRIVECAN_TX_GPIO_NUM, .rx_io = DRIVECAN_RX_GPIO_NUM,
                                   .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 64, .rx_queue_len = 64,
                                   .alerts_enabled = TWAI_ALERT_NONE, .clkout_divider = 0 };
             
twai_message_t init_twai_message(uint32_t id) {
    twai_message_t msg = { .identifier = id, .data_length_code = 8 };
    memset(msg.data, 0, sizeof(msg.data));
    return msg;
}

void canTransmit(void *arg)
{
    ESP_LOGI(can_log, "CAN Transmit Task Started");

    // Setup CAN Packets
    twai_message_t tx_msg[3];
    for(size_t i = 0; i <= 3; ++i){
        tx_msg[i] = init_twai_message(CAN_BASEID + i);
    }

    while(1){

        for(int i = 0; i <= 9; i++){
           scaled_voltages[i] = getScaledMillivolts(i, true);
        }

        // Charge Cooler Inlet Pressure
        scaled_pressures[0] = (getSensorPressure(scaled_voltages[0], 500, 4500, 0, 600) * 10); // Kpa
        // Exhaust Back Pressure
        scaled_pressures[1] = (getSensorPressure(scaled_voltages[1], 500, 4500, 0, 150) * 10); // PSI
        
        // Base Message
        tx_msg[0].data[0] = (int8_t) getCpuTemperature(); // CPU Temperature (-128C > +127C)
        tx_msg[0].data[1] = 0x00; // Unused / Spare
        tx_msg[0].data[2] = scaled_voltages[0] & 0xFF; // Analog Input 1 - Charge Cooler Inlet Pressure
        tx_msg[0].data[3] = (scaled_voltages[0] >> 8) & 0xFF;
        tx_msg[0].data[4] = scaled_voltages[1]; // Analog Input 2 - Exhaust Back Pressure
        tx_msg[0].data[5] = (scaled_voltages[1] >> 8) & 0xFF;
        tx_msg[0].data[6] = scaled_voltages[2]; // Analog Input 3
        tx_msg[0].data[7] = (scaled_voltages[2] >> 8) & 0xFF;
        twai_transmit(&tx_msg[0], pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(10));

        // BASE + 1
        tx_msg[1].data[0] = scaled_voltages[3]; // Analog Input 4
        tx_msg[1].data[1] = (scaled_voltages[3] >> 8) & 0xFF;; 
        tx_msg[1].data[2] = scaled_voltages[4]; // Analog Input 5
        tx_msg[1].data[3] = (scaled_voltages[4] >> 8) & 0xFF;;
        tx_msg[1].data[4] = scaled_voltages[5]; // Analog Input 6
        tx_msg[1].data[5] = (scaled_voltages[5] >> 8) & 0xFF;;
        tx_msg[1].data[6] = scaled_voltages[6]; // Analog Input 7
        tx_msg[1].data[7] = (scaled_voltages[6] >> 8) & 0xFF;;
        twai_transmit(&tx_msg[1], pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(10));

        // BASE + 2
        tx_msg[2].data[0] = scaled_voltages[7]; // Analog Input 8 - Charge Cooler Inlet Temperature
        tx_msg[2].data[1] = (scaled_voltages[7] >> 8) & 0xFF; 
        tx_msg[2].data[2] = scaled_voltages[8]; // Analog Input 9 - Charge Cooler Water Temperature
        tx_msg[2].data[3] = (scaled_voltages[8] >> 8) & 0xFF;
        tx_msg[2].data[4] = scaled_voltages[9]; // Analog Input 10 - Air Temperature
        tx_msg[2].data[5] = (scaled_voltages[9] >> 8) & 0xFF;
        twai_transmit(&tx_msg[2], pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // BASE + 3
        tx_msg[3].data[0] = getSensorTemperature(scaled_voltages[8], 2400, PULLUP_VREF_MV, BOSCH_0280130026); // Charge Cooler Water Temperature
        tx_msg[3].data[1] = getSensorTemperature(scaled_voltages[9], 2400, PULLUP_VREF_MV, BOSCH_0280130039); // Air Temperature
        tx_msg[3].data[2] = 0x00; // Charge Cooler Inlet Temperature
        tx_msg[3].data[3] = scaled_pressures[0]; // Charge Cooler Inlet Pressure (kPa)
        tx_msg[3].data[4] = (scaled_pressures[0] >> 8) & 0xFF;
        tx_msg[3].data[5] = scaled_pressures[1]; // Exhaust Back Pressure (PSI)
        tx_msg[3].data[6] = (scaled_pressures[1] >> 8) & 0xFF;
        twai_transmit(&tx_msg[3], pdMS_TO_TICKS(1000));

        vTaskDelay(pdMS_TO_TICKS(80)); // 10Hz
    }
    vTaskDelete(NULL);
}