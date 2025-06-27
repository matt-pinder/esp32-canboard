#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define USB_PRESENCE_DETECT GPIO_NUM_38

#include "inc/inputs.h"
#include "inc/can.h"

void app_main(void)
{
    initCpuTempSensor();
    ESP_LOGI(adc_log, "Current CPU Temperature: %u°C", getCpuTemperature());

    initAdcChannels();
    for(int i = 0; i <= 9; i++){
        ESP_LOGI(adc_log, "ADC %u, GPIO %u: %u mV", i, i+1, getScaledMillivolts(i));
    }

    if(twai_driver_install_v2(&can_config, &t_can_config, &f_config, &twai_can) == ESP_OK){
        ESP_LOGI(can_log, "TWAI Driver Installed");
        if(twai_start_v2(twai_can) == ESP_OK){
            ESP_LOGI(can_log, "TWAI Driver Started!");
        } else {
            ESP_LOGI(can_log, "Failed to Start TWAI Driver!");
            return;
        }
    } else {
        ESP_LOGI(can_log, "Failed to Install TWAI Driver!");
        return;
    }
   
    // Transmit CAN on Core 1
    xTaskCreatePinnedToCore(canTransmit, "canTransmit", 4096, NULL, 10, NULL, 1);
}
