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

void app_main(void)
{
    initCpuTempSensor();
    ESP_LOGI(adc_log, "Current CPU Temperature: %u°C", getCpuTemperature());

    initAdcChannels();
    for(int i = 0; i <= 9; i++){ // Display voltage readings on initial startup.
        ESP_LOGI(adc_log, "ADC %u, GPIO %u: %u mV", i, i+1, getScaledMillivolts(i, false));
        ESP_LOGI(adc_log, "ADC %u, GPIO %u: %u mV (Scaled 0-5v)", i, i+1, getScaledMillivolts(i, true));
    }

    if(twai_driver_install_v2(&can_config, &t_can_config, &f_config, &twai_can) == ESP_OK){
        ESP_LOGI(can_log, "TWAI Driver Installed");
        if(twai_start_v2(twai_can) == ESP_OK){
            ESP_LOGI(can_log, "TWAI Driver Started!");
        } else {
            ESP_LOGI(can_log, "Failed to Start TWAI Driver!");
            abort();
        }
    } else {
        ESP_LOGI(can_log, "Failed to Install TWAI Driver!");
        abort();
    }

    filtered_voltages_mutex = xSemaphoreCreateMutex();
    scaled_pressures_mutex = xSemaphoreCreateMutex();
    if (filtered_voltages_mutex == NULL || scaled_pressures_mutex == NULL) {
        ESP_LOGE(adc_log, "Failed to create mutexes! Aborting...");
        abort();
    }

    // Process ADCs on Core 1
    xTaskCreatePinnedToCore(adcProcess, "adcProcess", 4096, NULL, 5, NULL, 1);

    // Process Pressures on Core 0
    xTaskCreatePinnedToCore(pressureProcess, "pressureProcess", 4096, NULL, 5, NULL, 0);
   
    // Transmit CAN on Core 0
    xTaskCreatePinnedToCore(canTransmit, "canTransmit", 4096, NULL, 10, NULL, 0);
}
