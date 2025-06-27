#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/temperature_sensor.h"

#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL_START ADC_CHANNEL_0
#define ADC_CHANNEL_END ADC_CHANNEL_9

typedef enum {
    BOSCH_0280130039,
    BOSCH_0280130026
} ntc_sensor_model_t;

static const char *adc_log = "adc";

esp_err_t initCpuTempSensor(void);
int8_t getCpuTemperature(void);

void initAdcChannels(void);

extern uint16_t scaled_voltages[10];

int getSensorTemperature(int v_mv, float r_pullup, ntc_sensor_model_t model);
uint16_t getScaledMillivolts(adc_channel_t channel);