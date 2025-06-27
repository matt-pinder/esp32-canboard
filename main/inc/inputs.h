#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/temperature_sensor.h"

#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL_START ADC_CHANNEL_0
#define ADC_CHANNEL_END ADC_CHANNEL_9
#define PULLUP_VREF_MV 5025

static const char *adc_log = "adc";

typedef struct {
    int16_t temp_c;      // Temperature in °C
    int32_t resistance;  // Resistance in ohms
} ntc_point_t;

esp_err_t initCpuTempSensor(void);
int8_t getCpuTemperature(void);

void initAdcChannels(void);

extern uint16_t scaled_voltages[10];
extern uint16_t scaled_pressures[2];

int8_t getSensorTemperature(int v_mv, int r_pullup, int v_ref_mv);
uint16_t getSensorPressure(int v_mv, int v_min_mv, int v_max_mv, float p_min, float p_max);
uint16_t getScaledMillivolts(adc_channel_t channel, bool scaled);
