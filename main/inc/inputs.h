#pragma once

#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/temperature_sensor.h"

#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL_START ADC_CHANNEL_0
#define ADC_CHANNEL_END ADC_CHANNEL_9
#define NUM_ADC_CHANNELS (ADC_CHANNEL_END - ADC_CHANNEL_START + 1)
#define PULLUP_VREF_MV 5015
#define FILTER_DEPTH 5
#define NTC_TABLE_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static const char *adc_log = "adc";

typedef struct {
    int16_t temp_c;      // Temperature in °C
    int32_t resistance;  // Resistance in ohms
} ntc_point_t;

esp_err_t initCpuTempSensor(void);
int8_t getCpuTemperature(void);

void initAdcChannels(void);

extern SemaphoreHandle_t filtered_voltages_mutex;
extern SemaphoreHandle_t scaled_pressures_mutex;

extern volatile uint16_t scaled_pressures[4];
extern volatile uint16_t filtered_voltages[NUM_ADC_CHANNELS];

int8_t getSensorTemperature(int v_mv, int r_pullup, int v_ref_mv, const ntc_point_t *table, size_t table_size);
int8_t getSensorTemperatureEquation(int v_mv, int r_pullup, int v_ref_mv);
uint16_t getSensorPressure(int v_mv, int v_min_mv, int v_max_mv, float p_min, float p_max);
uint16_t getScaledMillivolts(adc_channel_t channel, bool scaled, float scaling_factor);
uint16_t medianFilterHelper(uint16_t *samples, int count);
void adcProcess(void *arg);
void pressureProcess(void *arg);

// static const ntc_point_t ntc_table[] = { // Bosch 0280130026, Bosch 0280130039
//     { -20, 52000 }, { -4, 47000  }, {   2, 37600  }, {  10,  13500 },
//     {  21, 9400  }, {  31,  5471 }, {  40, 3600  }, 
//     {  50, 2400 },
//     {  60,  1612 }, {  71, 1230 }, {  80, 960 }, {  92, 687 },
//     { 100,  536}, { 110,  200 }, { 120,   201 }, { 130,    202 }, { 140,    203 }
// };

static const ntc_point_t ntc_table[] = {
    { -20, 25317 }, { -4, 22855 }, {  2, 18144 }, { 10,  8233 },
    { 15, 5847 }, { 21, 4376 }, { 31, 2549 }, { 42, 1656 },
    { 50, 1100 }, { 60,  789 }, { 71,  552 }, { 80,  429 },
    { 92,  305 }, {100,  240 }, {109, 198 }, {150,  76 }
};

static const ntc_point_t tmap_table[] = { // BMW TMAP 13627843531
    { -40, 23615 }, { -30, 14436 }, { -20,  9175 }, { -10,  6035 },
    {   0,  4094 }, {  10,  2854 }, {  20,  2039 }, {  30,  1489 },
    {  40,  1110 }, {  50,   842 }, {  60,   650 }, {  70,   509 },
    {  80,   404 }, {  90,   325 }, { 100,   265 }, { 110,   218 },
    { 120,   181 }, { 130,   151 }, { 140,   128 },
};


// INPUT 1 filtered voltage 0 - TEMP SENSOR
