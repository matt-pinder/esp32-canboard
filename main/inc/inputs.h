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
#define PULLUP_VREF_MV 3823
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
uint16_t getSensorPressure(int v_mv, int v_min_mv, int v_max_mv, float p_min, float p_max);
uint16_t getScaledMillivolts(adc_channel_t channel, bool scaled, float scaling_factor);
uint16_t medianFilterHelper(uint16_t *samples, int count);
void adcProcess(void *arg);
void pressureProcess(void *arg);

static const ntc_point_t ntc_table[] = { // Bosch 0280130026, Bosch 0280130039
    { -40, 45313 }, { -30, 26114 }, { -20, 15462 }, { -10,  9397 },
    {   0,  5896 }, {  10,  3792 }, {  20,  2500 }, {  30,  1707 },
    {  40,  1175 }, {  50,   834 }, {  60,   596 }, {  70,   436 },
    {  80,   323 }, {  90,   243 }, { 100,   187 }, { 110,   144 },
    { 120,   113 }, { 130,    89 }, { 140,    71 }
};

static const ntc_point_t tmap_table[] = { // BMW TMAP 13627843531
    { -40, 23615 }, { -30, 14436 }, { -20,  9175 }, { -10,  6035 },
    {   0,  4094 }, {  10,  2854 }, {  20,  2039 }, {  30,  1489 },
    {  40,  1110 }, {  50,   842 }, {  60,   650 }, {  70,   509 },
    {  80,   404 }, {  90,   325 }, { 100,   265 }, { 110,   218 },
    { 120,   181 }, { 130,   151 }, { 140,   128 },
};