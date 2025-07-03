#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#include "driver/temperature_sensor.h"
#include "inc/inputs.h"

#include <math.h>
#include <stdint.h>

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handles[ADC_CHANNEL_END + 1] = {NULL};

static temperature_sensor_handle_t tempSensor_handle = NULL;
static bool tempSensor_initialized = false;

uint16_t scaled_voltages[10];
uint16_t scaled_pressures[4];

esp_err_t initCpuTempSensor(void){
    if(tempSensor_initialized) return ESP_OK;
    temperature_sensor_config_t config = { .range_min = 10, .range_max = 50 };

    esp_err_t err = temperature_sensor_install(&config, &tempSensor_handle);
    if(err != ESP_OK) return err;

    err = temperature_sensor_enable(tempSensor_handle);
    if(err != ESP_OK) return err;

    tempSensor_initialized = true;
    return ESP_OK;
}

int8_t getCpuTemperature(void){
    if(!tempSensor_initialized){
        if(initCpuTempSensor() != ESP_OK) {
            return -128;
        }
    }
    float cpuTemp = -128;
    esp_err_t err = temperature_sensor_get_celsius(tempSensor_handle, &cpuTemp);
    if (err != ESP_OK) return -128;
    return (int8_t)cpuTemp;
}

void initAdcChannels(void){
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    for(adc_channel_t ch = ADC_CHANNEL_START; ch <= ADC_CHANNEL_END; ch++){
        adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
        esp_err_t err = adc_oneshot_config_channel(adc_handle, ch, &chan_cfg);
        if(err != ESP_OK){
            ESP_LOGW(adc_log, "Failed to Configure ADC Channel: %d (%s)", ch, esp_err_to_name(err));
            continue;
        } else {
            ESP_LOGI(adc_log, "Configured ADC Channel: %d", ch);
        }

        adc_cali_curve_fitting_config_t cali_cfg = { .unit_id = ADC_UNIT, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };

        adc_cali_handle_t cali_handle = NULL;
        esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);
        if(ret == ESP_OK){
            cali_handles[ch] = cali_handle;
            ESP_LOGI(adc_log, "Calibration Created for ADC Channel: %d", ch);
        } else {
            ESP_LOGW(adc_log, "Failed to Create Calibration for ADC Channel: %d (%s)", ch, esp_err_to_name(ret));
        }
    }
}

uint16_t getSensorPressure(int v_mv, int v_min_mv, int v_max_mv, float p_min, float p_max)
{
    if (v_mv < v_min_mv) v_mv = v_min_mv;
    if (v_mv > v_max_mv) v_mv = v_max_mv;

    float voltage_span = v_max_mv - v_min_mv;
    float pressure_span = p_max - p_min;
    float relative_voltage = v_mv - v_min_mv;

    float pressure = p_min + (relative_voltage / voltage_span) * pressure_span;

    if(pressure < 0.0f) pressure = 0.0f; 
    return (uint16_t)(pressure * 100.0f);
}

int8_t getSensorTemperature(int v_mv, int r_pullup, int v_ref_mv)
{
    if (v_mv <= 0 || v_mv >= v_ref_mv || r_pullup <= 0 || v_ref_mv <= 0)
        return (int8_t)-128;

    float v_ntc = v_mv / 1000.0f;
    float v_ref = v_ref_mv / 1000.0f;
    float r_ntc_f = (r_pullup * v_ntc) / (v_ref - v_ntc);
    int32_t r_ntc = (int32_t)(r_ntc_f + 0.5f);

    if (r_ntc >= ntc_table[0].resistance) return ntc_table[0].temp_c;
    if (r_ntc <= ntc_table[sizeof(ntc_table) / sizeof(ntc_table[0]) - 1].resistance)
        return ntc_table[sizeof(ntc_table) / sizeof(ntc_table[0]) - 1].temp_c;

    for (int i = 0; i < sizeof(ntc_table) / sizeof(ntc_table[0]) - 1; i++) {
        int32_t r1 = ntc_table[i].resistance;
        int32_t r2 = ntc_table[i + 1].resistance;

        if (r_ntc <= r1 && r_ntc > r2) {
            int16_t t1 = ntc_table[i].temp_c;
            int16_t t2 = ntc_table[i + 1].temp_c;

            float frac = (float)(r_ntc - r2) / (r1 - r2);
            int8_t temp = (int8_t)(t2 + frac * (t1 - t2));
            return (temp <= 127) ? temp : 127;
        }
    }

    return (int8_t)-128;
}

uint16_t getScaledMillivolts(adc_channel_t channel, bool scaled){
    if(channel < ADC_CHANNEL_START || channel > ADC_CHANNEL_END){
        ESP_LOGE(adc_log, "Requested ADC Channel %d Out of Range!", channel);
        return 0;
    }

    if(cali_handles[channel] == NULL){
        ESP_LOGE(adc_log, "No Calibration Handle for ADC Channel: %d", channel);
        return 0;
    }

    int raw, voltage = 0;
    esp_err_t err = adc_oneshot_read(adc_handle, channel, &raw);
    if(err != ESP_OK){
        ESP_LOGE(adc_log, "ADC Read Failed on Channel: %d (%s)", channel, esp_err_to_name(err));
        return 0;
    }

    err = adc_cali_raw_to_voltage(cali_handles[channel], raw, &voltage);
    if(err != ESP_OK){
        ESP_LOGE(adc_log, "Voltage Conversion Failed for ADC Channel: %d (%s)", channel, esp_err_to_name(err));
        return 0;
    }

    // Scale to original voltage input based on divider of R1 = 4K7 and R2 = 10K (* 1.47) if TRUE.
    // Compensation for offset of pullup voltage is built into the NTC function.
    float v_input_mv;
    v_input_mv = (scaled) ? voltage * 1.47 : voltage; 
    
    return (int)v_input_mv;
}