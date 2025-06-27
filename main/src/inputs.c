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
uint16_t scaled_pressures[2];

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

    if (pressure < 0.0f) pressure = 0.0f; 
    return (uint16_t)(pressure * 10.0f + 0.5f); // Scale for .x precision
}

int8_t getSensorTemperature(int v_mv, int r_pullup, int v_ref_mv, ntc_sensor_model_t model)
{
    if (v_mv <= 0 || v_mv >= v_ref_mv || r_pullup <= 0.0f || v_ref_mv <= 0)
        return (int8_t)-128;

    float v_ntc = v_mv / 1000.0f;
    float v_ref = v_ref_mv / 1000.0f;
    float r_ntc = (r_pullup * v_ntc) / (v_ref - v_ntc);

    float A, B, C;
    switch (model) {
        case BOSCH_0280130039:
            //A = 1.129148e-3f; B = 2.341250e-4f; C = 8.767410e-8f; break;
            A = 1.4051e-3f; B = 2.3696e-4f; C = 1.0199e-7f; break;
        case BOSCH_0280130026:
            //A = 1.132430e-3f; B = 2.351000e-4f; C = 8.775468e-8f; break;
            A = 1.4051e-3f; B = 2.3696e-4f; C = 1.0199e-7f; break;
        default:
            return (int8_t)-128;
    }

    float ln_r = logf(r_ntc);
    float temp_k = 1.0f / (A + B * ln_r + C * ln_r * ln_r * ln_r);
    return (int8_t)(temp_k - 273.15f);
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

    float v_input_mv;
    
    v_input_mv = (scaled) ? voltage * 1.47 : voltage; 
    
    // Scale to original voltage input based on divider of R1 = 4K7 and R2 = 10K (* 1.47).
    // Compensation for offset of pullup voltage is built into the NTC function.

    return (int)v_input_mv;
}