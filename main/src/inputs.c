#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/temperature_sensor.h"
#include "inc/inputs.h"

#include <math.h>
#include <stdint.h>

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handles[ADC_CHANNEL_END + 1] = {NULL};

static temperature_sensor_handle_t tempSensor_handle = NULL;
static bool tempSensor_initialized = false;

SemaphoreHandle_t filtered_voltages_mutex;
SemaphoreHandle_t scaled_pressures_mutex;

volatile uint16_t scaled_pressures[4];
volatile uint16_t filtered_voltages[NUM_ADC_CHANNELS];

/**
 * @brief Initializes the CPU temperature sensor.
 *
 * This function installs and enables the temperature sensor if it hasn't been
 * initialized yet. It configures the sensor to operate within a temperature
 * range of 10°C to 50°C. If the initialization is successful, it marks the
 * sensor as initialized to prevent re-initialization.
 *
 * @return
 *      - ESP_OK on successful initialization
 *      - Error code from `temperature_sensor_install` or `temperature_sensor_enable`
 *        if initialization fails
 */

esp_err_t initCpuTempSensor(void){
    if (tempSensor_initialized) return ESP_OK;
    temperature_sensor_config_t config = { .range_min = 10, .range_max = 50 };

    esp_err_t err = temperature_sensor_install(&config, &tempSensor_handle);
    if (err != ESP_OK) return err;

    err = temperature_sensor_enable(tempSensor_handle);
    if (err != ESP_OK) return err;

    tempSensor_initialized = true;
    return ESP_OK;
}

/**
 * @brief Returns the current CPU temperature in degrees Celsius.
 *
 * If the CPU temperature sensor has not been initialized yet, this function
 * initializes it. If the initialization fails, -128 is returned.
 *
 * @return
 *      - The current CPU temperature in degrees Celsius
 *      - -128 if the initialization or readout fails
 */
int8_t getCpuTemperature(void){
    if (!tempSensor_initialized) {
        if(initCpuTempSensor() != ESP_OK) {
            return -128;
        }
    }
    float cpuTemp = -128;
    esp_err_t err = temperature_sensor_get_celsius(tempSensor_handle, &cpuTemp);
    if (err != ESP_OK) return -128;
    return (int8_t)cpuTemp;
}

/**
 * @brief Initializes the ADC channels for the ESP32.
 *
 * This function initializes the ADC unit and configures each channel to use a
 * 12-bit ADC with an attenuation of 12 dB. It then creates a calibration scheme
 * for each channel using the curve fitting method. The calibration handles are
 * stored in the `cali_handles` array.
 *
 * @note If a channel's configuration or calibration fails, the function will
 *       log a warning message and continue to the next channel.
 */
void initAdcChannels(void){
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    for (adc_channel_t ch = ADC_CHANNEL_START; ch <= ADC_CHANNEL_END; ch++) {
        adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
        esp_err_t err = adc_oneshot_config_channel(adc_handle, ch, &chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(adc_log, "Failed to Configure ADC Channel: %d (%s)", ch, esp_err_to_name(err));
            continue;
        } else {
            ESP_LOGI(adc_log, "Configured ADC Channel: %d", ch);
        }

        adc_cali_curve_fitting_config_t cali_cfg = { .unit_id = ADC_UNIT, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };

        adc_cali_handle_t cali_handle = NULL;
        esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);
        if (ret == ESP_OK) {
            cali_handles[ch] = cali_handle;
            ESP_LOGI(adc_log, "Calibration Created for ADC Channel: %d", ch);
        } else {
            ESP_LOGW(adc_log, "Failed to Create Calibration for ADC Channel: %d (%s)", ch, esp_err_to_name(ret));
        }
    }
}

/**
 * @brief Calculates the pressure from the given voltage, given min and max voltage
 *        and pressure values.
 *
 * The function first clamps the given voltage to the valid range, then calculates
 * the relative voltage to the total voltage span. This relative voltage is then
 * multiplied with the total pressure span to get the absolute pressure value.
 *
 * The result is then multiplied by 100 and returned as an integer.
 *
 * @param v_mv The measured voltage in millivolts
 * @param v_min_mv The minimum valid voltage in millivolts
 * @param v_max_mv The maximum valid voltage in millivolts
 * @param p_min The minimum pressure value
 * @param p_max The maximum pressure value
 * @return The calculated pressure value multiplied by 100
 */
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

/**
 * @brief Calculates the temperature from the given voltage, given pull-up resistor
 *        value and reference voltage.
 *
 * The function first clamps the given voltage to the valid range, then calculates
 * the NTC resistance from the given values. This resistance is then used to
 * interpolate the temperature from the pre-defined NTC table.
 *
 * The result is then clamped to the valid range and returned as an integer.
 *
 * @param v_mv The measured voltage in millivolts
 * @param r_pullup The pull-up resistor value in ohms
 * @param v_ref_mv The reference voltage in millivolts
 * @return The calculated temperature value in degrees Celsius (int8_t)
 */
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
            return temp;
        }
    }

    return (int8_t)-128;
}

/**
 * @brief Gets the scaled millivolts from the given ADC channel.
 *
 * If the 'scaled' parameter is true, the voltage is scaled to the range of 0 to 5000 mV.
 *
 * @param channel The ADC channel to read from
 * @param scaled Whether to scale the voltage to the range of 0 to 5000 mV
 * @return The scaled voltage in millivolts
 */
uint16_t getScaledMillivolts(adc_channel_t channel, bool scaled) {
    if (channel < ADC_CHANNEL_START || channel > ADC_CHANNEL_END) {
        ESP_LOGE(adc_log, "Requested ADC Channel %d Out of Range!", channel);
        return 0;
    }

    if (cali_handles[channel] == NULL) {
        ESP_LOGE(adc_log, "No Calibration Handle for ADC Channel: %d", channel);
        return 0;
    }

    int raw, voltage = 0;
    esp_err_t err = adc_oneshot_read(adc_handle, channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(adc_log, "ADC Read Failed on Channel: %d (%s)", channel, esp_err_to_name(err));
        return 0;
    }

    err = adc_cali_raw_to_voltage(cali_handles[channel], raw, &voltage);
    if (err != ESP_OK) {
        ESP_LOGE(adc_log, "Voltage Conversion Failed for ADC Channel: %d (%s)", channel, esp_err_to_name(err));
        return 0;
    }

    float v_input_mv = (scaled) ? voltage * 1.47f : (float)voltage;

    return (uint16_t)(v_input_mv);
}

/**
 * @brief A helper function for median filtering.
 *
 * This function takes an array of uint16_t samples and the number of samples
 * as input, and returns the median value of the samples. The median is
 * calculated by sorting the samples in ascending order and returning the
 * middle value.
 *
 * @param samples The array of uint16_t samples
 * @param count The number of samples
 * @return The median value of the samples
 */
uint16_t medianFilterHelper(uint16_t *samples, int count) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (samples[j] < samples[i]) {
                uint16_t tmp = samples[i];
                samples[i] = samples[j];
                samples[j] = tmp;
            }
        }
    }
    return samples[count / 2];
}

/**
 * @brief Processes the ADC values in the background.
 *
 * This function runs in its own task and continuously reads the ADC values
 * from all channels. It applies a median filter to the samples and stores the
 * filtered values in the filtered_voltages array.
 */
void adcProcess(void *arg) {
    ESP_LOGI(adc_log, "ADC Processing Task Started");
    uint16_t samples[FILTER_DEPTH];
    while (1) {
        for (int ch = ADC_CHANNEL_START; ch <= ADC_CHANNEL_END; ch++) {
            for (int i = 0; i < FILTER_DEPTH; i++) {
                samples[i] = getScaledMillivolts(ch, true); 
                vTaskDelay(pdMS_TO_TICKS(2)); 
            }

            uint16_t filtered = medianFilterHelper(samples, FILTER_DEPTH);
            if (xSemaphoreTake(filtered_voltages_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                filtered_voltages[ch] = filtered;
                xSemaphoreGive(filtered_voltages_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void pressureProcess(void *arg) {
    ESP_LOGI(adc_log, "Pressure Sensor Processing Task Started");
    while (1) {
        if (xSemaphoreTake(scaled_pressures_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {

            // Charge Cooler Inlet Pressure
            //scaled_pressures[0] = (scaled_voltages[0] > 0) ? getSensorPressure(scaled_voltages[0], 498, 4539, 50, 356) : 0; // kPa
            scaled_pressures[0] = (filtered_voltages[0] > 0) ? (uint16_t)((-2.502 * (filtered_voltages[0]) * (filtered_voltages[0]) + 72.145 * (filtered_voltages[0]) + 30.300) * 100.0f) : 0; // kPa
            // Exhaust Back Pressure (0-30psi)
            scaled_pressures[1] = (filtered_voltages[1] > 0) ? getSensorPressure(filtered_voltages[1], 500, 4500, 0, 30) : 0; // Psi
            // Crank Case Pressure (Bosch MAP 0261230119)
            scaled_pressures[2] = (filtered_voltages[2] > 0) ? getSensorPressure(filtered_voltages[2], 400, 4650, 20, 300) : 0; // kPa
            // Turbo Regulator Oil Pressure (0-150psi / 0-10bar)
            scaled_pressures[3] = (filtered_voltages[3] > 0) ? getSensorPressure(filtered_voltages[3], 500, 4500, 0, 6.89) : 0; // Bar

            xSemaphoreGive(scaled_pressures_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    vTaskDelete(NULL);
}