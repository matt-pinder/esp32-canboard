
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "inc/config.h"
#include "inc/can.h"
#include "inc/can_receive_dispatch.h"
#include "inc/inputs.h"
#include "inc/espnow_transport.h"
#include "inc/mk60_emulator.h"

extern board_config_t board_cfg;

twai_handle_t twai_can = NULL;
static bool can_driver_active = false;
static SemaphoreHandle_t can_driver_mutex = NULL;
twai_timing_config_t t_can_config = TWAI_TIMING_CONFIG_500KBITS();
/// Default filter rejects incoming messages unless CAN-to-ESP-NOW relay is enabled.
twai_filter_config_t f_config = { .acceptance_code = 0xFFFFFFFF, .acceptance_mask = 0x00000000, .single_filter = true };
twai_general_config_t can_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM, TWAI_MODE_NORMAL);

/**
 * @brief Initialize and start TWAI/CAN driver with dynamic speed configuration
 * Reads CAN speed from board_config_t (125/250/500/1000 kbps) and applies appropriate timing.
 * Must be called before creating canTransmit task.
 * @return ESP_OK on success, ESP_FAIL on driver initialization error
 */
esp_err_t can_init(void) {
    if (!board_cfg.can_enabled && !config_has_espnow_relay_client(&board_cfg) &&
        !board_cfg.mk60_emulator.enabled) {
        ESP_LOGI(can_log, "CAN transmission, relay, and MK60 emulator disabled; TWAI driver not started");
        return ESP_OK;
    }

    if (can_driver_mutex == NULL) {
        can_driver_mutex = xSemaphoreCreateMutex();
        if (can_driver_mutex == NULL) {
            ESP_LOGE(can_log, "Failed to create CAN driver mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    if (xSemaphoreTake(can_driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (can_driver_active) {
        xSemaphoreGive(can_driver_mutex);
        return ESP_OK;
    }

    // Select timing config based on board configuration
    if (board_cfg.can_speed_kbps == 125) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_125KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 125 kbps");
    } else if (board_cfg.can_speed_kbps == 250) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_250KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 250 kbps");
    } else if (board_cfg.can_speed_kbps == 1000) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_1MBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 1000 kbps");
    } else if (board_cfg.can_speed_kbps == 500) {
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_500KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
        ESP_LOGI(can_log, "CAN speed set to 500 kbps");
    } else {
        ESP_LOGW(can_log, "Invalid CAN speed %lu, defaulting to 500 kbps", board_cfg.can_speed_kbps);
        static const twai_timing_config_t temp_config = TWAI_TIMING_CONFIG_500KBITS();
        memcpy(&t_can_config, &temp_config, sizeof(twai_timing_config_t));
    }

    if (config_has_espnow_relay_client(&board_cfg) || board_cfg.mk60_emulator.enabled) {
        static const twai_filter_config_t accept_all = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        memcpy(&f_config, &accept_all, sizeof(f_config));
        can_config.rx_queue_len = 128;
        ESP_LOGI(can_log, "CAN receive filter enabled for dispatcher");
    } else {
        static const twai_filter_config_t reject_all = {
            .acceptance_code = 0xFFFFFFFF,
            .acceptance_mask = 0x00000000,
            .single_filter = true
        };
        memcpy(&f_config, &reject_all, sizeof(f_config));
        can_config.rx_queue_len = 5;
    }

    // Install TWAI driver
    esp_err_t err = twai_driver_install_v2(&can_config, &t_can_config, &f_config, &twai_can);
    if (err != ESP_OK) {
        ESP_LOGE(can_log, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        xSemaphoreGive(can_driver_mutex);
        return ESP_FAIL;
    }
    ESP_LOGI(can_log, "TWAI driver installed");

    // Start TWAI driver
    err = twai_start_v2(twai_can);
    if (err != ESP_OK) {
        ESP_LOGE(can_log, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        twai_driver_uninstall_v2(twai_can);
        twai_can = NULL;
        xSemaphoreGive(can_driver_mutex);
        return ESP_FAIL;
    }
    ESP_LOGI(can_log, "TWAI driver started");
    if (!board_cfg.can_enabled) {
        ESP_LOGI(can_log, "CAN sensor transmission disabled; requested CAN services remain active");
    }

    can_driver_active = true;
    xSemaphoreGive(can_driver_mutex);
    return ESP_OK;
}

esp_err_t can_transmit_service_frame(const twai_message_t *message) {
    if (message == NULL || can_driver_mutex == NULL) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(can_driver_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (can_driver_active && twai_can != NULL) {
        twai_status_info_t status = {0};
        err = twai_get_status_info_v2(twai_can, &status);
        if (err == ESP_OK && status.state == TWAI_STATE_RUNNING) {
            err = twai_transmit_v2(twai_can, message, pdMS_TO_TICKS(20));
        } else if (err == ESP_OK) {
            err = ESP_ERR_INVALID_STATE;
        }
    }
    xSemaphoreGive(can_driver_mutex);
    return err;
}

esp_err_t can_deinit(void) {
    if (can_driver_mutex == NULL) {
        can_driver_active = false;
        twai_can = NULL;
        return ESP_OK;
    }

    if (xSemaphoreTake(can_driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!can_driver_active || twai_can == NULL) {
        can_driver_active = false;
        twai_can = NULL;
        xSemaphoreGive(can_driver_mutex);
        return ESP_OK;
    }

    esp_err_t first_err = ESP_OK;
    esp_err_t err = twai_stop_v2(twai_can);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(can_log, "Failed to stop TWAI driver: %s", esp_err_to_name(err));
        first_err = err;
    }

    err = twai_driver_uninstall_v2(twai_can);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(can_log, "Failed to uninstall TWAI driver: %s", esp_err_to_name(err));
        if (first_err == ESP_OK) {
            first_err = err;
        }
    } else {
        can_driver_active = false;
        twai_can = NULL;
        ESP_LOGI(can_log, "TWAI driver stopped");
    }

    xSemaphoreGive(can_driver_mutex);
    return first_err;
}

void can_transmit_frame(const twai_message_t *message, const char *label) {
    static TickType_t last_espnow_warn = 0;

    if (board_cfg.can_enabled && can_driver_mutex != NULL &&
        xSemaphoreTake(can_driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (can_driver_active && twai_can != NULL) {
            esp_err_t err = twai_transmit_v2(twai_can, message, pdMS_TO_TICKS(1000));
            if (err != ESP_OK) {
                ESP_LOGW(can_log, "Failed to transmit %s via CAN: %s", label, esp_err_to_name(err));
            }
        }
        xSemaphoreGive(can_driver_mutex);
    }

    if (board_cfg.espnow_enabled) {
        esp_err_t err = espnow_transport_enqueue_twai(message);
        if (err != ESP_OK) {
            TickType_t now = xTaskGetTickCount();
            if (last_espnow_warn == 0 || (now - last_espnow_warn) >= pdMS_TO_TICKS(1000)) {
                ESP_LOGW(can_log, "Failed to transmit %s via ESP-NOW: %s", label, esp_err_to_name(err));
                last_espnow_warn = now;
            }
        }
    }
}

static bool dispatch_to_espnow(const void *frame, void *context)
{
    (void)context;
    return espnow_transport_enqueue_relay_twai(frame) == ESP_OK;
}

static bool dispatch_to_mk60(const void *frame, void *context)
{
    (void)context;
    return mk60_emulator_dispatch_frame(frame);
}

void canReceiveDispatch(void *arg)
{
    (void)arg;
    ESP_LOGI(can_log, "CAN receive dispatcher task started");
    ESP_LOGI(can_log, "CAN receive configuration: can_tx=%d espnow=%d relay=%d mk60=%d driver=%d",
             board_cfg.can_enabled,
             board_cfg.espnow_enabled,
             config_has_espnow_relay_client(&board_cfg),
             board_cfg.mk60_emulator.enabled,
             can_driver_active);

    bool first_frame_logged = false;
    uint32_t received_count = 0;
    uint32_t enqueue_error_count = 0;
    uint32_t emulator_error_count = 0;
    TickType_t last_status_log = xTaskGetTickCount();

    while (true) {
        if ((!config_has_espnow_relay_client(&board_cfg) && !board_cfg.mk60_emulator.enabled) ||
            can_driver_mutex == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        uint32_t drained = 0U;
        while (drained < 32U) {
            twai_message_t message = {0};
            esp_err_t receive_err = ESP_ERR_TIMEOUT;
            if (xSemaphoreTake(can_driver_mutex, 0) != pdTRUE) {
                break;
            }
            if (can_driver_active && twai_can != NULL) {
                receive_err = twai_receive_v2(twai_can, &message, 0);
            }
            xSemaphoreGive(can_driver_mutex);
            if (receive_err != ESP_OK) {
                break;
            }
            drained++;
            received_count++;
            if (!first_frame_logged) {
                ESP_LOGI(can_log, "CAN relay received first frame: id=0x%lX dlc=%u%s",
                         (unsigned long)message.identifier,
                         (unsigned)message.data_length_code,
                         message.extd ? " extended" : "");
                first_frame_logged = true;
            }
            can_receive_dispatch_result_t dispatch = can_receive_dispatch_fanout(
                &message,
                config_has_espnow_relay_client(&board_cfg), dispatch_to_espnow, NULL,
                board_cfg.mk60_emulator.enabled, dispatch_to_mk60, NULL);
            if (dispatch.relay_called && !dispatch.relay_ok) ++enqueue_error_count;
            if (dispatch.emulator_called && !dispatch.emulator_ok) ++emulator_error_count;
        }
        if (drained == 0U || drained == 32U) {
            vTaskDelay(1);
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_status_log) >= pdMS_TO_TICKS(5000)) {
            twai_status_info_t status = {0};
            bool have_status = false;
            if (xSemaphoreTake(can_driver_mutex, 0) == pdTRUE) {
                if (can_driver_active && twai_can != NULL &&
                    twai_get_status_info_v2(twai_can, &status) == ESP_OK) {
                    have_status = true;
                }
                xSemaphoreGive(can_driver_mutex);
            }

            if (have_status) {
                ESP_LOGI(can_log,
                         "CAN dispatcher status: rx=%lu relay_errors=%lu emulator_errors=%lu queued=%lu missed=%lu overrun=%lu state=%d",
                         (unsigned long)received_count,
                         (unsigned long)enqueue_error_count,
                         (unsigned long)emulator_error_count,
                         (unsigned long)status.msgs_to_rx,
                         (unsigned long)status.rx_missed_count,
                         (unsigned long)status.rx_overrun_count,
                         (int)status.state);
            } else {
                ESP_LOGI(can_log,
                         "CAN dispatcher status: rx=%lu relay_errors=%lu emulator_errors=%lu",
                         (unsigned long)received_count,
                         (unsigned long)enqueue_error_count,
                         (unsigned long)emulator_error_count);
            }
            last_status_log = now;
        }
    }
}

/**
 * @brief FreeRTOS task that transmits sensor data over CAN bus.
 *
 * Continuously reads filtered voltage data from all ADC channels and transmits
 * them as a set of CAN messages. New layout:
 * - Message 1 (can_start_id): inputs 0..3 (four uint16 LE)
 * - Message 2 (can_start_id+1): inputs 4..7 (four uint16 LE)
 * - Message 3 (can_start_id+2): inputs 8..9 (two uint16 LE in bytes 0..3)
 *   followed immediately by the first two dynamic signals in bytes 4..7.
 * Dynamic signals (10 total) are encoded based on channel configuration and
 * are placed into msg3 (first two), then msg4 (can_start_id+3, four signals),
 * then msg5 (can_start_id+4, final four signals). Each dynamic signal is 2
 * bytes (uint16/int16 LE) with encoding rules:
 *  - Raw: output 0 (uint16)
 *  - Pressure: unsigned uint16 = pressure_kPa * 100 (factor 0.01)
 *  - NTC: signed int16 = temperature_C * 1 (factor 1)
 *
 * Voltages are encoded as uint16_t little-endian with 0.001 V scale (i.e., mV
 * stored directly). Endianness: little-endian (LSB first) for multi-byte fields.
 *
 * @param arg Unused (FreeRTOS task parameter)
 *
 * @note Runs in infinite loop until task is deleted.
 *       Protected access to filtered_voltages array via mutex.
 *       All CAN transmit failures are logged as warnings (non-fatal).
 *       Message timing: ~1-2ms spacing between messages in a cycle.
 *       Overall loop cadence is configurable via board_cfg.can_tx_hz (25 or 50 Hz).
 */
void canTransmit(void *arg)
{
    ESP_LOGI(can_log, "Transmit Task Started");
    
    while(1) {
        TickType_t loop_start = xTaskGetTickCount();
        uint8_t can_tx_hz_snapshot = board_cfg.can_tx_hz;
        uint16_t voltages_copy[NUM_ADC_CHANNELS];
        
        // Safely copy voltage data
        if (xSemaphoreTake(filtered_voltages_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(voltages_copy, filtered_voltages, sizeof(voltages_copy));
            xSemaphoreGive(filtered_voltages_mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        // Message 1: inputs 0..3 (each uint16 LE)
        twai_message_t msg1 = init_twai_message(board_cfg.can_start_id);
        msg1.data[0] = voltages_copy[0] & 0xFF;  // input 0 LSB
        msg1.data[1] = (voltages_copy[0] >> 8) & 0xFF; // input 0 MSB
        msg1.data[2] = voltages_copy[1] & 0xFF;  // input 1 LSB
        msg1.data[3] = (voltages_copy[1] >> 8) & 0xFF; // input 1 MSB
        msg1.data[4] = voltages_copy[2] & 0xFF;  // input 2 LSB
        msg1.data[5] = (voltages_copy[2] >> 8) & 0xFF; // input 2 MSB
        msg1.data[6] = voltages_copy[3] & 0xFF;  // input 3 LSB
        msg1.data[7] = (voltages_copy[3] >> 8) & 0xFF; // input 3 MSB
        
        can_transmit_frame(&msg1, "analogVoltage_1");
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // Message 2: inputs 4..7 (each uint16 LE)
        twai_message_t msg2 = init_twai_message(board_cfg.can_start_id + 1);
        msg2.data[0] = voltages_copy[4] & 0xFF; // input 4 LSB
        msg2.data[1] = (voltages_copy[4] >> 8) & 0xFF; // input 4 MSB
        msg2.data[2] = voltages_copy[5] & 0xFF; // input 5 LSB
        msg2.data[3] = (voltages_copy[5] >> 8) & 0xFF; // input 5 MSB
        msg2.data[4] = voltages_copy[6] & 0xFF; // input 6 LSB
        msg2.data[5] = (voltages_copy[6] >> 8) & 0xFF; // input 6 MSB
        msg2.data[6] = voltages_copy[7] & 0xFF; // input 7 LSB
        msg2.data[7] = (voltages_copy[7] >> 8) & 0xFF; // input 7 MSB
        
        can_transmit_frame(&msg2, "analogVoltage_2");
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // Message 3: inputs 8..9 (each uint16 LE) and first two dynamic signals
        twai_message_t msg3 = init_twai_message(board_cfg.can_start_id + 2);
        msg3.data[0] = voltages_copy[8] & 0xFF; // input 8 LSB
        msg3.data[1] = (voltages_copy[8] >> 8) & 0xFF; // input 8 MSB
        msg3.data[2] = voltages_copy[9] & 0xFF; // input 9 LSB
        msg3.data[3] = (voltages_copy[9] >> 8) & 0xFF; // input 9 MSB

        // Prepare dynamic signals (10 signals, one per channel), encoded per config
        uint16_t dyn[10];
        for (int i = 0; i < 10; ++i) {
            if (board_cfg.channels[i].type == SENSOR_RAW) {
                dyn[i] = 0;
            } else if (board_cfg.channels[i].type == SENSOR_PRESSURE) {
                // getSensorPressure returns kPa * 100 (0.01 kPa resolution)
                uint16_t p = getSensorPressure(voltages_copy[i],
                                               board_cfg.channels[i].params.pressure.min_mv,
                                               board_cfg.channels[i].params.pressure.max_mv,
                                               board_cfg.channels[i].params.pressure.min_kpa,
                                               board_cfg.channels[i].params.pressure.max_kpa);
                dyn[i] = p;
            } else if (board_cfg.channels[i].type == SENSOR_NTC) {
                const ntc_table_def_t *t = ntc_get_table(board_cfg.channels[i].params.ntc.table_id);
                int8_t temp = getSensorTemperature(voltages_copy[i], board_cfg.channels[i].pullup_ohms, board_cfg.pullup_vref_mv,
                                                   t ? t->points : NULL, t ? t->points_count : 0);
                if (temp == (int8_t)-128) temp = 0;
                int16_t t16 = (int16_t)temp;
                dyn[i] = (uint16_t)((uint16_t)t16 & 0xFFFF);
            } else {
                dyn[i] = 0;
            }
        }

        // Place first two dynamic signals into msg3 bytes 4..7
        msg3.data[4] = dyn[0] & 0xFF;
        msg3.data[5] = (dyn[0] >> 8) & 0xFF;
        msg3.data[6] = dyn[1] & 0xFF;
        msg3.data[7] = (dyn[1] >> 8) & 0xFF;

        can_transmit_frame(&msg3, "analogVoltage_3/msg3");
        vTaskDelay(pdMS_TO_TICKS(1));

        // Message 4: dynamic signals 2..5 (four uint16)
        twai_message_t msg4 = init_twai_message(board_cfg.can_start_id + 3);
        msg4.data[0] = dyn[2] & 0xFF;
        msg4.data[1] = (dyn[2] >> 8) & 0xFF;
        msg4.data[2] = dyn[3] & 0xFF;
        msg4.data[3] = (dyn[3] >> 8) & 0xFF;
        msg4.data[4] = dyn[4] & 0xFF;
        msg4.data[5] = (dyn[4] >> 8) & 0xFF;
        msg4.data[6] = dyn[5] & 0xFF;
        msg4.data[7] = (dyn[5] >> 8) & 0xFF;

        can_transmit_frame(&msg4, "dynamic msg4");
        vTaskDelay(pdMS_TO_TICKS(1));

        // Message 5: dynamic signals 6..9 (four uint16)
        twai_message_t msg5 = init_twai_message(board_cfg.can_start_id + 4);
        msg5.data[0] = dyn[6] & 0xFF;
        msg5.data[1] = (dyn[6] >> 8) & 0xFF;
        msg5.data[2] = dyn[7] & 0xFF;
        msg5.data[3] = (dyn[7] >> 8) & 0xFF;
        msg5.data[4] = dyn[8] & 0xFF;
        msg5.data[5] = (dyn[8] >> 8) & 0xFF;
        msg5.data[6] = dyn[9] & 0xFF;
        msg5.data[7] = (dyn[9] >> 8) & 0xFF;

        can_transmit_frame(&msg5, "dynamic msg5");

        bool any_emub = false;
        uint8_t emub_bytes[8] = {0};
        for (int i = 0; i < 10; ++i) {
            if (board_cfg.channels[i].emub_tx > EMUB_TX_DISABLED && board_cfg.channels[i].emub_tx <= EMUB_TX_CAN_ANALOG_16) {
                uint32_t scaled = ((uint32_t)voltages_copy[i] * 5 + 49) / 98; // 19.6 mV per count
                if (scaled > 255) scaled = 255;
                emub_bytes[board_cfg.channels[i].emub_tx - 1] = (uint8_t)scaled;
                any_emub = true;
            }
        }

        if (any_emub) {
            twai_message_t emub_msg = init_twai_message(0x66B);
            memcpy(emub_msg.data, emub_bytes, sizeof(emub_bytes));
            can_transmit_frame(&emub_msg, "EMUB TX msg");
        }

        TickType_t target_period_ticks = pdMS_TO_TICKS((can_tx_hz_snapshot == 50) ? 20 : 40);
        TickType_t elapsed_ticks = xTaskGetTickCount() - loop_start;
        if (elapsed_ticks < target_period_ticks) {
            vTaskDelay(target_period_ticks - elapsed_ticks);
        } else {
            taskYIELD();
        }
    }
    vTaskDelete(NULL);
}
