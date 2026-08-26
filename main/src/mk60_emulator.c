#include "inc/mk60_emulator.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "inc/can.h"

#define MK60_TRIGGER_QUEUE_DEPTH 8U
#define MK60_REPORT_INTERVAL_MS 5000U

static const char *TAG = "MK60";
static QueueHandle_t trigger_queue;
static SemaphoreHandle_t config_mutex;
static mk60_emulator_config_t active_config;
static bool fault_latched;
static uint32_t trigger_count;
static uint32_t rejected_count;
static uint32_t queue_drop_count;
static uint32_t burst_count;
static uint32_t transmitted_count;
static uint32_t failure_count;

typedef struct {
    esp_err_t error;
    uint32_t sent_count;
    uint8_t failure_index;
} burst_runtime_t;

static void delay_response(uint8_t delay_ms, void *context)
{
    (void)context;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

static bool transmit_response(const mk60_response_frame_config_t *response,
                              uint8_t response_index,
                              void *context)
{
    burst_runtime_t *runtime = context;
    twai_message_t outgoing = {
        .identifier = response->identifier,
        .extd = 0,
        .rtr = 0,
        .data_length_code = response->data_length_code,
    };
    memcpy(outgoing.data, response->data, sizeof(outgoing.data));
    runtime->error = can_transmit_service_frame(&outgoing);
    if (runtime->error != ESP_OK) {
        runtime->failure_index = response_index;
        return false;
    }
    runtime->sent_count++;
    return true;
}

static void copy_config(mk60_emulator_config_t *destination)
{
    if (config_mutex != NULL && xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *destination = active_config;
        xSemaphoreGive(config_mutex);
    } else {
        memset(destination, 0, sizeof(*destination));
    }
}

void mk60_emulator_apply_config(const mk60_emulator_config_t *config)
{
    if (config == NULL || !mk60_emulator_profile_valid(config) || config_mutex == NULL) return;
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    active_config = *config;
    fault_latched = false;
    xSemaphoreGive(config_mutex);
    if (trigger_queue != NULL) xQueueReset(trigger_queue);
    ESP_LOGI(TAG, "Profile applied: enabled=%d trigger=0x%lX dlc=%u responses=%u",
             config->enabled, (unsigned long)config->trigger_id,
             (unsigned)config->trigger_dlc, (unsigned)config->response_count);
}

bool mk60_emulator_dispatch_frame(const twai_message_t *message)
{
    if (message == NULL || trigger_queue == NULL) return false;
    if (message->identifier != MK60_TRIGGER_ID) return true;

    mk60_emulator_config_t config;
    copy_config(&config);
    if (!config.enabled || message->identifier != config.trigger_id) return true;
    if (xQueueSend(trigger_queue, message, 0) != pdTRUE) {
        ++queue_drop_count;
        return false;
    }
    return true;
}

static void emulator_task(void *arg)
{
    (void)arg;
    TickType_t last_report = xTaskGetTickCount();
    while (true) {
        twai_message_t message = {0};
        if (xQueueReceive(trigger_queue, &message, pdMS_TO_TICKS(1000)) == pdTRUE) {
            mk60_emulator_config_t config;
            copy_config(&config);
            const mk60_received_frame_t received = {
                .identifier = message.identifier,
                .data_length_code = message.data_length_code,
                .extended = message.extd,
                .rtr = message.rtr,
            };
            if (!mk60_emulator_is_trigger(&config, &received)) {
                ++rejected_count;
            } else if (!fault_latched) {
                ++trigger_count;
                burst_runtime_t runtime = {.error = ESP_OK};
                bool burst_ok = mk60_emulator_run_burst(
                    &config, delay_response, transmit_response, &runtime);
                transmitted_count += runtime.sent_count;
                if (burst_ok) {
                    ++burst_count;
                } else {
                    ++failure_count;
                    fault_latched = true;
                    ESP_LOGE(TAG, "Service transmit failed at response %u: %s; emulator latched off",
                             (unsigned)runtime.failure_index, esp_err_to_name(runtime.error));
                }
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_report) >= pdMS_TO_TICKS(MK60_REPORT_INTERVAL_MS)) {
            mk60_emulator_config_t config;
            copy_config(&config);
            if (config.enabled || trigger_count || rejected_count || queue_drop_count || failure_count) {
                ESP_LOGI(TAG,
                         "status enabled=%d fault=%d triggers=%lu rejected=%lu queue_drops=%lu bursts=%lu tx=%lu failures=%lu",
                         config.enabled, fault_latched, (unsigned long)trigger_count,
                         (unsigned long)rejected_count, (unsigned long)queue_drop_count,
                         (unsigned long)burst_count, (unsigned long)transmitted_count,
                         (unsigned long)failure_count);
            }
            last_report = now;
        }
    }
}

esp_err_t mk60_emulator_start(const mk60_emulator_config_t *config)
{
    if (config == NULL || !mk60_emulator_profile_valid(config)) return ESP_ERR_INVALID_ARG;
    if (config_mutex == NULL) config_mutex = xSemaphoreCreateMutex();
    if (trigger_queue == NULL) trigger_queue = xQueueCreate(MK60_TRIGGER_QUEUE_DEPTH, sizeof(twai_message_t));
    if (config_mutex == NULL || trigger_queue == NULL) return ESP_ERR_NO_MEM;

    mk60_emulator_apply_config(config);
    BaseType_t result = xTaskCreatePinnedToCore(emulator_task, "mk60Emulator", 4096, NULL, 11, NULL, 0);
    return result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
