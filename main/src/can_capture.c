#include "inc/can_capture.h"

#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define CAN_CAPTURE_QUEUE_DEPTH 512U

static QueueHandle_t capture_queue;
static portMUX_TYPE capture_mux = portMUX_INITIALIZER_UNLOCKED;
static bool capture_active;
static int64_t capture_start_us;
static can_capture_stats_t capture_stats;

esp_err_t can_capture_init(void)
{
    if (capture_queue != NULL) return ESP_OK;
    capture_queue = xQueueCreate(CAN_CAPTURE_QUEUE_DEPTH, sizeof(can_capture_record_t));
    return capture_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

bool can_capture_begin(void)
{
    if (capture_queue == NULL && can_capture_init() != ESP_OK) return false;

    portENTER_CRITICAL(&capture_mux);
    if (capture_active) {
        portEXIT_CRITICAL(&capture_mux);
        return false;
    }
    portEXIT_CRITICAL(&capture_mux);

    xQueueReset(capture_queue);
    portENTER_CRITICAL(&capture_mux);
    memset(&capture_stats, 0, sizeof(capture_stats));
    capture_start_us = esp_timer_get_time();
    capture_stats.start_uptime_ms = (uint32_t)(capture_start_us / 1000LL);
    capture_active = true;
    portEXIT_CRITICAL(&capture_mux);
    return true;
}

void can_capture_offer(const twai_message_t *message)
{
    if (message == NULL || capture_queue == NULL) return;

    int64_t start_us;
    portENTER_CRITICAL(&capture_mux);
    if (!capture_active) {
        portEXIT_CRITICAL(&capture_mux);
        return;
    }
    start_us = capture_start_us;
    capture_stats.frames_seen++;
    portEXIT_CRITICAL(&capture_mux);

    can_capture_record_t record = {
        .timestamp_us = (uint32_t)(esp_timer_get_time() - start_us),
        .identifier = message->identifier,
        .dlc = message->data_length_code <= 8U ? message->data_length_code : 8U,
        .extended = message->extd,
        .rtr = message->rtr,
    };
    memcpy(record.data, message->data, sizeof(record.data));

    if (xQueueSend(capture_queue, &record, 0) == pdTRUE) return;

    can_capture_record_t discarded;
    bool frame_lost = false;
    if (xQueueReceive(capture_queue, &discarded, 0) == pdTRUE) {
        frame_lost = true;
        (void)xQueueSend(capture_queue, &record, 0);
    } else if (xQueueSend(capture_queue, &record, 0) != pdTRUE) {
        frame_lost = true;
    }
    if (frame_lost) {
        portENTER_CRITICAL(&capture_mux);
        capture_stats.queue_drops++;
        portEXIT_CRITICAL(&capture_mux);
    }
}

bool can_capture_receive(can_capture_record_t *record, uint32_t wait_ms)
{
    if (record == NULL || capture_queue == NULL) return false;
    return xQueueReceive(capture_queue, record, pdMS_TO_TICKS(wait_ms)) == pdTRUE;
}

void can_capture_finish(can_capture_stats_t *stats)
{
    portENTER_CRITICAL(&capture_mux);
    capture_active = false;
    if (stats != NULL) *stats = capture_stats;
    portEXIT_CRITICAL(&capture_mux);
}

void can_capture_get_stats(can_capture_stats_t *stats)
{
    if (stats == NULL) return;
    portENTER_CRITICAL(&capture_mux);
    *stats = capture_stats;
    portEXIT_CRITICAL(&capture_mux);
}

bool can_capture_is_active(void)
{
    portENTER_CRITICAL(&capture_mux);
    const bool active = capture_active;
    portEXIT_CRITICAL(&capture_mux);
    return active;
}
