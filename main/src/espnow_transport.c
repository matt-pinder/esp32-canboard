#include "inc/espnow_transport.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "inc/config.h"
#include "inc/espnow_can_protocol.h"

extern board_config_t board_cfg;

#define ESPNOW_FRAME_QUEUE_DEPTH 128U
#define ESPNOW_BATCH_WINDOW_US 5000ULL
#define ESPNOW_STATUS_INTERVAL_MS 5000U
#define ESPNOW_BATCH_TASK_PRIORITY 8U
#define ESPNOW_BATCH_TASK_STACK 4096U
#define ESPNOW_NOTIFY_FRAME BIT0
#define ESPNOW_NOTIFY_FLUSH BIT1
#define ESPNOW_NOTIFY_STOP BIT2

typedef struct {
    espnow_can_frame_t frame;
    bool relayed;
} espnow_queued_frame_t;

typedef struct {
    uint32_t queued_frames;
    uint32_t batches;
    uint32_t sent_packets;
    uint32_t batched_frames;
    uint32_t max_batch_size;
    uint32_t queue_drops;
    uint32_t send_failures;
    uint64_t radio_busy_us;
} espnow_transport_stats_t;

static const char *TAG = "ESPNOW";
static volatile bool espnow_started;
static espnow_client_config_t active_clients[ESPNOW_MAX_CLIENTS];
static uint16_t active_sequences[ESPNOW_MAX_CLIENTS];
static uint8_t active_client_count;
static QueueHandle_t frame_queue;
static SemaphoreHandle_t transport_mutex;
static TaskHandle_t batch_task_handle;
static esp_timer_handle_t flush_timer;
static espnow_transport_stats_t transport_stats;
static portMUX_TYPE stats_mux = portMUX_INITIALIZER_UNLOCKED;

static bool mac_is_empty(const uint8_t mac[ESP_NOW_ETH_ALEN])
{
    const uint8_t zero[ESP_NOW_ETH_ALEN] = {0};
    return memcmp(mac, zero, ESP_NOW_ETH_ALEN) == 0;
}

static void increment_send_failure(void)
{
    portENTER_CRITICAL(&stats_mux);
    transport_stats.send_failures++;
    portEXIT_CRITICAL(&stats_mux);
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status != ESP_NOW_SEND_SUCCESS)
    {
        increment_send_failure();
    }
}

static void flush_timer_callback(void *arg)
{
    (void)arg;
    if (batch_task_handle != NULL)
    {
        xTaskNotify(batch_task_handle, ESPNOW_NOTIFY_FLUSH, eSetBits);
    }
}

static void log_transport_summary(TickType_t *last_log_tick, espnow_transport_stats_t *last_stats)
{
    const TickType_t now = xTaskGetTickCount();
    if ((now - *last_log_tick) < pdMS_TO_TICKS(ESPNOW_STATUS_INTERVAL_MS))
    {
        return;
    }

    espnow_transport_stats_t current;
    portENTER_CRITICAL(&stats_mux);
    current = transport_stats;
    transport_stats.max_batch_size = 0U;
    portEXIT_CRITICAL(&stats_mux);

    const uint32_t batches = current.batches - last_stats->batches;
    const uint32_t frames = current.batched_frames - last_stats->batched_frames;
    const uint32_t average_times_10 = batches == 0U ? 0U : ((frames * 10U) / batches);
    ESP_LOGI(TAG,
             "Transport 5s: clients=%u queued=%lu batches=%lu packets=%lu avg_batch=%lu.%lu max_batch=%lu queue_drops=%lu send_failures=%lu send_api_us=%llu depth=%u",
             (unsigned)active_client_count,
             (unsigned long)(current.queued_frames - last_stats->queued_frames),
             (unsigned long)batches,
             (unsigned long)(current.sent_packets - last_stats->sent_packets),
             (unsigned long)(average_times_10 / 10U),
             (unsigned long)(average_times_10 % 10U),
             (unsigned long)current.max_batch_size,
             (unsigned long)(current.queue_drops - last_stats->queue_drops),
             (unsigned long)(current.send_failures - last_stats->send_failures),
             (unsigned long long)(current.radio_busy_us - last_stats->radio_busy_us),
             frame_queue == NULL ? 0U : (unsigned)uxQueueMessagesWaiting(frame_queue));
    *last_stats = current;
    *last_log_tick = now;
}

static void batch_transmitter_task(void *arg)
{
    (void)arg;
    TickType_t last_log_tick = xTaskGetTickCount();
    espnow_transport_stats_t last_stats = {0};
    espnow_queued_frame_t queued_frames[ESPNOW_CAN_MAX_FRAMES];
    espnow_can_frame_t client_frames[ESPNOW_CAN_MAX_FRAMES];
    uint8_t packet[ESPNOW_CAN_MAX_PACKET_SIZE];

    while (true)
    {
        log_transport_summary(&last_log_tick, &last_stats);
        if (!espnow_started || frame_queue == NULL)
        {
            uint32_t notification = 0U;
            xTaskNotifyWait(0U, UINT32_MAX, &notification, pdMS_TO_TICKS(100));
            continue;
        }
        if (xQueueReceive(frame_queue, &queued_frames[0], pdMS_TO_TICKS(100)) != pdTRUE)
        {
            continue;
        }

        uint8_t frame_count = 1U;
        xTaskNotifyStateClear(NULL);
        esp_err_t timer_err = esp_timer_start_once(flush_timer, ESPNOW_BATCH_WINDOW_US);
        if (timer_err != ESP_OK)
        {
            ESP_LOGW(TAG, "Could not start batch timer: %s", esp_err_to_name(timer_err));
        }

        bool flush = timer_err != ESP_OK;
        while (!flush && frame_count < ESPNOW_CAN_MAX_FRAMES)
        {
            while (frame_count < ESPNOW_CAN_MAX_FRAMES &&
                   xQueueReceive(frame_queue, &queued_frames[frame_count], 0) == pdTRUE)
            {
                frame_count++;
            }
            if (frame_count >= ESPNOW_CAN_MAX_FRAMES)
            {
                break;
            }
            uint32_t notification = 0U;
            xTaskNotifyWait(0U, UINT32_MAX, &notification, portMAX_DELAY);
            flush = (notification & (ESPNOW_NOTIFY_FLUSH | ESPNOW_NOTIFY_STOP)) != 0U;
        }
        timer_err = esp_timer_stop(flush_timer);
        if (timer_err != ESP_OK && timer_err != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGW(TAG, "Could not stop batch timer: %s", esp_err_to_name(timer_err));
        }
        if (!espnow_started)
        {
            continue;
        }

        espnow_transport_stats_t stats_snapshot;
        portENTER_CRITICAL(&stats_mux);
        stats_snapshot = transport_stats;
        portEXIT_CRITICAL(&stats_mux);
        portENTER_CRITICAL(&stats_mux);
        transport_stats.batches++;
        transport_stats.batched_frames += frame_count;
        if (frame_count > transport_stats.max_batch_size)
        {
            transport_stats.max_batch_size = frame_count;
        }
        portEXIT_CRITICAL(&stats_mux);

        if (xSemaphoreTake(transport_mutex, portMAX_DELAY) != pdTRUE) continue;
        for (uint8_t client_index = 0; client_index < active_client_count && espnow_started; ++client_index) {
            uint8_t client_frame_count = 0U;
            for (uint8_t frame_index = 0; frame_index < frame_count; ++frame_index) {
                if (!queued_frames[frame_index].relayed || active_clients[client_index].relay_can) {
                    client_frames[client_frame_count++] = queued_frames[frame_index].frame;
                }
            }
            if (client_frame_count == 0U) continue;

            const espnow_can_batch_meta_t meta = {
                .sequence = active_sequences[client_index]++,
                .sender_uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
                .sender_frame_drops = (uint16_t)stats_snapshot.queue_drops,
                .sender_send_failures = (uint16_t)stats_snapshot.send_failures,
            };
            const size_t packet_size = espnow_can_encode_batch(packet, sizeof(packet), &meta,
                                                                client_frames, client_frame_count);
            if (packet_size == 0U) {
                increment_send_failure();
                continue;
            }

            const int64_t send_start_us = esp_timer_get_time();
            const esp_err_t send_err = esp_now_send(active_clients[client_index].mac, packet, packet_size);
            const uint64_t send_api_us = (uint64_t)(esp_timer_get_time() - send_start_us);
            portENTER_CRITICAL(&stats_mux);
            transport_stats.sent_packets++;
            transport_stats.radio_busy_us += send_api_us;
            portEXIT_CRITICAL(&stats_mux);
            if (send_err != ESP_OK) increment_send_failure();
        }
        xSemaphoreGive(transport_mutex);
    }
}

static esp_err_t ensure_transport_resources(void)
{
    if (frame_queue == NULL)
    {
        frame_queue = xQueueCreate(ESPNOW_FRAME_QUEUE_DEPTH, sizeof(espnow_queued_frame_t));
        if (frame_queue == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }
    if (transport_mutex == NULL)
    {
        transport_mutex = xSemaphoreCreateMutex();
        if (transport_mutex == NULL) return ESP_ERR_NO_MEM;
    }
    if (flush_timer == NULL)
    {
        const esp_timer_create_args_t timer_args = {
            .callback = flush_timer_callback,
            .name = "espnow_can_flush",
        };
        ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &flush_timer), TAG, "Could not create batch timer");
    }
    if (batch_task_handle == NULL)
    {
        if (xTaskCreatePinnedToCore(batch_transmitter_task,
                                    "espnowCanTx",
                                    ESPNOW_BATCH_TASK_STACK,
                                    NULL,
                                    ESPNOW_BATCH_TASK_PRIORITY,
                                    &batch_task_handle,
                                    0) != pdPASS)
        {
            batch_task_handle = NULL;
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

esp_err_t espnow_transport_start(void)
{
    if (!board_cfg.espnow_enabled)
    {
        return ESP_OK;
    }
    if (board_cfg.espnow_client_count == 0U || board_cfg.espnow_client_count > ESPNOW_MAX_CLIENTS)
    {
        ESP_LOGW(TAG, "ESP-NOW enabled but no clients are configured");
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < board_cfg.espnow_client_count; ++i) {
        if (mac_is_empty(board_cfg.espnow_clients[i].mac)) return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(ensure_transport_resources(), TAG, "Could not allocate transport resources");

    uint8_t primary_channel = 0U;
    wifi_second_chan_t secondary_channel = WIFI_SECOND_CHAN_NONE;
    ESP_RETURN_ON_ERROR(esp_wifi_get_channel(&primary_channel, &secondary_channel), TAG, "Could not verify WiFi channel");
    if (primary_channel != ESPNOW_WIFI_CHANNEL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "Could not initialize ESP-NOW");
    esp_err_t err = esp_now_register_send_cb(espnow_send_callback);
    if (err != ESP_OK)
    {
        esp_now_deinit();
        return err;
    }

    memset(active_clients, 0, sizeof(active_clients));
    memset(active_sequences, 0, sizeof(active_sequences));
    active_client_count = 0U;
    for (uint8_t i = 0; i < board_cfg.espnow_client_count; ++i) {
        esp_now_peer_info_t peer_info = {0};
        memcpy(peer_info.peer_addr, board_cfg.espnow_clients[i].mac, ESP_NOW_ETH_ALEN);
        /* WiFi owns the AP and channel.  Channel 0 makes the peer use the
         * channel already selected by the running WiFi interface. */
        peer_info.channel = 0;
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false;
        err = esp_now_add_peer(&peer_info);
        if (err != ESP_OK) {
            for (uint8_t added = 0; added < active_client_count; ++added) {
                esp_now_del_peer(active_clients[added].mac);
            }
            esp_now_unregister_send_cb();
            esp_now_deinit();
            active_client_count = 0U;
            return err;
        }
        active_clients[active_client_count++] = board_cfg.espnow_clients[i];
        ESP_LOGI(TAG, "Client %u active: %02X:%02X:%02X:%02X:%02X:%02X relay_can=%d",
                 (unsigned)(i + 1U),
                 peer_info.peer_addr[0], peer_info.peer_addr[1], peer_info.peer_addr[2],
                 peer_info.peer_addr[3], peer_info.peer_addr[4], peer_info.peer_addr[5],
                 board_cfg.espnow_clients[i].relay_can);
    }

    espnow_started = true;
    xTaskNotify(batch_task_handle, ESPNOW_NOTIFY_FRAME, eSetBits);
    ESP_LOGI(TAG, "Batched CAN transport active on channel %u for %u client(s)",
             (unsigned)primary_channel, (unsigned)active_client_count);
    return ESP_OK;
}

esp_err_t espnow_transport_stop(void)
{
    if (!espnow_started)
    {
        return ESP_OK;
    }
    espnow_started = false;
    if (batch_task_handle != NULL)
    {
        xTaskNotify(batch_task_handle, ESPNOW_NOTIFY_STOP, eSetBits);
    }
    if (flush_timer != NULL)
    {
        esp_timer_stop(flush_timer);
    }
    if (transport_mutex != NULL) xSemaphoreTake(transport_mutex, portMAX_DELAY);
    if (frame_queue != NULL)
    {
        xQueueReset(frame_queue);
    }
    for (uint8_t i = 0; i < active_client_count; ++i) {
        esp_now_del_peer(active_clients[i].mac);
    }
    memset(active_clients, 0, sizeof(active_clients));
    active_client_count = 0U;
    esp_now_unregister_send_cb();
    const esp_err_t err = esp_now_deinit();
    if (transport_mutex != NULL) xSemaphoreGive(transport_mutex);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to deinitialize ESP-NOW: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t espnow_transport_apply_config(void)
{
    if (espnow_started) {
        const esp_err_t stop_err = espnow_transport_stop();
        if (stop_err != ESP_OK) return stop_err;
    }
    return board_cfg.espnow_enabled ? espnow_transport_start() : ESP_OK;
}

static esp_err_t enqueue_twai(const twai_message_t *message, bool relayed)
{
    if (!board_cfg.espnow_enabled || !espnow_started)
    {
        return ESP_OK;
    }
    if (message == NULL || message->data_length_code > 8U ||
        message->identifier > (message->extd ? ESPNOW_CAN_ID_MASK : 0x7FFU))
    {
        return ESP_ERR_INVALID_ARG;
    }

    espnow_queued_frame_t queued_frame = {
        .frame = {
            .identifier = message->identifier,
            .data_length_code = message->data_length_code,
            .extd = message->extd,
            .rtr = message->rtr,
        },
        .relayed = relayed,
    };
    memcpy(queued_frame.frame.data, message->data, sizeof(queued_frame.frame.data));

    if (xQueueSend(frame_queue, &queued_frame, 0) != pdTRUE)
    {
        espnow_queued_frame_t discarded;
        if (xQueueReceive(frame_queue, &discarded, 0) == pdTRUE)
        {
            portENTER_CRITICAL(&stats_mux);
            transport_stats.queue_drops++;
            portEXIT_CRITICAL(&stats_mux);
        }
        if (xQueueSend(frame_queue, &queued_frame, 0) != pdTRUE)
        {
            portENTER_CRITICAL(&stats_mux);
            transport_stats.queue_drops++;
            portEXIT_CRITICAL(&stats_mux);
            return ESP_ERR_NO_MEM;
        }
    }
    portENTER_CRITICAL(&stats_mux);
    transport_stats.queued_frames++;
    portEXIT_CRITICAL(&stats_mux);
    xTaskNotify(batch_task_handle, ESPNOW_NOTIFY_FRAME, eSetBits);
    return ESP_OK;
}

esp_err_t espnow_transport_enqueue_twai(const twai_message_t *message)
{
    return enqueue_twai(message, false);
}

esp_err_t espnow_transport_enqueue_relay_twai(const twai_message_t *message)
{
    if (!config_has_espnow_relay_client(&board_cfg)) return ESP_OK;
    return enqueue_twai(message, true);
}
