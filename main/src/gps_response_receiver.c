#include "inc/gps_response_receiver.h"

#include <string.h>

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "inc/can.h"
#include "inc/config.h"
#include "inc/gps_snapshot_protocol.h"
#include "inc/gps_time_codec.h"

#define GPS_RESPONSE_QUEUE_DEPTH 8U
#define GPS_RESPONSE_TASK_STACK 4096U
#define GPS_RESPONSE_TASK_PRIORITY 6U

typedef struct {
    uint8_t source_mac[ESP_NOW_ETH_ALEN];
    int64_t received_at_ms;
    uint8_t payload[GPS_SNAPSHOT_PACKET_SIZE];
} gps_response_packet_t;

extern board_config_t board_cfg;

static const char *TAG = "gps_response_rx";
static QueueHandle_t packet_queue;
static SemaphoreHandle_t cache_mutex;
static gps_snapshot_t cached_snapshot;
static int64_t cached_at_ms;
static bool have_cached_snapshot;
static bool sequence_initialised;
static uint16_t last_sequence;
static uint32_t last_sender_uptime_ms;
static uint8_t configured_peer[ESP_NOW_ETH_ALEN];

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)(value >> 24);
}

static twai_message_t gps_frame(uint32_t identifier)
{
    twai_message_t frame = {0};
    frame.identifier = identifier;
    frame.data_length_code = 8U;
    return frame;
}

static bool accept_sequence(const gps_snapshot_t *snapshot)
{
    if (!sequence_initialised || snapshot->sender_uptime_ms < last_sender_uptime_ms) {
        sequence_initialised = true;
        last_sequence = snapshot->sequence;
        last_sender_uptime_ms = snapshot->sender_uptime_ms;
        return true;
    }
    const uint16_t delta = (uint16_t)(snapshot->sequence - last_sequence);
    if (delta == 0U || delta >= 0x8000U) return false;
    last_sequence = snapshot->sequence;
    last_sender_uptime_ms = snapshot->sender_uptime_ms;
    return true;
}

static void decoder_task(void *argument)
{
    (void)argument;
    gps_response_packet_t packet;
    while (xQueueReceive(packet_queue, &packet, portMAX_DELAY) == pdTRUE) {
        gps_snapshot_t snapshot;
        if (!gps_snapshot_decode(packet.payload, sizeof(packet.payload), &snapshot)) {
            continue;
        }
        int64_t captured_at_ms = packet.received_at_ms - (int64_t)snapshot.sample_age_ms;
        if (captured_at_ms < 0) captured_at_ms = 0;
        xSemaphoreTake(cache_mutex, portMAX_DELAY);
        if (memcmp(packet.source_mac, configured_peer, ESP_NOW_ETH_ALEN) == 0 &&
            accept_sequence(&snapshot)) {
            cached_snapshot = snapshot;
            cached_at_ms = captured_at_ms;
            have_cached_snapshot = true;
        }
        xSemaphoreGive(cache_mutex);
    }
    vTaskDelete(NULL);
}

void gps_response_receive_callback(const esp_now_recv_info_t *info,
                                   const uint8_t *data, int data_length)
{
    if (info == NULL || info->src_addr == NULL || data == NULL ||
        data_length != (int)GPS_SNAPSHOT_PACKET_SIZE || packet_queue == NULL ||
        !board_cfg.gps_enabled ||
        board_cfg.gps_source != GPS_SOURCE_ESPNOW_RESPONSE ||
        memcmp(info->src_addr, board_cfg.gps_espnow_peer_mac,
               ESP_NOW_ETH_ALEN) != 0) {
        return;
    }
    gps_response_packet_t packet = {
        .received_at_ms = esp_timer_get_time() / 1000,
    };
    memcpy(packet.source_mac, info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(packet.payload, data, sizeof(packet.payload));
    if (xQueueSend(packet_queue, &packet, 0) != pdTRUE) {
        gps_response_packet_t discarded;
        (void)xQueueReceive(packet_queue, &discarded, 0);
        (void)xQueueSend(packet_queue, &packet, 0);
    }
}

void gps_response_receiver_apply_config(void)
{
    if (cache_mutex == NULL) return;
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    if (board_cfg.gps_source != GPS_SOURCE_ESPNOW_RESPONSE ||
        memcmp(configured_peer, board_cfg.gps_espnow_peer_mac,
               ESP_NOW_ETH_ALEN) != 0) {
        have_cached_snapshot = false;
        sequence_initialised = false;
    }
    memcpy(configured_peer, board_cfg.gps_espnow_peer_mac, ESP_NOW_ETH_ALEN);
    xSemaphoreGive(cache_mutex);
    if (packet_queue != NULL) xQueueReset(packet_queue);
}

esp_err_t gps_response_receiver_start(void)
{
    if (packet_queue != NULL) {
        gps_response_receiver_apply_config();
        return ESP_OK;
    }
    packet_queue = xQueueCreate(GPS_RESPONSE_QUEUE_DEPTH,
                                sizeof(gps_response_packet_t));
    cache_mutex = xSemaphoreCreateMutex();
    if (packet_queue == NULL || cache_mutex == NULL) return ESP_ERR_NO_MEM;
    gps_response_receiver_apply_config();
    if (xTaskCreatePinnedToCore(decoder_task, "gpsResponseRx",
                                GPS_RESPONSE_TASK_STACK, NULL,
                                GPS_RESPONSE_TASK_PRIORITY, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void gps_response_publish_cached(void)
{
    if (!board_cfg.gps_enabled ||
        board_cfg.gps_source != GPS_SOURCE_ESPNOW_RESPONSE ||
        cache_mutex == NULL) {
        return;
    }

    gps_snapshot_t snapshot = {0};
    int64_t captured_at_ms = 0;
    bool available;
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    available = have_cached_snapshot;
    snapshot = cached_snapshot;
    captured_at_ms = cached_at_ms;
    xSemaphoreGive(cache_mutex);

    const int64_t now_ms = esp_timer_get_time() / 1000;
    const bool fresh = available && now_ms >= captured_at_ms &&
                       (uint64_t)(now_ms - captured_at_ms) <=
                           board_cfg.gps_response_timeout_ms;
    const bool fix_valid = fresh &&
        (snapshot.flags & GPS_SNAPSHOT_FLAG_FIX_VALID) != 0U;

    twai_message_t status = gps_frame(board_cfg.gps_can_start_id);
    status.data[0] = fix_valid ? 2U : 0U;
    status.data[2] = fix_valid ? snapshot.satellites : 0U;
    can_transmit_frame(&status, fix_valid ? "ESP-NOW GPS status" :
                                            "ESP-NOW GPS invalid status");
    if (!fix_valid) return;

    twai_message_t motion = gps_frame(board_cfg.gps_can_start_id + 1U);
    write_u32_le(&motion.data[0], snapshot.ground_speed_mm_s);
    write_u32_le(&motion.data[4],
                 (uint32_t)snapshot.heading_1e5_deg);
    can_transmit_frame(&motion, "ESP-NOW GPS motion");

    twai_message_t position = gps_frame(board_cfg.gps_can_start_id + 2U);
    write_u32_le(&position.data[0],
                 (uint32_t)snapshot.latitude_1e7_deg);
    write_u32_le(&position.data[4],
                 (uint32_t)snapshot.longitude_1e7_deg);
    can_transmit_frame(&position, "ESP-NOW GPS position");

    twai_message_t altitude_time = gps_frame(board_cfg.gps_can_start_id + 3U);
    (void)gps_can_pack_altitude_utc(altitude_time.data,
                                    snapshot.altitude_mm,
                                    snapshot.utc_unix_s,
                                    snapshot.utc_millisecond);
    can_transmit_frame(&altitude_time, "ESP-NOW GPS altitude/time");
}
