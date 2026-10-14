#ifndef ESPNOW_TRANSPORT_H
#define ESPNOW_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai_types_legacy.h"
#include "esp_err.h"

typedef struct {
    bool started;
    uint8_t client_count;
    uint32_t queued_frames;
    uint32_t batches;
    uint32_t sent_packets;
    uint32_t queue_depth;
    uint32_t queue_drops;
    uint32_t send_failures;
    uint32_t callback_timeouts;
    uint64_t radio_busy_us;
} espnow_transport_status_t;

#define ESPNOW_WIFI_CHANNEL 1

/// @brief Attach ESP-NOW to the already-running WiFi radio and add configured peers.
/// Does not change WiFi mode, channel, AP configuration, or radio lifetime.
esp_err_t espnow_transport_start(void);

/// @brief Stop ESP-NOW transport if it is active.
esp_err_t espnow_transport_stop(void);

/// @brief Apply current board_cfg ESP-NOW settings at runtime.
esp_err_t espnow_transport_apply_config(void);

/// @brief Enqueue a TWAI frame for freshness-first batched ESP-NOW transport.
esp_err_t espnow_transport_enqueue_twai(const twai_message_t *message);

/// @brief Enqueue a frame received from the physical CAN bus for relay-enabled clients only.
esp_err_t espnow_transport_enqueue_relay_twai(const twai_message_t *message);

/// @brief Take a lock-safe snapshot for the web diagnostics endpoint.
void espnow_transport_get_status(espnow_transport_status_t *status);

#endif // ESPNOW_TRANSPORT_H
