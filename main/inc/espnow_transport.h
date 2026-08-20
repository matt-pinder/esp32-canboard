#ifndef ESPNOW_TRANSPORT_H
#define ESPNOW_TRANSPORT_H

#include "driver/twai_types_legacy.h"
#include "esp_err.h"

#define ESPNOW_WIFI_CHANNEL 1

/// @brief Initialize ESP-NOW and add the configured target peer.
esp_err_t espnow_transport_start(void);

/// @brief Stop ESP-NOW transport if it is active.
esp_err_t espnow_transport_stop(void);

/// @brief Apply current board_cfg ESP-NOW settings at runtime.
esp_err_t espnow_transport_apply_config(void);

/// @brief Enqueue a TWAI frame for freshness-first batched ESP-NOW transport.
esp_err_t espnow_transport_enqueue_twai(const twai_message_t *message);

/// @brief Enqueue a frame received from the physical CAN bus for relay-enabled clients only.
esp_err_t espnow_transport_enqueue_relay_twai(const twai_message_t *message);

#endif // ESPNOW_TRANSPORT_H
