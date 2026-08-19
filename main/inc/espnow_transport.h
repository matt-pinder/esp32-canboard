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

/// @brief Send a TWAI message byte-for-byte over ESP-NOW.
esp_err_t espnow_transport_send_twai(const twai_message_t *message);

/// @brief Best-effort nonblocking send for lower-priority relayed CAN traffic.
esp_err_t espnow_transport_try_relay_twai(const twai_message_t *message);

#endif // ESPNOW_TRANSPORT_H
