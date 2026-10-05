#pragma once

#include "esp_err.h"
#include "esp_now.h"

esp_err_t gps_response_receiver_start(void);
void gps_response_receiver_apply_config(void);
void gps_response_receive_callback(const esp_now_recv_info_t *info,
                                   const uint8_t *data, int data_length);
void gps_response_publish_cached(void);

