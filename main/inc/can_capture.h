#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai_types_legacy.h"
#include "esp_err.h"

#define CAN_CAPTURE_DURATION_MS 10000U

typedef struct {
    uint32_t timestamp_us;
    uint32_t identifier;
    uint8_t data[8];
    uint8_t dlc;
    bool extended;
    bool rtr;
} can_capture_record_t;

typedef struct {
    uint32_t start_uptime_ms;
    uint32_t frames_seen;
    uint32_t queue_drops;
} can_capture_stats_t;

esp_err_t can_capture_init(void);
bool can_capture_begin(void);
void can_capture_offer(const twai_message_t *message);
bool can_capture_receive(can_capture_record_t *record, uint32_t wait_ms);
void can_capture_finish(can_capture_stats_t *stats);
void can_capture_get_stats(can_capture_stats_t *stats);
bool can_capture_is_active(void);
