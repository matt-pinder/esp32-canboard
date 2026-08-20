#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ESPNOW_CAN_MAGIC_0 0x43U
#define ESPNOW_CAN_MAGIC_1 0x41U
#define ESPNOW_CAN_PROTOCOL_VERSION 1U
#define ESPNOW_CAN_HEADER_SIZE 14U
#define ESPNOW_CAN_FRAME_SIZE 13U
#define ESPNOW_CAN_MAX_FRAMES 18U
#define ESPNOW_CAN_MAX_PACKET_SIZE (ESPNOW_CAN_HEADER_SIZE + (ESPNOW_CAN_MAX_FRAMES * ESPNOW_CAN_FRAME_SIZE))

#define ESPNOW_CAN_ID_MASK 0x1FFFFFFFU
#define ESPNOW_CAN_FLAG_EXTD (1UL << 29)
#define ESPNOW_CAN_FLAG_RTR (1UL << 30)
#define ESPNOW_CAN_FLAG_RESERVED (1UL << 31)

typedef struct {
    uint16_t sequence;
    uint32_t sender_uptime_ms;
    uint16_t sender_frame_drops;
    uint16_t sender_send_failures;
} espnow_can_batch_meta_t;

typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
    bool extd;
    bool rtr;
} espnow_can_frame_t;

size_t espnow_can_encode_batch(uint8_t *output,
                               size_t output_size,
                               const espnow_can_batch_meta_t *meta,
                               const espnow_can_frame_t *frames,
                               uint8_t frame_count);

bool espnow_can_decode_header(const uint8_t *packet,
                              size_t packet_size,
                              espnow_can_batch_meta_t *meta,
                              uint8_t *frame_count);

bool espnow_can_decode_frame(const uint8_t *packet,
                             size_t packet_size,
                             uint8_t frame_index,
                             espnow_can_frame_t *frame);
