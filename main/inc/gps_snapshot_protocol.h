#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GPS_SNAPSHOT_MAGIC_0 0x47U
#define GPS_SNAPSHOT_MAGIC_1 0x50U
#define GPS_SNAPSHOT_PROTOCOL_VERSION 1U
#define GPS_SNAPSHOT_PACKET_SIZE 42U

#define GPS_SNAPSHOT_FLAG_FIX_VALID (1U << 0)
#define GPS_SNAPSHOT_FLAG_TIME_VALID (1U << 1)
#define GPS_SNAPSHOT_FLAG_COURSE_VALID (1U << 2)
#define GPS_SNAPSHOT_FLAG_MASK                                                     \
    (GPS_SNAPSHOT_FLAG_FIX_VALID | GPS_SNAPSHOT_FLAG_TIME_VALID |                 \
     GPS_SNAPSHOT_FLAG_COURSE_VALID)

#define GPS_WEEK_MILLISECONDS 604800000UL

typedef struct {
    uint8_t flags;
    uint16_t sequence;
    uint32_t sender_uptime_ms;
    uint32_t sample_age_ms;
    uint32_t i_tow_ms;
    int32_t latitude_1e7_deg;
    int32_t longitude_1e7_deg;
    uint32_t ground_speed_mm_s;
    int32_t heading_1e5_deg;
    int32_t altitude_mm;
    uint8_t satellites;
    uint8_t fix_quality;
    uint16_t hdop_x100;
} gps_snapshot_t;

size_t gps_snapshot_encode(uint8_t *output, size_t output_size,
                           const gps_snapshot_t *snapshot);

bool gps_snapshot_decode(const uint8_t *packet, size_t packet_size,
                         gps_snapshot_t *snapshot);
