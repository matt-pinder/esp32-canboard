#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "gps_snapshot_protocol.h"
#include "gps_time_codec.h"

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

int main(void)
{
    const gps_snapshot_t input = {
        .flags = GPS_SNAPSHOT_FLAG_FIX_VALID |
                 GPS_SNAPSHOT_FLAG_TIME_VALID |
                 GPS_SNAPSHOT_FLAG_COURSE_VALID,
        .sequence = 65535U,
        .sender_uptime_ms = 123456U,
        .sample_age_ms = 37U,
        .utc_unix_s = 1791069382U,
        .latitude_1e7_deg = -512345678,
        .longitude_1e7_deg = 1799999999,
        .ground_speed_mm_s = 43210U,
        .heading_1e5_deg = 35999999,
        .altitude_mm = -12500,
        .satellites = 17U,
        .fix_quality = 2U,
        .hdop_x100 = 73U,
        .utc_millisecond = 900U,
    };
    uint8_t packet[GPS_SNAPSHOT_PACKET_SIZE];
    assert(gps_snapshot_encode(packet, sizeof(packet), &input) == sizeof(packet));
    gps_snapshot_t output;
    assert(gps_snapshot_decode(packet, sizeof(packet), &output));
    assert(memcmp(&input, &output, sizeof(input)) == 0);

    assert(!gps_snapshot_decode(packet, sizeof(packet) - 1U, &output));
    assert(!gps_snapshot_decode(packet, sizeof(packet) + 1U, &output));
    packet[3] |= 0x80U;
    assert(!gps_snapshot_decode(packet, sizeof(packet), &output));
    packet[3] &= GPS_SNAPSHOT_FLAG_MASK;

    gps_snapshot_t invalid = input;
    invalid.utc_millisecond = 1000U;
    assert(gps_snapshot_encode(packet, sizeof(packet), &invalid) == 0U);
    invalid = input;
    invalid.utc_unix_s = 0U;
    assert(gps_snapshot_encode(packet, sizeof(packet), &invalid) == 0U);
    invalid = input;
    invalid.latitude_1e7_deg = 900000001;
    assert(gps_snapshot_encode(packet, sizeof(packet), &invalid) == 0U);
    invalid = input;
    invalid.longitude_1e7_deg = -1800000001;
    assert(gps_snapshot_encode(packet, sizeof(packet), &invalid) == 0U);
    invalid = input;
    invalid.heading_1e5_deg = 36000000;
    assert(gps_snapshot_encode(packet, sizeof(packet), &invalid) == 0U);
    invalid = input;
    invalid.altitude_mm = 100000001;
    assert(gps_snapshot_encode(packet, sizeof(packet), &invalid) == 0U);
    invalid = input;
    invalid.fix_quality = 9U;
    assert(gps_snapshot_encode(packet, sizeof(packet), &invalid) == 0U);

    uint32_t utc_unix_s = 0U;
    uint16_t utc_millisecond = 0U;
    assert(gps_utc_from_calendar(2026U, 10U, 3U, 23U, 16U, 22U,
                                 900000000, &utc_unix_s,
                                 &utc_millisecond));
    assert(utc_unix_s == 1791069382U);
    assert(utc_millisecond == 900U);
    assert(gps_utc_from_calendar(2026U, 10U, 3U, 23U, 16U, 22U,
                                 -100000000, &utc_unix_s,
                                 &utc_millisecond));
    assert(utc_unix_s == 1791069381U);
    assert(utc_millisecond == 900U);
    assert(gps_utc_from_calendar(2026U, 10U, 3U, 23U, 16U, 22U,
                                 1000000000, &utc_unix_s,
                                 &utc_millisecond));
    assert(utc_unix_s == 1791069383U);
    assert(utc_millisecond == 0U);
    assert(gps_utc_from_calendar(2026U, 10U, 3U, 23U, 16U, 22U,
                                 -1000000000, &utc_unix_s,
                                 &utc_millisecond));
    assert(utc_unix_s == 1791069381U);
    assert(utc_millisecond == 0U);
    assert(!gps_utc_from_calendar(2026U, 2U, 29U, 0U, 0U, 0U, 0,
                                  &utc_unix_s, &utc_millisecond));

    uint8_t can_data[8];
    assert(gps_can_pack_altitude_utc(can_data, 123400, 1791069382U, 789U));
    const uint32_t altitude_time = read_u32_le(can_data);
    assert((altitude_time & 0x003FFFFFU) == 12340U);
    assert((altitude_time >> 22) == 789U);
    assert(read_u32_le(can_data + 4) == 1791069382U);

    puts("gps_snapshot_protocol tests passed");
    return 0;
}
