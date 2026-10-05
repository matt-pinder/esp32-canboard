#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "gps_snapshot_protocol.h"

int main(void)
{
    const gps_snapshot_t input = {
        .flags = GPS_SNAPSHOT_FLAG_FIX_VALID |
                 GPS_SNAPSHOT_FLAG_TIME_VALID |
                 GPS_SNAPSHOT_FLAG_COURSE_VALID,
        .sequence = 65535U,
        .sender_uptime_ms = 123456U,
        .sample_age_ms = 37U,
        .i_tow_ms = GPS_WEEK_MILLISECONDS - 1U,
        .latitude_1e7_deg = -512345678,
        .longitude_1e7_deg = 1799999999,
        .ground_speed_mm_s = 43210U,
        .heading_1e5_deg = 35999999,
        .altitude_mm = -12500,
        .satellites = 17U,
        .fix_quality = 2U,
        .hdop_x100 = 73U,
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
    invalid.i_tow_ms = GPS_WEEK_MILLISECONDS;
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

    puts("gps_snapshot_protocol tests passed");
    return 0;
}
