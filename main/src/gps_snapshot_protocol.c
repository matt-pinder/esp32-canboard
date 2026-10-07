#include "inc/gps_snapshot_protocol.h"

static void write_u16_le(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8) & 0xFFU);
    output[2] = (uint8_t)((value >> 16) & 0xFFU);
    output[3] = (uint8_t)(value >> 24);
}

static uint16_t read_u16_le(const uint8_t *input)
{
    return (uint16_t)input[0] | ((uint16_t)input[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *input)
{
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8) |
           ((uint32_t)input[2] << 16) | ((uint32_t)input[3] << 24);
}

static bool snapshot_valid(const gps_snapshot_t *snapshot)
{
    return snapshot != NULL &&
           (snapshot->flags & ~GPS_SNAPSHOT_FLAG_MASK) == 0U &&
           snapshot->utc_millisecond <= 999U &&
           (((snapshot->flags & GPS_SNAPSHOT_FLAG_TIME_VALID) == 0U) ||
            snapshot->utc_unix_s > 0U) &&
           snapshot->latitude_1e7_deg >= -900000000 &&
           snapshot->latitude_1e7_deg <= 900000000 &&
           snapshot->longitude_1e7_deg >= -1800000000 &&
           snapshot->longitude_1e7_deg <= 1800000000 &&
           snapshot->heading_1e5_deg >= 0 &&
           snapshot->heading_1e5_deg < 36000000 &&
           snapshot->altitude_mm >= -1000000 &&
           snapshot->altitude_mm <= 100000000 &&
           snapshot->fix_quality <= 8U;
}

size_t gps_snapshot_encode(uint8_t *output, size_t output_size,
                           const gps_snapshot_t *snapshot)
{
    if (output == NULL || output_size < GPS_SNAPSHOT_PACKET_SIZE ||
        !snapshot_valid(snapshot)) {
        return 0U;
    }

    output[0] = GPS_SNAPSHOT_MAGIC_0;
    output[1] = GPS_SNAPSHOT_MAGIC_1;
    output[2] = GPS_SNAPSHOT_PROTOCOL_VERSION;
    output[3] = snapshot->flags;
    write_u16_le(output + 4, snapshot->sequence);
    write_u32_le(output + 6, snapshot->sender_uptime_ms);
    write_u32_le(output + 10, snapshot->sample_age_ms);
    write_u32_le(output + 14, snapshot->utc_unix_s);
    write_u32_le(output + 18, (uint32_t)snapshot->latitude_1e7_deg);
    write_u32_le(output + 22, (uint32_t)snapshot->longitude_1e7_deg);
    write_u32_le(output + 26, snapshot->ground_speed_mm_s);
    write_u32_le(output + 30, (uint32_t)snapshot->heading_1e5_deg);
    write_u32_le(output + 34, (uint32_t)snapshot->altitude_mm);
    output[38] = snapshot->satellites;
    output[39] = snapshot->fix_quality;
    write_u16_le(output + 40, snapshot->hdop_x100);
    write_u16_le(output + 42, snapshot->utc_millisecond);
    return GPS_SNAPSHOT_PACKET_SIZE;
}

bool gps_snapshot_decode(const uint8_t *packet, size_t packet_size,
                         gps_snapshot_t *snapshot)
{
    if (packet == NULL || snapshot == NULL ||
        packet_size != GPS_SNAPSHOT_PACKET_SIZE ||
        packet[0] != GPS_SNAPSHOT_MAGIC_0 || packet[1] != GPS_SNAPSHOT_MAGIC_1 ||
        packet[2] != GPS_SNAPSHOT_PROTOCOL_VERSION) {
        return false;
    }

    const gps_snapshot_t decoded = {
        .flags = packet[3],
        .sequence = read_u16_le(packet + 4),
        .sender_uptime_ms = read_u32_le(packet + 6),
        .sample_age_ms = read_u32_le(packet + 10),
        .utc_unix_s = read_u32_le(packet + 14),
        .latitude_1e7_deg = (int32_t)read_u32_le(packet + 18),
        .longitude_1e7_deg = (int32_t)read_u32_le(packet + 22),
        .ground_speed_mm_s = read_u32_le(packet + 26),
        .heading_1e5_deg = (int32_t)read_u32_le(packet + 30),
        .altitude_mm = (int32_t)read_u32_le(packet + 34),
        .satellites = packet[38],
        .fix_quality = packet[39],
        .hdop_x100 = read_u16_le(packet + 40),
        .utc_millisecond = read_u16_le(packet + 42),
    };
    if (!snapshot_valid(&decoded)) {
        return false;
    }
    *snapshot = decoded;
    return true;
}
