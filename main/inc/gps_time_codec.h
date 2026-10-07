#pragma once

#include <stdbool.h>
#include <stdint.h>

bool gps_utc_from_calendar(unsigned year, unsigned month, unsigned day,
                           unsigned hour, unsigned minute, unsigned second,
                           int32_t nanosecond, uint32_t *utc_unix_s,
                           uint16_t *utc_millisecond);

bool gps_can_pack_altitude_utc(uint8_t output[8], int32_t altitude_mm,
                               uint32_t utc_unix_s,
                               uint16_t utc_millisecond);
