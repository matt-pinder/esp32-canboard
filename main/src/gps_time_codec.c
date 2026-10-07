#include "inc/gps_time_codec.h"

#include <limits.h>
#include <stddef.h>

#define GPS_CAN_ALTITUDE_BITS 22U
#define GPS_CAN_ALTITUDE_MASK ((1U << GPS_CAN_ALTITUDE_BITS) - 1U)
#define GPS_CAN_ALTITUDE_MIN_CM (-(1L << (GPS_CAN_ALTITUDE_BITS - 1U)))
#define GPS_CAN_ALTITUDE_MAX_CM ((1L << (GPS_CAN_ALTITUDE_BITS - 1U)) - 1L)

static bool leap_year(unsigned year)
{
    return (year % 4U == 0U && year % 100U != 0U) || year % 400U == 0U;
}

static unsigned days_in_month(unsigned year, unsigned month)
{
    static const uint8_t days[] = {31U, 28U, 31U, 30U, 31U, 30U,
                                   31U, 31U, 30U, 31U, 30U, 31U};
    if (month < 1U || month > 12U) return 0U;
    return month == 2U && leap_year(year) ? 29U : days[month - 1U];
}

static int64_t days_from_civil(int year, unsigned month, unsigned day)
{
    year -= month <= 2U;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned year_of_era = (unsigned)(year - era * 400);
    const unsigned day_of_year =
        (153U * (month > 2U ? month - 3U : month + 9U) + 2U) / 5U + day - 1U;
    const unsigned day_of_era =
        year_of_era * 365U + year_of_era / 4U - year_of_era / 100U + day_of_year;
    return (int64_t)era * 146097LL + (int64_t)day_of_era - 719468LL;
}

static void write_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8) & 0xFFU);
    output[2] = (uint8_t)((value >> 16) & 0xFFU);
    output[3] = (uint8_t)(value >> 24);
}

bool gps_utc_from_calendar(unsigned year, unsigned month, unsigned day,
                           unsigned hour, unsigned minute, unsigned second,
                           int32_t nanosecond, uint32_t *utc_unix_s,
                           uint16_t *utc_millisecond)
{
    if (utc_unix_s == NULL || utc_millisecond == NULL || year < 1970U ||
        year > 2106U || day < 1U || day > days_in_month(year, month) ||
        hour > 23U || minute > 59U || second > 60U ||
        nanosecond < -1000000000 || nanosecond > 1000000000) {
        return false;
    }

    const int64_t whole_seconds =
        days_from_civil((int)year, month, day) * 86400LL +
        (int64_t)hour * 3600LL + (int64_t)minute * 60LL + second;
    int64_t fractional_ms = nanosecond / 1000000;
    if (nanosecond < 0 && nanosecond % 1000000 != 0) --fractional_ms;
    const int64_t unix_ms = whole_seconds * 1000LL + fractional_ms;
    if (unix_ms <= 0 || unix_ms / 1000LL > UINT32_MAX) return false;

    *utc_unix_s = (uint32_t)(unix_ms / 1000LL);
    *utc_millisecond = (uint16_t)(unix_ms % 1000LL);
    return true;
}

bool gps_can_pack_altitude_utc(uint8_t output[8], int32_t altitude_mm,
                               uint32_t utc_unix_s,
                               uint16_t utc_millisecond)
{
    if (output == NULL || utc_millisecond > 999U) return false;

    int64_t altitude_cm = altitude_mm >= 0
                              ? ((int64_t)altitude_mm + 5LL) / 10LL
                              : ((int64_t)altitude_mm - 5LL) / 10LL;
    if (altitude_cm < GPS_CAN_ALTITUDE_MIN_CM) altitude_cm = GPS_CAN_ALTITUDE_MIN_CM;
    if (altitude_cm > GPS_CAN_ALTITUDE_MAX_CM) altitude_cm = GPS_CAN_ALTITUDE_MAX_CM;

    const uint32_t altitude_time =
        ((uint32_t)altitude_cm & GPS_CAN_ALTITUDE_MASK) |
        ((uint32_t)utc_millisecond << GPS_CAN_ALTITUDE_BITS);
    write_u32_le(output, altitude_time);
    write_u32_le(output + 4, utc_unix_s);
    return true;
}
