#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "mk60_emulator_protocol.h"

static mk60_emulator_config_t valid_profile(void)
{
    mk60_emulator_config_t profile = {
        .enabled = true,
        .trigger_id = MK60_TRIGGER_ID,
        .trigger_dlc = 8,
        .response_count = 3,
        .responses = {
            {.identifier = 0x610, .data_length_code = 8, .delay_before_ms = 0,
             .data = {0x12, 0x34, 0x56, 0x41, 0x42, 0, 0, 0}},
            {.identifier = 0x613, .data_length_code = 8, .delay_before_ms = 5,
             .data = {1, 2, 3, 4, 5, 6, 7, 8}},
            {.identifier = 0x615, .data_length_code = 8, .delay_before_ms = 5,
             .data = {8, 7, 6, 5, 4, 3, 2, 1}},
        },
    };
    return profile;
}

static void test_profile_validation(void)
{
    mk60_emulator_config_t disabled = {.trigger_id = MK60_TRIGGER_ID, .trigger_dlc = 8};
    assert(mk60_emulator_profile_valid(&disabled));

    mk60_emulator_config_t profile = valid_profile();
    assert(mk60_emulator_profile_valid(&profile));
    assert(profile.responses[0].identifier == 0x610);
    assert(profile.responses[1].delay_before_ms == 5);
    assert(profile.responses[1].data[7] == 8);

    mk60_emulator_config_t invalid = profile;
    invalid.response_count = 0;
    assert(!mk60_emulator_profile_valid(&invalid));
    invalid = profile;
    invalid.responses[0].identifier = 0x613;
    assert(!mk60_emulator_profile_valid(&invalid));
    invalid = profile;
    invalid.responses[2].identifier = 0x613;
    assert(!mk60_emulator_profile_valid(&invalid));
    invalid = profile;
    invalid.responses[1].identifier = 0x123;
    assert(!mk60_emulator_profile_valid(&invalid));
    invalid = profile;
    invalid.responses[1].data_length_code = 9;
    assert(!mk60_emulator_profile_valid(&invalid));
    invalid = profile;
    invalid.responses[1].delay_before_ms = MK60_MAX_INTERFRAME_DELAY_MS + 1;
    assert(!mk60_emulator_profile_valid(&invalid));
}

static void test_trigger_matching(void)
{
    mk60_emulator_config_t profile = valid_profile();
    mk60_received_frame_t request = {
        .identifier = MK60_TRIGGER_ID,
        .data_length_code = 8,
        .extended = false,
        .rtr = true,
    };
    assert(mk60_emulator_is_trigger(&profile, &request));

    request.rtr = false;
    assert(!mk60_emulator_is_trigger(&profile, &request));
    request.rtr = true;
    request.extended = true;
    assert(!mk60_emulator_is_trigger(&profile, &request));
    request.extended = false;
    request.identifier = 0x611;
    assert(!mk60_emulator_is_trigger(&profile, &request));
    request.identifier = MK60_TRIGGER_ID;
    request.data_length_code = 0;
    assert(!mk60_emulator_is_trigger(&profile, &request));
    request.data_length_code = 8;
    profile.enabled = false;
    assert(!mk60_emulator_is_trigger(&profile, &request));
}

typedef struct {
    uint8_t delay_count;
    uint8_t delays[MK60_MAX_RESPONSE_FRAMES];
    uint8_t transmit_count;
    mk60_response_frame_config_t transmitted[MK60_MAX_RESPONSE_FRAMES];
    int fail_at;
} burst_capture_t;

static void capture_delay(uint8_t delay_ms, void *context)
{
    burst_capture_t *capture = context;
    capture->delays[capture->delay_count++] = delay_ms;
}

static bool capture_transmit(const mk60_response_frame_config_t *response,
                             uint8_t response_index,
                             void *context)
{
    burst_capture_t *capture = context;
    if ((int)response_index == capture->fail_at) return false;
    capture->transmitted[capture->transmit_count++] = *response;
    return true;
}

static void test_exact_burst(void)
{
    mk60_emulator_config_t profile = valid_profile();
    burst_capture_t capture = {.fail_at = -1};
    assert(mk60_emulator_run_burst(&profile, capture_delay, capture_transmit, &capture));
    assert(capture.delay_count == 2);
    assert(capture.delays[0] == 5 && capture.delays[1] == 5);
    assert(capture.transmit_count == profile.response_count);
    for (uint8_t index = 0; index < profile.response_count; ++index) {
        assert(memcmp(&capture.transmitted[index], &profile.responses[index],
                      sizeof(profile.responses[index])) == 0);
    }

    memset(&capture, 0, sizeof(capture));
    capture.fail_at = 1;
    assert(!mk60_emulator_run_burst(&profile, capture_delay, capture_transmit, &capture));
    assert(capture.transmit_count == 1);
}

int main(void)
{
    test_profile_validation();
    test_trigger_matching();
    test_exact_burst();
    puts("MK60 emulator protocol tests passed");
    return 0;
}
