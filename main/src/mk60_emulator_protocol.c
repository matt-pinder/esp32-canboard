#include "inc/mk60_emulator_protocol.h"

static bool response_id_allowed(uint32_t identifier)
{
    static const uint32_t allowed[] = {0x610U, 0x613U, 0x615U, 0x316U, 0x329U};
    for (size_t i = 0; i < sizeof(allowed) / sizeof(allowed[0]); ++i) {
        if (identifier == allowed[i]) return true;
    }
    return false;
}

bool mk60_emulator_profile_valid(const mk60_emulator_config_t *config)
{
    if (config == NULL || config->trigger_id != MK60_TRIGGER_ID || config->trigger_dlc > 8U) {
        return false;
    }
    if (config->response_count == 0U) {
        return !config->enabled;
    }
    if (config->response_count > MK60_MAX_RESPONSE_FRAMES ||
        config->responses[0].identifier != MK60_TRIGGER_ID ||
        config->responses[0].delay_before_ms != 0U) {
        return false;
    }

    uint32_t seen_ids[MK60_MAX_RESPONSE_FRAMES] = {0};
    for (uint8_t i = 0; i < config->response_count; ++i) {
        const mk60_response_frame_config_t *response = &config->responses[i];
        if (!response_id_allowed(response->identifier) || response->data_length_code > 8U ||
            response->delay_before_ms > MK60_MAX_INTERFRAME_DELAY_MS) {
            return false;
        }
        for (uint8_t j = 0; j < i; ++j) {
            if (seen_ids[j] == response->identifier) return false;
        }
        seen_ids[i] = response->identifier;
    }
    return true;
}

bool mk60_emulator_is_trigger(const mk60_emulator_config_t *config,
                              const mk60_received_frame_t *frame)
{
    return config != NULL && frame != NULL && config->enabled && mk60_emulator_profile_valid(config) &&
           frame->identifier == config->trigger_id &&
           frame->data_length_code == config->trigger_dlc &&
           !frame->extended && frame->rtr;
}

bool mk60_emulator_run_burst(const mk60_emulator_config_t *config,
                             mk60_delay_callback_t delay_callback,
                             mk60_transmit_callback_t transmit_callback,
                             void *context)
{
    if (config == NULL || !config->enabled || !mk60_emulator_profile_valid(config) ||
        transmit_callback == NULL) {
        return false;
    }
    for (uint8_t index = 0; index < config->response_count; ++index) {
        const mk60_response_frame_config_t *response = &config->responses[index];
        if (response->delay_before_ms > 0U && delay_callback != NULL) {
            delay_callback(response->delay_before_ms, context);
        }
        if (!transmit_callback(response, index, context)) return false;
    }
    return true;
}
