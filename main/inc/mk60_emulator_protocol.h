#ifndef MK60_EMULATOR_PROTOCOL_H
#define MK60_EMULATOR_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MK60_TRIGGER_ID 0x610U
#define MK60_MAX_RESPONSE_FRAMES 5U
#define MK60_MAX_INTERFRAME_DELAY_MS 50U

typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t delay_before_ms;
    uint8_t data[8];
} mk60_response_frame_config_t;

typedef struct {
    bool enabled;
    uint32_t trigger_id;
    uint8_t trigger_dlc;
    uint8_t response_count;
    mk60_response_frame_config_t responses[MK60_MAX_RESPONSE_FRAMES];
} mk60_emulator_config_t;

typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    bool extended;
    bool rtr;
} mk60_received_frame_t;

typedef void (*mk60_delay_callback_t)(uint8_t delay_ms, void *context);
typedef bool (*mk60_transmit_callback_t)(const mk60_response_frame_config_t *response,
                                         uint8_t response_index,
                                         void *context);

bool mk60_emulator_profile_valid(const mk60_emulator_config_t *config);
bool mk60_emulator_is_trigger(const mk60_emulator_config_t *config,
                              const mk60_received_frame_t *frame);
bool mk60_emulator_run_burst(const mk60_emulator_config_t *config,
                             mk60_delay_callback_t delay_callback,
                             mk60_transmit_callback_t transmit_callback,
                             void *context);

#endif
