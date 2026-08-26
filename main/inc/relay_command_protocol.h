#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RELAY_COMMAND_PROTOCOL_VERSION 1U
#define RELAY_COMMAND_RULE_COUNT 16U
#define RELAY_COMMAND_FRAME_DLC 8U

typedef struct {
    uint16_t state_mask;
    uint16_t valid_mask;
    uint16_t pulse_mask;
    uint8_t counter;
} relay_command_t;

static inline uint32_t relay_command_can_id(uint32_t can_start_id)
{
    return can_start_id + 5U;
}

static inline void relay_command_encode(uint8_t data[RELAY_COMMAND_FRAME_DLC],
                                        const relay_command_t *command)
{
    data[0] = (uint8_t)command->state_mask;
    data[1] = (uint8_t)(command->state_mask >> 8);
    data[2] = (uint8_t)command->valid_mask;
    data[3] = (uint8_t)(command->valid_mask >> 8);
    data[4] = (uint8_t)command->pulse_mask;
    data[5] = (uint8_t)(command->pulse_mask >> 8);
    data[6] = command->counter;
    data[7] = RELAY_COMMAND_PROTOCOL_VERSION;
}

static inline bool relay_command_decode(const uint8_t data[RELAY_COMMAND_FRAME_DLC],
                                        relay_command_t *command)
{
    if (data == NULL || command == NULL ||
        data[7] != RELAY_COMMAND_PROTOCOL_VERSION) return false;
    command->state_mask = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    command->valid_mask = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    command->pulse_mask = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    command->counter = data[6];
    return true;
}
