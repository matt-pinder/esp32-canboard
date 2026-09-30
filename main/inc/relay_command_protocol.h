#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define OUTPUT_COMMAND_PROTOCOL_VERSION 2U
#define OUTPUT_COMMAND_COUNT 8U
#define OUTPUT_BINARY_FRAME_DLC 8U
#define OUTPUT_DUTY_FRAME_DLC 8U
#define OUTPUT_CAN_BASE_MAX 0x7F9U

typedef struct {
    uint8_t state_mask;
    uint8_t valid_mask;
    uint8_t pulse_mask;
    uint8_t duty_percent[OUTPUT_COMMAND_COUNT];
    uint8_t counter;
} output_command_t;

static inline bool output_command_base_can_id_valid(uint32_t can_start_id)
{
    return can_start_id <= OUTPUT_CAN_BASE_MAX;
}

static inline uint32_t output_binary_can_id(uint32_t can_start_id)
{
    return can_start_id + 5U;
}

static inline uint32_t output_duty_can_id(uint32_t can_start_id)
{
    return can_start_id + 6U;
}

static inline bool output_command_can_id_reserved(uint32_t can_start_id, uint32_t identifier)
{
    return identifier == output_binary_can_id(can_start_id) ||
           identifier == output_duty_can_id(can_start_id);
}

static inline bool output_binary_encode(uint8_t data[OUTPUT_BINARY_FRAME_DLC],
                                        const output_command_t *command)
{
    if (data == NULL || command == NULL) return false;
    data[0] = command->state_mask;
    data[1] = 0U;
    data[2] = command->valid_mask;
    data[3] = 0U;
    data[4] = command->pulse_mask;
    data[5] = 0U;
    data[6] = command->counter;
    data[7] = OUTPUT_COMMAND_PROTOCOL_VERSION;
    return true;
}

static inline bool output_binary_decode(const uint8_t data[OUTPUT_BINARY_FRAME_DLC],
                                        output_command_t *command)
{
    if (data == NULL || command == NULL ||
        data[7] != OUTPUT_COMMAND_PROTOCOL_VERSION ||
        data[1] != 0U || data[3] != 0U || data[5] != 0U) {
        return false;
    }
    memset(command, 0, sizeof(*command));
    command->state_mask = data[0];
    command->valid_mask = data[2];
    command->pulse_mask = data[4];
    command->counter = data[6];
    return true;
}

static inline bool output_duty_encode(uint8_t data[OUTPUT_DUTY_FRAME_DLC],
                                      const output_command_t *command)
{
    if (data == NULL || command == NULL) return false;
    memset(data, 0, OUTPUT_DUTY_FRAME_DLC);
    for (unsigned output = 0; output < OUTPUT_COMMAND_COUNT; ++output) {
        const uint8_t duty = command->duty_percent[output];
        if (duty > 100U) return false;
        const unsigned start_bit = output * 7U;
        const unsigned byte_index = start_bit / 8U;
        const unsigned bit_offset = start_bit % 8U;
        const uint16_t packed = (uint16_t)duty << bit_offset;
        data[byte_index] |= (uint8_t)packed;
        if (bit_offset + 7U > 8U) data[byte_index + 1U] |= (uint8_t)(packed >> 8U);
    }
    data[7] = command->counter;
    return true;
}

static inline bool output_duty_decode(const uint8_t data[OUTPUT_DUTY_FRAME_DLC],
                                      output_command_t *command)
{
    if (data == NULL || command == NULL) return false;
    for (unsigned output = 0; output < OUTPUT_COMMAND_COUNT; ++output) {
        const unsigned start_bit = output * 7U;
        const unsigned byte_index = start_bit / 8U;
        const unsigned bit_offset = start_bit % 8U;
        uint16_t packed = data[byte_index];
        if (bit_offset + 7U > 8U) packed |= (uint16_t)data[byte_index + 1U] << 8U;
        const uint8_t duty = (uint8_t)((packed >> bit_offset) & 0x7FU);
        if (duty > 100U) return false;
        command->duty_percent[output] = duty;
    }
    command->counter = data[7];
    return true;
}

static inline bool output_command_decode_pair(const uint8_t binary[OUTPUT_BINARY_FRAME_DLC],
                                              const uint8_t duty[OUTPUT_DUTY_FRAME_DLC],
                                              output_command_t *command)
{
    if (command == NULL) return false;
    output_command_t binary_command;
    output_command_t duty_command = {0};
    if (!output_binary_decode(binary, &binary_command) ||
        !output_duty_decode(duty, &duty_command) ||
        binary_command.counter != duty_command.counter) {
        return false;
    }
    binary_command.counter = duty_command.counter;
    memcpy(binary_command.duty_percent, duty_command.duty_percent,
           sizeof(binary_command.duty_percent));
    *command = binary_command;
    return true;
}
