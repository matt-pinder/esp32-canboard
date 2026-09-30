#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "espnow_can_protocol.h"
#include "relay_command_protocol.h"

static void assert_frame_equal(const espnow_can_frame_t *actual, const espnow_can_frame_t *expected)
{
    assert(actual->identifier == expected->identifier);
    assert(actual->data_length_code == expected->data_length_code);
    assert(actual->extd == expected->extd);
    assert(actual->rtr == expected->rtr);
    if (!expected->rtr)
    {
        assert(memcmp(actual->data, expected->data, expected->data_length_code) == 0);
    }
}

static void test_round_trips(void)
{
    espnow_can_frame_t frames[ESPNOW_CAN_MAX_FRAMES] = {0};
    for (uint8_t i = 0U; i < ESPNOW_CAN_MAX_FRAMES; ++i)
    {
        frames[i].identifier = (i & 1U) ? (0x1ABCDE0U + i) : (0x500U + i);
        frames[i].extd = (i & 1U) != 0U;
        frames[i].rtr = i == 7U;
        frames[i].data_length_code = i == 7U ? 4U : 8U;
        for (uint8_t j = 0U; j < 8U; ++j)
        {
            frames[i].data[j] = (uint8_t)(i * 8U + j);
        }
    }
    const espnow_can_batch_meta_t meta = {
        .sequence = UINT16_MAX,
        .sender_uptime_ms = 0x12345678U,
        .sender_frame_drops = 12U,
        .sender_send_failures = 34U,
    };
    uint8_t packet[ESPNOW_CAN_MAX_PACKET_SIZE];

    size_t size = espnow_can_encode_batch(packet, sizeof(packet), &meta, frames, 1U);
    assert(size == ESPNOW_CAN_HEADER_SIZE + ESPNOW_CAN_FRAME_SIZE);
    espnow_can_batch_meta_t decoded_meta;
    uint8_t decoded_count;
    assert(espnow_can_decode_header(packet, size, &decoded_meta, &decoded_count));
    assert(decoded_count == 1U);
    assert(memcmp(&meta, &decoded_meta, sizeof(meta)) == 0);
    espnow_can_frame_t decoded;
    assert(espnow_can_decode_frame(packet, size, 0U, &decoded));
    assert_frame_equal(&decoded, &frames[0]);

    espnow_can_batch_meta_t wrapped_meta = meta;
    wrapped_meta.sequence = 0U;
    size = espnow_can_encode_batch(packet, sizeof(packet), &wrapped_meta, frames, ESPNOW_CAN_MAX_FRAMES);
    assert(size == ESPNOW_CAN_MAX_PACKET_SIZE);
    assert(espnow_can_decode_header(packet, size, &decoded_meta, &decoded_count));
    assert((uint16_t)(decoded_meta.sequence - meta.sequence) == 1U);
    for (uint8_t i = 0U; i < ESPNOW_CAN_MAX_FRAMES; ++i)
    {
        assert(espnow_can_decode_frame(packet, size, i, &decoded));
        assert_frame_equal(&decoded, &frames[i]);
    }
}

static void test_malformed_packets(void)
{
    const espnow_can_batch_meta_t meta = {0};
    espnow_can_frame_t frame = {.identifier = 0x123U, .data_length_code = 1U, .data = {0xAA}};
    uint8_t packet[ESPNOW_CAN_MAX_PACKET_SIZE];
    const size_t size = espnow_can_encode_batch(packet, sizeof(packet), &meta, &frame, 1U);
    espnow_can_batch_meta_t decoded_meta;
    uint8_t count;

    uint8_t saved = packet[0];
    packet[0] = 0U;
    assert(!espnow_can_decode_header(packet, size, &decoded_meta, &count));
    packet[0] = saved;
    saved = packet[2];
    packet[2]++;
    assert(!espnow_can_decode_header(packet, size, &decoded_meta, &count));
    packet[2] = saved;
    saved = packet[3];
    packet[3] = 0U;
    assert(!espnow_can_decode_header(packet, size, &decoded_meta, &count));
    packet[3] = ESPNOW_CAN_MAX_FRAMES + 1U;
    assert(!espnow_can_decode_header(packet, size, &decoded_meta, &count));
    packet[3] = saved;
    assert(!espnow_can_decode_header(packet, size - 1U, &decoded_meta, &count));

    packet[ESPNOW_CAN_HEADER_SIZE + 4U] = 9U;
    espnow_can_frame_t decoded;
    assert(!espnow_can_decode_frame(packet, size, 0U, &decoded));
    frame.data_length_code = 9U;
    assert(espnow_can_encode_batch(packet, sizeof(packet), &meta, &frame, 1U) == 0U);
    frame.data_length_code = 1U;
    frame.identifier = 0x800U;
    assert(espnow_can_encode_batch(packet, sizeof(packet), &meta, &frame, 1U) == 0U);
    assert(espnow_can_encode_batch(packet, sizeof(packet), &meta, &frame, 0U) == 0U);
}

static void reference_pack_duty(uint8_t data[OUTPUT_DUTY_FRAME_DLC],
                                const uint8_t duties[OUTPUT_COMMAND_COUNT], uint8_t counter)
{
    memset(data, 0, OUTPUT_DUTY_FRAME_DLC);
    for (unsigned output = 0; output < OUTPUT_COMMAND_COUNT; ++output)
    {
        for (unsigned bit = 0; bit < 7U; ++bit)
        {
            if ((duties[output] & (1U << bit)) == 0U) continue;
            const unsigned packed_bit = output * 7U + bit;
            data[packed_bit / 8U] |= (uint8_t)(1U << (packed_bit % 8U));
        }
    }
    data[7] = counter;
}

static void test_output_binary_encoding(void)
{
    const output_command_t command = {
        .state_mask = 0x81U,
        .valid_mask = 0x42U,
        .pulse_mask = 0x24U,
        .counter = 0xA5U,
    };
    uint8_t bytes[OUTPUT_BINARY_FRAME_DLC];
    assert(output_binary_encode(bytes, &command));
    const uint8_t expected[OUTPUT_BINARY_FRAME_DLC] =
        {0x81U, 0x00U, 0x42U, 0x00U, 0x24U, 0x00U, 0xA5U, 0x02U};
    assert(memcmp(bytes, expected, sizeof(bytes)) == 0);

    output_command_t decoded;
    assert(output_binary_decode(bytes, &decoded));
    assert(decoded.state_mask == command.state_mask);
    assert(decoded.valid_mask == command.valid_mask);
    assert(decoded.pulse_mask == command.pulse_mask);
    assert(decoded.counter == command.counter);

    bytes[7] = 1U;
    assert(!output_binary_decode(bytes, &decoded));
    bytes[7] = OUTPUT_COMMAND_PROTOCOL_VERSION;
    bytes[1] = 1U;
    assert(!output_binary_decode(bytes, &decoded));
    bytes[1] = 0U;
    bytes[3] = 1U;
    assert(!output_binary_decode(bytes, &decoded));
    bytes[3] = 0U;
    bytes[5] = 1U;
    assert(!output_binary_decode(bytes, &decoded));

    assert(output_command_base_can_id_valid(0x000U));
    assert(output_command_base_can_id_valid(0x7F9U));
    assert(!output_command_base_can_id_valid(0x7FAU));
    assert(output_binary_can_id(0x7F9U) == 0x7FEU);
    assert(output_duty_can_id(0x7F9U) == 0x7FFU);
    assert(output_command_can_id_reserved(0x100U, 0x105U));
    assert(output_command_can_id_reserved(0x100U, 0x106U));
    assert(!output_command_can_id_reserved(0x100U, 0x104U));
}

static void test_output_duty_encoding(void)
{
    output_command_t command = {
        .duty_percent = {0U, 1U, 99U, 100U, 1U, 99U, 100U, 0U},
        .counter = 0xFEU,
    };
    uint8_t bytes[OUTPUT_DUTY_FRAME_DLC];
    uint8_t expected[OUTPUT_DUTY_FRAME_DLC];
    reference_pack_duty(expected, command.duty_percent, command.counter);
    assert(output_duty_encode(bytes, &command));
    assert(memcmp(bytes, expected, sizeof(bytes)) == 0);

    output_command_t decoded = {0};
    assert(output_duty_decode(bytes, &decoded));
    assert(memcmp(decoded.duty_percent, command.duty_percent,
                  sizeof(command.duty_percent)) == 0);
    assert(decoded.counter == command.counter);

    static const uint8_t boundaries[] = {0U, 1U, 99U, 100U};
    for (unsigned output = 0; output < OUTPUT_COMMAND_COUNT; ++output)
    {
        for (unsigned value = 0; value < sizeof(boundaries); ++value)
        {
            memset(&command, 0, sizeof(command));
            command.duty_percent[output] = boundaries[value];
            command.counter = (uint8_t)(output * 16U + value);
            reference_pack_duty(expected, command.duty_percent, command.counter);
            assert(output_duty_encode(bytes, &command));
            assert(memcmp(bytes, expected, sizeof(bytes)) == 0);
            memset(&decoded, 0, sizeof(decoded));
            assert(output_duty_decode(bytes, &decoded));
            assert(decoded.duty_percent[output] == boundaries[value]);
            assert(decoded.counter == command.counter);
        }
    }

    for (unsigned output = 0; output < OUTPUT_COMMAND_COUNT; ++output)
    {
        for (unsigned invalid = 101U; invalid <= 127U; ++invalid)
        {
            memset(&command, 0, sizeof(command));
            command.duty_percent[output] = (uint8_t)invalid;
            assert(!output_duty_encode(bytes, &command));

            memset(bytes, 0, sizeof(bytes));
            uint8_t malformed[OUTPUT_COMMAND_COUNT] = {0};
            malformed[output] = (uint8_t)invalid;
            reference_pack_duty(bytes, malformed, 0U);
            assert(!output_duty_decode(bytes, &decoded));
        }
    }
}

static void test_output_pair_espnow_order(void)
{
    const output_command_t command = {
        .state_mask = 0x80U,
        .valid_mask = 0xFFU,
        .pulse_mask = 0x01U,
        .duty_percent = {0U, 1U, 25U, 50U, 75U, 99U, 100U, 0U},
        .counter = 0x5AU,
    };
    espnow_can_frame_t frames[2] = {
        {.identifier = output_binary_can_id(0x100U), .data_length_code = 8U},
        {.identifier = output_duty_can_id(0x100U), .data_length_code = 8U},
    };
    assert(output_binary_encode(frames[0].data, &command));
    assert(output_duty_encode(frames[1].data, &command));

    const espnow_can_batch_meta_t meta = {.sequence = 9U};
    uint8_t packet[ESPNOW_CAN_MAX_PACKET_SIZE];
    const size_t size = espnow_can_encode_batch(packet, sizeof(packet), &meta, frames, 2U);
    assert(size != 0U);

    espnow_can_batch_meta_t decoded_meta;
    uint8_t count = 0U;
    assert(espnow_can_decode_header(packet, size, &decoded_meta, &count));
    assert(count == 2U);
    espnow_can_frame_t binary_frame;
    espnow_can_frame_t duty_frame;
    assert(espnow_can_decode_frame(packet, size, 0U, &binary_frame));
    assert(espnow_can_decode_frame(packet, size, 1U, &duty_frame));
    assert(binary_frame.identifier == 0x105U);
    assert(duty_frame.identifier == 0x106U);

    output_command_t paired;
    assert(output_command_decode_pair(binary_frame.data, duty_frame.data, &paired));
    assert(paired.counter == command.counter);
    assert(memcmp(paired.duty_percent, command.duty_percent,
                  sizeof(command.duty_percent)) == 0);
}

static void test_output_pairing(void)
{
    output_command_t command = {
        .state_mask = 0x01U,
        .valid_mask = 0x03U,
        .pulse_mask = 0x02U,
        .duty_percent = {100U, 0U, 1U, 99U, 50U, 25U, 75U, 100U},
        .counter = 0xFFU,
    };
    uint8_t binary[OUTPUT_BINARY_FRAME_DLC];
    uint8_t duty[OUTPUT_DUTY_FRAME_DLC];
    assert(output_binary_encode(binary, &command));
    assert(output_duty_encode(duty, &command));

    output_command_t decoded;
    assert(output_command_decode_pair(binary, duty, &decoded));
    assert(decoded.counter == 0xFFU);
    assert(decoded.state_mask == command.state_mask);
    assert(memcmp(decoded.duty_percent, command.duty_percent,
                  sizeof(command.duty_percent)) == 0);

    duty[7] = 0U; /* Normal counter wrap for the next pair, mismatch for this pair. */
    assert(!output_command_decode_pair(binary, duty, &decoded));
    binary[6] = 0U;
    assert(output_command_decode_pair(binary, duty, &decoded));
    assert(decoded.counter == 0U);
}

int main(void)
{
    test_round_trips();
    test_malformed_packets();
    test_output_binary_encoding();
    test_output_duty_encoding();
    test_output_pair_espnow_order();
    test_output_pairing();
    puts("espnow_can_protocol tests passed");
    return 0;
}
