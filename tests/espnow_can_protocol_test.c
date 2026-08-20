#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "espnow_can_protocol.h"

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

int main(void)
{
    test_round_trips();
    test_malformed_packets();
    puts("espnow_can_protocol tests passed");
    return 0;
}
