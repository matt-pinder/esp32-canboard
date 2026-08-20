#include "inc/espnow_can_protocol.h"

#include <string.h>

_Static_assert(ESPNOW_CAN_MAX_PACKET_SIZE == 248U, "ESP-NOW CAN packet must remain below 250 bytes");

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

size_t espnow_can_encode_batch(uint8_t *output,
                               size_t output_size,
                               const espnow_can_batch_meta_t *meta,
                               const espnow_can_frame_t *frames,
                               uint8_t frame_count)
{
    if (output == NULL || meta == NULL || frames == NULL || frame_count == 0U ||
        frame_count > ESPNOW_CAN_MAX_FRAMES)
    {
        return 0U;
    }

    const size_t packet_size = ESPNOW_CAN_HEADER_SIZE + ((size_t)frame_count * ESPNOW_CAN_FRAME_SIZE);
    if (output_size < packet_size)
    {
        return 0U;
    }

    output[0] = ESPNOW_CAN_MAGIC_0;
    output[1] = ESPNOW_CAN_MAGIC_1;
    output[2] = ESPNOW_CAN_PROTOCOL_VERSION;
    output[3] = frame_count;
    write_u16_le(output + 4, meta->sequence);
    write_u32_le(output + 6, meta->sender_uptime_ms);
    write_u16_le(output + 10, meta->sender_frame_drops);
    write_u16_le(output + 12, meta->sender_send_failures);

    for (uint8_t i = 0; i < frame_count; ++i)
    {
        const espnow_can_frame_t *frame = &frames[i];
        if (frame->data_length_code > sizeof(frame->data) ||
            frame->identifier > (frame->extd ? ESPNOW_CAN_ID_MASK : 0x7FFU))
        {
            return 0U;
        }

        uint8_t *encoded = output + ESPNOW_CAN_HEADER_SIZE + ((size_t)i * ESPNOW_CAN_FRAME_SIZE);
        uint32_t id_flags = frame->identifier;
        if (frame->extd)
        {
            id_flags |= ESPNOW_CAN_FLAG_EXTD;
        }
        if (frame->rtr)
        {
            id_flags |= ESPNOW_CAN_FLAG_RTR;
        }
        write_u32_le(encoded, id_flags);
        encoded[4] = frame->data_length_code;
        memset(encoded + 5, 0, 8);
        if (!frame->rtr && frame->data_length_code > 0U)
        {
            memcpy(encoded + 5, frame->data, frame->data_length_code);
        }
    }

    return packet_size;
}

bool espnow_can_decode_header(const uint8_t *packet,
                              size_t packet_size,
                              espnow_can_batch_meta_t *meta,
                              uint8_t *frame_count)
{
    if (packet == NULL || meta == NULL || frame_count == NULL || packet_size < ESPNOW_CAN_HEADER_SIZE ||
        packet[0] != ESPNOW_CAN_MAGIC_0 || packet[1] != ESPNOW_CAN_MAGIC_1 ||
        packet[2] != ESPNOW_CAN_PROTOCOL_VERSION)
    {
        return false;
    }

    const uint8_t count = packet[3];
    if (count == 0U || count > ESPNOW_CAN_MAX_FRAMES ||
        packet_size != ESPNOW_CAN_HEADER_SIZE + ((size_t)count * ESPNOW_CAN_FRAME_SIZE))
    {
        return false;
    }

    meta->sequence = read_u16_le(packet + 4);
    meta->sender_uptime_ms = read_u32_le(packet + 6);
    meta->sender_frame_drops = read_u16_le(packet + 10);
    meta->sender_send_failures = read_u16_le(packet + 12);
    *frame_count = count;
    return true;
}

bool espnow_can_decode_frame(const uint8_t *packet,
                             size_t packet_size,
                             uint8_t frame_index,
                             espnow_can_frame_t *frame)
{
    espnow_can_batch_meta_t meta;
    uint8_t frame_count;
    if (frame == NULL || !espnow_can_decode_header(packet, packet_size, &meta, &frame_count) ||
        frame_index >= frame_count)
    {
        return false;
    }

    const uint8_t *encoded = packet + ESPNOW_CAN_HEADER_SIZE + ((size_t)frame_index * ESPNOW_CAN_FRAME_SIZE);
    const uint32_t id_flags = read_u32_le(encoded);
    if ((id_flags & ESPNOW_CAN_FLAG_RESERVED) != 0U || encoded[4] > sizeof(frame->data))
    {
        return false;
    }

    frame->identifier = id_flags & ESPNOW_CAN_ID_MASK;
    frame->extd = (id_flags & ESPNOW_CAN_FLAG_EXTD) != 0U;
    frame->rtr = (id_flags & ESPNOW_CAN_FLAG_RTR) != 0U;
    frame->data_length_code = encoded[4];
    if ((!frame->extd && frame->identifier > 0x7FFU) || frame->identifier > ESPNOW_CAN_ID_MASK)
    {
        return false;
    }
    memcpy(frame->data, encoded + 5, sizeof(frame->data));
    return true;
}
