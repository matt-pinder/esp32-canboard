#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
    bool extd;
    bool rtr;
} twai_message_t;
