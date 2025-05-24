#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"

#define DRIVECAN_TX_GPIO_NUM       GPIO_NUM_12
#define DRIVECAN_RX_GPIO_NUM       GPIO_NUM_11

#define CAN_BASEID 0x620

extern twai_handle_t twai_can;

static const char* can_log = "can";

extern twai_timing_config_t t_can_config;
extern twai_filter_config_t f_config;
extern twai_general_config_t can_config; 

void canTransmit(void *arg);
