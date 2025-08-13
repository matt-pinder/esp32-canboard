#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"

#define DRIVECAN_TX_GPIO_NUM       GPIO_NUM_12
#define DRIVECAN_RX_GPIO_NUM       GPIO_NUM_11

#define CAN_BASEID 0x076c

extern twai_handle_t twai_can;

static const char* can_log = "can";

static twai_timing_config_t t_can_config = TWAI_TIMING_CONFIG_500KBITS();
static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static twai_general_config_t can_config = { .controller_id = 0, .mode = TWAI_MODE_NORMAL, .tx_io = DRIVECAN_TX_GPIO_NUM, .rx_io = DRIVECAN_RX_GPIO_NUM,
                                   .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 64, .rx_queue_len = 64,
                                   .alerts_enabled = TWAI_ALERT_NONE, .clkout_divider = 0 };

void canTransmit(void *arg);
