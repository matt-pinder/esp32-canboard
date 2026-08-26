#ifndef MK60_EMULATOR_H
#define MK60_EMULATOR_H

#include <stdbool.h>

#include "driver/twai_types_legacy.h"
#include "esp_err.h"
#include "mk60_emulator_protocol.h"

esp_err_t mk60_emulator_start(const mk60_emulator_config_t *config);
void mk60_emulator_apply_config(const mk60_emulator_config_t *config);
bool mk60_emulator_dispatch_frame(const twai_message_t *message);

#endif
