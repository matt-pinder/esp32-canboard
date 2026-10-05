#pragma once

#include <stdbool.h>

#include "cJSON.h"
#include "inc/relay_rule_engine.h"

/* Create/parse the Outputs object nested inside the aggregate board config. */
cJSON *relay_rule_config_json_create(void);
bool relay_rule_config_json_parse(const cJSON *root, relay_rule_config_t *config);

/* Create sparse live telemetry arrays nested inside /api/live_values. */
cJSON *relay_condition_status_json_create(void);
cJSON *relay_rule_status_json_create(void);
