#pragma once

#include <stdbool.h>

#include "cJSON.h"
#include "inc/relay_rule_engine.h"

/* Create/parse the rule object nested inside the aggregate board config. */
cJSON *relay_rule_config_json_create(void);
bool relay_rule_config_json_parse(const cJSON *root, relay_rule_config_t *config);

/* Create the sparse rule telemetry array nested inside /api/live_values. */
cJSON *relay_rule_status_json_create(void);
