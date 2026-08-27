#include "inc/relay_rule_http.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inc/relay_rule_engine.h"

static bool number(const cJSON *object, const char *key, double *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
    *value = item->valuedouble;
    return true;
}

static bool boolean(const cJSON *object, const char *key, bool *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsBool(item)) return false;
    *value = cJSON_IsTrue(item);
    return true;
}

static cJSON *can_source_json(const relay_source_config_t *source)
{
    cJSON *item = cJSON_CreateObject();
    cJSON_AddStringToObject(item, "name", source->name);
    cJSON_AddNumberToObject(item, "can_id", source->can_id);
    cJSON_AddBoolToObject(item, "extended", source->extended);
    cJSON_AddNumberToObject(item, "start_bit", source->start_bit);
    cJSON_AddNumberToObject(item, "bit_length", source->bit_length);
    cJSON_AddBoolToObject(item, "little_endian", source->little_endian);
    cJSON_AddBoolToObject(item, "is_signed", source->is_signed);
    cJSON_AddNumberToObject(item, "factor", source->factor);
    cJSON_AddNumberToObject(item, "offset", source->offset);
    cJSON_AddNumberToObject(item, "zero_confirm_samples", source->zero_confirm_samples);
    return item;
}

static cJSON *rules_json(const relay_rule_config_t *config)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "version", config->version);
    cJSON_AddNumberToObject(root, "signal_timeout_ms", config->signal_timeout_ms);
    bool used_sources[RELAY_RULE_MAX_SOURCES] = {0};
    for (unsigned r = 0; r < RELAY_RULE_MAX_RULES; ++r) {
        const relay_output_rule_t *rule = &config->rules[r];
        if (!rule->enabled && rule->case_count == 0U) continue;
        for (unsigned c = 0; c < rule->case_count; ++c) {
            const relay_rule_case_t *entry = &rule->cases[c];
            for (unsigned t = 0; t < entry->test_count; ++t)
                if (entry->tests[t].type == RELAY_TEST_SOURCE)
                    used_sources[entry->tests[t].source_index] = true;
            if (entry->action == RELAY_ACTION_PULSE)
                used_sources[entry->pulse_source_index] = true;
        }
    }
    cJSON *sources = cJSON_AddArrayToObject(root, "sources");
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i)
        if (used_sources[i] && config->sources[i].type == RELAY_SOURCE_CAN)
            cJSON_AddItemToArray(sources, can_source_json(&config->sources[i]));
    cJSON *rules = cJSON_AddArrayToObject(root, "rules");
    for (unsigned r = 0; r < RELAY_RULE_MAX_RULES; ++r) {
        const relay_output_rule_t *rule = &config->rules[r];
        if (!rule->enabled && rule->case_count == 0U) continue;
        cJSON *rule_json = cJSON_CreateObject();
        cJSON_AddNumberToObject(rule_json, "rule", r + 1U);
        cJSON_AddStringToObject(rule_json, "label", rule->label);
        cJSON_AddBoolToObject(rule_json, "enabled", rule->enabled);
        cJSON *cases = cJSON_AddArrayToObject(rule_json, "cases");
        for (unsigned c = 0; c < rule->case_count; ++c) {
            const relay_rule_case_t *entry = &rule->cases[c];
            cJSON *case_json = cJSON_CreateObject();
            cJSON_AddNumberToObject(case_json, "action", entry->action);
            if (entry->action == RELAY_ACTION_PULSE) {
                cJSON_AddStringToObject(case_json, "pulse_source_name",
                                        config->sources[entry->pulse_source_index].name);
                cJSON_AddNumberToObject(case_json, "pulse_hysteresis", entry->pulse_hysteresis);
            }
            cJSON *tests = cJSON_AddArrayToObject(case_json, "tests");
            for (unsigned t = 0; t < entry->test_count; ++t) {
                const relay_rule_test_t *test = &entry->tests[t];
                cJSON *test_json = cJSON_CreateObject();
                cJSON_AddNumberToObject(test_json, "type", test->type);
                if (test->type == RELAY_TEST_SOURCE)
                    cJSON_AddStringToObject(test_json, "source_name",
                                            config->sources[test->source_index].name);
                cJSON_AddNumberToObject(test_json, "comparison", test->comparison);
                cJSON_AddBoolToObject(test_json, "hysteresis_enabled", test->hysteresis_enabled);
                cJSON_AddNumberToObject(test_json, "threshold", test->threshold);
                cJSON_AddNumberToObject(test_json, "hysteresis", test->hysteresis);
                cJSON_AddItemToArray(tests, test_json);
            }
            if (entry->action == RELAY_ACTION_PULSE) {
                cJSON *points = cJSON_AddArrayToObject(case_json, "pulse_points");
                for (unsigned p = 0; p < entry->pulse_point_count; ++p) {
                    cJSON *point = cJSON_CreateObject();
                    cJSON_AddNumberToObject(point, "input_value", entry->pulse_points[p].input_value);
                    cJSON_AddNumberToObject(point, "on_time_ms", entry->pulse_points[p].on_time_ms);
                    cJSON_AddNumberToObject(point, "period_ms", entry->pulse_points[p].period_ms);
                    cJSON_AddItemToArray(points, point);
                }
            }
            cJSON_AddItemToArray(cases, case_json);
        }
        cJSON_AddItemToArray(rules, rule_json);
    }
    return root;
}

static bool parse_can_source(cJSON *item, relay_source_config_t *source)
{
    double value;
    cJSON *name = cJSON_GetObjectItemCaseSensitive(item, "name");
    if (!cJSON_IsString(name) || name->valuestring == NULL || name->valuestring[0] == '\0' ||
        strlen(name->valuestring) >= sizeof(source->name)) return false;
    strlcpy(source->name, name->valuestring, sizeof(source->name));
    source->type = RELAY_SOURCE_CAN;
    if (!number(item, "can_id", &value) || value < 0 || value > 0x1FFFFFFF) return false;
    source->can_id = (uint32_t)value;
    if (!boolean(item, "extended", &source->extended) || !number(item, "start_bit", &value) || value < 0 || value > 63) return false;
    source->start_bit = (uint8_t)value;
    if (!number(item, "bit_length", &value) || value < 0 || value > 64) return false;
    source->bit_length = (uint8_t)value;
    if (!boolean(item, "little_endian", &source->little_endian) || !boolean(item, "is_signed", &source->is_signed) ||
        !number(item, "factor", &value)) return false;
    source->factor = (float)value;
    if (!number(item, "offset", &value)) return false;
    source->offset = (float)value;
    /* Zero means use the engine default (11 samples). Unused source slots in
     * the aggregate configuration intentionally serialize as zero. */
    if (!number(item, "zero_confirm_samples", &value) || value < 0 || value > UINT8_MAX) return false;
    source->zero_confirm_samples = (uint8_t)value;
    return true;
}

static int source_index_by_name(const relay_rule_config_t *config, const char *name)
{
    if (name == NULL || name[0] == '\0') return -1;
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i)
        if (config->sources[i].type != RELAY_SOURCE_UNUSED &&
            strcmp(config->sources[i].name, name) == 0) return (int)i;
    return -1;
}

static bool parse_case(cJSON *item, relay_rule_case_t *entry,
                       const relay_rule_config_t *config)
{
    double value;
    cJSON *tests = cJSON_GetObjectItemCaseSensitive(item, "tests");
    cJSON *points = cJSON_GetObjectItemCaseSensitive(item, "pulse_points");
    if (!number(item, "action", &value) || value < RELAY_ACTION_OFF || value > RELAY_ACTION_PULSE ||
        !cJSON_IsArray(tests)) return false;
    entry->action = (relay_action_t)cJSON_GetObjectItemCaseSensitive(item, "action")->valueint;
    if (entry->action == RELAY_ACTION_PULSE) {
        cJSON *pulse_source = cJSON_GetObjectItemCaseSensitive(item, "pulse_source_name");
        cJSON *pulse_hysteresis = cJSON_GetObjectItemCaseSensitive(item, "pulse_hysteresis");
        if (!cJSON_IsString(pulse_source) || pulse_source->valuestring == NULL ||
            !cJSON_IsNumber(pulse_hysteresis) || !isfinite(pulse_hysteresis->valuedouble) ||
            pulse_hysteresis->valuedouble < 0.0 || !cJSON_IsArray(points)) return false;
        const int source_index = source_index_by_name(config, pulse_source->valuestring);
        if (source_index < 0) return false;
        entry->pulse_source_index = (uint8_t)source_index;
        entry->pulse_hysteresis = (float)pulse_hysteresis->valuedouble;
    }
    const int test_count = cJSON_GetArraySize(tests);
    const int point_count = entry->action == RELAY_ACTION_PULSE ? cJSON_GetArraySize(points) : 0;
    if (test_count < 1 || test_count > RELAY_RULE_MAX_TESTS || point_count > RELAY_RULE_MAX_PULSE_POINTS) return false;
    entry->test_count = (uint8_t)test_count;
    entry->pulse_point_count = (uint8_t)point_count;
    for (int t = 0; t < test_count; ++t) {
        cJSON *test_json = cJSON_GetArrayItem(tests, t);
        relay_rule_test_t *test = &entry->tests[t];
        if (!cJSON_IsObject(test_json) || !number(test_json, "type", &value) || value < RELAY_TEST_SOURCE || value > RELAY_TEST_UPTIME) return false;
        test->type = (relay_test_type_t)value;
        if (test->type == RELAY_TEST_SOURCE) {
            cJSON *source_name = cJSON_GetObjectItemCaseSensitive(test_json, "source_name");
            if (!cJSON_IsString(source_name) || source_name->valuestring == NULL) return false;
            const int source_index = source_index_by_name(config, source_name->valuestring);
            if (source_index < 0) return false;
            test->source_index = (uint8_t)source_index;
        }
        if (!number(test_json, "comparison", &value) || value < RELAY_COMPARE_GT || value > RELAY_COMPARE_NE) return false;
        test->comparison = (relay_compare_t)value;
        if (!boolean(test_json, "hysteresis_enabled", &test->hysteresis_enabled) || !number(test_json, "threshold", &value)) return false;
        test->threshold = (float)value;
        if (!number(test_json, "hysteresis", &value)) return false;
        test->hysteresis = (float)value;
    }
    for (int p = 0; p < point_count; ++p) {
        cJSON *point_json = cJSON_GetArrayItem(points, p);
        relay_pulse_point_t *point = &entry->pulse_points[p];
        if (!cJSON_IsObject(point_json) || !number(point_json, "input_value", &value)) return false;
        point->input_value = (float)value;
        if (!number(point_json, "on_time_ms", &value) || value < 0 || value > UINT32_MAX) return false;
        point->on_time_ms = (uint32_t)value;
        if (!number(point_json, "period_ms", &value) || value < 0 || value > UINT32_MAX) return false;
        point->period_ms = (uint32_t)value;
    }
    return true;
}

bool relay_rule_config_json_parse(const cJSON *root, relay_rule_config_t *config)
{
    double value;
    relay_rule_engine_set_defaults(config);
    if (!number(root, "signal_timeout_ms", &value) || value < 100 || value > 60000) return false;
    config->signal_timeout_ms = (uint32_t)value;
    cJSON *sources = cJSON_GetObjectItemCaseSensitive(root, "sources");
    cJSON *rules = cJSON_GetObjectItemCaseSensitive(root, "rules");
    if (!cJSON_IsArray(sources) || cJSON_GetArraySize(sources) > RELAY_RULE_MAX_SOURCES - 20U ||
        !cJSON_IsArray(rules) || cJSON_GetArraySize(rules) > RELAY_RULE_MAX_RULES) return false;
    const unsigned source_count = (unsigned)cJSON_GetArraySize(sources);
    for (unsigned source = 0; source < source_count; ++source) {
        relay_source_config_t parsed = {0};
        if (!parse_can_source(cJSON_GetArrayItem(sources, source), &parsed) ||
            source_index_by_name(config, parsed.name) >= 0) return false;
        config->sources[20U + source] = parsed;
    }
    bool used[RELAY_RULE_MAX_RULES] = {0};
    const unsigned rule_count = (unsigned)cJSON_GetArraySize(rules);
    for (unsigned position = 0; position < rule_count; ++position) {
        cJSON *rule_json = cJSON_GetArrayItem(rules, position);
        cJSON *rule_number = cJSON_GetObjectItemCaseSensitive(rule_json, "rule");
        unsigned r = position; /* Backward compatibility with the former full array. */
        if (rule_number != NULL) {
            if (!cJSON_IsNumber(rule_number) || rule_number->valueint < 1 ||
                rule_number->valueint > RELAY_RULE_MAX_RULES) return false;
            r = (unsigned)rule_number->valueint - 1U;
        }
        if (used[r]) return false;
        used[r] = true;
        cJSON *label = cJSON_GetObjectItemCaseSensitive(rule_json, "label");
        cJSON *cases = cJSON_GetObjectItemCaseSensitive(rule_json, "cases");
        relay_output_rule_t *rule = &config->rules[r];
        if (!cJSON_IsObject(rule_json) || !cJSON_IsString(label) || label->valuestring == NULL ||
            strlen(label->valuestring) >= sizeof(rule->label) || !boolean(rule_json, "enabled", &rule->enabled) || !cJSON_IsArray(cases) ||
            cJSON_GetArraySize(cases) > RELAY_RULE_MAX_CASES) return false;
        strlcpy(rule->label, label->valuestring, sizeof(rule->label));
        rule->case_count = (uint8_t)cJSON_GetArraySize(cases);
        for (unsigned c = 0; c < rule->case_count; ++c) {
            if (!parse_case(cJSON_GetArrayItem(cases, c), &rule->cases[c], config)) return false;
        }
    }
    return true;
}

cJSON *relay_rule_config_json_create(void)
{
    relay_rule_config_t *config = malloc(sizeof(*config));
    if (config == NULL) return NULL;
    relay_rule_engine_snapshot(config);
    cJSON *root = rules_json(config);
    free(config);
    return root;
}

cJSON *relay_rule_status_json_create(void)
{
    relay_rule_status_t *rules = malloc(sizeof(*rules) * RELAY_RULE_MAX_RULES);
    relay_rule_source_status_t *sources = malloc(sizeof(*sources) * RELAY_RULE_MAX_SOURCES);
    if (rules == NULL || sources == NULL) {
        free(rules);
        free(sources);
        return NULL;
    }
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    relay_rule_engine_get_status(rules, sources, now_ms);
    cJSON *rule_array = cJSON_CreateArray();
    if (rule_array == NULL) {
        free(rules);
        free(sources);
        return NULL;
    }
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i) {
        if (!rules[i].configured) continue;
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "rule", i + 1U);
        cJSON_AddBoolToObject(item, "enabled", rules[i].enabled);
        if (!rules[i].enabled) {
            cJSON_AddItemToArray(rule_array, item);
            continue;
        }
        cJSON_AddBoolToObject(item, "valid", rules[i].valid);
        cJSON_AddBoolToObject(item, "state", rules[i].state);
        cJSON_AddBoolToObject(item, "pulse_active", rules[i].pulse_active);
        if (rules[i].valid) {
            cJSON_AddNumberToObject(item, "selected_case", rules[i].selected_case);
        }
        if (rules[i].valid && rules[i].pulse_active) {
            cJSON_AddNumberToObject(item, "pulse_on_time_ms", rules[i].pulse_on_time_ms);
            cJSON_AddNumberToObject(item, "pulse_period_ms", rules[i].pulse_period_ms);
            cJSON_AddNumberToObject(item, "pulse_next_on_ms", rules[i].pulse_next_on_ms);
        }
        if (!rules[i].valid) {
            cJSON_AddNumberToObject(item, "invalid_case", rules[i].invalid_case);
            cJSON_AddNumberToObject(item, "invalid_test", rules[i].invalid_test);
            cJSON_AddBoolToObject(item, "invalid_pulse_source", rules[i].invalid_pulse_source);
            cJSON_AddNumberToObject(item, "invalid_reason", rules[i].invalid_reason);
            if (rules[i].invalid_source >= 0 &&
                (unsigned)rules[i].invalid_source < RELAY_RULE_MAX_SOURCES) {
                const relay_rule_source_status_t *source =
                    &sources[(unsigned)rules[i].invalid_source];
                cJSON_AddStringToObject(item, "invalid_source_name", source->name);
                cJSON_AddNumberToObject(item, "invalid_source_age_ms", source->age_ms);
                cJSON_AddNumberToObject(item, "invalid_source_zero_streak", source->zero_streak);
                cJSON_AddNumberToObject(item, "invalid_source_zero_confirm_samples", source->zero_confirm_samples);
            }
        }
        cJSON_AddItemToArray(rule_array, item);
    }
    free(rules);
    free(sources);
    return rule_array;
}
