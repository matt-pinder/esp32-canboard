#include "inc/relay_rule_http.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inc/relay_rule_engine.h"

#define OUTPUT_CONFIG_VERSION 2U

static bool number(const cJSON *object, const char *key, double *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
    *value = item->valuedouble;
    return true;
}

static bool integer(const cJSON *object, const char *key, double minimum, double maximum,
                    double *value)
{
    if (!number(object, key, value) || *value < minimum || *value > maximum ||
        floor(*value) != *value) {
        return false;
    }
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

static cJSON *outputs_json(const relay_rule_config_t *config)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "version", config->version);
    cJSON_AddNumberToObject(root, "signal_timeout_ms", config->signal_timeout_ms);

    bool used_sources[RELAY_RULE_MAX_SOURCES] = {0};
    for (unsigned r = 0; r < RELAY_RULE_MAX_RULES; ++r) {
        const relay_output_rule_t *output = &config->rules[r];
        if (!output->enabled && output->case_count == 0U) continue;
        for (unsigned c = 0; c < output->case_count; ++c) {
            const relay_rule_case_t *entry = &output->cases[c];
            for (unsigned t = 0; t < entry->test_count; ++t) {
                if (entry->tests[t].type == RELAY_TEST_SOURCE)
                    used_sources[entry->tests[t].source_index] = true;
            }
            if (entry->action == RELAY_ACTION_PULSE)
                used_sources[entry->pulse_source_index] = true;
            else if (entry->action == RELAY_ACTION_PWM)
                used_sources[entry->pwm_source_index] = true;
        }
    }

    cJSON *sources = cJSON_AddArrayToObject(root, "sources");
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) {
        if (used_sources[i] && config->sources[i].type == RELAY_SOURCE_CAN)
            cJSON_AddItemToArray(sources, can_source_json(&config->sources[i]));
    }

    cJSON *outputs = cJSON_AddArrayToObject(root, "outputs");
    for (unsigned r = 0; r < RELAY_RULE_MAX_RULES; ++r) {
        const relay_output_rule_t *output = &config->rules[r];
        if (!output->enabled && output->case_count == 0U) continue;
        cJSON *output_json = cJSON_CreateObject();
        cJSON_AddNumberToObject(output_json, "output", r + 1U);
        cJSON_AddStringToObject(output_json, "label", output->label);
        cJSON_AddBoolToObject(output_json, "enabled", output->enabled);
        cJSON *cases = cJSON_AddArrayToObject(output_json, "cases");
        for (unsigned c = 0; c < output->case_count; ++c) {
            const relay_rule_case_t *entry = &output->cases[c];
            cJSON *case_json = cJSON_CreateObject();
            cJSON_AddNumberToObject(case_json, "action", entry->action);
            if (entry->action == RELAY_ACTION_PULSE) {
                cJSON_AddStringToObject(case_json, "pulse_source_name",
                                        config->sources[entry->pulse_source_index].name);
                cJSON_AddNumberToObject(case_json, "pulse_hysteresis", entry->pulse_hysteresis);
            } else if (entry->action == RELAY_ACTION_PWM) {
                cJSON_AddStringToObject(case_json, "pwm_source_name",
                                        config->sources[entry->pwm_source_index].name);
                cJSON_AddNumberToObject(case_json, "pwm_hysteresis", entry->pwm_hysteresis);
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
            } else if (entry->action == RELAY_ACTION_PWM) {
                cJSON *points = cJSON_AddArrayToObject(case_json, "pwm_points");
                for (unsigned p = 0; p < entry->pwm_point_count; ++p) {
                    cJSON *point = cJSON_CreateObject();
                    cJSON_AddNumberToObject(point, "input_value", entry->pwm_points[p].input_value);
                    cJSON_AddNumberToObject(point, "duty_percent", entry->pwm_points[p].duty_percent);
                    cJSON_AddItemToArray(points, point);
                }
            }
            cJSON_AddItemToArray(cases, case_json);
        }
        cJSON_AddItemToArray(outputs, output_json);
    }
    return root;
}

static bool parse_can_source(cJSON *item, relay_source_config_t *source)
{
    double value;
    cJSON *name = cJSON_GetObjectItemCaseSensitive(item, "name");
    if (!cJSON_IsObject(item) || !cJSON_IsString(name) || name->valuestring == NULL ||
        name->valuestring[0] == '\0' || strlen(name->valuestring) >= sizeof(source->name)) {
        return false;
    }
    strlcpy(source->name, name->valuestring, sizeof(source->name));
    source->type = RELAY_SOURCE_CAN;
    if (!integer(item, "can_id", 0, 0x1FFFFFFF, &value)) return false;
    source->can_id = (uint32_t)value;
    if (!boolean(item, "extended", &source->extended) ||
        !integer(item, "start_bit", 0, 63, &value)) return false;
    source->start_bit = (uint8_t)value;
    if (!integer(item, "bit_length", 1, 64, &value)) return false;
    source->bit_length = (uint8_t)value;
    if (!boolean(item, "little_endian", &source->little_endian) ||
        !boolean(item, "is_signed", &source->is_signed) || !number(item, "factor", &value)) {
        return false;
    }
    source->factor = (float)value;
    if (!number(item, "offset", &value)) return false;
    source->offset = (float)value;
    if (!integer(item, "zero_confirm_samples", 0, UINT8_MAX, &value)) return false;
    source->zero_confirm_samples = (uint8_t)value;
    return true;
}

static int source_index_by_name(const relay_rule_config_t *config, const char *name)
{
    if (name == NULL || name[0] == '\0') return -1;
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) {
        if (config->sources[i].type != RELAY_SOURCE_UNUSED &&
            strcmp(config->sources[i].name, name) == 0) return (int)i;
    }
    return -1;
}

static bool parse_case(cJSON *item, relay_rule_case_t *entry,
                       const relay_rule_config_t *config)
{
    double value;
    cJSON *tests = cJSON_GetObjectItemCaseSensitive(item, "tests");
    if (!cJSON_IsObject(item) ||
        !integer(item, "action", RELAY_ACTION_OFF, RELAY_ACTION_PWM, &value) ||
        !cJSON_IsArray(tests)) {
        return false;
    }
    entry->action = (relay_action_t)value;

    cJSON *pulse_points = NULL;
    cJSON *pwm_points = NULL;
    if (entry->action == RELAY_ACTION_PULSE) {
        cJSON *pulse_source = cJSON_GetObjectItemCaseSensitive(item, "pulse_source_name");
        pulse_points = cJSON_GetObjectItemCaseSensitive(item, "pulse_points");
        if (!cJSON_IsString(pulse_source) || pulse_source->valuestring == NULL ||
            !number(item, "pulse_hysteresis", &value) || value < 0.0 ||
            !cJSON_IsArray(pulse_points)) return false;
        const int source_index = source_index_by_name(config, pulse_source->valuestring);
        if (source_index < 0) return false;
        entry->pulse_source_index = (uint8_t)source_index;
        entry->pulse_hysteresis = (float)value;
    } else if (entry->action == RELAY_ACTION_PWM) {
        cJSON *pwm_source = cJSON_GetObjectItemCaseSensitive(item, "pwm_source_name");
        pwm_points = cJSON_GetObjectItemCaseSensitive(item, "pwm_points");
        if (!cJSON_IsString(pwm_source) || pwm_source->valuestring == NULL ||
            !number(item, "pwm_hysteresis", &value) || value < 0.0 ||
            !cJSON_IsArray(pwm_points)) return false;
        const int source_index = source_index_by_name(config, pwm_source->valuestring);
        if (source_index < 0) return false;
        entry->pwm_source_index = (uint8_t)source_index;
        entry->pwm_hysteresis = (float)value;
    }

    const int test_count = cJSON_GetArraySize(tests);
    const int pulse_point_count = entry->action == RELAY_ACTION_PULSE ?
                                  cJSON_GetArraySize(pulse_points) : 0;
    const int pwm_point_count = entry->action == RELAY_ACTION_PWM ?
                                cJSON_GetArraySize(pwm_points) : 0;
    if (test_count < 1 || test_count > (int)RELAY_RULE_MAX_TESTS ||
        pulse_point_count < 0 || pulse_point_count > (int)RELAY_RULE_MAX_PULSE_POINTS ||
        pwm_point_count < 0 || pwm_point_count > (int)RELAY_RULE_MAX_PWM_POINTS ||
        (entry->action == RELAY_ACTION_PULSE && pulse_point_count < 1) ||
        (entry->action == RELAY_ACTION_PWM && pwm_point_count < 1)) {
        return false;
    }
    entry->test_count = (uint8_t)test_count;
    entry->pulse_point_count = (uint8_t)pulse_point_count;
    entry->pwm_point_count = (uint8_t)pwm_point_count;

    for (int t = 0; t < test_count; ++t) {
        cJSON *test_json = cJSON_GetArrayItem(tests, t);
        relay_rule_test_t *test = &entry->tests[t];
        if (!cJSON_IsObject(test_json) ||
            !integer(test_json, "type", RELAY_TEST_SOURCE, RELAY_TEST_UPTIME, &value)) return false;
        test->type = (relay_test_type_t)value;
        if (test->type == RELAY_TEST_SOURCE) {
            cJSON *source_name = cJSON_GetObjectItemCaseSensitive(test_json, "source_name");
            if (!cJSON_IsString(source_name) || source_name->valuestring == NULL) return false;
            const int source_index = source_index_by_name(config, source_name->valuestring);
            if (source_index < 0) return false;
            test->source_index = (uint8_t)source_index;
        }
        if (!integer(test_json, "comparison", RELAY_COMPARE_GT, RELAY_COMPARE_NE, &value)) return false;
        test->comparison = (relay_compare_t)value;
        if (!boolean(test_json, "hysteresis_enabled", &test->hysteresis_enabled) ||
            !number(test_json, "threshold", &value)) return false;
        test->threshold = (float)value;
        if (!number(test_json, "hysteresis", &value) || value < 0.0) return false;
        test->hysteresis = (float)value;
    }

    for (int p = 0; p < pulse_point_count; ++p) {
        cJSON *point_json = cJSON_GetArrayItem(pulse_points, p);
        relay_pulse_point_t *point = &entry->pulse_points[p];
        if (!cJSON_IsObject(point_json) || !number(point_json, "input_value", &value)) return false;
        point->input_value = (float)value;
        if (!integer(point_json, "on_time_ms", 0, UINT32_MAX, &value)) return false;
        point->on_time_ms = (uint32_t)value;
        if (!integer(point_json, "period_ms", 0, UINT32_MAX, &value)) return false;
        point->period_ms = (uint32_t)value;
    }

    for (int p = 0; p < pwm_point_count; ++p) {
        cJSON *point_json = cJSON_GetArrayItem(pwm_points, p);
        relay_pwm_point_t *point = &entry->pwm_points[p];
        if (!cJSON_IsObject(point_json) || !number(point_json, "input_value", &value)) return false;
        point->input_value = (float)value;
        if (!integer(point_json, "duty_percent", 0, 100, &value)) return false;
        point->duty_percent = (uint8_t)value;
    }
    return true;
}

bool relay_rule_config_json_parse(const cJSON *root, relay_rule_config_t *config)
{
    double value;
    if (root == NULL || config == NULL || !cJSON_IsObject(root)) return false;
    relay_rule_engine_set_defaults(config);
    if (!integer(root, "version", OUTPUT_CONFIG_VERSION, OUTPUT_CONFIG_VERSION, &value) ||
        !integer(root, "signal_timeout_ms", 100, 60000, &value)) return false;
    config->version = OUTPUT_CONFIG_VERSION;
    config->signal_timeout_ms = (uint32_t)value;

    cJSON *sources = cJSON_GetObjectItemCaseSensitive(root, "sources");
    cJSON *outputs = cJSON_GetObjectItemCaseSensitive(root, "outputs");
    if (!cJSON_IsArray(sources) || cJSON_GetArraySize(sources) > (int)(RELAY_RULE_MAX_SOURCES - 20U) ||
        !cJSON_IsArray(outputs) || cJSON_GetArraySize(outputs) > (int)RELAY_RULE_MAX_RULES) return false;

    const unsigned source_count = (unsigned)cJSON_GetArraySize(sources);
    for (unsigned source = 0; source < source_count; ++source) {
        relay_source_config_t parsed = {0};
        if (!parse_can_source(cJSON_GetArrayItem(sources, source), &parsed) ||
            source_index_by_name(config, parsed.name) >= 0) return false;
        config->sources[20U + source] = parsed;
    }

    bool used[RELAY_RULE_MAX_RULES] = {0};
    const unsigned output_count = (unsigned)cJSON_GetArraySize(outputs);
    for (unsigned position = 0; position < output_count; ++position) {
        cJSON *output_json = cJSON_GetArrayItem(outputs, position);
        if (!cJSON_IsObject(output_json) ||
            !integer(output_json, "output", 1, RELAY_RULE_MAX_RULES, &value)) return false;
        const unsigned r = (unsigned)value - 1U;
        if (used[r]) return false;
        used[r] = true;

        cJSON *label = cJSON_GetObjectItemCaseSensitive(output_json, "label");
        cJSON *cases = cJSON_GetObjectItemCaseSensitive(output_json, "cases");
        relay_output_rule_t *output = &config->rules[r];
        if (!cJSON_IsString(label) || label->valuestring == NULL ||
            strlen(label->valuestring) >= sizeof(output->label) ||
            !boolean(output_json, "enabled", &output->enabled) || !cJSON_IsArray(cases) ||
            cJSON_GetArraySize(cases) > (int)RELAY_RULE_MAX_CASES) return false;
        strlcpy(output->label, label->valuestring, sizeof(output->label));
        output->case_count = (uint8_t)cJSON_GetArraySize(cases);
        for (unsigned c = 0; c < output->case_count; ++c) {
            if (!parse_case(cJSON_GetArrayItem(cases, c), &output->cases[c], config)) return false;
        }
    }
    return true;
}

cJSON *relay_rule_config_json_create(void)
{
    relay_rule_config_t *config = malloc(sizeof(*config));
    if (config == NULL) return NULL;
    relay_rule_engine_snapshot(config);
    cJSON *root = outputs_json(config);
    free(config);
    return root;
}

cJSON *relay_rule_status_json_create(void)
{
    relay_rule_status_t *outputs = malloc(sizeof(*outputs) * RELAY_RULE_MAX_RULES);
    relay_rule_source_status_t *sources = malloc(sizeof(*sources) * RELAY_RULE_MAX_SOURCES);
    if (outputs == NULL || sources == NULL) {
        free(outputs);
        free(sources);
        return NULL;
    }
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    relay_rule_engine_get_status(outputs, sources, now_ms);
    cJSON *output_array = cJSON_CreateArray();
    if (output_array == NULL) {
        free(outputs);
        free(sources);
        return NULL;
    }
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i) {
        if (!outputs[i].configured) continue;
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "output", i + 1U);
        cJSON_AddBoolToObject(item, "enabled", outputs[i].enabled);
        if (!outputs[i].enabled) {
            cJSON_AddItemToArray(output_array, item);
            continue;
        }
        cJSON_AddBoolToObject(item, "valid", outputs[i].valid);
        cJSON_AddBoolToObject(item, "state", outputs[i].state);
        cJSON_AddBoolToObject(item, "pulse_active", outputs[i].pulse_active);
        cJSON_AddBoolToObject(item, "pwm_active", outputs[i].pwm_active);
        if (outputs[i].valid) cJSON_AddNumberToObject(item, "selected_case", outputs[i].selected_case);
        if (outputs[i].valid && outputs[i].pulse_active) {
            cJSON_AddNumberToObject(item, "pulse_on_time_ms", outputs[i].pulse_on_time_ms);
            cJSON_AddNumberToObject(item, "pulse_period_ms", outputs[i].pulse_period_ms);
            cJSON_AddNumberToObject(item, "pulse_next_on_ms", outputs[i].pulse_next_on_ms);
        }
        if (outputs[i].valid && outputs[i].pwm_active)
            cJSON_AddNumberToObject(item, "duty_percent", outputs[i].duty_percent);
        if (!outputs[i].valid) {
            cJSON_AddNumberToObject(item, "invalid_case", outputs[i].invalid_case);
            cJSON_AddNumberToObject(item, "invalid_test", outputs[i].invalid_test);
            cJSON_AddBoolToObject(item, "invalid_pulse_source", outputs[i].invalid_pulse_source);
            cJSON_AddBoolToObject(item, "invalid_pwm_source", outputs[i].invalid_pwm_source);
            cJSON_AddNumberToObject(item, "invalid_reason", outputs[i].invalid_reason);
            if (outputs[i].invalid_source >= 0 &&
                (unsigned)outputs[i].invalid_source < RELAY_RULE_MAX_SOURCES) {
                const relay_rule_source_status_t *source = &sources[(unsigned)outputs[i].invalid_source];
                cJSON_AddStringToObject(item, "invalid_source_name", source->name);
                cJSON_AddNumberToObject(item, "invalid_source_age_ms", source->age_ms);
                cJSON_AddNumberToObject(item, "invalid_source_zero_streak", source->zero_streak);
                cJSON_AddNumberToObject(item, "invalid_source_zero_confirm_samples", source->zero_confirm_samples);
            }
        }
        cJSON_AddItemToArray(output_array, item);
    }
    free(outputs);
    free(sources);
    return output_array;
}
