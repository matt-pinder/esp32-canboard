#include "inc/relay_rule_http.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inc/relay_rule_engine.h"

#define TAG "RULE_HTTP"
#define RULE_REQUEST_LIMIT (128U * 1024U)

static bool number(cJSON *object, const char *key, double *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
    *value = item->valuedouble;
    return true;
}

static bool boolean(cJSON *object, const char *key, bool *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsBool(item)) return false;
    *value = cJSON_IsTrue(item);
    return true;
}

static cJSON *source_json(const relay_source_config_t *source)
{
    cJSON *item = cJSON_CreateObject();
    cJSON_AddStringToObject(item, "name", source->name);
    cJSON_AddNumberToObject(item, "type", source->type);
    cJSON_AddNumberToObject(item, "local_channel", source->local_channel);
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
    cJSON *sources = cJSON_AddArrayToObject(root, "sources");
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i)
        cJSON_AddItemToArray(sources, source_json(&config->sources[i]));
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
            cJSON_AddNumberToObject(case_json, "pulse_source_index", entry->pulse_source_index);
            cJSON *tests = cJSON_AddArrayToObject(case_json, "tests");
            for (unsigned t = 0; t < entry->test_count; ++t) {
                const relay_rule_test_t *test = &entry->tests[t];
                cJSON *test_json = cJSON_CreateObject();
                cJSON_AddNumberToObject(test_json, "type", test->type);
                cJSON_AddNumberToObject(test_json, "source_index", test->source_index);
                cJSON_AddNumberToObject(test_json, "comparison", test->comparison);
                cJSON_AddBoolToObject(test_json, "hysteresis_enabled", test->hysteresis_enabled);
                cJSON_AddNumberToObject(test_json, "threshold", test->threshold);
                cJSON_AddNumberToObject(test_json, "hysteresis", test->hysteresis);
                cJSON_AddItemToArray(tests, test_json);
            }
            cJSON *points = cJSON_AddArrayToObject(case_json, "pulse_points");
            for (unsigned p = 0; p < entry->pulse_point_count; ++p) {
                cJSON *point = cJSON_CreateObject();
                cJSON_AddNumberToObject(point, "input_value", entry->pulse_points[p].input_value);
                cJSON_AddNumberToObject(point, "on_time_ms", entry->pulse_points[p].on_time_ms);
                cJSON_AddNumberToObject(point, "period_ms", entry->pulse_points[p].period_ms);
                cJSON_AddItemToArray(points, point);
            }
            cJSON_AddItemToArray(cases, case_json);
        }
        cJSON_AddItemToArray(rules, rule_json);
    }
    return root;
}

static bool parse_source(cJSON *item, relay_source_config_t *source)
{
    double value;
    cJSON *name = cJSON_GetObjectItemCaseSensitive(item, "name");
    if (!cJSON_IsString(name) || name->valuestring == NULL || strlen(name->valuestring) >= sizeof(source->name) ||
        !number(item, "type", &value) || value < RELAY_SOURCE_UNUSED || value > RELAY_SOURCE_LOCAL_VALUE) return false;
    strlcpy(source->name, name->valuestring, sizeof(source->name));
    source->type = (relay_source_type_t)value;
    if (!number(item, "local_channel", &value) || value < 0 || value > UINT8_MAX) return false;
    source->local_channel = (uint8_t)value;
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
    /* Zero means use the engine default (11 samples).  Unused source slots
     * returned by GET /api/rules intentionally serialize this as zero, so a
     * GET followed by an unchanged POST must remain valid. */
    if (!number(item, "zero_confirm_samples", &value) || value < 0 || value > UINT8_MAX) return false;
    source->zero_confirm_samples = (uint8_t)value;
    return true;
}

static bool parse_case(cJSON *item, relay_rule_case_t *entry)
{
    double value;
    cJSON *tests = cJSON_GetObjectItemCaseSensitive(item, "tests");
    cJSON *points = cJSON_GetObjectItemCaseSensitive(item, "pulse_points");
    if (!number(item, "action", &value) || value < RELAY_ACTION_OFF || value > RELAY_ACTION_PULSE ||
        !number(item, "pulse_source_index", &value) || value < 0 || value >= RELAY_RULE_MAX_SOURCES ||
        !cJSON_IsArray(tests) || !cJSON_IsArray(points)) return false;
    entry->action = (relay_action_t)cJSON_GetObjectItemCaseSensitive(item, "action")->valueint;
    entry->pulse_source_index = (uint8_t)value;
    const int test_count = cJSON_GetArraySize(tests);
    const int point_count = cJSON_GetArraySize(points);
    if (test_count < 1 || test_count > RELAY_RULE_MAX_TESTS || point_count > RELAY_RULE_MAX_PULSE_POINTS) return false;
    entry->test_count = (uint8_t)test_count;
    entry->pulse_point_count = (uint8_t)point_count;
    for (int t = 0; t < test_count; ++t) {
        cJSON *test_json = cJSON_GetArrayItem(tests, t);
        relay_rule_test_t *test = &entry->tests[t];
        if (!cJSON_IsObject(test_json) || !number(test_json, "type", &value) || value < RELAY_TEST_SOURCE || value > RELAY_TEST_UPTIME) return false;
        test->type = (relay_test_type_t)value;
        if (!number(test_json, "source_index", &value) || value < 0 || value >= RELAY_RULE_MAX_SOURCES) return false;
        test->source_index = (uint8_t)value;
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

static bool parse_rules(cJSON *root, relay_rule_config_t *config)
{
    double value;
    relay_rule_engine_set_defaults(config);
    if (!number(root, "signal_timeout_ms", &value) || value < 100 || value > 60000) return false;
    config->signal_timeout_ms = (uint32_t)value;
    cJSON *sources = cJSON_GetObjectItemCaseSensitive(root, "sources");
    cJSON *rules = cJSON_GetObjectItemCaseSensitive(root, "rules");
    if (!cJSON_IsArray(sources) || cJSON_GetArraySize(sources) != RELAY_RULE_MAX_SOURCES ||
        !cJSON_IsArray(rules) || cJSON_GetArraySize(rules) > RELAY_RULE_MAX_RULES) return false;
    for (unsigned source = 0; source < RELAY_RULE_MAX_SOURCES; ++source) {
        if (!parse_source(cJSON_GetArrayItem(sources, source), &config->sources[source])) return false;
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
            if (!parse_case(cJSON_GetArrayItem(cases, c), &rule->cases[c])) return false;
        }
    }
    return true;
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root)
{
    char *text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (text == NULL) return httpd_resp_send_500(req), ESP_FAIL;
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = httpd_resp_sendstr(req, text);
    free(text);
    return result;
}

static esp_err_t rules_get(httpd_req_t *req)
{
    relay_rule_config_t *config = malloc(sizeof(*config));
    if (config == NULL) return httpd_resp_send_500(req), ESP_FAIL;
    relay_rule_engine_snapshot(config);
    cJSON *root = rules_json(config);
    free(config);
    return send_json(req, root);
}

static esp_err_t rules_post(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > RULE_REQUEST_LIMIT) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request size"), ESP_FAIL;
    char *body = malloc((size_t)req->content_len + 1U);
    relay_rule_config_t *config = malloc(sizeof(*config));
    if (body == NULL || config == NULL) { free(body); free(config); return httpd_resp_send_500(req), ESP_FAIL; }
    size_t received = 0;
    while (received < (size_t)req->content_len) {
        int read = httpd_req_recv(req, body + received, req->content_len - received);
        if (read <= 0) { free(body); free(config); return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete request"), ESP_FAIL; }
        received += (size_t)read;
    }
    body[received] = '\0';
    cJSON *root = cJSON_Parse(body);
    free(body);
    /* replace_and_save validates against the active 25/50 Hz board cadence. */
    if (root == NULL || !parse_rules(root, config) || !relay_rule_engine_replace_and_save(config)) {
        cJSON_Delete(root); free(config); return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid rule configuration"), ESP_FAIL;
    }
    cJSON_Delete(root);
    root = rules_json(config);
    free(config);
    return send_json(req, root);
}

static esp_err_t status_get(httpd_req_t *req)
{
    relay_rule_status_t *rules = malloc(sizeof(*rules) * RELAY_RULE_MAX_RULES);
    relay_rule_source_status_t *sources = malloc(sizeof(*sources) * RELAY_RULE_MAX_SOURCES);
    if (rules == NULL || sources == NULL) {
        free(rules);
        free(sources);
        return httpd_resp_send_500(req), ESP_FAIL;
    }
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    relay_rule_engine_get_status(rules, sources, now_ms);
    cJSON *root = cJSON_CreateObject();
    cJSON *rule_array = cJSON_AddArrayToObject(root, "rules");
    cJSON *source_array = cJSON_AddArrayToObject(root, "sources");
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddBoolToObject(item, "enabled", rules[i].enabled);
        cJSON_AddBoolToObject(item, "valid", rules[i].valid);
        cJSON_AddBoolToObject(item, "state", rules[i].state);
        cJSON_AddBoolToObject(item, "pulse_active", rules[i].pulse_active);
        cJSON_AddNumberToObject(item, "selected_case", rules[i].selected_case);
        cJSON_AddItemToArray(rule_array, item);
    }
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddBoolToObject(item, "present", sources[i].present);
        cJSON_AddBoolToObject(item, "accepted_valid", sources[i].accepted_valid);
        cJSON_AddNumberToObject(item, "value", sources[i].value);
        cJSON_AddNumberToObject(item, "age_ms", sources[i].age_ms);
        cJSON_AddNumberToObject(item, "zero_streak", sources[i].zero_streak);
        cJSON_AddItemToArray(source_array, item);
    }
    free(rules);
    free(sources);
    return send_json(req, root);
}

void relay_rule_http_register(httpd_handle_t server)
{
    const httpd_uri_t get_uri = {.uri = "/api/rules", .method = HTTP_GET, .handler = rules_get};
    const httpd_uri_t post_uri = {.uri = "/api/rules", .method = HTTP_POST, .handler = rules_post};
    const httpd_uri_t status_uri = {.uri = "/api/rules/status", .method = HTTP_GET, .handler = status_get};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &get_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &post_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status_uri));
    ESP_LOGI(TAG, "Rule API registered");
}
