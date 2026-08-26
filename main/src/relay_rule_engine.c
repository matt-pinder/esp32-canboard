#include "inc/relay_rule_engine.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

#define TAG "RELAY_RULES"
#define RULE_CONFIG_VERSION 1U
#define RULE_CONFIG_NAMESPACE "relay_rules"
#define RULE_CONFIG_ACTIVE_KEY "active"
#define RULE_CONFIG_BACKUP_KEY "backup"
#define ZERO_CONFIRM_DEFAULT 11U
#define RULE_COMPACT_MAGIC 0x52554C32U
#define RULE_COMPACT_VERSION 1U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t source_count;
    uint16_t rule_count;
    uint16_t reserved;
    uint32_t total_size;
    uint32_t signal_timeout_ms;
    uint32_t crc32;
} compact_header_t;

typedef struct {
    uint8_t slot;
    uint8_t reserved[3];
    relay_source_config_t source;
} compact_source_t;

typedef struct {
    uint8_t slot;
    uint8_t reserved[3];
    relay_output_rule_t rule;
} compact_rule_t;

typedef struct {
    bool present;
    bool accepted_valid;
    float accepted_value;
    uint32_t last_seen_ms;
    uint8_t zero_streak;
} source_runtime_t;

typedef struct {
    bool state;
    bool valid;
    bool pulse_active;
    int8_t selected_case;
    bool hysteresis[RELAY_RULE_MAX_CASES][RELAY_RULE_MAX_TESTS];
    uint32_t pulse_started_ms;
} rule_runtime_t;

static relay_rule_config_t *active_config;
static source_runtime_t *source_runtime;
static rule_runtime_t *rule_runtime;
static SemaphoreHandle_t rule_mutex;
static uint8_t command_counter;
static uint8_t publish_rate_hz = 25U;

static uint32_t crc32(const void *data, size_t length)
{
    uint32_t crc = UINT32_MAX;
    const uint8_t *bytes = data;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : crc >> 1;
        }
    }
    return ~crc;
}

static void reset_runtime(void)
{
    memset(source_runtime, 0, sizeof(*source_runtime) * RELAY_RULE_MAX_SOURCES);
    memset(rule_runtime, 0, sizeof(*rule_runtime) * RELAY_RULE_MAX_RULES);
}

void relay_rule_engine_set_defaults(relay_rule_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->version = RULE_CONFIG_VERSION;
    config->signal_timeout_ms = 1000U;
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i) {
        snprintf(config->rules[i].label, sizeof(config->rules[i].label),
                 "Rule %u", i + 1U);
    }
    /* These are always available; the remaining slots are for reusable DBC
     * signals received from CAN. */
    for (unsigned channel = 0; channel < 10U; ++channel) {
        relay_source_config_t *voltage = &config->sources[channel];
        voltage->type = RELAY_SOURCE_LOCAL_VOLTAGE;
        voltage->local_channel = channel;
        voltage->zero_confirm_samples = ZERO_CONFIRM_DEFAULT;
        snprintf(voltage->name, sizeof(voltage->name), "Input %u voltage", channel + 1U);

        relay_source_config_t *converted = &config->sources[10U + channel];
        converted->type = RELAY_SOURCE_LOCAL_VALUE;
        converted->local_channel = channel;
        converted->zero_confirm_samples = ZERO_CONFIRM_DEFAULT;
        snprintf(converted->name, sizeof(converted->name), "Input %u value", channel + 1U);
    }
}

static bool finite_float(float value) { return isfinite(value); }

static bool source_valid(const relay_source_config_t *source)
{
    if (source->type == RELAY_SOURCE_UNUSED) return true;
    if (source->type == RELAY_SOURCE_LOCAL_VOLTAGE || source->type == RELAY_SOURCE_LOCAL_VALUE)
        return source->local_channel < 10U;
    if (source->type != RELAY_SOURCE_CAN || source->can_id > (source->extended ? 0x1FFFFFFFU : 0x7FFU) ||
        source->bit_length == 0U || source->bit_length > 64U || source->start_bit > 63U ||
        !finite_float(source->factor) || !finite_float(source->offset)) return false;
    return !source->little_endian || (unsigned)source->start_bit + source->bit_length <= 64U;
}

bool relay_rule_engine_validate(const relay_rule_config_t *config, uint8_t can_tx_hz)
{
    if (config == NULL || config->version != RULE_CONFIG_VERSION ||
        config->signal_timeout_ms < 100U || config->signal_timeout_ms > 60000U ||
        (can_tx_hz != 25U && can_tx_hz != 50U)) return false;
    const uint32_t interval = 1000U / can_tx_hz;
    for (unsigned source = 0; source < RELAY_RULE_MAX_SOURCES; ++source) {
        if (!source_valid(&config->sources[source])) return false;
    }
    for (unsigned rule = 0; rule < RELAY_RULE_MAX_RULES; ++rule) {
        const relay_output_rule_t *output = &config->rules[rule];
        if (strnlen(output->label, sizeof(output->label)) >= sizeof(output->label) ||
            output->case_count > RELAY_RULE_MAX_CASES) return false;
        if (!output->enabled) continue;
        if (output->case_count == 0U) return false;
        for (unsigned c = 0; c < output->case_count; ++c) {
            const relay_rule_case_t *entry = &output->cases[c];
            if (entry->test_count == 0U || entry->test_count > RELAY_RULE_MAX_TESTS ||
                entry->action > RELAY_ACTION_PULSE) return false;
            for (unsigned test = 0; test < entry->test_count; ++test) {
                const relay_rule_test_t *predicate = &entry->tests[test];
                if (predicate->type > RELAY_TEST_UPTIME || predicate->comparison > RELAY_COMPARE_NE ||
                    !finite_float(predicate->threshold) || !finite_float(predicate->hysteresis) ||
                    predicate->hysteresis < 0.0f) return false;
                if (predicate->type == RELAY_TEST_SOURCE &&
                    (predicate->source_index >= RELAY_RULE_MAX_SOURCES ||
                     config->sources[predicate->source_index].type == RELAY_SOURCE_UNUSED)) return false;
            }
            if (entry->action == RELAY_ACTION_PULSE) {
                if (entry->pulse_source_index >= RELAY_RULE_MAX_SOURCES ||
                    config->sources[entry->pulse_source_index].type == RELAY_SOURCE_UNUSED ||
                    entry->pulse_point_count == 0U || entry->pulse_point_count > RELAY_RULE_MAX_PULSE_POINTS) return false;
                for (unsigned point = 0; point < entry->pulse_point_count; ++point) {
                    const relay_pulse_point_t *p = &entry->pulse_points[point];
                    if (!finite_float(p->input_value) || p->period_ms == 0U || p->on_time_ms > p->period_ms ||
                        (p->on_time_ms != 0U && p->on_time_ms < interval) ||
                        (p->on_time_ms != p->period_ms && p->period_ms - p->on_time_ms < interval) ||
                        (point > 0U && p->input_value <= entry->pulse_points[point - 1U].input_value)) return false;
                }
            }
        }
    }
    return true;
}

static bool load_config(relay_rule_config_t *config)
{
    nvs_handle_t handle;
    if (nvs_open_from_partition("config", RULE_CONFIG_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;
    size_t length = 0U;
    esp_err_t result = nvs_get_blob(handle, RULE_CONFIG_ACTIVE_KEY, NULL, &length);
    if (result != ESP_OK) { nvs_close(handle); return false; }
    if (length == sizeof(*config)) {
        result = nvs_get_blob(handle, RULE_CONFIG_ACTIVE_KEY, config, &length);
        nvs_close(handle);
        return result == ESP_OK && config->crc32 == crc32(config, offsetof(relay_rule_config_t, crc32)) &&
               relay_rule_engine_validate(config, publish_rate_hz);
    }
    uint8_t *blob = malloc(length);
    if (blob == NULL) { nvs_close(handle); return false; }
    result = nvs_get_blob(handle, RULE_CONFIG_ACTIVE_KEY, blob, &length);
    nvs_close(handle);
    if (result != ESP_OK || length < sizeof(compact_header_t)) { free(blob); return false; }
    compact_header_t *header = (compact_header_t *)blob;
    const size_t expected = sizeof(*header) + header->source_count * sizeof(compact_source_t) +
                            header->rule_count * sizeof(compact_rule_t);
    const uint32_t saved_crc = header->crc32;
    header->crc32 = 0U;
    if (header->magic != RULE_COMPACT_MAGIC || header->version != RULE_COMPACT_VERSION ||
        header->source_count > RELAY_RULE_MAX_SOURCES || header->rule_count > RELAY_RULE_MAX_RULES ||
        header->total_size != length || expected != length || saved_crc != crc32(blob, length)) {
        free(blob);
        return false;
    }
    relay_rule_engine_set_defaults(config);
    config->signal_timeout_ms = header->signal_timeout_ms;
    size_t offset = sizeof(*header);
    bool used_sources[RELAY_RULE_MAX_SOURCES] = {0};
    bool used_rules[RELAY_RULE_MAX_RULES] = {0};
    for (unsigned i = 0; i < header->source_count; ++i) {
        compact_source_t entry;
        memcpy(&entry, blob + offset, sizeof(entry)); offset += sizeof(entry);
        if (entry.slot >= RELAY_RULE_MAX_SOURCES || used_sources[entry.slot]) { free(blob); return false; }
        used_sources[entry.slot] = true;
        config->sources[entry.slot] = entry.source;
    }
    for (unsigned i = 0; i < header->rule_count; ++i) {
        compact_rule_t entry;
        memcpy(&entry, blob + offset, sizeof(entry)); offset += sizeof(entry);
        if (entry.slot >= RELAY_RULE_MAX_RULES || used_rules[entry.slot]) { free(blob); return false; }
        used_rules[entry.slot] = true;
        config->rules[entry.slot] = entry.rule;
    }
    free(blob);
    return relay_rule_engine_validate(config, publish_rate_hz);
}

static bool save_config(const relay_rule_config_t *config)
{
    uint16_t source_count = 0U, rule_count = 0U;
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i)
        if (config->sources[i].type == RELAY_SOURCE_CAN) ++source_count;
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i)
        if (config->rules[i].enabled || config->rules[i].case_count > 0U) ++rule_count;
    const size_t stored_length = sizeof(compact_header_t) + source_count * sizeof(compact_source_t) +
                                 rule_count * sizeof(compact_rule_t);
    uint8_t *stored = calloc(1U, stored_length);
    if (stored == NULL) return false;
    compact_header_t *header = (compact_header_t *)stored;
    *header = (compact_header_t){.magic = RULE_COMPACT_MAGIC, .version = RULE_COMPACT_VERSION,
        .source_count = source_count, .rule_count = rule_count, .total_size = stored_length,
        .signal_timeout_ms = config->signal_timeout_ms, .crc32 = 0U};
    size_t offset = sizeof(*header);
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) {
        if (config->sources[i].type != RELAY_SOURCE_CAN) continue;
        compact_source_t entry = {.slot = i, .source = config->sources[i]};
        memcpy(stored + offset, &entry, sizeof(entry)); offset += sizeof(entry);
    }
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i) {
        if (!config->rules[i].enabled && config->rules[i].case_count == 0U) continue;
        compact_rule_t entry = {.slot = i, .rule = config->rules[i]};
        memcpy(stored + offset, &entry, sizeof(entry)); offset += sizeof(entry);
    }
    header->crc32 = crc32(stored, stored_length);
    nvs_handle_t handle;
    if (nvs_open_from_partition("config", RULE_CONFIG_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        free(stored);
        return false;
    }
    size_t previous_length = 0U;
    if (nvs_get_blob(handle, RULE_CONFIG_ACTIVE_KEY, NULL, &previous_length) == ESP_OK && previous_length > 0U) {
        uint8_t *previous = malloc(previous_length);
        if (previous != NULL && nvs_get_blob(handle, RULE_CONFIG_ACTIVE_KEY, previous, &previous_length) == ESP_OK)
            nvs_set_blob(handle, RULE_CONFIG_BACKUP_KEY, previous, previous_length);
        free(previous);
    }
    esp_err_t result = nvs_set_blob(handle, RULE_CONFIG_ACTIVE_KEY, stored, stored_length);
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Saved compact rules: %u configured rules, %u CAN sources, %u bytes",
                 (unsigned)rule_count, (unsigned)source_count, (unsigned)stored_length);
    }
    free(stored);
    return result == ESP_OK;
}

void relay_rule_engine_init(void)
{
    active_config = heap_caps_calloc(1U, sizeof(*active_config),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    source_runtime = heap_caps_calloc(RELAY_RULE_MAX_SOURCES, sizeof(*source_runtime),
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    rule_runtime = heap_caps_calloc(RELAY_RULE_MAX_RULES, sizeof(*rule_runtime),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (active_config == NULL || source_runtime == NULL || rule_runtime == NULL) {
        ESP_LOGE(TAG, "Could not allocate rule engine state in PSRAM");
        abort();
    }
    rule_mutex = xSemaphoreCreateMutex();
    if (rule_mutex == NULL) abort();
    if (!load_config(active_config)) {
        relay_rule_engine_set_defaults(active_config);
        if (!save_config(active_config)) ESP_LOGW(TAG, "Could not persist default rules");
    }
    reset_runtime();
}

void relay_rule_engine_set_publish_rate(uint8_t can_tx_hz)
{
    if (can_tx_hz == 25U || can_tx_hz == 50U) publish_rate_hz = can_tx_hz;
}

bool relay_rule_engine_replace_and_save(const relay_rule_config_t *config)
{
    if (!relay_rule_engine_validate(config, publish_rate_hz) || rule_mutex == NULL) return false;
    xSemaphoreTake(rule_mutex, portMAX_DELAY);
    const bool saved = save_config(config);
    if (saved) {
        *active_config = *config;
        active_config->version = RULE_CONFIG_VERSION;
        active_config->crc32 = crc32(active_config, offsetof(relay_rule_config_t, crc32));
        reset_runtime();
    }
    xSemaphoreGive(rule_mutex);
    return saved;
}

void relay_rule_engine_snapshot(relay_rule_config_t *config)
{
    if (config == NULL || rule_mutex == NULL) return;
    xSemaphoreTake(rule_mutex, portMAX_DELAY);
    *config = *active_config;
    xSemaphoreGive(rule_mutex);
}

static bool extract(const twai_message_t *message, const relay_source_config_t *source, float *value)
{
    const unsigned bits = message->data_length_code * 8U;
    uint64_t raw = 0U;
    if (source->little_endian) {
        if ((unsigned)source->start_bit + source->bit_length > bits) return false;
        for (unsigned i = 0; i < message->data_length_code; ++i) raw |= (uint64_t)message->data[i] << (i * 8U);
        raw >>= source->start_bit;
        if (source->bit_length < 64U) raw &= (1ULL << source->bit_length) - 1ULL;
    } else {
        int bit = source->start_bit;
        for (unsigned i = 0; i < source->bit_length; ++i) {
            if (bit < 0 || bit >= (int)bits) return false;
            raw = (raw << 1U) | ((message->data[bit / 8] >> (bit % 8)) & 1U);
            bit = bit % 8 == 0 ? bit + 15 : bit - 1;
        }
    }
    float numeric;
    if (source->is_signed) {
        int64_t signed_raw = source->bit_length == 64U ? (int64_t)raw :
            ((raw & (1ULL << (source->bit_length - 1U))) ? (int64_t)(raw | ~((1ULL << source->bit_length) - 1ULL)) : (int64_t)raw);
        numeric = (float)signed_raw;
    } else numeric = (float)raw;
    *value = numeric * source->factor + source->offset;
    return finite_float(*value);
}

static void ingest_value(unsigned index, float value, uint32_t now_ms)
{
    source_runtime_t *runtime = &source_runtime[index];
    const relay_source_config_t *source = &active_config->sources[index];
    runtime->present = true;
    runtime->last_seen_ms = now_ms;
    if (value == 0.0f) {
        if (runtime->zero_streak < UINT8_MAX) ++runtime->zero_streak;
        const uint8_t required = source->zero_confirm_samples == 0U ? ZERO_CONFIRM_DEFAULT : source->zero_confirm_samples;
        if (runtime->zero_streak < required) return;
    } else runtime->zero_streak = 0U;
    runtime->accepted_value = value;
    runtime->accepted_valid = true;
}

void relay_rule_engine_ingest_can(const twai_message_t *message, uint32_t now_ms)
{
    if (message == NULL || message->rtr || message->data_length_code > 8U || rule_mutex == NULL) return;
    xSemaphoreTake(rule_mutex, portMAX_DELAY);
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) {
        const relay_source_config_t *source = &active_config->sources[i];
        if (source->type != RELAY_SOURCE_CAN || source->can_id != message->identifier || source->extended != (bool)message->extd) continue;
        float value;
        if (extract(message, source, &value)) ingest_value(i, value, now_ms);
    }
    xSemaphoreGive(rule_mutex);
}

void relay_rule_engine_ingest_local(uint8_t channel, bool converted, float value, uint32_t now_ms)
{
    if (channel >= 10U || !finite_float(value) || rule_mutex == NULL) return;
    xSemaphoreTake(rule_mutex, portMAX_DELAY);
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) {
        const relay_source_config_t *source = &active_config->sources[i];
        if ((converted && source->type == RELAY_SOURCE_LOCAL_VALUE) || (!converted && source->type == RELAY_SOURCE_LOCAL_VOLTAGE)) {
            if (source->local_channel == channel) ingest_value(i, value, now_ms);
        }
    }
    xSemaphoreGive(rule_mutex);
}

static bool compare(float value, relay_compare_t op, float threshold, float hysteresis, bool prior)
{
    switch (op) {
        case RELAY_COMPARE_GT: return prior && hysteresis > 0.0f ? value >= threshold - hysteresis : value > threshold;
        case RELAY_COMPARE_GE: return prior && hysteresis > 0.0f ? value >= threshold - hysteresis : value >= threshold;
        case RELAY_COMPARE_LT: return prior && hysteresis > 0.0f ? value <= threshold + hysteresis : value < threshold;
        case RELAY_COMPARE_LE: return prior && hysteresis > 0.0f ? value <= threshold + hysteresis : value <= threshold;
        case RELAY_COMPARE_EQ: return fabsf(value - threshold) <= hysteresis;
        case RELAY_COMPARE_NE: return fabsf(value - threshold) > hysteresis;
        default: return false;
    }
}

typedef enum { TEST_FALSE, TEST_TRUE, TEST_UNKNOWN } test_result_t;

static test_result_t evaluate_test(const relay_rule_test_t *test, bool *latch, uint32_t now_ms)
{
    if (test->type == RELAY_TEST_UPTIME) {
        *latch = compare((float)now_ms, test->comparison, test->threshold,
                         test->hysteresis_enabled ? test->hysteresis : 0.0f, *latch);
        return *latch ? TEST_TRUE : TEST_FALSE;
    }
    const source_runtime_t *source = &source_runtime[test->source_index];
    if (!source->accepted_valid || now_ms - source->last_seen_ms >= active_config->signal_timeout_ms) return TEST_UNKNOWN;
    *latch = compare(source->accepted_value, test->comparison, test->threshold,
                     test->hysteresis_enabled ? test->hysteresis : 0.0f, *latch);
    return *latch ? TEST_TRUE : TEST_FALSE;
}

static bool pulse_value(const relay_rule_case_t *entry, float input, uint32_t now_ms, uint32_t *started)
{
    unsigned upper = 0U;
    while (upper + 1U < entry->pulse_point_count && input > entry->pulse_points[upper + 1U].input_value) ++upper;
    const relay_pulse_point_t *a = &entry->pulse_points[upper];
    const relay_pulse_point_t *b = upper + 1U < entry->pulse_point_count ? &entry->pulse_points[upper + 1U] : a;
    float ratio = a == b ? 0.0f : (input - a->input_value) / (b->input_value - a->input_value);
    uint32_t on = (uint32_t)lroundf(a->on_time_ms + ratio * ((float)b->on_time_ms - a->on_time_ms));
    uint32_t period = (uint32_t)lroundf(a->period_ms + ratio * ((float)b->period_ms - a->period_ms));
    if (period == 0U) return false;
    if (*started == 0U) *started = now_ms;
    return on >= period || ((now_ms - *started) % period) < on;
}

void relay_rule_engine_make_command(uint32_t now_ms, relay_command_t *command)
{
    if (command == NULL || rule_mutex == NULL) return;
    memset(command, 0, sizeof(*command));
    xSemaphoreTake(rule_mutex, portMAX_DELAY);
    command->counter = command_counter++;
    for (unsigned r = 0; r < RELAY_RULE_MAX_RULES; ++r) {
        const relay_output_rule_t *output = &active_config->rules[r];
        rule_runtime_t *runtime = &rule_runtime[r];
        runtime->state = false; runtime->valid = false; runtime->pulse_active = false; runtime->selected_case = -1;
        if (!output->enabled) continue;
        bool invalid = false;
        for (unsigned c = 0; c < output->case_count; ++c) {
            const relay_rule_case_t *entry = &output->cases[c];
            bool false_seen = false, unknown_seen = false;
            for (unsigned t = 0; t < entry->test_count; ++t) {
                const test_result_t result = evaluate_test(&entry->tests[t], &runtime->hysteresis[c][t], now_ms);
                false_seen |= result == TEST_FALSE;
                unknown_seen |= result == TEST_UNKNOWN;
            }
            if (false_seen) continue;
            if (unknown_seen) { invalid = true; break; }
            runtime->selected_case = (int8_t)c;
            runtime->valid = true;
            if (entry->action == RELAY_ACTION_ON) runtime->state = true;
            else if (entry->action == RELAY_ACTION_PULSE) {
                const source_runtime_t *source = &source_runtime[entry->pulse_source_index];
                if (!source->accepted_valid || now_ms - source->last_seen_ms >= active_config->signal_timeout_ms) { runtime->valid = false; invalid = true; break; }
                runtime->pulse_active = true;
                runtime->state = pulse_value(entry, source->accepted_value, now_ms, &runtime->pulse_started_ms);
            }
            break;
        }
        if (runtime->selected_case < 0 && !invalid) runtime->valid = true;
        if (runtime->selected_case < 0) runtime->pulse_started_ms = 0U;
        if (runtime->state) command->state_mask |= (uint16_t)(1U << r);
        if (runtime->valid) command->valid_mask |= (uint16_t)(1U << r);
        if (runtime->pulse_active) command->pulse_mask |= (uint16_t)(1U << r);
    }
    xSemaphoreGive(rule_mutex);
}

void relay_rule_engine_get_status(relay_rule_status_t rules[RELAY_RULE_MAX_RULES], relay_rule_source_status_t sources[RELAY_RULE_MAX_SOURCES], uint32_t now_ms)
{
    if (rule_mutex == NULL) return;
    xSemaphoreTake(rule_mutex, portMAX_DELAY);
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i) {
        rules[i] = (relay_rule_status_t){.enabled = active_config->rules[i].enabled, .valid = rule_runtime[i].valid, .state = rule_runtime[i].state, .pulse_active = rule_runtime[i].pulse_active, .selected_case = rule_runtime[i].selected_case};
    }
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) {
        sources[i] = (relay_rule_source_status_t){.present = source_runtime[i].present, .accepted_valid = source_runtime[i].accepted_valid, .value = source_runtime[i].accepted_value, .age_ms = source_runtime[i].present ? now_ms - source_runtime[i].last_seen_ms : UINT32_MAX, .zero_streak = source_runtime[i].zero_streak};
    }
    xSemaphoreGive(rule_mutex);
}

bool relay_rule_engine_has_external_sources(void)
{
    bool found = false;
    if (rule_mutex == NULL) return false;
    xSemaphoreTake(rule_mutex, portMAX_DELAY);
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) found |= active_config->sources[i].type == RELAY_SOURCE_CAN;
    xSemaphoreGive(rule_mutex);
    return found;
}
