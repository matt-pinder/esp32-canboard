#include "inc/relay_rule_engine.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "RELAY_RULES"
#define RULE_CONFIG_VERSION 1U
#define RULE_CONFIG_PARTITION "rules"
#define ZERO_CONFIRM_DEFAULT 11U
#define RULE_COMPACT_MAGIC 0x52554C32U
#define RULE_COMPACT_VERSION 2U
#define RULE_SLOT_MAGIC 0x52534C54U
#define RULE_SLOT_COUNT 2U

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
    uint32_t magic;
    uint32_t generation;
    uint32_t payload_size;
    uint32_t payload_crc32;
    uint32_t header_crc32;
} rule_slot_header_t;

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

/* Version 1 stored the rule structs before pulse-source hysteresis was added. */
typedef struct {
    uint8_t test_count;
    relay_rule_test_t tests[RELAY_RULE_MAX_TESTS];
    relay_action_t action;
    uint8_t pulse_source_index;
    uint8_t pulse_point_count;
    relay_pulse_point_t pulse_points[RELAY_RULE_MAX_PULSE_POINTS];
} relay_rule_case_v1_t;

typedef struct {
    char label[RELAY_RULE_NAME_LENGTH];
    bool enabled;
    uint8_t case_count;
    relay_rule_case_v1_t cases[RELAY_RULE_MAX_CASES];
} relay_output_rule_v1_t;

typedef struct {
    uint8_t slot;
    uint8_t reserved[3];
    relay_output_rule_v1_t rule;
} compact_rule_v1_t;

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
    uint32_t pulse_on_time_ms;
    uint32_t pulse_period_ms;
    float pulse_input_value;
    bool pulse_input_valid;
    int8_t invalid_case;
    int8_t invalid_test;
    int8_t invalid_source;
    bool invalid_pulse_source;
    relay_rule_invalid_reason_t invalid_reason;
} rule_runtime_t;

static relay_rule_config_t *active_config;
static source_runtime_t *source_runtime;
static rule_runtime_t *rule_runtime;
static SemaphoreHandle_t rule_mutex;
static uint8_t command_counter;
static uint8_t publish_rate_hz = 25U;

static const char *comparison_name(relay_compare_t comparison)
{
    static const char *const names[] = {">", ">=", "<", "<=", "==", "!="};
    return comparison <= RELAY_COMPARE_NE ? names[comparison] : "?";
}

static const char *action_name(relay_action_t action)
{
    static const char *const names[] = {"OFF", "ON", "PULSE"};
    return action <= RELAY_ACTION_PULSE ? names[action] : "?";
}

static void log_source(unsigned index, const relay_source_config_t *source, const char *indent)
{
    if (source->type == RELAY_SOURCE_CAN) {
        ESP_LOGI(TAG,
                 "%ssource[%u]: name=\"%s\" CAN id=0x%lX %s start_bit=%u length=%u %s %s factor=%.6g offset=%.6g zero_confirm=%u",
                 indent, index, source->name, (unsigned long)source->can_id,
                 source->extended ? "extended" : "standard",
                 (unsigned)source->start_bit, (unsigned)source->bit_length,
                 source->little_endian ? "Intel" : "Motorola",
                 source->is_signed ? "signed" : "unsigned",
                 (double)source->factor, (double)source->offset,
                 (unsigned)source->zero_confirm_samples);
    } else {
        const char *measurement = source->type == RELAY_SOURCE_LOCAL_VOLTAGE ? "voltage" : "value";
        ESP_LOGI(TAG, "%ssource[%u]: name=\"%s\" Input %u %s zero_confirm=%u",
                 indent, index, source->name, (unsigned)source->local_channel + 1U,
                 measurement, (unsigned)source->zero_confirm_samples);
    }
}

static void log_rule_config(const relay_rule_config_t *config)
{
    unsigned configured_rules = 0U;
    unsigned can_sources = 0U;
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i)
        configured_rules += config->rules[i].enabled || config->rules[i].case_count > 0U;
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i)
        can_sources += config->sources[i].type == RELAY_SOURCE_CAN;

    ESP_LOGI(TAG, "Loaded relay_rule_config_t:");
    ESP_LOGI(TAG, "  version=%lu signal_timeout_ms=%lu configured_rules=%u can_sources=%u publish_rate_hz=%u",
             (unsigned long)config->version, (unsigned long)config->signal_timeout_ms,
             configured_rules, can_sources, (unsigned)publish_rate_hz);

    for (unsigned r = 0; r < RELAY_RULE_MAX_RULES; ++r) {
        const relay_output_rule_t *rule = &config->rules[r];
        if (!rule->enabled && rule->case_count == 0U) continue;
        ESP_LOGI(TAG, "  rule[%u]: label=\"%s\" enabled=%d published_bit=%u cases=%u",
                 r, rule->label, rule->enabled, r + 1U, (unsigned)rule->case_count);
        for (unsigned c = 0; c < rule->case_count; ++c) {
            const relay_rule_case_t *entry = &rule->cases[c];
            ESP_LOGI(TAG, "    case[%u]: tests=%u action=%s pulse_hysteresis=%.6g", c,
                     (unsigned)entry->test_count, action_name(entry->action),
                     (double)entry->pulse_hysteresis);
            for (unsigned t = 0; t < entry->test_count; ++t) {
                const relay_rule_test_t *test = &entry->tests[t];
                if (test->type == RELAY_TEST_UPTIME) {
                    ESP_LOGI(TAG,
                             "      test[%u]: uptime_ms %s %.6g hysteresis=%s %.6g",
                             t, comparison_name(test->comparison), (double)test->threshold,
                             test->hysteresis_enabled ? "on" : "off",
                             (double)test->hysteresis);
                } else {
                    ESP_LOGI(TAG,
                             "      test[%u]: source[%u] %s %.6g hysteresis=%s %.6g",
                             t, (unsigned)test->source_index,
                             comparison_name(test->comparison), (double)test->threshold,
                             test->hysteresis_enabled ? "on" : "off",
                             (double)test->hysteresis);
                    log_source(test->source_index, &config->sources[test->source_index], "        ");
                }
            }
            if (entry->action != RELAY_ACTION_PULSE) continue;
            log_source(entry->pulse_source_index,
                       &config->sources[entry->pulse_source_index], "      pulse ");
            for (unsigned p = 0; p < entry->pulse_point_count; ++p) {
                const relay_pulse_point_t *point = &entry->pulse_points[p];
                ESP_LOGI(TAG,
                         "      pulse_point[%u]: input=%.6g on=%lu.%03lu s period=%lu.%03lu s",
                         p, (double)point->input_value,
                         (unsigned long)(point->on_time_ms / 1000U),
                         (unsigned long)(point->on_time_ms % 1000U),
                         (unsigned long)(point->period_ms / 1000U),
                         (unsigned long)(point->period_ms % 1000U));
            }
        }
    }
}

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
        if (config->sources[source].type == RELAY_SOURCE_UNUSED) continue;
        if (config->sources[source].name[0] == '\0' ||
            strnlen(config->sources[source].name, sizeof(config->sources[source].name)) >=
                sizeof(config->sources[source].name)) return false;
        for (unsigned previous = 0; previous < source; ++previous)
            if (config->sources[previous].type != RELAY_SOURCE_UNUSED &&
                strcmp(config->sources[previous].name, config->sources[source].name) == 0)
                return false;
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
                    entry->pulse_point_count == 0U || entry->pulse_point_count > RELAY_RULE_MAX_PULSE_POINTS ||
                    !finite_float(entry->pulse_hysteresis) || entry->pulse_hysteresis < 0.0f) return false;
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

static bool decode_config(uint8_t *blob, size_t length, relay_rule_config_t *config)
{
    if (blob == NULL || length < sizeof(compact_header_t)) return false;
    compact_header_t *header = (compact_header_t *)blob;
    const bool legacy_v1 = header->version == 1U;
    const size_t rule_size = legacy_v1 ? sizeof(compact_rule_v1_t) : sizeof(compact_rule_t);
    const size_t expected = sizeof(*header) + header->source_count * sizeof(compact_source_t) +
                            header->rule_count * rule_size;
    const uint32_t saved_crc = header->crc32;
    header->crc32 = 0U;
    if (header->magic != RULE_COMPACT_MAGIC ||
        (!legacy_v1 && header->version != RULE_COMPACT_VERSION) ||
        header->source_count > RELAY_RULE_MAX_SOURCES || header->rule_count > RELAY_RULE_MAX_RULES ||
        header->total_size != length || expected != length || saved_crc != crc32(blob, length)) {
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
        if (entry.slot >= RELAY_RULE_MAX_SOURCES || used_sources[entry.slot]) return false;
        used_sources[entry.slot] = true;
        config->sources[entry.slot] = entry.source;
    }
    for (unsigned i = 0; i < header->rule_count; ++i) {
        if (legacy_v1) {
            compact_rule_v1_t entry;
            memcpy(&entry, blob + offset, sizeof(entry)); offset += sizeof(entry);
            if (entry.slot >= RELAY_RULE_MAX_RULES || used_rules[entry.slot] ||
                entry.rule.case_count > RELAY_RULE_MAX_CASES) return false;
            used_rules[entry.slot] = true;
            relay_output_rule_t *rule = &config->rules[entry.slot];
            memcpy(rule->label, entry.rule.label, sizeof(rule->label));
            rule->enabled = entry.rule.enabled;
            rule->case_count = entry.rule.case_count;
            for (unsigned c = 0; c < rule->case_count; ++c) {
                const relay_rule_case_v1_t *old_case = &entry.rule.cases[c];
                relay_rule_case_t *new_case = &rule->cases[c];
                new_case->test_count = old_case->test_count;
                memcpy(new_case->tests, old_case->tests, sizeof(new_case->tests));
                new_case->action = old_case->action;
                new_case->pulse_source_index = old_case->pulse_source_index;
                new_case->pulse_point_count = old_case->pulse_point_count;
                memcpy(new_case->pulse_points, old_case->pulse_points,
                       sizeof(new_case->pulse_points));
                new_case->pulse_hysteresis = 0.0f;
            }
            continue;
        }
        compact_rule_t entry;
        memcpy(&entry, blob + offset, sizeof(entry)); offset += sizeof(entry);
        if (entry.slot >= RELAY_RULE_MAX_RULES || used_rules[entry.slot]) return false;
        used_rules[entry.slot] = true;
        config->rules[entry.slot] = entry.rule;
    }
    return relay_rule_engine_validate(config, publish_rate_hz);
}

static bool read_rule_slot(const esp_partition_t *partition, unsigned slot,
                           rule_slot_header_t *header, uint8_t **payload)
{
    if (partition == NULL || header == NULL || payload == NULL || slot >= RULE_SLOT_COUNT ||
        partition->size % RULE_SLOT_COUNT != 0U) return false;
    const size_t slot_size = partition->size / RULE_SLOT_COUNT;
    const size_t slot_offset = slot * slot_size;
    if (esp_partition_read(partition, slot_offset, header, sizeof(*header)) != ESP_OK) return false;

    const uint32_t saved_header_crc = header->header_crc32;
    header->header_crc32 = 0U;
    const bool header_valid = header->magic == RULE_SLOT_MAGIC &&
        saved_header_crc == crc32(header, sizeof(*header)) &&
        header->payload_size >= sizeof(compact_header_t) &&
        header->payload_size <= slot_size - sizeof(*header);
    header->header_crc32 = saved_header_crc;
    if (!header_valid) return false;

    *payload = malloc(header->payload_size);
    if (*payload == NULL) return false;
    if (esp_partition_read(partition, slot_offset + sizeof(*header), *payload,
                           header->payload_size) != ESP_OK ||
        crc32(*payload, header->payload_size) != header->payload_crc32) {
        free(*payload);
        *payload = NULL;
        return false;
    }
    return true;
}

static int newest_slot(const bool valid[RULE_SLOT_COUNT],
                       const rule_slot_header_t headers[RULE_SLOT_COUNT])
{
    if (!valid[0]) return valid[1] ? 1 : -1;
    if (!valid[1]) return 0;
    return (int32_t)(headers[1].generation - headers[0].generation) > 0 ? 1 : 0;
}

static bool load_config(relay_rule_config_t *config)
{
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, RULE_CONFIG_PARTITION);
    rule_slot_header_t headers[RULE_SLOT_COUNT] = {0};
    uint8_t *payloads[RULE_SLOT_COUNT] = {0};
    bool valid[RULE_SLOT_COUNT] = {0};
    for (unsigned slot = 0; slot < RULE_SLOT_COUNT; ++slot)
        valid[slot] = read_rule_slot(partition, slot, &headers[slot], &payloads[slot]);

    const int newest = newest_slot(valid, headers);
    bool loaded = newest >= 0 && decode_config(payloads[newest], headers[newest].payload_size, config);
    if (!loaded && newest >= 0) {
        const unsigned other = (unsigned)newest ^ 1U;
        loaded = valid[other] && decode_config(payloads[other], headers[other].payload_size, config);
    }
    for (unsigned slot = 0; slot < RULE_SLOT_COUNT; ++slot) free(payloads[slot]);
    return loaded;
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

    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, RULE_CONFIG_PARTITION);
    if (partition == NULL || partition->size % RULE_SLOT_COUNT != 0U ||
        stored_length > partition->size / RULE_SLOT_COUNT - sizeof(rule_slot_header_t)) {
        free(stored);
        return false;
    }

    rule_slot_header_t headers[RULE_SLOT_COUNT] = {0};
    uint8_t *old_payloads[RULE_SLOT_COUNT] = {0};
    bool valid[RULE_SLOT_COUNT] = {0};
    for (unsigned slot = 0; slot < RULE_SLOT_COUNT; ++slot)
        valid[slot] = read_rule_slot(partition, slot, &headers[slot], &old_payloads[slot]);
    const int current = newest_slot(valid, headers);
    const unsigned target = current == 0 ? 1U : 0U;
    const uint32_t generation = current >= 0 ? headers[current].generation + 1U : 1U;
    for (unsigned slot = 0; slot < RULE_SLOT_COUNT; ++slot) free(old_payloads[slot]);

    const size_t slot_size = partition->size / RULE_SLOT_COUNT;
    const size_t slot_offset = target * slot_size;
    esp_err_t result = esp_partition_erase_range(partition, slot_offset, slot_size);
    if (result == ESP_OK)
        result = esp_partition_write(partition, slot_offset + sizeof(rule_slot_header_t),
                                     stored, stored_length);

    rule_slot_header_t slot_header = {
        .magic = RULE_SLOT_MAGIC,
        .generation = generation,
        .payload_size = stored_length,
        .payload_crc32 = crc32(stored, stored_length),
        .header_crc32 = 0U,
    };
    slot_header.header_crc32 = crc32(&slot_header, sizeof(slot_header));
    if (result == ESP_OK)
        result = esp_partition_write(partition, slot_offset, &slot_header, sizeof(slot_header));

    rule_slot_header_t verified_header = {0};
    uint8_t *verified_payload = NULL;
    const bool verified = result == ESP_OK &&
        read_rule_slot(partition, target, &verified_header, &verified_payload) &&
        verified_header.generation == generation &&
        verified_header.payload_size == stored_length &&
        memcmp(verified_payload, stored, stored_length) == 0;
    free(verified_payload);
    if (!verified) result = ESP_FAIL;
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
    if (load_config(active_config)) {
        ESP_LOGI(TAG, "Relay rule config loaded from dedicated rules partition");
    } else {
        ESP_LOGW(TAG, "No valid relay rule config found; using defaults");
        relay_rule_engine_set_defaults(active_config);
        if (!save_config(active_config)) ESP_LOGW(TAG, "Could not persist default rules");
    }
    log_rule_config(active_config);
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

static relay_rule_invalid_reason_t invalid_source_reason(const source_runtime_t *source,
                                                         uint32_t now_ms)
{
    if (!source->present) return RELAY_RULE_INVALID_SOURCE_NOT_RECEIVED;
    if (!source->accepted_valid) return RELAY_RULE_INVALID_SOURCE_UNCONFIRMED;
    if (now_ms - source->last_seen_ms >= active_config->signal_timeout_ms)
        return RELAY_RULE_INVALID_SOURCE_STALE;
    return RELAY_RULE_INVALID_NONE;
}

static void interpolate_pulse_timing(const relay_rule_case_t *entry, float input,
                                     uint32_t *on_time_ms, uint32_t *period_ms)
{
    unsigned upper = 0U;
    while (upper + 1U < entry->pulse_point_count && input > entry->pulse_points[upper + 1U].input_value) ++upper;
    const relay_pulse_point_t *a = &entry->pulse_points[upper];
    const relay_pulse_point_t *b = upper + 1U < entry->pulse_point_count ? &entry->pulse_points[upper + 1U] : a;
    float ratio = a == b ? 0.0f : (input - a->input_value) / (b->input_value - a->input_value);
    *on_time_ms = (uint32_t)lroundf(a->on_time_ms + ratio * ((float)b->on_time_ms - a->on_time_ms));
    *period_ms = (uint32_t)lroundf(a->period_ms + ratio * ((float)b->period_ms - a->period_ms));
}

static bool pulse_value(const relay_rule_case_t *entry, float input, uint32_t now_ms,
                        bool continue_cycle, bool was_on, rule_runtime_t *runtime)
{
    if (!continue_cycle || !runtime->pulse_input_valid ||
        runtime->pulse_started_ms == 0U || runtime->pulse_period_ms == 0U) {
        runtime->pulse_input_value = input;
        runtime->pulse_input_valid = true;
        runtime->pulse_started_ms = now_ms;
        interpolate_pulse_timing(entry, input, &runtime->pulse_on_time_ms,
                                 &runtime->pulse_period_ms);
    } else {
        /* Hysteresis is relative to the value captured at the start of this
         * cycle, rather than to each successive sample. Recalculate against
         * the same start timestamp so a source change adjusts the deadline
         * without restarting the countdown. */
        const float timing_input =
            fabsf(input - runtime->pulse_input_value) > entry->pulse_hysteresis
                ? input
                : runtime->pulse_input_value;
        uint32_t updated_on_time_ms;
        uint32_t updated_period_ms;
        interpolate_pulse_timing(entry, timing_input, &updated_on_time_ms,
                                 &updated_period_ms);
        if (was_on) {
            runtime->pulse_on_time_ms = updated_on_time_ms < runtime->pulse_period_ms
                                            ? updated_on_time_ms
                                            : runtime->pulse_period_ms;
        } else {
            runtime->pulse_on_time_ms = updated_on_time_ms;
            runtime->pulse_period_ms = updated_period_ms;
        }
        if (now_ms - runtime->pulse_started_ms >= runtime->pulse_period_ms) {
            runtime->pulse_input_value = input;
            runtime->pulse_started_ms = now_ms;
            interpolate_pulse_timing(entry, input, &runtime->pulse_on_time_ms,
                                     &runtime->pulse_period_ms);
        }
    }
    if (runtime->pulse_period_ms == 0U) return false;
    return runtime->pulse_on_time_ms >= runtime->pulse_period_ms ||
           now_ms - runtime->pulse_started_ms < runtime->pulse_on_time_ms;
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
        const bool was_pulse_active = runtime->pulse_active;
        const bool was_on = runtime->state;
        const int8_t previous_case = runtime->selected_case;
        runtime->state = false; runtime->valid = false; runtime->pulse_active = false; runtime->selected_case = -1;
        runtime->invalid_case = -1; runtime->invalid_test = -1; runtime->invalid_source = -1;
        runtime->invalid_pulse_source = false; runtime->invalid_reason = RELAY_RULE_INVALID_NONE;
        if (!output->enabled) continue;
        bool invalid = false;
        for (unsigned c = 0; c < output->case_count; ++c) {
            const relay_rule_case_t *entry = &output->cases[c];
            bool false_seen = false, unknown_seen = false;
            int first_unknown_test = -1;
            for (unsigned t = 0; t < entry->test_count; ++t) {
                const test_result_t result = evaluate_test(&entry->tests[t], &runtime->hysteresis[c][t], now_ms);
                false_seen |= result == TEST_FALSE;
                unknown_seen |= result == TEST_UNKNOWN;
                if (result == TEST_UNKNOWN && first_unknown_test < 0) first_unknown_test = (int)t;
            }
            if (false_seen) continue;
            if (unknown_seen) {
                const relay_rule_test_t *test = &entry->tests[first_unknown_test];
                runtime->invalid_case = (int8_t)c;
                runtime->invalid_test = (int8_t)first_unknown_test;
                runtime->invalid_source = (int8_t)test->source_index;
                runtime->invalid_reason = invalid_source_reason(&source_runtime[test->source_index], now_ms);
                invalid = true;
                break;
            }
            runtime->selected_case = (int8_t)c;
            runtime->valid = true;
            if (entry->action == RELAY_ACTION_ON) runtime->state = true;
            else if (entry->action == RELAY_ACTION_PULSE) {
                const source_runtime_t *source = &source_runtime[entry->pulse_source_index];
                if (!source->accepted_valid || now_ms - source->last_seen_ms >= active_config->signal_timeout_ms) {
                    runtime->valid = false;
                    runtime->invalid_case = (int8_t)c;
                    runtime->invalid_source = (int8_t)entry->pulse_source_index;
                    runtime->invalid_pulse_source = true;
                    runtime->invalid_reason = invalid_source_reason(source, now_ms);
                    invalid = true;
                    break;
                }
                runtime->pulse_active = true;
                runtime->state = pulse_value(entry, source->accepted_value, now_ms,
                                             was_pulse_active && previous_case == (int8_t)c,
                                             was_on, runtime);
            }
            break;
        }
        if (runtime->selected_case < 0 && !invalid) runtime->valid = true;
        if (!runtime->pulse_active) {
            runtime->pulse_started_ms = 0U;
            runtime->pulse_on_time_ms = 0U;
            runtime->pulse_period_ms = 0U;
            runtime->pulse_input_value = 0.0f;
            runtime->pulse_input_valid = false;
        }
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
        const rule_runtime_t *runtime = &rule_runtime[i];
        uint32_t next_on_ms = 0U;
        if (runtime->pulse_active && runtime->pulse_period_ms > 0U &&
            runtime->pulse_on_time_ms > 0U &&
            runtime->pulse_on_time_ms < runtime->pulse_period_ms) {
            const uint32_t phase_ms = (now_ms - runtime->pulse_started_ms) % runtime->pulse_period_ms;
            next_on_ms = runtime->pulse_period_ms - phase_ms;
        }
        rules[i] = (relay_rule_status_t){
            .configured = active_config->rules[i].enabled || active_config->rules[i].case_count > 0U,
            .enabled = active_config->rules[i].enabled,
            .valid = runtime->valid,
            .state = runtime->state,
            .pulse_active = runtime->pulse_active,
            .selected_case = runtime->selected_case,
            .pulse_on_time_ms = runtime->pulse_on_time_ms,
            .pulse_period_ms = runtime->pulse_period_ms,
            .pulse_next_on_ms = next_on_ms,
            .invalid_case = runtime->invalid_case,
            .invalid_test = runtime->invalid_test,
            .invalid_source = runtime->invalid_source,
            .invalid_pulse_source = runtime->invalid_pulse_source,
            .invalid_reason = runtime->invalid_reason,
        };
    }
    for (unsigned i = 0; i < RELAY_RULE_MAX_SOURCES; ++i) {
        sources[i] = (relay_rule_source_status_t){.present = source_runtime[i].present, .accepted_valid = source_runtime[i].accepted_valid, .value = source_runtime[i].accepted_value, .age_ms = source_runtime[i].present ? now_ms - source_runtime[i].last_seen_ms : UINT32_MAX, .zero_streak = source_runtime[i].zero_streak, .zero_confirm_samples = active_config->sources[i].zero_confirm_samples};
        strlcpy(sources[i].name, active_config->sources[i].name, sizeof(sources[i].name));
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
