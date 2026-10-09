#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_partition.h"
#include "inc/relay_rule_engine.h"

#define TEST_RULE_COMPACT_MAGIC 0x52554C32U
#define TEST_RULE_SLOT_MAGIC 0x52534C54U
#define TEST_OLD_COMPACT_VERSION 2U
#define TEST_LEGACY_COMPACT_VERSION 4U
#define TEST_NEW_COMPACT_VERSION 5U
#define TEST_SLOT_COUNT 2U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t source_count;
    uint16_t rule_count;
    uint16_t reserved;
    uint32_t total_size;
    uint32_t signal_timeout_ms;
    uint32_t crc32;
} test_compact_header_t;

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint32_t payload_size;
    uint32_t payload_crc32;
    uint32_t header_crc32;
} test_slot_header_t;

typedef struct {
    char name[RELAY_RULE_NAME_LENGTH];
    relay_source_type_t type;
    uint8_t local_channel;
    uint32_t can_id;
    bool extended;
    uint8_t start_bit;
    uint8_t bit_length;
    bool little_endian;
    bool is_signed;
    float factor;
    float offset;
    uint8_t zero_confirm_samples;
} test_source_v4_t;

typedef struct {
    uint8_t slot;
    uint8_t reserved[3];
    test_source_v4_t source;
} test_compact_source_v4_t;

static uint32_t test_crc32(const void *data, size_t length)
{
    uint32_t crc = UINT32_MAX;
    const uint8_t *bytes = data;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : crc >> 1;
    }
    return ~crc;
}

static void install_v4_record(void)
{
    test_partition_reset();
    uint8_t *storage = test_partition_data();
    const size_t payload_size = sizeof(test_compact_header_t) +
                                sizeof(test_compact_source_v4_t);
    uint8_t payload[sizeof(test_compact_header_t) + sizeof(test_compact_source_v4_t)] = {0};
    test_compact_header_t *header = (test_compact_header_t *)payload;
    *header = (test_compact_header_t){
        .magic = TEST_RULE_COMPACT_MAGIC,
        .version = TEST_LEGACY_COMPACT_VERSION,
        .source_count = 1U,
        .total_size = payload_size,
        .signal_timeout_ms = 4321U,
    };
    test_compact_source_v4_t *entry =
        (test_compact_source_v4_t *)(payload + sizeof(*header));
    entry->slot = 20U;
    strcpy(entry->source.name, "OilTemp.OilTemperature");
    entry->source.type = RELAY_SOURCE_CAN;
    entry->source.can_id = 0x588U;
    entry->source.start_bit = 56U;
    entry->source.bit_length = 8U;
    entry->source.little_endian = true;
    entry->source.factor = 1.0f;
    entry->source.offset = -60.0f;
    entry->source.zero_confirm_samples = 11U;
    header->crc32 = test_crc32(payload, payload_size);

    test_slot_header_t slot = {
        .magic = TEST_RULE_SLOT_MAGIC,
        .generation = 3U,
        .payload_size = payload_size,
        .payload_crc32 = test_crc32(payload, payload_size),
    };
    slot.header_crc32 = test_crc32(&slot, sizeof(slot));
    memcpy(storage, &slot, sizeof(slot));
    memcpy(storage + sizeof(slot), payload, payload_size);
}

static void assert_v4_record_migrates_in_memory(void)
{
    install_v4_record();
    relay_rule_engine_set_publish_rate(25U);
    relay_rule_engine_init();

    relay_rule_config_t config;
    relay_rule_engine_snapshot(&config);
    assert(config.version == 5U);
    assert(config.signal_timeout_ms == 4321U);
    assert(strcmp(config.sources[20].name, "OilTemp.OilTemperature") == 0);
    assert(config.sources[20].can_id == 0x588U);
    assert(!config.sources[20].range_enabled);
    assert(!config.sources[20].invalid_raw_enabled);
}

static void install_old_v2_record(void)
{
    test_partition_reset();
    uint8_t *storage = test_partition_data();

    test_compact_header_t payload = {
        .magic = TEST_RULE_COMPACT_MAGIC,
        .version = TEST_OLD_COMPACT_VERSION,
        .source_count = 0U,
        .rule_count = 0U,
        .reserved = 0U,
        .total_size = sizeof(payload),
        .signal_timeout_ms = 4321U,
        .crc32 = 0U,
    };
    payload.crc32 = test_crc32(&payload, sizeof(payload));

    test_slot_header_t slot = {
        .magic = TEST_RULE_SLOT_MAGIC,
        .generation = 7U,
        .payload_size = sizeof(payload),
        .payload_crc32 = test_crc32(&payload, sizeof(payload)),
        .header_crc32 = 0U,
    };
    slot.header_crc32 = test_crc32(&slot, sizeof(slot));

    memcpy(storage, &slot, sizeof(slot));
    memcpy(storage + sizeof(slot), &payload, sizeof(payload));
}

static void assert_legacy_record_rejected_and_replaced(void)
{
    install_old_v2_record();
    relay_rule_engine_set_publish_rate(25U);
    relay_rule_engine_init();

    relay_rule_config_t config;
    relay_rule_engine_snapshot(&config);
    assert(config.version == 5U);
    assert(config.signal_timeout_ms == 1000U);
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i) {
        char expected[RELAY_RULE_NAME_LENGTH];
        snprintf(expected, sizeof(expected), "Output %u", i + 1U);
        assert(strcmp(config.rules[i].label, expected) == 0);
        assert(!config.rules[i].enabled);
        assert(config.rules[i].case_count == 0U);
    }

    const size_t slot_size = test_partition_size() / TEST_SLOT_COUNT;
    const uint8_t *storage = test_partition_data();
    for (unsigned slot_index = 0U; slot_index < TEST_SLOT_COUNT; ++slot_index) {
        const uint8_t *slot_base = storage + slot_index * slot_size;
        const test_slot_header_t *slot = (const test_slot_header_t *)slot_base;
        assert(slot->magic == TEST_RULE_SLOT_MAGIC);
        assert(slot->generation == (slot_index == 0U ? 9U : 8U));

        test_slot_header_t header_copy = *slot;
        const uint32_t header_crc = header_copy.header_crc32;
        header_copy.header_crc32 = 0U;
        assert(header_crc == test_crc32(&header_copy, sizeof(header_copy)));
        assert(slot->payload_crc32 ==
               test_crc32(slot_base + sizeof(*slot), slot->payload_size));

        const test_compact_header_t *new_payload =
            (const test_compact_header_t *)(slot_base + sizeof(*slot));
        assert(new_payload->magic == TEST_RULE_COMPACT_MAGIC);
        assert(new_payload->version == TEST_NEW_COMPACT_VERSION);
        assert(new_payload->source_count == 0U);
        assert(new_payload->rule_count == 0U);
        assert(new_payload->signal_timeout_ms == 1000U);
    }
}

static relay_rule_test_t uptime_test(relay_compare_t comparison, float threshold)
{
    return (relay_rule_test_t){
        .type = RELAY_TEST_UPTIME,
        .comparison = comparison,
        .threshold = threshold,
    };
}

static relay_rule_test_t source_test(uint8_t source, relay_compare_t comparison,
                                     float threshold, float hysteresis)
{
    return (relay_rule_test_t){
        .type = RELAY_TEST_SOURCE,
        .source_index = source,
        .comparison = comparison,
        .hysteresis_enabled = hysteresis > 0.0f,
        .threshold = threshold,
        .hysteresis = hysteresis,
    };
}

static relay_rule_test_t triggered_timer_test(uint8_t source, relay_compare_t comparison,
                                                float threshold, float hysteresis,
                                                uint32_t duration_ms)
{
    return (relay_rule_test_t){
        .type = RELAY_TEST_TRIGGER_TIMER,
        .source_index = source,
        .comparison = comparison,
        .hysteresis_enabled = hysteresis > 0.0f,
        .threshold = threshold,
        .hysteresis = hysteresis,
        .trigger_duration_ms = duration_ms,
    };
}

static relay_rule_test_t condition_test(uint8_t condition, bool expected)
{
    return (relay_rule_test_t){
        .type = RELAY_TEST_CONDITION,
        .source_index = condition,
        .comparison = RELAY_COMPARE_EQ,
        .threshold = expected ? 1.0f : 0.0f,
    };
}

static relay_rule_test_t triggered_condition_timer_test(uint8_t condition, bool expected,
                                                         uint32_t duration_ms)
{
    return (relay_rule_test_t){
        .type = RELAY_TEST_TRIGGER_TIMER,
        .source_index = (uint8_t)(RELAY_RULE_TRIGGER_CONDITION_FLAG | condition),
        .comparison = RELAY_COMPARE_EQ,
        .threshold = expected ? 1.0f : 0.0f,
        .trigger_duration_ms = duration_ms,
    };
}

static void configure_condition(relay_rule_config_t *config, uint8_t slot, const char *label,
                                uint8_t source, relay_compare_t comparison, float threshold,
                                float hysteresis, relay_condition_stale_behavior_t stale_behavior)
{
    relay_condition_config_t *condition = &config->conditions[slot];
    memset(condition, 0, sizeof(*condition));
    snprintf(condition->label, sizeof(condition->label), "%s", label);
    condition->source_index = source;
    condition->comparison = comparison;
    condition->threshold = threshold;
    condition->hysteresis_enabled = hysteresis > 0.0f;
    condition->hysteresis = hysteresis;
    condition->stale_behavior = stale_behavior;
}

static void configure_can_rpm(relay_rule_config_t *config, uint8_t slot)
{
    relay_source_config_t *source = &config->sources[slot];
    strcpy(source->name, "DME1.RPM");
    source->type = RELAY_SOURCE_CAN;
    source->can_id = 0x316U;
    source->start_bit = 0U;
    source->bit_length = 16U;
    source->little_endian = true;
    source->factor = 1.0f;
    source->zero_confirm_samples = 1U;
}

static void ingest_rpm(uint16_t rpm, uint32_t now_ms)
{
    twai_message_t message = {
        .identifier = 0x316U,
        .data_length_code = 2U,
        .data = {(uint8_t)(rpm & 0xFFU), (uint8_t)(rpm >> 8)},
    };
    relay_rule_engine_ingest_can(&message, now_ms);
}

static relay_rule_config_t fresh_config(void)
{
    relay_rule_config_t config;
    relay_rule_engine_set_defaults(&config);
    return config;
}

static relay_rule_case_t always_case(relay_action_t action)
{
    relay_rule_case_t entry = {0};
    entry.test_count = 1U;
    entry.tests[0] = uptime_test(RELAY_COMPARE_GE, 0.0f);
    entry.action = action;
    return entry;
}

static void install_config(const relay_rule_config_t *config)
{
    assert(relay_rule_engine_validate(config, 25U));
    assert(relay_rule_engine_replace_and_save(config));
}

static void assert_output(const output_command_t *command, unsigned output,
                          bool valid, bool state, bool pulse, uint8_t duty)
{
    const uint8_t bit = (uint8_t)(1U << output);
    assert(((command->valid_mask & bit) != 0U) == valid);
    assert(((command->state_mask & bit) != 0U) == state);
    assert(((command->pulse_mask & bit) != 0U) == pulse);
    assert(command->duty_percent[output] == duty);
}

static void test_ordered_off_on_and_timer(void)
{
    relay_rule_config_t config = fresh_config();
    relay_output_rule_t *output = &config.rules[0];
    output->enabled = true;
    output->case_count = 2U;
    output->cases[0] = always_case(RELAY_ACTION_OFF);
    output->cases[0].tests[0] = uptime_test(RELAY_COMPARE_GE, 1000.0f);
    output->cases[1] = always_case(RELAY_ACTION_ON);
    install_config(&config);

    output_command_t command;
    relay_rule_engine_make_command(500U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    relay_rule_engine_make_command(1000U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
}

static void test_all_eight_binary_outputs(void)
{
    relay_rule_config_t config = fresh_config();
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i) {
        config.rules[i].enabled = true;
        config.rules[i].case_count = 1U;
        config.rules[i].cases[0] = always_case((i & 1U) == 0U ? RELAY_ACTION_ON : RELAY_ACTION_OFF);
    }
    install_config(&config);

    output_command_t command;
    relay_rule_engine_make_command(100U, &command);
    assert(command.valid_mask == 0xFFU);
    assert(command.state_mask == 0x55U);
    assert(command.pulse_mask == 0x00U);
    for (unsigned i = 0; i < RELAY_RULE_MAX_RULES; ++i)
        assert(command.duty_percent[i] == 0U);
}

static void test_source_hysteresis(void)
{
    relay_rule_config_t config = fresh_config();
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
    config.rules[0].cases[0].tests[0] = source_test(10U, RELAY_COMPARE_GT, 10.0f, 2.0f);
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 11.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    relay_rule_engine_ingest_local(0U, true, 9.5f, 110U);
    relay_rule_engine_make_command(110U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    relay_rule_engine_ingest_local(0U, true, 8.0f, 120U);
    relay_rule_engine_make_command(120U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    relay_rule_engine_ingest_local(0U, true, 7.9f, 130U);
    relay_rule_engine_make_command(130U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
}

static void assert_comparison(relay_compare_t comparison, float value, bool expected_state)
{
    relay_rule_config_t config = fresh_config();
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
    config.rules[0].cases[0].tests[0] = source_test(10U, comparison, 10.0f, 0.0f);
    install_config(&config);

    relay_rule_engine_ingest_local(0U, true, value, 100U);
    output_command_t command;
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, expected_state, false, 0U);
}

static void test_all_comparisons_and_and_logic(void)
{
    assert_comparison(RELAY_COMPARE_GT, 10.0f, false);
    assert_comparison(RELAY_COMPARE_GT, 10.1f, true);
    assert_comparison(RELAY_COMPARE_GE, 9.9f, false);
    assert_comparison(RELAY_COMPARE_GE, 10.0f, true);
    assert_comparison(RELAY_COMPARE_LT, 10.0f, false);
    assert_comparison(RELAY_COMPARE_LT, 9.9f, true);
    assert_comparison(RELAY_COMPARE_LE, 10.1f, false);
    assert_comparison(RELAY_COMPARE_LE, 10.0f, true);
    assert_comparison(RELAY_COMPARE_EQ, 9.9f, false);
    assert_comparison(RELAY_COMPARE_EQ, 10.0f, true);
    assert_comparison(RELAY_COMPARE_NE, 10.0f, false);
    assert_comparison(RELAY_COMPARE_NE, 10.1f, true);

    relay_rule_config_t config = fresh_config();
    config.rules[0].enabled = true;
    config.rules[0].case_count = 2U;
    config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
    config.rules[0].cases[0].test_count = 2U;
    config.rules[0].cases[0].tests[0] = uptime_test(RELAY_COMPARE_GE, 100.0f);
    config.rules[0].cases[0].tests[1] = uptime_test(RELAY_COMPARE_LT, 200.0f);
    config.rules[0].cases[1] = always_case(RELAY_ACTION_OFF);
    install_config(&config);

    output_command_t command;
    relay_rule_engine_make_command(150U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    relay_rule_engine_make_command(250U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
}

static void test_pulse_regression(void)
{
    relay_rule_config_t config = fresh_config();
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    relay_rule_case_t *entry = &config.rules[0].cases[0];
    *entry = always_case(RELAY_ACTION_PULSE);
    entry->pulse_source_index = 10U;
    entry->pulse_point_count = 1U;
    entry->pulse_points[0] = (relay_pulse_point_t){
        .input_value = 1.0f,
        .on_time_ms = 100U,
        .period_ms = 200U,
    };
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 1.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, true, true, 0U);
    relay_rule_engine_make_command(199U, &command);
    assert_output(&command, 0U, true, true, true, 0U);
    relay_rule_engine_make_command(200U, &command);
    assert_output(&command, 0U, true, false, true, 0U);
    relay_rule_engine_make_command(300U, &command);
    assert_output(&command, 0U, true, true, true, 0U);
}

static void test_pulse_lookup_interpolation(void)
{
    relay_rule_config_t config = fresh_config();
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    relay_rule_case_t *entry = &config.rules[0].cases[0];
    *entry = always_case(RELAY_ACTION_PULSE);
    entry->pulse_source_index = 10U;
    entry->pulse_point_count = 2U;
    entry->pulse_points[0] = (relay_pulse_point_t){
        .input_value = 0.0f, .on_time_ms = 100U, .period_ms = 200U};
    entry->pulse_points[1] = (relay_pulse_point_t){
        .input_value = 10.0f, .on_time_ms = 200U, .period_ms = 400U};
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 5.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, true, true, 0U);

    relay_rule_status_t outputs[RELAY_RULE_MAX_RULES];
    relay_rule_source_status_t sources[RELAY_RULE_MAX_SOURCES];
    relay_rule_engine_get_status(outputs, sources, 100U);
    assert(outputs[0].pulse_on_time_ms == 150U);
    assert(outputs[0].pulse_period_ms == 300U);

    relay_rule_engine_make_command(249U, &command);
    assert_output(&command, 0U, true, true, true, 0U);
    relay_rule_engine_make_command(250U, &command);
    assert_output(&command, 0U, true, false, true, 0U);
    relay_rule_engine_make_command(400U, &command);
    assert_output(&command, 0U, true, true, true, 0U);
}

static void test_pulse_continuous_on_off_and_validation(void)
{
    relay_rule_config_t config = fresh_config();
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    relay_rule_case_t *entry = &config.rules[0].cases[0];
    *entry = always_case(RELAY_ACTION_PULSE);
    entry->pulse_source_index = 10U;
    entry->pulse_point_count = 1U;
    entry->pulse_points[0] = (relay_pulse_point_t){
        .input_value = 1.0f,
        .on_time_ms = 200U,
        .period_ms = 200U,
    };

    /* ON time equal to period is the explicit continuous-ON representation. */
    assert(relay_rule_engine_validate(&config, 25U));
    install_config(&config);
    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 1.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, true, true, 0U);
    relay_rule_engine_make_command(500U, &command);
    assert_output(&command, 0U, true, true, true, 0U);

    /* ON time zero is the explicit continuous-OFF representation. */
    relay_rule_config_t off = config;
    off.rules[0].cases[0].pulse_points[0].on_time_ms = 0U;
    assert(relay_rule_engine_validate(&off, 25U));
    install_config(&off);
    relay_rule_engine_ingest_local(0U, true, 1.0f, 600U);
    relay_rule_engine_make_command(600U, &command);
    assert_output(&command, 0U, true, false, true, 0U);

    relay_rule_config_t bad = config;
    bad.rules[0].cases[0].pulse_points[0].period_ms = 0U;
    assert(!relay_rule_engine_validate(&bad, 25U));

    bad = config;
    bad.rules[0].cases[0].pulse_points[0].on_time_ms = 201U;
    assert(!relay_rule_engine_validate(&bad, 25U));
}

static relay_rule_config_t pwm_config(float hysteresis, uint32_t timeout_ms)
{
    relay_rule_config_t config = fresh_config();
    config.signal_timeout_ms = timeout_ms;
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    relay_rule_case_t *entry = &config.rules[0].cases[0];
    *entry = always_case(RELAY_ACTION_PWM);
    entry->pwm_source_index = 10U;
    entry->pwm_hysteresis = hysteresis;
    entry->pwm_point_count = 2U;
    entry->pwm_points[0] = (relay_pwm_point_t){.input_value = 0.0f, .duty_percent = 0U};
    entry->pwm_points[1] = (relay_pwm_point_t){.input_value = 10.0f, .duty_percent = 100U};
    return config;
}

static void test_pwm_interpolation_clamp_hysteresis_and_state(void)
{
    relay_rule_config_t config = pwm_config(1.0f, 1000U);
    install_config(&config);
    output_command_t command;

    relay_rule_engine_ingest_local(0U, true, 5.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, false, false, 50U);

    relay_rule_engine_ingest_local(0U, true, 5.5f, 110U);
    relay_rule_engine_make_command(110U, &command);
    assert_output(&command, 0U, true, false, false, 50U);

    relay_rule_engine_ingest_local(0U, true, 6.0f, 120U);
    relay_rule_engine_make_command(120U, &command);
    assert_output(&command, 0U, true, false, false, 50U);

    relay_rule_engine_ingest_local(0U, true, 6.01f, 130U);
    relay_rule_engine_make_command(130U, &command);
    assert_output(&command, 0U, true, false, false, 60U);

    relay_rule_engine_ingest_local(0U, true, -1.0f, 140U);
    relay_rule_engine_make_command(140U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    relay_rule_engine_ingest_local(0U, true, 11.0f, 150U);
    relay_rule_engine_make_command(150U, &command);
    assert_output(&command, 0U, true, true, false, 100U);
}

static void test_pwm_from_imported_can_source(void)
{
    relay_rule_config_t config = fresh_config();
    relay_source_config_t *source = &config.sources[20U];
    strcpy(source->name, "Imported.PWM");
    source->type = RELAY_SOURCE_CAN;
    source->can_id = 0x321U;
    source->start_bit = 0U;
    source->bit_length = 8U;
    source->little_endian = true;
    source->factor = 1.0f;
    source->zero_confirm_samples = 1U;

    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    relay_rule_case_t *entry = &config.rules[0].cases[0];
    *entry = always_case(RELAY_ACTION_PWM);
    entry->pwm_source_index = 20U;
    entry->pwm_point_count = 2U;
    entry->pwm_points[0] = (relay_pwm_point_t){.input_value = 0.0f, .duty_percent = 0U};
    entry->pwm_points[1] = (relay_pwm_point_t){.input_value = 10.0f, .duty_percent = 100U};
    install_config(&config);

    twai_message_t message = {
        .identifier = 0x321U,
        .data_length_code = 1U,
        .data = {5U},
    };
    relay_rule_engine_ingest_can(&message, 100U);
    output_command_t command;
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, false, false, 50U);
}

static void test_pwm_rounding(void)
{
    relay_rule_config_t config = pwm_config(0.0f, 1000U);
    relay_rule_case_t *entry = &config.rules[0].cases[0];
    entry->pwm_points[1] = (relay_pwm_point_t){.input_value = 2.0f, .duty_percent = 1U};
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 1.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, false, false, 1U);
}

static void test_pwm_timeout_and_status(void)
{
    relay_rule_config_t config = pwm_config(0.0f, 100U);
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 5.0f, 100U);
    relay_rule_engine_make_command(199U, &command);
    assert_output(&command, 0U, true, false, false, 50U);
    relay_rule_engine_make_command(200U, &command);
    assert_output(&command, 0U, false, false, false, 0U);

    relay_rule_status_t outputs[RELAY_RULE_MAX_RULES];
    relay_rule_source_status_t sources[RELAY_RULE_MAX_SOURCES];
    relay_rule_engine_get_status(outputs, sources, 200U);
    assert(!outputs[0].valid);
    assert(outputs[0].invalid_pwm_source);
    assert(outputs[0].invalid_reason == RELAY_RULE_INVALID_SOURCE_STALE);
    assert(outputs[0].duty_percent == 0U);

    relay_rule_engine_ingest_local(0U, true, 5.5f, 210U);
    relay_rule_engine_make_command(210U, &command);
    assert_output(&command, 0U, true, false, false, 55U);
}

static void test_pwm_zero_confirmation(void)
{
    relay_rule_config_t config = pwm_config(0.0f, 1000U);
    config.sources[10].zero_confirm_samples = 2U;
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 0.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, false, false, false, 0U);

    relay_rule_status_t outputs[RELAY_RULE_MAX_RULES];
    relay_rule_source_status_t sources[RELAY_RULE_MAX_SOURCES];
    relay_rule_engine_get_status(outputs, sources, 100U);
    assert(outputs[0].invalid_pwm_source);
    assert(outputs[0].invalid_reason == RELAY_RULE_INVALID_SOURCE_UNCONFIRMED);

    relay_rule_engine_ingest_local(0U, true, 0.0f, 110U);
    relay_rule_engine_make_command(110U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
}

static void test_pwm_case_change_recalculates_immediately(void)
{
    relay_rule_config_t config = fresh_config();
    config.rules[0].enabled = true;
    config.rules[0].case_count = 2U;

    relay_rule_case_t *first = &config.rules[0].cases[0];
    *first = always_case(RELAY_ACTION_PWM);
    first->tests[0] = source_test(0U, RELAY_COMPARE_GT, 0.0f, 0.0f);
    first->pwm_source_index = 10U;
    first->pwm_hysteresis = 10.0f;
    first->pwm_point_count = 2U;
    first->pwm_points[0] = (relay_pwm_point_t){.input_value = 0.0f, .duty_percent = 0U};
    first->pwm_points[1] = (relay_pwm_point_t){.input_value = 10.0f, .duty_percent = 100U};

    relay_rule_case_t *second = &config.rules[0].cases[1];
    *second = always_case(RELAY_ACTION_PWM);
    second->pwm_source_index = 10U;
    second->pwm_hysteresis = 10.0f;
    second->pwm_point_count = 2U;
    second->pwm_points[0] = (relay_pwm_point_t){.input_value = 0.0f, .duty_percent = 0U};
    second->pwm_points[1] = (relay_pwm_point_t){.input_value = 10.0f, .duty_percent = 50U};
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, false, 1.0f, 100U);
    relay_rule_engine_ingest_local(0U, true, 8.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, false, false, 80U);

    relay_rule_engine_ingest_local(0U, false, -1.0f, 110U);
    relay_rule_engine_ingest_local(0U, true, 8.5f, 110U);
    relay_rule_engine_make_command(110U, &command);
    assert_output(&command, 0U, true, false, false, 43U);
}

static void test_live_config_replacement_resets_pwm_runtime(void)
{
    relay_rule_config_t config = pwm_config(10.0f, 1000U);
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 8.0f, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, false, false, 80U);

    assert(relay_rule_engine_replace_and_save(&config));
    relay_rule_engine_make_command(110U, &command);
    assert_output(&command, 0U, false, false, false, 0U);

    relay_rule_engine_ingest_local(0U, true, 8.5f, 120U);
    relay_rule_engine_make_command(120U, &command);
    assert_output(&command, 0U, true, false, false, 85U);
}

static void test_pwm_validation(void)
{
    relay_rule_config_t config = pwm_config(0.0f, 1000U);
    assert(relay_rule_engine_validate(&config, 25U));

    relay_rule_config_t bad = config;
    bad.rules[0].cases[0].pwm_point_count = 0U;
    assert(!relay_rule_engine_validate(&bad, 25U));

    bad = config;
    bad.rules[0].cases[0].pwm_point_count = RELAY_RULE_MAX_PWM_POINTS + 1U;
    assert(!relay_rule_engine_validate(&bad, 25U));

    bad = config;
    bad.rules[0].cases[0].pwm_points[1].input_value = 0.0f;
    assert(!relay_rule_engine_validate(&bad, 25U));

    bad = config;
    bad.rules[0].cases[0].pwm_points[0].input_value = NAN;
    assert(!relay_rule_engine_validate(&bad, 25U));

    bad = config;
    bad.rules[0].cases[0].pwm_points[0].duty_percent = 101U;
    assert(!relay_rule_engine_validate(&bad, 25U));

    bad = config;
    bad.rules[0].cases[0].pwm_hysteresis = -0.01f;
    assert(!relay_rule_engine_validate(&bad, 25U));

    bad = config;
    bad.rules[0].enabled = false;
    bad.rules[0].cases[0].pwm_points[1].input_value = 0.0f;
    assert(!relay_rule_engine_validate(&bad, 25U));

    bad = config;
    bad.rules[0].cases[0].pwm_source_index = RELAY_RULE_MAX_SOURCES;
    assert(!relay_rule_engine_validate(&bad, 25U));
}

static relay_rule_config_t inline_trigger_timer_config(uint32_t duration_ms, float hysteresis)
{
    relay_rule_config_t config = fresh_config();
    config.signal_timeout_ms = 1000U;
    configure_can_rpm(&config, 20U);
    relay_output_rule_t *output = &config.rules[0];
    output->enabled = true;
    output->case_count = 1U;
    output->cases[0] = always_case(RELAY_ACTION_ON);
    output->cases[0].test_count = 2U;
    output->cases[0].tests[0] = source_test(10U, RELAY_COMPARE_GT, 100.0f, 0.0f);
    output->cases[0].tests[1] = triggered_timer_test(
        20U, RELAY_COMPARE_LT, 600.0f, hysteresis, duration_ms);
    return config;
}

static void test_inline_triggered_timer_post_shutdown(void)
{
    relay_rule_config_t config = inline_trigger_timer_config(300000U, 0.0f);
    install_config(&config);
    output_command_t command;

    /* Booting with RPM already below the threshold must not invent a shutdown. */
    relay_rule_engine_ingest_local(0U, true, 105.0f, 100U);
    ingest_rpm(500U, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    /* Observing the trigger false arms it. */
    relay_rule_engine_ingest_local(0U, true, 105.0f, 200U);
    ingest_rpm(1000U, 200U);
    relay_rule_engine_make_command(200U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    /* The false -> true edge starts the 300 second latched timer. */
    relay_rule_engine_ingest_local(0U, true, 105.0f, 300U);
    ingest_rpm(500U, 300U);
    relay_rule_engine_make_command(300U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    /* Once active, stale CAN must not cancel the timer. */
    relay_rule_engine_ingest_local(0U, true, 105.0f, 2000U);
    relay_rule_engine_make_command(2000U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    /* Other AND tests can stop the Output without stopping the timer. */
    relay_rule_engine_ingest_local(0U, true, 95.0f, 2500U);
    relay_rule_engine_make_command(2500U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
    relay_rule_engine_ingest_local(0U, true, 105.0f, 3000U);
    relay_rule_engine_make_command(3000U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    /* Expiry makes the timer test false, not invalid. */
    relay_rule_engine_ingest_local(0U, true, 105.0f, 300300U);
    relay_rule_engine_make_command(300300U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    /* A new running -> stopped transition starts a fresh full period. */
    relay_rule_engine_ingest_local(0U, true, 105.0f, 301000U);
    ingest_rpm(1000U, 301000U);
    relay_rule_engine_make_command(301000U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
    relay_rule_engine_ingest_local(0U, true, 105.0f, 301100U);
    ingest_rpm(500U, 301100U);
    relay_rule_engine_make_command(301100U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
}

static void test_inline_triggered_timer_updates_behind_higher_priority_case(void)
{
    relay_rule_config_t config = fresh_config();
    configure_can_rpm(&config, 20U);
    relay_output_rule_t *output = &config.rules[0];
    output->enabled = true;
    output->case_count = 2U;
    output->cases[0] = always_case(RELAY_ACTION_ON);
    output->cases[0].tests[0] = source_test(10U, RELAY_COMPARE_GT, 110.0f, 0.0f);
    output->cases[1] = always_case(RELAY_ACTION_ON);
    output->cases[1].tests[0] = triggered_timer_test(
        20U, RELAY_COMPARE_LT, 600.0f, 0.0f, 300000U);
    install_config(&config);

    output_command_t command;
    relay_rule_engine_ingest_local(0U, true, 120.0f, 100U);
    ingest_rpm(1000U, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    /* Case 1 still wins here, but the case-2 trigger must latch in parallel. */
    relay_rule_engine_ingest_local(0U, true, 120.0f, 200U);
    ingest_rpm(500U, 200U);
    relay_rule_engine_make_command(200U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    /* When case 1 stops matching, case 2 sees the already-running timer. */
    relay_rule_engine_ingest_local(0U, true, 100.0f, 300U);
    relay_rule_engine_make_command(300U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
}

static void test_inline_triggered_timer_hysteresis_and_reset(void)
{
    relay_rule_config_t config = fresh_config();
    configure_can_rpm(&config, 20U);
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
    config.rules[0].cases[0].tests[0] = triggered_timer_test(
        20U, RELAY_COMPARE_LT, 600.0f, 50.0f, 1000U);
    install_config(&config);

    output_command_t command;
    ingest_rpm(700U, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
    ingest_rpm(590U, 200U);
    relay_rule_engine_make_command(200U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    /* Prior-true LT hysteresis holds through 620 RPM, so it does not re-arm. */
    ingest_rpm(620U, 300U);
    relay_rule_engine_make_command(300U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    ingest_rpm(590U, 400U);
    relay_rule_engine_make_command(400U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    /* Above threshold+hysteresis re-arms; another drop restarts the full timer. */
    ingest_rpm(651U, 500U);
    relay_rule_engine_make_command(500U, &command);
    ingest_rpm(590U, 600U);
    relay_rule_engine_make_command(600U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    relay_rule_engine_make_command(1500U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    relay_rule_engine_make_command(1600U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    /* Replacing configuration clears armed/active state. */
    ingest_rpm(700U, 1700U);
    relay_rule_engine_make_command(1700U, &command);
    ingest_rpm(590U, 1800U);
    relay_rule_engine_make_command(1800U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    assert(relay_rule_engine_replace_and_save(&config));
    relay_rule_engine_make_command(1810U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
}

static void test_inline_triggered_timer_validation(void)
{
    relay_rule_config_t config = fresh_config();
    configure_can_rpm(&config, 20U);
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
    config.rules[0].cases[0].tests[0] = triggered_timer_test(
        20U, RELAY_COMPARE_LT, 600.0f, 0.0f, 300000U);
    assert(relay_rule_engine_validate(&config, 25U));

    relay_rule_config_t bad = config;
    bad.rules[0].cases[0].tests[0].trigger_duration_ms = 0U;
    assert(!relay_rule_engine_validate(&bad, 25U));
    bad = config;
    bad.rules[0].cases[0].tests[0].trigger_duration_ms = 86400001U;
    assert(!relay_rule_engine_validate(&bad, 25U));
    bad = config;
    bad.rules[0].cases[0].tests[0].source_index = RELAY_RULE_MAX_SOURCES;
    assert(!relay_rule_engine_validate(&bad, 25U));
    bad = config;
    bad.rules[0].cases[0].tests[0].hysteresis = -0.1f;
    assert(!relay_rule_engine_validate(&bad, 25U));
}

static void test_conditions_reused_and_stale_false(void)
{
    relay_rule_config_t config = fresh_config();
    config.signal_timeout_ms = 1000U;
    configure_can_rpm(&config, 20U);
    configure_condition(&config, 0U, "Engine running", 20U, RELAY_COMPARE_GE, 600.0f,
                        0.0f, RELAY_CONDITION_STALE_FALSE);

    for (unsigned output = 0U; output < 2U; ++output) {
        config.rules[output].enabled = true;
        config.rules[output].case_count = 1U;
        config.rules[output].cases[0] = always_case(RELAY_ACTION_ON);
        config.rules[output].cases[0].tests[0] = condition_test(0U, true);
    }
    install_config(&config);

    output_command_t command;
    relay_rule_engine_make_command(0U, &command);
    assert_output(&command, 0U, false, false, false, 0U);
    assert_output(&command, 1U, false, false, false, 0U);

    relay_condition_status_t conditions[RELAY_RULE_MAX_CONDITIONS] = {0};
    relay_rule_engine_get_condition_status(conditions, 0U);
    assert(conditions[0].configured);
    assert(!conditions[0].valid);
    assert(!conditions[0].established);

    ingest_rpm(1000U, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    assert_output(&command, 1U, true, true, false, 0U);
    relay_rule_engine_get_condition_status(conditions, 100U);
    assert(conditions[0].valid);
    assert(conditions[0].value);
    assert(conditions[0].established);
    assert(conditions[0].source_current);

    /* Source disappearance deliberately maps to false only after a valid sample. */
    relay_rule_engine_make_command(1100U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
    assert_output(&command, 1U, true, false, false, 0U);
    relay_rule_engine_get_condition_status(conditions, 1100U);
    assert(conditions[0].valid);
    assert(!conditions[0].value);
    assert(conditions[0].established);
    assert(!conditions[0].source_current);
    assert(conditions[0].invalid_reason == RELAY_RULE_INVALID_SOURCE_STALE);
}

static void test_condition_stale_behaviors(void)
{
    for (unsigned behavior = RELAY_CONDITION_STALE_INVALID;
         behavior <= RELAY_CONDITION_STALE_HOLD_LAST; ++behavior) {
        relay_rule_config_t config = fresh_config();
        config.signal_timeout_ms = 1000U;
        configure_can_rpm(&config, 20U);
        configure_condition(&config, 0U, "Engine running", 20U, RELAY_COMPARE_GE, 600.0f,
                            0.0f, (relay_condition_stale_behavior_t)behavior);
        config.rules[0].enabled = true;
        config.rules[0].case_count = 1U;
        config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
        config.rules[0].cases[0].tests[0] = condition_test(0U, true);
        install_config(&config);

        ingest_rpm(1000U, 10U);
        output_command_t command;
        relay_rule_engine_make_command(10U, &command);
        assert_output(&command, 0U, true, true, false, 0U);
        relay_rule_engine_make_command(1010U, &command);

        if (behavior == RELAY_CONDITION_STALE_INVALID) {
            assert_output(&command, 0U, false, false, false, 0U);
        } else if (behavior == RELAY_CONDITION_STALE_FALSE) {
            assert_output(&command, 0U, true, false, false, 0U);
        } else {
            assert_output(&command, 0U, true, true, false, 0U);
        }
    }
}

static void test_condition_trigger_timer_on_can_disappearance(void)
{
    relay_rule_config_t config = fresh_config();
    config.signal_timeout_ms = 1000U;
    configure_can_rpm(&config, 20U);
    configure_condition(&config, 0U, "Engine running", 20U, RELAY_COMPARE_GE, 600.0f,
                        0.0f, RELAY_CONDITION_STALE_FALSE);
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
    config.rules[0].cases[0].tests[0] =
        triggered_condition_timer_test(0U, false, 300000U);
    install_config(&config);

    output_command_t command;

    /* No valid RPM since boot: stale->false must not fabricate a shutdown. */
    relay_rule_engine_make_command(5000U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    /* Seeing Engine running=true makes the timer predicate false and arms it. */
    ingest_rpm(1000U, 6000U);
    relay_rule_engine_make_command(6000U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    /* No 200/0 RPM frame is needed: source timeout makes the established
     * Condition false, producing the false->true timer edge. */
    relay_rule_engine_make_command(7000U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    relay_rule_engine_make_command(306999U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    relay_rule_engine_make_command(307000U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    /* A first-ever valid sample that is already false also must not trigger. */
    assert(relay_rule_engine_replace_and_save(&config));
    ingest_rpm(0U, 400000U);
    relay_rule_engine_make_command(400000U, &command);
    assert_output(&command, 0U, true, false, false, 0U);
}

static void test_condition_hysteresis_and_validation(void)
{
    relay_rule_config_t config = fresh_config();
    configure_can_rpm(&config, 20U);
    configure_condition(&config, 0U, "Engine running", 20U, RELAY_COMPARE_GT, 600.0f,
                        50.0f, RELAY_CONDITION_STALE_INVALID);
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
    config.rules[0].cases[0].tests[0] = condition_test(0U, true);
    assert(relay_rule_engine_validate(&config, 25U));
    install_config(&config);

    output_command_t command;
    ingest_rpm(700U, 100U);
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    ingest_rpm(560U, 200U);
    relay_rule_engine_make_command(200U, &command);
    assert_output(&command, 0U, true, true, false, 0U);
    ingest_rpm(549U, 300U);
    relay_rule_engine_make_command(300U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    relay_rule_config_t bad = config;
    bad.conditions[1] = bad.conditions[0];
    assert(!relay_rule_engine_validate(&bad, 25U));
    bad = config;
    bad.conditions[0].source_index = RELAY_RULE_MAX_SOURCES;
    assert(!relay_rule_engine_validate(&bad, 25U));
    bad = config;
    bad.conditions[0].stale_behavior = (relay_condition_stale_behavior_t)99;
    assert(!relay_rule_engine_validate(&bad, 25U));
    bad = config;
    bad.rules[0].cases[0].tests[0] = condition_test(1U, true);
    assert(!relay_rule_engine_validate(&bad, 25U));
    bad = config;
    bad.rules[0].cases[0].tests[0] = triggered_condition_timer_test(1U, false, 1000U);
    assert(!relay_rule_engine_validate(&bad, 25U));
}

static void test_dbc_invalid_values_become_stale_without_replacing_last_value(void)
{
    relay_rule_config_t config = fresh_config();
    config.signal_timeout_ms = 1000U;
    relay_source_config_t *source = &config.sources[20U];
    strcpy(source->name, "OilTemp.OilTemperature");
    source->type = RELAY_SOURCE_CAN;
    source->can_id = 0x588U;
    source->start_bit = 56U;
    source->bit_length = 8U;
    source->little_endian = true;
    source->factor = 1.0f;
    source->offset = -60.0f;
    source->zero_confirm_samples = 1U;
    source->range_enabled = true;
    source->minimum = -10.0f;
    source->maximum = 195.0f;
    source->invalid_raw_enabled = true;
    source->invalid_raw = 1U;
    configure_condition(&config, 0U, "Oil hot", 20U, RELAY_COMPARE_GE, 100.0f,
                        0.0f, RELAY_CONDITION_STALE_HOLD_LAST);
    config.rules[0].enabled = true;
    config.rules[0].case_count = 1U;
    config.rules[0].cases[0] = always_case(RELAY_ACTION_ON);
    config.rules[0].cases[0].tests[0] = condition_test(0U, true);
    install_config(&config);

    twai_message_t message = {
        .identifier = 0x588U,
        .data_length_code = 8U,
        .data = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 180U},
    };
    output_command_t command;
    relay_rule_engine_ingest_can(&message, 100U); /* 120 C: accepted and hot. */
    relay_rule_engine_make_command(100U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    message.data[7] = 1U; /* -59 C: explicit DBC invalid raw value. */
    relay_rule_engine_ingest_can(&message, 500U);
    relay_rule_engine_make_command(500U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    message.data[7] = 49U; /* -11 C: outside the DBC physical range. */
    relay_rule_engine_ingest_can(&message, 900U);
    relay_rule_engine_make_command(1100U, &command);
    assert_output(&command, 0U, true, true, false, 0U);

    relay_rule_status_t outputs[RELAY_RULE_MAX_RULES];
    relay_rule_source_status_t sources[RELAY_RULE_MAX_SOURCES];
    relay_rule_engine_get_status(outputs, sources, 1100U);
    assert(sources[20U].accepted_valid);
    assert(sources[20U].value == 120.0f);
    assert(sources[20U].age_ms == 1000U);

    relay_condition_status_t conditions[RELAY_RULE_MAX_CONDITIONS];
    relay_rule_engine_get_condition_status(conditions, 1100U);
    assert(conditions[0].valid);
    assert(conditions[0].value);
    assert(!conditions[0].source_current);
    assert(conditions[0].invalid_reason == RELAY_RULE_INVALID_SOURCE_STALE);

    message.data[7] = 50U; /* -10 C: inclusive lower boundary is valid. */
    relay_rule_engine_ingest_can(&message, 1200U);
    relay_rule_engine_make_command(1200U, &command);
    assert_output(&command, 0U, true, false, false, 0U);

    relay_rule_config_t bad = config;
    bad.sources[20U].minimum = 20.0f;
    bad.sources[20U].maximum = 10.0f;
    assert(!relay_rule_engine_validate(&bad, 25U));
    bad = config;
    bad.sources[20U].invalid_raw = 256U;
    assert(!relay_rule_engine_validate(&bad, 25U));
}

static void test_maximum_compact_config_fits_existing_partition(void)
{
    relay_rule_config_t config = fresh_config();
    for (unsigned i = 20U; i < RELAY_RULE_MAX_SOURCES; ++i) {
        relay_source_config_t *source = &config.sources[i];
        snprintf(source->name, sizeof(source->name), "CAN%u", i);
        source->type = RELAY_SOURCE_CAN;
        source->can_id = 0x100U + i;
        source->start_bit = 0U;
        source->bit_length = 8U;
        source->little_endian = true;
        source->factor = 1.0f;
        source->zero_confirm_samples = 1U;
    }
    for (unsigned i = 0U; i < RELAY_RULE_MAX_CONDITIONS; ++i) {
        char label[RELAY_RULE_NAME_LENGTH];
        snprintf(label, sizeof(label), "Condition %u", i + 1U);
        configure_condition(&config, (uint8_t)i, label, (uint8_t)(20U + (i % 44U)),
                            RELAY_COMPARE_GT, (float)i, 0.0f,
                            RELAY_CONDITION_STALE_INVALID);
    }
    for (unsigned i = 0U; i < RELAY_RULE_MAX_RULES; ++i) {
        config.rules[i].enabled = true;
        config.rules[i].case_count = 1U;
        config.rules[i].cases[0] = always_case(RELAY_ACTION_OFF);
    }
    assert(relay_rule_engine_validate(&config, 25U));
    assert(relay_rule_engine_replace_and_save(&config));

    const size_t slot_size = test_partition_size() / TEST_SLOT_COUNT;
    const uint8_t *storage = test_partition_data();
    const test_slot_header_t *slot0 = (const test_slot_header_t *)storage;
    const test_slot_header_t *slot1 = (const test_slot_header_t *)(storage + slot_size);
    const unsigned newest = (int32_t)(slot1->generation - slot0->generation) > 0 ? 1U : 0U;
    const uint8_t *slot_base = storage + newest * slot_size;
    const test_compact_header_t *payload =
        (const test_compact_header_t *)(slot_base + sizeof(test_slot_header_t));
    assert(payload->version == TEST_NEW_COMPACT_VERSION);
    assert(payload->source_count == 44U);
    assert(payload->rule_count == RELAY_RULE_MAX_RULES);
    assert(payload->reserved == RELAY_RULE_MAX_CONDITIONS);
}

static void test_counter_wrap(void)
{
    relay_rule_config_t config = fresh_config();
    install_config(&config);

    output_command_t first;
    relay_rule_engine_make_command(0U, &first);
    output_command_t command = {0};
    for (unsigned i = 0; i < 256U; ++i)
        relay_rule_engine_make_command(i + 1U, &command);
    assert(command.counter == first.counter);
}

int main(void)
{
    assert(RELAY_RULE_MAX_RULES == 8U);
    assert_v4_record_migrates_in_memory();
    assert_legacy_record_rejected_and_replaced();
    test_ordered_off_on_and_timer();
    test_all_eight_binary_outputs();
    test_source_hysteresis();
    test_all_comparisons_and_and_logic();
    test_pulse_regression();
    test_pulse_lookup_interpolation();
    test_pulse_continuous_on_off_and_validation();
    test_pwm_interpolation_clamp_hysteresis_and_state();
    test_pwm_from_imported_can_source();
    test_pwm_rounding();
    test_pwm_timeout_and_status();
    test_pwm_zero_confirmation();
    test_pwm_case_change_recalculates_immediately();
    test_live_config_replacement_resets_pwm_runtime();
    test_pwm_validation();
    test_inline_triggered_timer_post_shutdown();
    test_inline_triggered_timer_updates_behind_higher_priority_case();
    test_inline_triggered_timer_hysteresis_and_reset();
    test_inline_triggered_timer_validation();
    test_conditions_reused_and_stale_false();
    test_condition_stale_behaviors();
    test_condition_trigger_timer_on_can_disappearance();
    test_condition_hysteresis_and_validation();
    test_dbc_invalid_values_become_stale_without_replacing_last_value();
    test_maximum_compact_config_fits_existing_partition();
    test_counter_wrap();
    puts("output_engine_test: PASS");
    return 0;
}
