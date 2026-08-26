#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai_types_legacy.h"
#include "relay_command_protocol.h"

#define RELAY_RULE_MAX_RULES 16U
#define RELAY_RULE_MAX_SOURCES 64U
#define RELAY_RULE_MAX_CASES 4U
#define RELAY_RULE_MAX_TESTS 4U
#define RELAY_RULE_MAX_PULSE_POINTS 8U
#define RELAY_RULE_NAME_LENGTH 32U

typedef enum {
    RELAY_SOURCE_UNUSED = 0,
    RELAY_SOURCE_CAN,
    RELAY_SOURCE_LOCAL_VOLTAGE,
    RELAY_SOURCE_LOCAL_VALUE,
} relay_source_type_t;

typedef enum {
    RELAY_COMPARE_GT = 0,
    RELAY_COMPARE_GE,
    RELAY_COMPARE_LT,
    RELAY_COMPARE_LE,
    RELAY_COMPARE_EQ,
    RELAY_COMPARE_NE,
} relay_compare_t;

typedef enum {
    RELAY_ACTION_OFF = 0,
    RELAY_ACTION_ON,
    RELAY_ACTION_PULSE,
} relay_action_t;

typedef enum {
    RELAY_TEST_SOURCE = 0,
    RELAY_TEST_UPTIME,
} relay_test_type_t;

typedef struct {
    float input_value;
    uint32_t on_time_ms;
    uint32_t period_ms;
} relay_pulse_point_t;

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
} relay_source_config_t;

typedef struct {
    relay_test_type_t type;
    uint8_t source_index;
    relay_compare_t comparison;
    bool hysteresis_enabled;
    float threshold;
    float hysteresis;
} relay_rule_test_t;

typedef struct {
    uint8_t test_count;
    relay_rule_test_t tests[RELAY_RULE_MAX_TESTS];
    relay_action_t action;
    uint8_t pulse_source_index;
    uint8_t pulse_point_count;
    relay_pulse_point_t pulse_points[RELAY_RULE_MAX_PULSE_POINTS];
} relay_rule_case_t;

typedef struct {
    char label[RELAY_RULE_NAME_LENGTH];
    bool enabled;
    uint8_t case_count;
    relay_rule_case_t cases[RELAY_RULE_MAX_CASES];
} relay_output_rule_t;

typedef struct {
    uint32_t version;
    uint32_t signal_timeout_ms;
    relay_source_config_t sources[RELAY_RULE_MAX_SOURCES];
    relay_output_rule_t rules[RELAY_RULE_MAX_RULES];
    uint32_t crc32;
} relay_rule_config_t;

typedef struct {
    bool present;
    bool accepted_valid;
    float value;
    uint32_t age_ms;
    uint8_t zero_streak;
} relay_rule_source_status_t;

typedef struct {
    bool enabled;
    bool valid;
    bool state;
    bool pulse_active;
    int8_t selected_case;
} relay_rule_status_t;

void relay_rule_engine_init(void);
/** Set the current sensor/CAN publish cadence used to validate pulse intervals. */
void relay_rule_engine_set_publish_rate(uint8_t can_tx_hz);
void relay_rule_engine_set_defaults(relay_rule_config_t *config);
bool relay_rule_engine_replace_and_save(const relay_rule_config_t *config);
void relay_rule_engine_snapshot(relay_rule_config_t *config);
bool relay_rule_engine_validate(const relay_rule_config_t *config, uint8_t can_tx_hz);
void relay_rule_engine_ingest_can(const twai_message_t *message, uint32_t now_ms);
void relay_rule_engine_ingest_local(uint8_t channel, bool converted, float value,
                                    uint32_t now_ms);
void relay_rule_engine_make_command(uint32_t now_ms, relay_command_t *command);
void relay_rule_engine_get_status(relay_rule_status_t rules[RELAY_RULE_MAX_RULES],
                                  relay_rule_source_status_t sources[RELAY_RULE_MAX_SOURCES],
                                  uint32_t now_ms);
bool relay_rule_engine_has_external_sources(void);
