#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai_types_legacy.h"
#include "relay_command_protocol.h"

#define RELAY_RULE_MAX_RULES OUTPUT_COMMAND_COUNT
#define RELAY_RULE_MAX_SOURCES 64U
#define RELAY_RULE_MAX_CONDITIONS 16U
#define RELAY_RULE_MAX_CASES 4U
#define RELAY_RULE_MAX_TESTS 4U
#define RELAY_RULE_MAX_PULSE_POINTS 8U
#define RELAY_RULE_MAX_PWM_POINTS 8U
#define RELAY_RULE_NAME_LENGTH 32U

/* source_index remains an 8-bit field in persisted Output tests. Raw source
 * references use 0..63. Triggered timers set this flag to reference a global
 * Condition instead, preserving the existing compact Output record layout. */
#define RELAY_RULE_TRIGGER_CONDITION_FLAG 0x80U
#define RELAY_RULE_TRIGGER_INDEX_MASK 0x7FU

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
    RELAY_ACTION_PWM,
} relay_action_t;

typedef enum {
    RELAY_TEST_SOURCE = 0,
    RELAY_TEST_UPTIME,
    RELAY_TEST_TRIGGER_TIMER,
    RELAY_TEST_CONDITION,
} relay_test_type_t;

typedef enum {
    RELAY_CONDITION_STALE_INVALID = 0,
    RELAY_CONDITION_STALE_FALSE,
    RELAY_CONDITION_STALE_TRUE,
    RELAY_CONDITION_STALE_HOLD_LAST,
} relay_condition_stale_behavior_t;

typedef struct {
    float input_value;
    uint32_t on_time_ms;
    uint32_t period_ms;
} relay_pulse_point_t;

typedef struct {
    float input_value;
    uint8_t duty_percent;
} relay_pwm_point_t;

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
    bool range_enabled;
    float minimum;
    float maximum;
    bool invalid_raw_enabled;
    uint64_t invalid_raw;
} relay_source_config_t;

typedef struct {
    char label[RELAY_RULE_NAME_LENGTH];
    uint8_t source_index;
    relay_compare_t comparison;
    bool hysteresis_enabled;
    float threshold;
    float hysteresis;
    relay_condition_stale_behavior_t stale_behavior;
} relay_condition_config_t;

typedef struct {
    relay_test_type_t type;
    /* RELAY_TEST_SOURCE: source index. RELAY_TEST_CONDITION: Condition index.
     * RELAY_TEST_TRIGGER_TIMER: raw source index, or CONDITION_FLAG | index. */
    uint8_t source_index;
    relay_compare_t comparison;
    bool hysteresis_enabled;
    float threshold;
    float hysteresis;
    uint32_t trigger_duration_ms;
} relay_rule_test_t;

typedef struct {
    uint8_t test_count;
    relay_rule_test_t tests[RELAY_RULE_MAX_TESTS];
    relay_action_t action;
    uint8_t pulse_source_index;
    uint8_t pulse_point_count;
    relay_pulse_point_t pulse_points[RELAY_RULE_MAX_PULSE_POINTS];
    float pulse_hysteresis;
    uint8_t pwm_source_index;
    uint8_t pwm_point_count;
    relay_pwm_point_t pwm_points[RELAY_RULE_MAX_PWM_POINTS];
    float pwm_hysteresis;
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
    relay_condition_config_t conditions[RELAY_RULE_MAX_CONDITIONS];
    relay_output_rule_t rules[RELAY_RULE_MAX_RULES];
    uint32_t crc32;
} relay_rule_config_t;

typedef enum {
    RELAY_RULE_INVALID_NONE = 0,
    RELAY_RULE_INVALID_SOURCE_NOT_RECEIVED,
    RELAY_RULE_INVALID_SOURCE_UNCONFIRMED,
    RELAY_RULE_INVALID_SOURCE_STALE,
} relay_rule_invalid_reason_t;

typedef struct {
    char name[RELAY_RULE_NAME_LENGTH];
    bool present;
    bool accepted_valid;
    float value;
    uint32_t age_ms;
    uint8_t zero_streak;
    uint8_t zero_confirm_samples;
} relay_rule_source_status_t;

typedef struct {
    char label[RELAY_RULE_NAME_LENGTH];
    bool configured;
    bool valid;
    bool value;
    bool established;
    bool source_current;
    relay_rule_invalid_reason_t invalid_reason;
    uint32_t source_age_ms;
} relay_condition_status_t;

typedef struct {
    bool configured;
    bool enabled;
    bool valid;
    bool state;
    bool pulse_active;
    bool pwm_active;
    uint8_t duty_percent;
    int8_t selected_case;
    uint32_t pulse_on_time_ms;
    uint32_t pulse_period_ms;
    uint32_t pulse_next_on_ms;
    int8_t invalid_case;
    int8_t invalid_test;
    int8_t invalid_source;
    int8_t invalid_condition;
    bool invalid_pulse_source;
    bool invalid_pwm_source;
    relay_rule_invalid_reason_t invalid_reason;
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
void relay_rule_engine_make_command(uint32_t now_ms, output_command_t *command);
void relay_rule_engine_get_status(relay_rule_status_t rules[RELAY_RULE_MAX_RULES],
                                  relay_rule_source_status_t sources[RELAY_RULE_MAX_SOURCES],
                                  uint32_t now_ms);
void relay_rule_engine_get_condition_status(
    relay_condition_status_t conditions[RELAY_RULE_MAX_CONDITIONS], uint32_t now_ms);
bool relay_rule_engine_has_external_sources(void);
