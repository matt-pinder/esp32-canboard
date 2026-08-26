#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_now.h"
#include "mk60_emulator_protocol.h"

#define CONFIG_CHANNELS 10       ///< Number of ADC input channels
#define CONFIG_NAME_LEN 32      ///< Maximum characters for channel name
#define CONFIG_VERSION 12       ///< Configuration structure version number (increment for new fields or layout changes)
#define ESPNOW_MAX_CLIENTS 4    ///< Maximum number of unencrypted ESP-NOW destinations

/// Per-channel median filter strength levels
/// stored in the 8‑bit `filtering` field below.
/// 0 = none, 1 = low (3‑sample), 2 = med (5‑sample), 3 = high (7‑sample)
typedef enum {
    FILTER_NONE = 0,
    FILTER_LOW  = 1,
    FILTER_MED  = 2,
    FILTER_HIGH = 3
} filter_level_t;

typedef enum {
    EMUB_TX_DISABLED = 0,
    EMUB_TX_CAN_ANALOG_9 = 1,
    EMUB_TX_CAN_ANALOG_10 = 2,
    EMUB_TX_CAN_ANALOG_11 = 3,
    EMUB_TX_CAN_ANALOG_12 = 4,
    EMUB_TX_CAN_ANALOG_13 = 5,
    EMUB_TX_CAN_ANALOG_14 = 6,
    EMUB_TX_CAN_ANALOG_15 = 7,
    EMUB_TX_CAN_ANALOG_16 = 8
} emub_tx_t;

/// Sensor measurement types for ADC channels
typedef enum {
    SENSOR_RAW = 0,
    SENSOR_NTC = 1,
    SENSOR_PRESSURE = 2
} sensor_type_t;

/// Per-channel configuration structure
typedef struct {
    char name[CONFIG_NAME_LEN]; ///< ASCII name/description of the channel
    uint32_t pullup_ohms;       ///< Optional pullup resistor value in ohms (0 if not present)
    sensor_type_t type;         ///< Sensor type (RAW, NTC, PRESSURE)
    uint8_t filtering;          ///< Median filter level (see filter_level_t enum; 0=none)
    uint8_t emub_tx;            ///< Optional EMUB TX mapping 0=Disabled, 1..8 => CAN Analog 9..16
    union {
        struct { ///< NTC thermistor configuration
            uint8_t table_id;   ///< Reference table index for NTC lookup
        } ntc;
        struct { ///< Pressure sensor configuration
            uint16_t min_mv;    ///< Minimum voltage in millivolts
            uint16_t max_mv;    ///< Maximum voltage in millivolts
            float min_kpa;      ///< Minimum pressure in kilopascals
            float max_kpa;      ///< Maximum pressure in kilopascals
        } pressure;
        struct { ///< Raw voltage measurement (no conversion)
        } raw;
    } params;
} channel_config_t;

typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN]; ///< Destination STA MAC address
    bool relay_can;                ///< Include frames received from the physical CAN bus
} espnow_client_config_t;

/// Runtime board configuration used by the firmware.
/// `pullup_vref_mv` is a live ADC-derived measurement and is not persisted.
typedef struct {
    uint32_t version;                      ///< Config version for migration/compatibility
    uint32_t can_start_id;                 ///< CAN message ID base (incremented for each message)
    uint32_t can_speed_kbps;               ///< CAN bus speed (125, 250, 500, or 1000 kbps)
    uint8_t can_tx_hz;                     ///< CAN transmit loop rate in Hz (supported: 25 or 50)
    bool can_enabled;                      ///< Enable physical CAN/TWAI transmission
    bool espnow_enabled;                   ///< Enable ESP-NOW transmission of TWAI messages
    uint8_t espnow_client_count;           ///< Number of configured entries in espnow_clients
    espnow_client_config_t espnow_clients[ESPNOW_MAX_CLIENTS]; ///< ESP-NOW destinations and per-client relay policy
    bool gps_enabled;                      ///< Enable external BLE GPS integration
    uint32_t gps_can_start_id;             ///< First CAN ID used by Dragy GPS/IMU frames; following frames increment from this
    uint8_t gps_target_mac[ESP_NOW_ETH_ALEN]; ///< Optional BLE GPS target MAC address
    uint16_t pullup_vref_mv;               ///< Live pull-up reference voltage in millivolts (runtime read-only display)
    uint16_t pullup_vref_divider_high_ohm; ///< Top resistor value of the pull-up Vref divider (persisted)
    mk60_emulator_config_t mk60_emulator;  ///< Opt-in capture-derived MK60 RTR response profile
    channel_config_t channels[CONFIG_CHANNELS]; ///< Per-channel configuration array
    uint32_t crc32;                        ///< CRC32 checksum of all fields above for integrity verification
} board_config_t;

/// @brief Persist board configuration to the dedicated NVS partition
/// @param cfg Pointer to configuration structure to save
/// @return true only after commit and read-back verification succeed
bool config_save(const board_config_t *cfg);

/// @brief Load configuration from NVS, importing legacy SPIFFS once if needed
/// @param cfg Pointer to configuration structure to populate
/// @return true if a valid NVS or legacy configuration was loaded
bool config_load(board_config_t *cfg);

/// @brief Initialize configuration structure with factory defaults
/// @param cfg Pointer to configuration structure to initialize
void config_set_defaults(board_config_t *cfg);

/// @brief Return true when at least one active ESP-NOW client accepts relayed CAN frames.
bool config_has_espnow_relay_client(const board_config_t *cfg);

#endif // CONFIG_H
