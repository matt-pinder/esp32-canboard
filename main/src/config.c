#include "inc/config.h"
#include <stddef.h> 
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "inc/relay_command_protocol.h"

#define CONFIG_NVS_PARTITION "config"
#define CONFIG_NVS_NAMESPACE "board_config"
#define CONFIG_NVS_KEY "active"
#define CONFIG_RECORD_MAGIC 0x31474643U
#define CONFIG_RECORD_SCHEMA_VERSION 1U
#define CONFIG_RECORD_HEADER_SIZE 16U
#define TAG "CONFIG"

typedef struct {
    uint32_t version;
    uint32_t can_start_id;
    uint32_t can_speed_kbps;
    uint8_t can_tx_hz;
    bool can_enabled;
    bool espnow_enabled;
    uint16_t pullup_vref_divider_high_ohm;
    channel_config_t channels[CONFIG_CHANNELS];
    bool gps_enabled;
    uint32_t gps_can_start_id;
    uint8_t gps_target_mac[ESP_NOW_ETH_ALEN];
    uint8_t espnow_client_count;
    espnow_client_config_t espnow_clients[ESPNOW_MAX_CLIENTS];
    mk60_emulator_config_t mk60_emulator;
    uint8_t gps_update_rate_hz;
    gps_source_t gps_source;
    uint8_t gps_espnow_peer_mac[ESP_NOW_ETH_ALEN];
    uint32_t gps_response_timeout_ms;
    uint32_t crc32;
} board_config_persist_t;

typedef enum {
    CONFIG_READ_OK,
    CONFIG_READ_NOT_FOUND,
    CONFIG_READ_INVALID,
    CONFIG_READ_ERROR,
} config_read_result_t;

static bool nvs_partition_initialized;

static void log_board_config(const board_config_t *cfg) {
    if (cfg == NULL) {
        return;
    }

    ESP_LOGI(TAG, "Loaded board_config_t:");
    ESP_LOGI(TAG, "  version=%lu can_enabled=%d can_start_id=0x%lX can_speed_kbps=%lu can_tx_hz=%u",
             (unsigned long)cfg->version,
             cfg->can_enabled,
             (unsigned long)cfg->can_start_id,
             (unsigned long)cfg->can_speed_kbps,
             (unsigned)cfg->can_tx_hz);
    ESP_LOGI(TAG, "  espnow_enabled=%d espnow_clients=%u relay_clients=%d",
             cfg->espnow_enabled,
             (unsigned)cfg->espnow_client_count,
             config_has_espnow_relay_client(cfg));
    for (uint8_t i = 0; i < cfg->espnow_client_count; ++i) {
        const espnow_client_config_t *client = &cfg->espnow_clients[i];
        ESP_LOGI(TAG, "    client[%u]=%02X:%02X:%02X:%02X:%02X:%02X label=\"%s\" relay_can=%d",
                 (unsigned)i,
                 client->mac[0], client->mac[1], client->mac[2],
                 client->mac[3], client->mac[4], client->mac[5],
                 client->label,
                 client->relay_can);
    }
    ESP_LOGI(TAG, "  pullup_vref_divider_high_ohm=%u crc32=0x%08lX",
             (unsigned)cfg->pullup_vref_divider_high_ohm,
             (unsigned long)cfg->crc32);
    ESP_LOGI(TAG, "  gps_enabled=%d gps_rate=%uHz gps_can_start_id=0x%lX gps_target_mac=%02X:%02X:%02X:%02X:%02X:%02X",
             cfg->gps_enabled,
             (unsigned)cfg->gps_update_rate_hz,
             (unsigned long)cfg->gps_can_start_id,
             cfg->gps_target_mac[0], cfg->gps_target_mac[1], cfg->gps_target_mac[2],
             cfg->gps_target_mac[3], cfg->gps_target_mac[4], cfg->gps_target_mac[5]);
    ESP_LOGI(TAG, "  gps_source=%u response_peer=%02X:%02X:%02X:%02X:%02X:%02X response_timeout_ms=%lu",
             (unsigned)cfg->gps_source,
             cfg->gps_espnow_peer_mac[0], cfg->gps_espnow_peer_mac[1],
             cfg->gps_espnow_peer_mac[2], cfg->gps_espnow_peer_mac[3],
             cfg->gps_espnow_peer_mac[4], cfg->gps_espnow_peer_mac[5],
             (unsigned long)cfg->gps_response_timeout_ms);
    ESP_LOGI(TAG, "  mk60_emulator_enabled=%d trigger=0x%lX dlc=%u responses=%u",
             cfg->mk60_emulator.enabled,
             (unsigned long)cfg->mk60_emulator.trigger_id,
             (unsigned)cfg->mk60_emulator.trigger_dlc,
             (unsigned)cfg->mk60_emulator.response_count);

    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        const channel_config_t *ch = &cfg->channels[i];
        ESP_LOGI(TAG, "  channels[%d]: name=\"%s\" pullup_ohms=%lu type=%u filtering=%u emub_tx=%u",
                 i,
                 ch->name,
                 (unsigned long)ch->pullup_ohms,
                 (unsigned)ch->type,
                 (unsigned)ch->filtering,
                 (unsigned)ch->emub_tx);

        if (ch->type == SENSOR_NTC) {
            ESP_LOGI(TAG, "    params.ntc.table_id=%u", (unsigned)ch->params.ntc.table_id);
        } else if (ch->type == SENSOR_PRESSURE) {
            ESP_LOGI(TAG, "    params.pressure.min_mv=%u max_mv=%u min_kpa=%.2f max_kpa=%.2f",
                     (unsigned)ch->params.pressure.min_mv,
                     (unsigned)ch->params.pressure.max_mv,
                     ch->params.pressure.min_kpa,
                     ch->params.pressure.max_kpa);
        } else {
            ESP_LOGI(TAG, "    params.raw={}");
        }
    }
}

/**
 * @brief Computes CRC32 checksum of data buffer.
 *
 * Uses polynomial 0xEDB88320 for CRC32 calculation. Processes each byte with
 * bit-by-bit XOR operations. Used to verify integrity of configuration data
 * stored in persistent configuration records.
 *
 * @param data Pointer to data buffer
 * @param len Number of bytes to checksum
 *
 * @return CRC32 value (inverted final result)
 *
 * @note This is a software implementation, not hardware-accelerated.
 *       Initial CRC value is 0xFFFFFFFF, final result is inverted.
 *       Complexity: O(8n) where n is buffer length in bytes.
 */
static uint32_t crc32(const void *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *buf = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

static void persist_from_runtime(const board_config_t *cfg, board_config_persist_t *persisted) {
    memset(persisted, 0, sizeof(*persisted));
    persisted->version = cfg->version;
    persisted->can_start_id = cfg->can_start_id;
    persisted->can_speed_kbps = cfg->can_speed_kbps;
    persisted->can_tx_hz = cfg->can_tx_hz;
    persisted->can_enabled = cfg->can_enabled;
    persisted->espnow_enabled = cfg->espnow_enabled;
    persisted->pullup_vref_divider_high_ohm = cfg->pullup_vref_divider_high_ohm;
    memcpy(persisted->channels, cfg->channels, sizeof(persisted->channels));
    persisted->gps_enabled = cfg->gps_enabled;
    persisted->gps_can_start_id = cfg->gps_can_start_id;
    memcpy(persisted->gps_target_mac, cfg->gps_target_mac, sizeof(persisted->gps_target_mac));
    persisted->espnow_client_count = cfg->espnow_client_count;
    memcpy(persisted->espnow_clients, cfg->espnow_clients, sizeof(persisted->espnow_clients));
    persisted->mk60_emulator = cfg->mk60_emulator;
    persisted->gps_update_rate_hz = cfg->gps_update_rate_hz;
    persisted->gps_source = cfg->gps_source;
    memcpy(persisted->gps_espnow_peer_mac, cfg->gps_espnow_peer_mac,
           sizeof(persisted->gps_espnow_peer_mac));
    persisted->gps_response_timeout_ms = cfg->gps_response_timeout_ms;
    persisted->crc32 = crc32(persisted, offsetof(board_config_persist_t, crc32));
}

static void runtime_from_persist(const board_config_persist_t *persisted, board_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->version = persisted->version;
    cfg->can_start_id = persisted->can_start_id;
    cfg->can_speed_kbps = persisted->can_speed_kbps;
    cfg->can_tx_hz = persisted->can_tx_hz;
    cfg->can_enabled = persisted->can_enabled;
    cfg->espnow_enabled = persisted->espnow_enabled;
    cfg->pullup_vref_divider_high_ohm = persisted->pullup_vref_divider_high_ohm;
    memcpy(cfg->channels, persisted->channels, sizeof(cfg->channels));
    cfg->gps_enabled = persisted->gps_enabled;
    cfg->gps_can_start_id = persisted->gps_can_start_id;
    memcpy(cfg->gps_target_mac, persisted->gps_target_mac, sizeof(cfg->gps_target_mac));
    cfg->espnow_client_count = persisted->espnow_client_count;
    memcpy(cfg->espnow_clients, persisted->espnow_clients, sizeof(cfg->espnow_clients));
    cfg->mk60_emulator = persisted->mk60_emulator;
    cfg->gps_update_rate_hz = persisted->gps_update_rate_hz;
    cfg->gps_source = persisted->gps_source;
    memcpy(cfg->gps_espnow_peer_mac, persisted->gps_espnow_peer_mac,
           sizeof(cfg->gps_espnow_peer_mac));
    cfg->gps_response_timeout_ms = persisted->gps_response_timeout_ms;
    cfg->pullup_vref_mv = 5025;
    cfg->crc32 = persisted->crc32;
}

bool config_has_espnow_relay_client(const board_config_t *cfg) {
    if (cfg == NULL || !cfg->espnow_enabled) return false;
    for (uint8_t i = 0; i < cfg->espnow_client_count && i < ESPNOW_MAX_CLIENTS; ++i) {
        if (cfg->espnow_clients[i].relay_can) return true;
    }
    return false;
}

static bool config_semantically_valid(const board_config_t *cfg) {
    if (cfg->version != CONFIG_VERSION || !output_command_base_can_id_valid(cfg->can_start_id) ||
        cfg->gps_can_start_id > 0x7FA ||
        (cfg->can_speed_kbps != 125 && cfg->can_speed_kbps != 250 &&
         cfg->can_speed_kbps != 500 && cfg->can_speed_kbps != 1000) ||
        (cfg->can_tx_hz != 25 && cfg->can_tx_hz != 50) ||
        (cfg->gps_update_rate_hz != 10 && cfg->gps_update_rate_hz != 20 &&
         cfg->gps_update_rate_hz != 25) ||
        cfg->gps_source > GPS_SOURCE_ESPNOW_RESPONSE ||
        cfg->gps_response_timeout_ms < 100U || cfg->gps_response_timeout_ms > 60000U ||
        cfg->espnow_client_count > ESPNOW_MAX_CLIENTS ||
        (cfg->espnow_enabled && cfg->espnow_client_count == 0) ||
        (cfg->mk60_emulator.enabled && cfg->can_speed_kbps != 500) ||
        !mk60_emulator_profile_valid(&cfg->mk60_emulator)) {
        return false;
    }

    const uint8_t zero_mac[ESP_NOW_ETH_ALEN] = {0};
    bool gps_peer_configured = false;
    for (uint8_t i = 0; i < cfg->espnow_client_count; ++i) {
        if (memcmp(cfg->espnow_clients[i].mac, zero_mac, sizeof(zero_mac)) == 0 ||
            memchr(cfg->espnow_clients[i].label, '\0', ESPNOW_CLIENT_LABEL_LEN) == NULL) {
            return false;
        }
        for (uint8_t j = 0; j < i; ++j) {
            if (memcmp(cfg->espnow_clients[i].mac, cfg->espnow_clients[j].mac, ESP_NOW_ETH_ALEN) == 0) {
                return false;
            }
        }
        if (memcmp(cfg->espnow_clients[i].mac, cfg->gps_espnow_peer_mac,
                   ESP_NOW_ETH_ALEN) == 0) {
            gps_peer_configured = true;
        }
    }
    if (cfg->gps_enabled && cfg->gps_source == GPS_SOURCE_ESPNOW_RESPONSE &&
        (!cfg->espnow_enabled ||
         memcmp(cfg->gps_espnow_peer_mac, zero_mac, sizeof(zero_mac)) == 0 ||
         !gps_peer_configured)) {
        return false;
    }

    bool emub_seen[EMUB_TX_CAN_ANALOG_16 + 1] = {false};
    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        const channel_config_t *channel = &cfg->channels[i];
        if (memchr(channel->name, '\0', CONFIG_NAME_LEN) == NULL || channel->type > SENSOR_PRESSURE ||
            channel->filtering > FILTER_HIGH || channel->emub_tx > EMUB_TX_CAN_ANALOG_16) {
            return false;
        }
        if (channel->emub_tx != EMUB_TX_DISABLED) {
            if (emub_seen[channel->emub_tx]) {
                return false;
            }
            emub_seen[channel->emub_tx] = true;
        }
    }
    return true;
}

static esp_err_t config_nvs_init(void) {
    if (nvs_partition_initialized) return ESP_OK;

    esp_err_t err = nvs_flash_init_partition(CONFIG_NVS_PARTITION);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(TAG, "Config NVS requires recovery (%s); preserving it instead of erasing",
                 esp_err_to_name(err));
        return err;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize config NVS: %s", esp_err_to_name(err));
        return err;
    }
    nvs_partition_initialized = true;
    return ESP_OK;
}

static void record_write_u16(uint8_t *dst, uint16_t value) { memcpy(dst, &value, sizeof(value)); }
static void record_write_u32(uint8_t *dst, uint32_t value) { memcpy(dst, &value, sizeof(value)); }
static uint16_t record_read_u16(const uint8_t *src) { uint16_t value; memcpy(&value, src, sizeof(value)); return value; }
static uint32_t record_read_u32(const uint8_t *src) { uint32_t value; memcpy(&value, src, sizeof(value)); return value; }

static size_t config_record_size(void) {
    return CONFIG_RECORD_HEADER_SIZE + sizeof(board_config_persist_t);
}

static bool encode_record(const board_config_t *cfg, uint8_t *record, size_t record_size) {
    if (!config_semantically_valid(cfg) || record_size != config_record_size()) return false;

    board_config_persist_t *persisted = malloc(sizeof(*persisted));
    if (persisted == NULL) return false;

    persist_from_runtime(cfg, persisted);
    memset(record, 0, record_size);
    // Header offsets are explicit so its on-flash layout never depends on compiler padding.
    record_write_u32(record + 0, CONFIG_RECORD_MAGIC);
    record_write_u16(record + 4, CONFIG_RECORD_SCHEMA_VERSION);
    record_write_u16(record + 6, CONFIG_RECORD_HEADER_SIZE);
    record_write_u32(record + 8, sizeof(*persisted));
    memcpy(record + CONFIG_RECORD_HEADER_SIZE, persisted, sizeof(*persisted));
    record_write_u32(record + 12, crc32(record + CONFIG_RECORD_HEADER_SIZE, sizeof(*persisted)));
    free(persisted);
    return true;
}

static bool decode_record(const uint8_t *record, size_t record_size, board_config_t *cfg) {
    if (record_size < CONFIG_RECORD_HEADER_SIZE ||
        record_read_u32(record + 0) != CONFIG_RECORD_MAGIC ||
        record_read_u16(record + 4) != CONFIG_RECORD_SCHEMA_VERSION ||
        record_read_u16(record + 6) != CONFIG_RECORD_HEADER_SIZE) {
        return false;
    }
    const size_t payload_size = record_read_u32(record + 8);
    if (payload_size != record_size - CONFIG_RECORD_HEADER_SIZE) return false;
    const uint8_t *payload = record + CONFIG_RECORD_HEADER_SIZE;
    if (record_read_u32(record + 12) != crc32(payload, payload_size)) return false;

    if (payload_size != sizeof(board_config_persist_t)) return false;

    board_config_persist_t *persisted = malloc(sizeof(*persisted));
    if (persisted == NULL) return false;
    memcpy(persisted, payload, sizeof(*persisted));

    const bool valid = persisted->version == CONFIG_VERSION &&
                       persisted->crc32 == crc32(persisted, offsetof(board_config_persist_t, crc32));
    if (valid) runtime_from_persist(persisted, cfg);
    free(persisted);
    return valid && config_semantically_valid(cfg);
}

static config_read_result_t load_nvs_record(const char *key, board_config_t *cfg) {
    esp_err_t err = config_nvs_init();
    if (err != ESP_OK) return CONFIG_READ_ERROR;

    nvs_handle_t handle;
    err = nvs_open_from_partition(CONFIG_NVS_PARTITION, CONFIG_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err == ESP_ERR_NVS_NOT_FOUND ? CONFIG_READ_NOT_FOUND : CONFIG_READ_ERROR;

    size_t size = 0;
    err = nvs_get_blob(handle, key, NULL, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return CONFIG_READ_NOT_FOUND;
    }
    if (err != ESP_OK || size != config_record_size()) {
        nvs_close(handle);
        return CONFIG_READ_INVALID;
    }

    uint8_t *record = malloc(size);
    if (record == NULL) {
        nvs_close(handle);
        return CONFIG_READ_ERROR;
    }
    err = nvs_get_blob(handle, key, record, &size);
    nvs_close(handle);
    bool valid = err == ESP_OK && decode_record(record, size, cfg);
    free(record);
    return valid ? CONFIG_READ_OK : CONFIG_READ_INVALID;
}

bool config_save(const board_config_t *cfg) {
    if (cfg == NULL || !config_semantically_valid(cfg)) {
        ESP_LOGE(TAG, "Refusing to save invalid config");
        return false;
    }
    if (config_nvs_init() != ESP_OK) return false;

    size_t record_size = config_record_size();
    uint8_t *record = malloc(record_size);
    uint8_t *readback = malloc(record_size);
    if (record == NULL || readback == NULL || !encode_record(cfg, record, record_size)) {
        free(record); free(readback);
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open_from_partition(CONFIG_NVS_PARTITION, CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        free(record); free(readback);
        return false;
    }

    err = nvs_set_blob(handle, CONFIG_NVS_KEY, record, record_size);
    if (err == ESP_OK) err = nvs_commit(handle);

    size_t readback_size = record_size;
    if (err == ESP_OK) err = nvs_get_blob(handle, CONFIG_NVS_KEY, readback, &readback_size);
    bool verified = err == ESP_OK && readback_size == record_size && memcmp(record, readback, record_size) == 0;
    nvs_close(handle);
    free(readback);
    if (!verified) {
        ESP_LOGE(TAG, "Config NVS commit/read-back failed: %s", esp_err_to_name(err));
        free(record);
        return false;
    }

    uint32_t payload_crc = record_read_u32(record + 12);
    free(record);
    ESP_LOGI(TAG, "Config committed to NVS and verified, payload CRC=0x%08lX", (unsigned long)payload_crc);
    return true;
}

bool config_load(board_config_t *cfg) {
    if (cfg == NULL) return false;

    config_read_result_t nvs_result = load_nvs_record(CONFIG_NVS_KEY, cfg);
    if (nvs_result == CONFIG_READ_OK) {
        ESP_LOGI(TAG, "Config loaded from dedicated NVS partition");
        log_board_config(cfg);
        return true;
    }

    if (nvs_result == CONFIG_READ_INVALID)
        ESP_LOGE(TAG, "Active NVS config record is invalid or uses an unsupported schema");

    return false;
}

/**
 * @brief Initializes board configuration with sensible defaults.
 *
 * Sets all configuration fields to default values:
 * - CAN start ID: 0x100
 * - CAN bus speed: 500 kbps
 * - All channels: RAW sensor type, no filtering, no pullup resistor
 * - Channel names: "Input 1", "Input 2", etc.
 *
 * Clears entire structure with memset before populating defaults.
 * Useful for first-time setup or factory reset.
 *
 * @param cfg Pointer to board_config_t structure to initialize
 *
 * @return void
 *         Logs ERROR if cfg is NULL, otherwise logs INFO on success
 *
 * @note Does not save to NVS; use config_save() to persist.
 *       All array fields (channels) are automatically initialized.
 *       CRC32 field is NOT calculated; must save before using.
 *
 * @see config_save(), config_load(), board_config_t
 */
void config_set_defaults(board_config_t *cfg) {
    if (cfg == NULL) {
        ESP_LOGE(TAG, "Invalid config pointer");
        return;
    }
    
    memset(cfg, 0, sizeof(*cfg));
    cfg->version = CONFIG_VERSION;
    cfg->can_start_id = 0x100;
    cfg->can_speed_kbps = 500; // Default CAN speed 500 kbps
    cfg->can_tx_hz = 25; // Default CAN transmit rate 25 Hz
    cfg->can_enabled = true;
    cfg->espnow_enabled = false;
    cfg->espnow_client_count = 0;
    memset(cfg->espnow_clients, 0, sizeof(cfg->espnow_clients));
    cfg->gps_enabled = false;
    cfg->gps_can_start_id = 0x650;
    memset(cfg->gps_target_mac, 0, sizeof(cfg->gps_target_mac));
    cfg->gps_update_rate_hz = 10;
    cfg->gps_source = GPS_SOURCE_DRAGY;
    memset(cfg->gps_espnow_peer_mac, 0, sizeof(cfg->gps_espnow_peer_mac));
    cfg->gps_response_timeout_ms = 1000U;
    cfg->mk60_emulator.enabled = false;
    cfg->mk60_emulator.trigger_id = MK60_TRIGGER_ID;
    cfg->mk60_emulator.trigger_dlc = 8U;
    cfg->mk60_emulator.response_count = 0U;
    cfg->pullup_vref_mv = 5025; // Default live pull-up reference voltage (mV)
    cfg->pullup_vref_divider_high_ohm = 9475;
    
    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        snprintf(cfg->channels[i].name, CONFIG_NAME_LEN, "Input %d", i + 1);
        cfg->channels[i].pullup_ohms = 0;
        cfg->channels[i].type = SENSOR_RAW;
        cfg->channels[i].filtering = FILTER_NONE; // No filtering by default
        cfg->channels[i].emub_tx = EMUB_TX_DISABLED;
    }
    
    ESP_LOGI(TAG, "Default config set");
}
