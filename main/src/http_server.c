#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inc/config.h"
#include "inc/can.h"
#include "inc/ble_scan.h"
#include "inc/dragy_gps.h"
#include "inc/espnow_transport.h"
#include "inc/http_server.h"
#include "inc/inputs.h"
#include "inc/wifi_config.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include <sys/param.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * @brief Log tag for HTTP server module
 */
static const char *TAG = "HTTPD";
/**
 * @brief HTTP server handle
 */
static httpd_handle_t server = NULL;
extern board_config_t board_cfg;

static void format_mac(const uint8_t mac[ESP_NOW_ETH_ALEN], char *out, size_t out_len) {
    snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool parse_mac(const char *text, uint8_t mac[ESP_NOW_ETH_ALEN]) {
    if (text == NULL || mac == NULL) {
        return false;
    }

    unsigned int bytes[ESP_NOW_ETH_ALEN] = {0};
    int matched = sscanf(text, "%2x:%2x:%2x:%2x:%2x:%2x",
                         &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4], &bytes[5]);
    if (matched != ESP_NOW_ETH_ALEN) {
        matched = sscanf(text, "%2x-%2x-%2x-%2x-%2x-%2x",
                         &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4], &bytes[5]);
    }
    if (matched != ESP_NOW_ETH_ALEN && strlen(text) == 12) {
        matched = sscanf(text, "%2x%2x%2x%2x%2x%2x",
                         &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4], &bytes[5]);
    }
    if (matched != ESP_NOW_ETH_ALEN) {
        return false;
    }

    for (int i = 0; i < ESP_NOW_ETH_ALEN; ++i) {
        if (bytes[i] > 0xFF) {
            return false;
        }
        mac[i] = (uint8_t)bytes[i];
    }
    return true;
}

static bool read_transport_config(cJSON *root, board_config_t *cfg) {
    cJSON *can_enabled = cJSON_GetObjectItem(root, "can_enabled");
    if (can_enabled && cJSON_IsBool(can_enabled)) {
        cfg->can_enabled = cJSON_IsTrue(can_enabled);
    }

    cJSON *espnow_enabled = cJSON_GetObjectItem(root, "espnow_enabled");
    if (espnow_enabled && cJSON_IsBool(espnow_enabled)) {
        cfg->espnow_enabled = cJSON_IsTrue(espnow_enabled);
    }

    cJSON *can_relay_espnow_enabled = cJSON_GetObjectItem(root, "can_relay_espnow_enabled");
    if (can_relay_espnow_enabled && cJSON_IsBool(can_relay_espnow_enabled)) {
        cfg->can_relay_espnow_enabled = cJSON_IsTrue(can_relay_espnow_enabled);
    }

    cJSON *espnow_mac = cJSON_GetObjectItem(root, "espnow_target_mac");
    if (espnow_mac && cJSON_IsString(espnow_mac)) {
        if (espnow_mac->valuestring == NULL || espnow_mac->valuestring[0] == '\0') {
            memset(cfg->espnow_target_mac, 0, ESP_NOW_ETH_ALEN);
        } else if (!parse_mac(espnow_mac->valuestring, cfg->espnow_target_mac)) {
            return false;
        }
    }

    uint8_t zero_mac[ESP_NOW_ETH_ALEN] = {0};
    if (cfg->espnow_enabled && memcmp(cfg->espnow_target_mac, zero_mac, ESP_NOW_ETH_ALEN) == 0) {
        return false;
    }
    if (cfg->can_relay_espnow_enabled && !cfg->espnow_enabled) {
        return false;
    }

    return true;
}

static bool read_gps_config(cJSON *root, board_config_t *cfg) {
    cJSON *gps_enabled = cJSON_GetObjectItem(root, "gps_enabled");
    if (gps_enabled && cJSON_IsBool(gps_enabled)) {
        cfg->gps_enabled = cJSON_IsTrue(gps_enabled);
    }

    cJSON *gps_start = cJSON_GetObjectItem(root, "gps_can_start_id");
    if (gps_start) {
        if (cJSON_IsNumber(gps_start)) {
            cfg->gps_can_start_id = (uint32_t)gps_start->valueint;
        } else if (cJSON_IsString(gps_start)) {
            const char *s = gps_start->valuestring;
            if (s && strlen(s) > 0) {
                cfg->gps_can_start_id = (uint32_t)strtoul(s, NULL, 0);
            }
        }
    }

    cJSON *gps_mac = cJSON_GetObjectItem(root, "gps_target_mac");
    if (gps_mac && cJSON_IsString(gps_mac)) {
        if (gps_mac->valuestring == NULL || gps_mac->valuestring[0] == '\0') {
            memset(cfg->gps_target_mac, 0, ESP_NOW_ETH_ALEN);
        } else if (!parse_mac(gps_mac->valuestring, cfg->gps_target_mac)) {
            return false;
        }
    }

    if (cfg->gps_can_start_id > 0x7FA) {
        return false;
    }

    uint8_t zero_mac[ESP_NOW_ETH_ALEN] = {0};
    if (cfg->gps_enabled && memcmp(cfg->gps_target_mac, zero_mac, ESP_NOW_ETH_ALEN) == 0) {
        return false;
    }

    return true;
}

static bool apply_runtime_config(const board_config_t *cfg) {
    if (cfg == NULL) {
        return false;
    }

    board_config_t previous_cfg = board_cfg;
    bool can_changed = (previous_cfg.can_enabled != cfg->can_enabled) ||
                       (previous_cfg.can_speed_kbps != cfg->can_speed_kbps) ||
                       (previous_cfg.can_relay_espnow_enabled != cfg->can_relay_espnow_enabled);
    bool espnow_changed = (previous_cfg.espnow_enabled != cfg->espnow_enabled) ||
                          (memcmp(previous_cfg.espnow_target_mac, cfg->espnow_target_mac, ESP_NOW_ETH_ALEN) != 0);
    bool gps_changed = (previous_cfg.gps_enabled != cfg->gps_enabled) ||
                       (previous_cfg.gps_can_start_id != cfg->gps_can_start_id) ||
                       (memcmp(previous_cfg.gps_target_mac, cfg->gps_target_mac, ESP_NOW_ETH_ALEN) != 0);
    board_cfg = *cfg;

    if (can_changed) {
        esp_err_t err = can_deinit();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop CAN before config update: %s", esp_err_to_name(err));
            board_cfg = previous_cfg;
            return false;
        }
        if (can_init() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reinitialize CAN after config update, restoring previous config");
            board_cfg = previous_cfg;
            can_deinit();
            if (previous_cfg.can_enabled && can_init() != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore previous CAN configuration");
            }
            return false;
        }
        ESP_LOGI(TAG, "Runtime CAN settings updated: enabled=%d speed=%lu kbps relay_to_espnow=%d",
                 board_cfg.can_enabled, (unsigned long)board_cfg.can_speed_kbps,
                 board_cfg.can_relay_espnow_enabled);
    }

    if (espnow_changed) {
        esp_err_t err = espnow_transport_apply_config();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to apply ESP-NOW config: %s", esp_err_to_name(err));
            board_cfg = previous_cfg;
            if (can_changed) {
                can_deinit();
                if (previous_cfg.can_enabled && can_init() != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to restore previous CAN configuration");
                }
            }
            espnow_transport_apply_config();
            return false;
        }
    }

    if (gps_changed) {
        dragy_gps_apply_config();
        ESP_LOGI(TAG, "Runtime GPS settings updated: enabled=%d start_id=0x%lX",
                 board_cfg.gps_enabled, (unsigned long)board_cfg.gps_can_start_id);
    }

    if (!can_changed && !espnow_changed && !gps_changed) {
        return true;
    }

    return true;
}

// Delayed restart task: stops HTTP server and WiFi, then restarts the chip.
static void delayed_restart_task(void *arg) {
    ESP_LOGI(TAG, "Delayed restart task: stopping HTTP server and WiFi");
    stop_http_server();
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Restarting now");
    esp_restart();
    vTaskDelete(NULL);
}

/**
 * @brief HTTP GET handler for the pre-compressed web UI
 * Streams the gzipped HTML asset from SPIFFS for standard and wildcard routes.
 * @param req HTTP request context
 * @return ESP_OK on success, ESP_FAIL on file or response errors
 */
esp_err_t index_get_handler(httpd_req_t *req) {
    notify_client_connected();
    FILE *f = fopen("/spiffs/index.min.html.gz", "rb");
    if (!f) {
        ESP_LOGW(TAG, "Failed to open index.min.html.gz");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    uint8_t chunk[1024];
    size_t bytes_read;
    while ((bytes_read = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        if (httpd_resp_send_chunk(req, (const char *)chunk, bytes_read) != ESP_OK) {
            ESP_LOGE(TAG, "Failed while streaming compressed web UI");
            fclose(f);
            return ESP_FAIL;
        }
    }

    if (ferror(f)) {
        ESP_LOGE(TAG, "Failed while reading compressed web UI");
        fclose(f);
        return ESP_FAIL;
    }

    fclose(f);
    return httpd_resp_send_chunk(req, NULL, 0);
}

/**
 * @brief HTTP GET handler for board configuration (GET /api/config)
 * Returns current configuration as JSON including all channel settings, sensor types, and NTC table references.
 * @param req HTTP request context
 * @return ESP_OK on success, ESP_FAIL on allocation or config load error
 */
esp_err_t config_get_handler(httpd_req_t *req) {
    notify_client_connected();
    board_config_t cfg = board_cfg;
    
    // Use larger buffer for JSON output
    char *json = malloc(4096);
    if (!json) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    size_t json_pos = 0;
    const size_t json_max = 4096;
    char espnow_mac[18];
    char gps_mac[18];
    format_mac(cfg.espnow_target_mac, espnow_mac, sizeof(espnow_mac));
    format_mac(cfg.gps_target_mac, gps_mac, sizeof(gps_mac));
    
    // Start JSON object including persisted board and transport configuration.
    json_pos += snprintf(json + json_pos, json_max - json_pos,
        "{\"can_enabled\":%s,\"can_speed_kbps\":%lu,\"can_start_id\":%lu,\"can_tx_hz\":%u,\"espnow_enabled\":%s,\"can_relay_espnow_enabled\":%s,\"espnow_target_mac\":\"%s\",\"gps_enabled\":%s,\"gps_can_start_id\":%lu,\"gps_target_mac\":\"%s\",\"pullup_vref_divider_high_ohm\":%u,\"channels\":[",
        cfg.can_enabled ? "true" : "false",
        (unsigned long)cfg.can_speed_kbps,
        (unsigned long)cfg.can_start_id,
        (unsigned)cfg.can_tx_hz,
        cfg.espnow_enabled ? "true" : "false",
        cfg.can_relay_espnow_enabled ? "true" : "false",
        espnow_mac,
        cfg.gps_enabled ? "true" : "false",
        (unsigned long)cfg.gps_can_start_id,
        gps_mac,
        (unsigned)cfg.pullup_vref_divider_high_ohm);
    
    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        json_pos += snprintf(json + json_pos, json_max - json_pos,
            "%s{\"name\":\"%s\",\"pullup_ohms\":%lu,\"type\":%u,\"filtering\":%d,\"emub_tx\":%u,\"params\":{",
            i ? "," : "",
            cfg.channels[i].name,
            (unsigned long)cfg.channels[i].pullup_ohms,
            cfg.channels[i].type,
            cfg.channels[i].filtering,
            cfg.channels[i].emub_tx);
            
        if (cfg.channels[i].type == SENSOR_NTC) {
            const ntc_table_def_t* table = ntc_get_table(cfg.channels[i].params.ntc.table_id);
            json_pos += snprintf(json + json_pos, json_max - json_pos, 
                "\"ntc\":{\"table_id\":%u,\"table_name\":\"%s\"}}}",
                cfg.channels[i].params.ntc.table_id,
                table ? table->name : "Unknown");
        } else if (cfg.channels[i].type == SENSOR_PRESSURE) {
            json_pos += snprintf(json + json_pos, json_max - json_pos, 
                "\"pressure\":{\"min_mv\":%u,\"max_mv\":%u,\"min_kpa\":%.2f,\"max_kpa\":%.2f}}}",
                cfg.channels[i].params.pressure.min_mv,
                cfg.channels[i].params.pressure.max_mv,
                cfg.channels[i].params.pressure.min_kpa,
                cfg.channels[i].params.pressure.max_kpa);
        } else {
            json_pos += snprintf(json + json_pos, json_max - json_pos, "\"raw\":{}}}");
        }
        
        // Check for buffer overflow
        if (json_pos >= json_max - 100) {
            ESP_LOGE(TAG, "JSON buffer overflow");
            free(json);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }
    
    snprintf(json + json_pos, json_max - json_pos, "]}\n");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    return ESP_OK;
}

/**
 * @brief HTTP POST handler for board configuration update (POST /api/config)
 * Accepts JSON configuration and validates all channel settings before persisting to NVS.
 * @param req HTTP request context
 * @return ESP_OK on successful save, ESP_FAIL on JSON parse error or config validation failure
 */
esp_err_t config_post_handler(httpd_req_t *req) {
    const size_t REQ_BUF_SIZE = 2048;
    char *buf = malloc(REQ_BUF_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate request buffer");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server error");
        return ESP_FAIL;
    }

    int ret = httpd_req_recv(req, buf, REQ_BUF_SIZE - 1);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive request body");
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request error");
        return ESP_FAIL;
    }
    buf[ret] = 0;
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        ESP_LOGW(TAG, "Invalid JSON received");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    board_config_t cfg;
    config_set_defaults(&cfg);
    
    cJSON *channels = cJSON_GetObjectItem(root, "channels");
    if (!channels || !cJSON_IsArray(channels) || cJSON_GetArraySize(channels) != CONFIG_CHANNELS) {
        ESP_LOGW(TAG, "Invalid channels array in config");
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channels array");
        return ESP_FAIL;
    }
    
    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        cJSON *ch = cJSON_GetArrayItem(channels, i);
        if (!ch) {
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing channel entry");
            return ESP_FAIL;
        }
        
        cJSON *name = cJSON_GetObjectItem(ch, "name");
        cJSON *pullup = cJSON_GetObjectItem(ch, "pullup_ohms");
        cJSON *type = cJSON_GetObjectItem(ch, "type");
        cJSON *filtering = cJSON_GetObjectItem(ch, "filtering");
        cJSON *emub_tx = cJSON_GetObjectItem(ch, "emub_tx");
        
        if (!cJSON_IsString(name) || !cJSON_IsNumber(pullup) || !cJSON_IsNumber(type) || !cJSON_IsNumber(emub_tx)) {
            ESP_LOGW(TAG, "Invalid channel fields at index %d", i);
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel fields");
            return ESP_FAIL;
        }
        
        strncpy(cfg.channels[i].name, name->valuestring, CONFIG_NAME_LEN - 1);
        cfg.channels[i].name[CONFIG_NAME_LEN - 1] = 0;
        cfg.channels[i].pullup_ohms = (uint32_t)pullup->valueint;
        cfg.channels[i].type = (sensor_type_t)type->valueint;
        if (filtering && cJSON_IsNumber(filtering)) {
            int lvl = filtering->valueint;
            if (lvl < FILTER_NONE) lvl = FILTER_NONE;
            if (lvl > FILTER_HIGH) lvl = FILTER_HIGH;
            cfg.channels[i].filtering = (uint8_t)lvl;
        } else {
            cfg.channels[i].filtering = FILTER_NONE;
        }
        cfg.channels[i].emub_tx = (uint8_t)emub_tx->valueint;
        if (cfg.channels[i].emub_tx > EMUB_TX_CAN_ANALOG_16) {
            cfg.channels[i].emub_tx = EMUB_TX_DISABLED;
        }
        
        cJSON *params = cJSON_GetObjectItem(ch, "params");
        if (!params) {
            ESP_LOGW(TAG, "Missing params for channel %d", i);
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing channel params");
            return ESP_FAIL;
        }
        
        if (cfg.channels[i].type == SENSOR_NTC) {
            cJSON *ntc = cJSON_GetObjectItem(params, "ntc");
            if (!ntc) {
                cJSON_Delete(root);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing NTC params");
                return ESP_FAIL;
            }
            cJSON *table_id = cJSON_GetObjectItem(ntc, "table_id");
            if (!cJSON_IsNumber(table_id)) {
                cJSON_Delete(root);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid NTC table_id");
                return ESP_FAIL;
            }
            cfg.channels[i].params.ntc.table_id = (uint8_t)table_id->valueint;
        } else if (cfg.channels[i].type == SENSOR_PRESSURE) {
            cJSON *pressure = cJSON_GetObjectItem(params, "pressure");
            if (!pressure) {
                cJSON_Delete(root);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing pressure params");
                return ESP_FAIL;
            }
            
            cJSON *min_mv = cJSON_GetObjectItem(pressure, "min_mv");
            cJSON *max_mv = cJSON_GetObjectItem(pressure, "max_mv");
            cJSON *min_kpa = cJSON_GetObjectItem(pressure, "min_kpa");
            cJSON *max_kpa = cJSON_GetObjectItem(pressure, "max_kpa");
            
            if (!cJSON_IsNumber(min_mv) || !cJSON_IsNumber(max_mv) || 
                !cJSON_IsNumber(min_kpa) || !cJSON_IsNumber(max_kpa)) {
                cJSON_Delete(root);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid pressure params");
                return ESP_FAIL;
            }
            
            cfg.channels[i].params.pressure.min_mv = (uint16_t)min_mv->valueint;
            cfg.channels[i].params.pressure.max_mv = (uint16_t)max_mv->valueint;
            cfg.channels[i].params.pressure.min_kpa = (float)min_kpa->valuedouble;
            cfg.channels[i].params.pressure.max_kpa = (float)max_kpa->valuedouble;
        }
    }
    
    // Read optional top-level fields: can_speed_kbps and can_start_id
    cJSON *can_speed = cJSON_GetObjectItem(root, "can_speed_kbps");
    if (can_speed && cJSON_IsNumber(can_speed)) {
        cfg.can_speed_kbps = (uint32_t)can_speed->valueint;
    }
    cJSON *can_start = cJSON_GetObjectItem(root, "can_start_id");
    if (can_start) {
        if (cJSON_IsNumber(can_start)) {
            cfg.can_start_id = (uint32_t)can_start->valueint;
        } else if (cJSON_IsString(can_start)) {
            // support hex string like "0x100"
            const char *s = can_start->valuestring;
            if (s && strlen(s) > 0) {
                cfg.can_start_id = (uint32_t)strtoul(s, NULL, 0);
            }
        }
    }

    cJSON *can_tx_hz = cJSON_GetObjectItem(root, "can_tx_hz");
    if (can_tx_hz && cJSON_IsNumber(can_tx_hz)) {
        uint32_t requested_hz = (uint32_t)can_tx_hz->valueint;
        cfg.can_tx_hz = (requested_hz == 50) ? 50 : 25;
    }

    cJSON *pullup_vref_high = cJSON_GetObjectItem(root, "pullup_vref_divider_high_ohm");
    if (pullup_vref_high && cJSON_IsNumber(pullup_vref_high)) {
        cfg.pullup_vref_divider_high_ohm = (uint16_t)pullup_vref_high->valueint;
    }

    if (!read_transport_config(root, &cfg)) {
        ESP_LOGW(TAG, "Invalid transport configuration");
        cJSON_Delete(root);
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid transport configuration");
        return ESP_FAIL;
    }

    if (!read_gps_config(root, &cfg)) {
        ESP_LOGW(TAG, "Invalid GPS configuration");
        cJSON_Delete(root);
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid GPS configuration");
        return ESP_FAIL;
    }

    // Keep pullup_vref_mv driven by the ADC task rather than the HTTP API.
    cfg.pullup_vref_mv = board_cfg.pullup_vref_mv;

    // Validate unique EMUB TX assignments
    bool emub_seen[EMUB_TX_CAN_ANALOG_16 + 1] = { false };
    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        if (cfg.channels[i].emub_tx > EMUB_TX_DISABLED) {
            if (emub_seen[cfg.channels[i].emub_tx]) {
                ESP_LOGW(TAG, "Duplicate EMUB TX assignment: %u", cfg.channels[i].emub_tx);
                cJSON_Delete(root);
                free(buf);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Duplicate EMUB TX assignment");
                return ESP_FAIL;
            }
            emub_seen[cfg.channels[i].emub_tx] = true;
        }
    }

    cJSON_Delete(root);
    free(buf);

    if (!config_save(&cfg)) {
        ESP_LOGE(TAG, "Failed to save configuration");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save config");
        return ESP_FAIL;
    }

    if (!apply_runtime_config(&cfg)) {
        ESP_LOGW(TAG, "Config saved but failed to fully apply at runtime");
    }
    
    ESP_LOGI(TAG, "Configuration updated successfully");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/**
 * @brief HTTP POST handler for device reboot (POST /api/reboot)
 * Immediately triggers ESP32 restart via esp_restart().
 * @param req HTTP request context
 * @return ESP_OK (though will not reach if reboot succeeds)
 */
esp_err_t reboot_post_handler(httpd_req_t *req) {
    httpd_resp_sendstr(req, "Rebooting");
    // Schedule a short delayed restart to allow the HTTP response to complete
    // and to shut down services cleanly.
    BaseType_t xr = xTaskCreate(delayed_restart_task, "del_restart", 2048, NULL, 5, NULL);
    if (xr != pdPASS) {
        ESP_LOGW(TAG, "Failed to create restart task; restarting immediately");
        esp_restart();
    }
    return ESP_OK;
}

/**
 * @brief HTTP GET handler for available NTC lookup tables (GET /api/ntc_tables)
 * Returns JSON array of all available NTC thermistor lookup tables with metadata.
 * @param req HTTP request context
 * @return ESP_OK on success, ESP_FAIL on allocation error
 */
esp_err_t ntc_tables_get_handler(httpd_req_t *req) {
    char *json = malloc(2048);
    if (!json) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    size_t json_pos = 0;
    const size_t json_max = 2048;
    
    json_pos += snprintf(json + json_pos, json_max - json_pos, "{\"tables\":[");
    
    for (size_t i = 0; i < NUM_NTC_TABLES; i++) {
        const ntc_table_def_t* table = ntc_get_table(i);
        if (table) {
            json_pos += snprintf(json + json_pos, json_max - json_pos,
                "%s{\"id\":%zu,\"name\":\"%s\",\"description\":\"%s\",\"points\":%zu}",
                i ? "," : "",
                i,
                table->name,
                table->description,
                table->points_count);
            
            if (json_pos >= json_max - 100) {
                ESP_LOGE(TAG, "JSON buffer overflow in NTC tables");
                free(json);
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
        }
    }
    
    snprintf(json + json_pos, json_max - json_pos, "]}\n");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    return ESP_OK;
}

/**
 * @brief HTTP GET handler for live channel values (GET /api/live_values)
 * Returns current measured voltage and computed value (pressure/temperature) per channel.
 */
esp_err_t live_values_get_handler(httpd_req_t *req) {
    notify_client_connected();

    uint16_t voltages_copy[NUM_ADC_CHANNELS] = {0};
    if (xSemaphoreTake(filtered_voltages_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        memcpy(voltages_copy, (const void *)filtered_voltages, sizeof(voltages_copy));
        xSemaphoreGive(filtered_voltages_mutex);
    }

    char *json = malloc(4096);
    if (!json) {
        ESP_LOGE(TAG, "Failed to allocate live values JSON buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t json_pos = 0;
    const size_t json_max = 4096;

    int written = snprintf(json + json_pos, json_max - json_pos,
        "{\"derived_vref_mv\":%u,\"channels\":[",
        (unsigned)board_cfg.pullup_vref_mv);
    if (written < 0 || (size_t)written >= (json_max - json_pos)) {
        ESP_LOGE(TAG, "Failed to build live values JSON header");
        free(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    json_pos += (size_t)written;

    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        float voltage_v = (float)voltages_copy[i] / 1000.0f;

        if (board_cfg.channels[i].type == SENSOR_PRESSURE) {
            uint16_t pressure_x100 = getSensorPressure(
                voltages_copy[i],
                board_cfg.channels[i].params.pressure.min_mv,
                board_cfg.channels[i].params.pressure.max_mv,
                board_cfg.channels[i].params.pressure.min_kpa,
                board_cfg.channels[i].params.pressure.max_kpa);
            float pressure_kpa = (float)pressure_x100 / 100.0f;
            written = snprintf(json + json_pos, json_max - json_pos,
                "%s{\"voltage_v\":%.2f,\"value\":\"%.2f kPa\"}",
                i ? "," : "", voltage_v, pressure_kpa);
        } else if (board_cfg.channels[i].type == SENSOR_NTC) {
            const ntc_table_def_t *table = ntc_get_table(board_cfg.channels[i].params.ntc.table_id);
            int8_t temp_c = getSensorTemperature(
                voltages_copy[i],
                board_cfg.channels[i].pullup_ohms,
                board_cfg.pullup_vref_mv,
                table ? table->points : NULL,
                table ? table->points_count : 0);

            int32_t r_ntc = -1;
            if (voltages_copy[i] > 0 && voltages_copy[i] < board_cfg.pullup_vref_mv &&
                board_cfg.channels[i].pullup_ohms > 0 && board_cfg.pullup_vref_mv > 0) {
                float v_ntc = (float)voltages_copy[i] / 1000.0f;
                float v_ref = (float)board_cfg.pullup_vref_mv / 1000.0f;
                float r_eq_f = ((float)board_cfg.channels[i].pullup_ohms * v_ntc) / (v_ref - v_ntc);
                float denom = (float)DIVIDER_TOTAL_OHM - r_eq_f;
                if (denom > 0.0f) {
                    float r_ntc_f = (r_eq_f * (float)DIVIDER_TOTAL_OHM) / denom;
                    r_ntc = (int32_t)(r_ntc_f + 0.5f);
                }
            }

            if (temp_c == (int8_t)-128) {
                written = snprintf(json + json_pos, json_max - json_pos,
                    "%s{\"voltage_v\":%.2f,\"value\":\"\"}",
                    i ? "," : "", voltage_v);
            } else {
                if (r_ntc >= 0) {
                    written = snprintf(json + json_pos, json_max - json_pos,
                        "%s{\"voltage_v\":%.2f,\"value\":\"%d C (%ld ohm)\"}",
                        i ? "," : "", voltage_v, (int)temp_c, (long)r_ntc);
                } else {
                    written = snprintf(json + json_pos, json_max - json_pos,
                        "%s{\"voltage_v\":%.2f,\"value\":\"%d C\"}",
                        i ? "," : "", voltage_v, (int)temp_c);
                }
            }
        } else {
            written = snprintf(json + json_pos, json_max - json_pos,
                "%s{\"voltage_v\":%.2f,\"value\":\"\"}",
                i ? "," : "", voltage_v);
        }

        if (written < 0 || (size_t)written >= (json_max - json_pos)) {
            ESP_LOGE(TAG, "JSON buffer overflow in live values (channel %d)", i);
            free(json);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        json_pos += (size_t)written;
    }

    written = snprintf(json + json_pos, json_max - json_pos, "]}\n");
    if (written < 0 || (size_t)written >= (json_max - json_pos)) {
        ESP_LOGE(TAG, "Failed to finalize live values JSON");
        free(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    return ESP_OK;
}

/**
 * @brief HTTP GET handler to export configuration as downloadable JSON file
 * (GET /api/config/export.json)
 */
esp_err_t config_export_get_handler(httpd_req_t *req) {
    // Keep the configuration off the HTTP server task's relatively small stack.
    // config_load() also uses configuration-sized working buffers, and nesting
    // all of them on this stack can corrupt it when the load path logs details.
    board_config_t *cfg = calloc(1, sizeof(*cfg));
    if (!cfg) {
        ESP_LOGE(TAG, "Failed to allocate export configuration");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (!config_load(cfg)) {
        ESP_LOGW(TAG, "Failed to load persisted config for export; using runtime config");
        *cfg = board_cfg;
    }

    char *json = malloc(4096);
    if (!json) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer");
        free(cfg);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t json_pos = 0;
    const size_t json_max = 4096;
    char espnow_mac[18];
    char gps_mac[18];
    format_mac(cfg->espnow_target_mac, espnow_mac, sizeof(espnow_mac));
    format_mac(cfg->gps_target_mac, gps_mac, sizeof(gps_mac));
    json_pos += snprintf(json + json_pos, json_max - json_pos,
        "{\"can_enabled\":%s,\"can_speed_kbps\":%lu,\"can_start_id\":%lu,\"can_tx_hz\":%u,\"espnow_enabled\":%s,\"can_relay_espnow_enabled\":%s,\"espnow_target_mac\":\"%s\",\"gps_enabled\":%s,\"gps_can_start_id\":%lu,\"gps_target_mac\":\"%s\",\"pullup_vref_divider_high_ohm\":%u,\"channels\":[",
        cfg->can_enabled ? "true" : "false",
        (unsigned long)cfg->can_speed_kbps,
        (unsigned long)cfg->can_start_id,
        (unsigned)cfg->can_tx_hz,
        cfg->espnow_enabled ? "true" : "false",
        cfg->can_relay_espnow_enabled ? "true" : "false",
        espnow_mac,
        cfg->gps_enabled ? "true" : "false",
        (unsigned long)cfg->gps_can_start_id,
        gps_mac,
        (unsigned)cfg->pullup_vref_divider_high_ohm);

    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        json_pos += snprintf(json + json_pos, json_max - json_pos,
            "%s{\"name\":\"%s\",\"pullup_ohms\":%lu,\"type\":%u,\"filtering\":%d,\"emub_tx\":%u,\"params\":{",
            i ? "," : "",
            cfg->channels[i].name,
            (unsigned long)cfg->channels[i].pullup_ohms,
            cfg->channels[i].type,
            cfg->channels[i].filtering,
            cfg->channels[i].emub_tx);

        if (cfg->channels[i].type == SENSOR_NTC) {
            json_pos += snprintf(json + json_pos, json_max - json_pos,
                "\"ntc\":{\"table_id\":%u}}}", cfg->channels[i].params.ntc.table_id);
        } else if (cfg->channels[i].type == SENSOR_PRESSURE) {
            json_pos += snprintf(json + json_pos, json_max - json_pos,
                "\"pressure\":{\"min_mv\":%u,\"max_mv\":%u,\"min_kpa\":%.2f,\"max_kpa\":%.2f}}}",
                cfg->channels[i].params.pressure.min_mv,
                cfg->channels[i].params.pressure.max_mv,
                cfg->channels[i].params.pressure.min_kpa,
                cfg->channels[i].params.pressure.max_kpa);
        } else {
            json_pos += snprintf(json + json_pos, json_max - json_pos, "\"raw\":{}}}");
        }

        if (json_pos >= json_max - 100) {
            ESP_LOGE(TAG, "JSON buffer overflow (export)");
            free(json);
            free(cfg);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }

    snprintf(json + json_pos, json_max - json_pos, "]}\n");

    // Set headers to trigger download in browser
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=esp32-canboard-config.json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    free(cfg);
    return ESP_OK;
}


/**
 * @brief HTTP POST handler to import configuration JSON (POST /api/config/import.json)
 * Accepts raw JSON body, validates it, backs up existing config, and saves new config.
 */
esp_err_t config_import_post_handler(httpd_req_t *req) {
    const size_t REQ_BUF_SIZE = 4096;
    char *buf = malloc(REQ_BUF_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate import buffer");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server error");
        return ESP_FAIL;
    }

    int ret = httpd_req_recv(req, buf, REQ_BUF_SIZE - 1);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive import body");
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request error");
        return ESP_FAIL;
    }
    buf[ret] = 0;

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        ESP_LOGW(TAG, "Invalid JSON import");
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    board_config_t cfg;
    config_set_defaults(&cfg);

    cJSON *channels = cJSON_GetObjectItem(root, "channels");
    if (!channels || !cJSON_IsArray(channels) || cJSON_GetArraySize(channels) != CONFIG_CHANNELS) {
        ESP_LOGW(TAG, "Invalid channels array in import");
        cJSON_Delete(root);
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channels array");
        return ESP_FAIL;
    }

    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        cJSON *ch = cJSON_GetArrayItem(channels, i);
        if (!ch) {
            cJSON_Delete(root);
            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing channel entry");
            return ESP_FAIL;
        }

        cJSON *name = cJSON_GetObjectItem(ch, "name");
        cJSON *pullup = cJSON_GetObjectItem(ch, "pullup_ohms");
        cJSON *type = cJSON_GetObjectItem(ch, "type");
        cJSON *filtering = cJSON_GetObjectItem(ch, "filtering");
        cJSON *emub_tx = cJSON_GetObjectItem(ch, "emub_tx");

        if (!cJSON_IsString(name) || !cJSON_IsNumber(pullup) || !cJSON_IsNumber(type) || !cJSON_IsNumber(emub_tx)) {
            ESP_LOGW(TAG, "Invalid channel fields at index %d", i);
            cJSON_Delete(root);
            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel fields");
            return ESP_FAIL;
        }

        strncpy(cfg.channels[i].name, name->valuestring, CONFIG_NAME_LEN - 1);
        cfg.channels[i].name[CONFIG_NAME_LEN - 1] = 0;
        cfg.channels[i].pullup_ohms = (uint32_t)pullup->valueint;
        cfg.channels[i].type = (sensor_type_t)type->valueint;
        cfg.channels[i].emub_tx = (uint8_t)emub_tx->valueint;
        if (cfg.channels[i].emub_tx > EMUB_TX_CAN_ANALOG_16) {
            cfg.channels[i].emub_tx = EMUB_TX_DISABLED;
        }
        if (filtering && cJSON_IsNumber(filtering)) {
            int lvl = filtering->valueint;
            if (lvl < FILTER_NONE) lvl = FILTER_NONE;
            if (lvl > FILTER_HIGH) lvl = FILTER_HIGH;
            cfg.channels[i].filtering = (uint8_t)lvl;
        } else {
            cfg.channels[i].filtering = FILTER_NONE;
        }

        cJSON *params = cJSON_GetObjectItem(ch, "params");
        if (!params) {
            ESP_LOGW(TAG, "Missing params for channel %d", i);
            cJSON_Delete(root);
            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing channel params");
            return ESP_FAIL;
        }

        if (cfg.channels[i].type == SENSOR_NTC) {
            cJSON *ntc = cJSON_GetObjectItem(params, "ntc");
            if (!ntc) {
                cJSON_Delete(root);
                free(buf);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing NTC params");
                return ESP_FAIL;
            }
            cJSON *table_id = cJSON_GetObjectItem(ntc, "table_id");
            if (!cJSON_IsNumber(table_id)) {
                cJSON_Delete(root);
                free(buf);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid NTC table_id");
                return ESP_FAIL;
            }
            cfg.channels[i].params.ntc.table_id = (uint8_t)table_id->valueint;
        } else if (cfg.channels[i].type == SENSOR_PRESSURE) {
            cJSON *pressure = cJSON_GetObjectItem(params, "pressure");
            if (!pressure) {
                cJSON_Delete(root);
                free(buf);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing pressure params");
                return ESP_FAIL;
            }

            cJSON *min_mv = cJSON_GetObjectItem(pressure, "min_mv");
            cJSON *max_mv = cJSON_GetObjectItem(pressure, "max_mv");
            cJSON *min_kpa = cJSON_GetObjectItem(pressure, "min_kpa");
            cJSON *max_kpa = cJSON_GetObjectItem(pressure, "max_kpa");

            if (!cJSON_IsNumber(min_mv) || !cJSON_IsNumber(max_mv) || 
                !cJSON_IsNumber(min_kpa) || !cJSON_IsNumber(max_kpa)) {
                cJSON_Delete(root);
                free(buf);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid pressure params");
                return ESP_FAIL;
            }

            cfg.channels[i].params.pressure.min_mv = (uint16_t)min_mv->valueint;
            cfg.channels[i].params.pressure.max_mv = (uint16_t)max_mv->valueint;
            cfg.channels[i].params.pressure.min_kpa = (float)min_kpa->valuedouble;
            cfg.channels[i].params.pressure.max_kpa = (float)max_kpa->valuedouble;
        }
    }

    cJSON *can_speed = cJSON_GetObjectItem(root, "can_speed_kbps");
    if (can_speed && cJSON_IsNumber(can_speed)) {
        cfg.can_speed_kbps = (uint32_t)can_speed->valueint;
    }
    cJSON *can_start = cJSON_GetObjectItem(root, "can_start_id");
    if (can_start) {
        if (cJSON_IsNumber(can_start)) {
            cfg.can_start_id = (uint32_t)can_start->valueint;
        } else if (cJSON_IsString(can_start)) {
            const char *s = can_start->valuestring;
            if (s && strlen(s) > 0) {
                cfg.can_start_id = (uint32_t)strtoul(s, NULL, 0);
            }
        }
    }

    cJSON *can_tx_hz = cJSON_GetObjectItem(root, "can_tx_hz");
    if (can_tx_hz && cJSON_IsNumber(can_tx_hz)) {
        uint32_t requested_hz = (uint32_t)can_tx_hz->valueint;
        cfg.can_tx_hz = (requested_hz == 50) ? 50 : 25;
    }

    cJSON *pullup_vref_high = cJSON_GetObjectItem(root, "pullup_vref_divider_high_ohm");
    if (pullup_vref_high && cJSON_IsNumber(pullup_vref_high)) {
        cfg.pullup_vref_divider_high_ohm = (uint16_t)pullup_vref_high->valueint;
    }

    if (!read_transport_config(root, &cfg)) {
        ESP_LOGW(TAG, "Invalid transport configuration in import");
        cJSON_Delete(root);
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid transport configuration");
        return ESP_FAIL;
    }

    if (!read_gps_config(root, &cfg)) {
        ESP_LOGW(TAG, "Invalid GPS configuration in import");
        cJSON_Delete(root);
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid GPS configuration");
        return ESP_FAIL;
    }
    cfg.pullup_vref_mv = board_cfg.pullup_vref_mv;

    // config_save() commits the previous valid NVS record to its backup key
    // before replacing the active record.
    bool saved = config_save(&cfg);
    if (!saved) {
        ESP_LOGE(TAG, "Failed to save imported configuration");
        cJSON_Delete(root);
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save imported config");
        return ESP_FAIL;
    }

    if (!apply_runtime_config(&cfg)) {
        ESP_LOGW(TAG, "Imported config saved but failed to fully apply at runtime");
    }

    cJSON_Delete(root);
    free(buf);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/**
 * @brief HTTP POST handler to scan nearby BLE devices likely to be Dragy devices.
 */
esp_err_t ble_dragy_scan_post_handler(httpd_req_t *req) {
    notify_client_connected();

    ble_scan_result_t results[BLE_SCAN_MAX_RESULTS] = {0};
    size_t count = 0;
    esp_err_t err = ble_scan_dragy(5000, results, BLE_SCAN_MAX_RESULTS, &count);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE Dragy scan failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_FAIL;
    }

    char *json = malloc(2048);
    if (!json) {
        ESP_LOGE(TAG, "Failed to allocate BLE scan JSON buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t json_pos = 0;
    const size_t json_max = 2048;
    json_pos += snprintf(json + json_pos, json_max - json_pos, "{\"devices\":[");

    for (size_t i = 0; i < count; ++i) {
        if (!results[i].potential_dragy) {
            continue;
        }

        char mac[18];
        format_mac(results[i].mac, mac, sizeof(mac));
        int written = snprintf(json + json_pos, json_max - json_pos,
            "%s{\"mac\":\"%s\",\"name\":\"%s\",\"rssi\":%d,\"has_dragy_name\":%s,\"has_fd00_service\":%s}",
            json_pos > strlen("{\"devices\":[") ? "," : "",
            mac,
            results[i].name[0] ? results[i].name : "",
            (int)results[i].rssi,
            results[i].has_dragy_name ? "true" : "false",
            results[i].has_fd00_service ? "true" : "false");
        if (written < 0 || (size_t)written >= (json_max - json_pos)) {
            ESP_LOGE(TAG, "JSON buffer overflow in BLE scan results");
            free(json);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        json_pos += (size_t)written;
    }

    snprintf(json + json_pos, json_max - json_pos, "]}\n");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    return ESP_OK;
}

/**
 * @brief Start HTTP server with REST API endpoints and SPIFFS mount
 * Initializes SPIFFS filesystem and starts HTTP server on port 80 (http://192.168.4.1).
 * Registers five URI handlers for configuration management and device control.
 */
void start_http_server(void) {
    // Initialize SPIFFS
    esp_err_t ret = esp_vfs_spiffs_register(&(esp_vfs_spiffs_conf_t){
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    });

    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "SPIFFS already mounted");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS: %s", esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted successfully");
    }
    
    // Start HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 12;
    // Allow larger responses, such as the configuration UI, to traverse a
    // slower infrastructure WiFi path without hitting the 5-second default.
    config.send_wait_timeout = 15;
    
    ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register URI handlers
    httpd_uri_t index_uri = { 
        .uri = "/", 
        .method = HTTP_GET, 
        .handler = index_get_handler, 
        .user_ctx = NULL 
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    
    httpd_uri_t config_get_uri = { 
        .uri = "/api/config", 
        .method = HTTP_GET, 
        .handler = config_get_handler, 
        .user_ctx = NULL 
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_get_uri));
    
    httpd_uri_t config_post_uri = { 
        .uri = "/api/config", 
        .method = HTTP_POST, 
        .handler = config_post_handler, 
        .user_ctx = NULL 
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_post_uri));

    httpd_uri_t config_export_uri = {
        .uri = "/api/config/export.json",
        .method = HTTP_GET,
        .handler = config_export_get_handler,
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_export_uri));

    httpd_uri_t config_import_uri = {
        .uri = "/api/config/import.json",
        .method = HTTP_POST,
        .handler = config_import_post_handler,
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_import_uri));
    
    httpd_uri_t reboot_post_uri = { 
        .uri = "/api/reboot", 
        .method = HTTP_POST, 
        .handler = reboot_post_handler, 
        .user_ctx = NULL 
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &reboot_post_uri));
    
    httpd_uri_t ntc_tables_uri = { 
        .uri = "/api/ntc_tables", 
        .method = HTTP_GET, 
        .handler = ntc_tables_get_handler, 
        .user_ctx = NULL 
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ntc_tables_uri));

    httpd_uri_t live_values_uri = {
        .uri = "/api/live_values",
        .method = HTTP_GET,
        .handler = live_values_get_handler,
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &live_values_uri));

    httpd_uri_t ble_dragy_scan_uri = {
        .uri = "/api/ble/dragy_scan",
        .method = HTTP_POST,
        .handler = ble_dragy_scan_post_handler,
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ble_dragy_scan_uri));
    
    ESP_LOGI(TAG, "HTTP server started on http://192.168.4.1");
}

/**
 * @brief Stop HTTP server and unmount SPIFFS
 * Gracefully closes HTTP server and unmounts SPIFFS filesystem.
 */
void stop_http_server(void) {
    if (server) {
        ESP_LOGI(TAG, "Stopping HTTP server");
        httpd_stop(server);
        server = NULL;
    }
    
    // Unmount SPIFFS if needed
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}
