#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "inc/config.h"
#include "inc/inputs.h"
#include "cJSON.h"
#include <sys/param.h>
#include <string.h>
#include <stdlib.h>

/**
 * @brief Log tag for HTTP server module
 */
static const char *TAG = "HTTPD";
/**
 * @brief HTTP server handle
 */
static httpd_handle_t server = NULL;

/**
 * @brief HTTP GET handler for index.html
 * Serves the web UI from SPIFFS. Supports both standard requests and wildcard routes.
 * @param req HTTP request context
 * @return ESP_OK on success, ESP_FAIL on file not found or allocation error
 */
esp_err_t index_get_handler(httpd_req_t *req) {
    FILE *f = fopen("/spiffs/index.html", "r");
    if (!f) {
        ESP_LOGW(TAG, "Failed to open index.html");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    // Get file size
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    if (size <= 0 || size > 65536) {  // Arbitrary limit to prevent massive allocations
        ESP_LOGE(TAG, "Invalid file size: %ld", size);
        fclose(f);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    char *buf = malloc(size);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate memory for file");
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    size_t read = fread(buf, 1, size, f);
    fclose(f);
    
    if (read != (size_t)size) {
        ESP_LOGE(TAG, "Failed to read entire file");
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, buf, size);
    free(buf);
    return ESP_OK;
}

/**
 * @brief HTTP GET handler for board configuration (GET /api/config)
 * Returns current configuration as JSON including all channel settings, sensor types, and NTC table references.
 * @param req HTTP request context
 * @return ESP_OK on success, ESP_FAIL on allocation or config load error
 */
esp_err_t config_get_handler(httpd_req_t *req) {
    board_config_t cfg;
    if (!config_load(&cfg)) {
        config_set_defaults(&cfg);
    }
    
    // Use larger buffer for JSON output
    char *json = malloc(4096);
    if (!json) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    size_t json_pos = 0;
    const size_t json_max = 4096;
    
    // Start JSON object
    json_pos += snprintf(json + json_pos, json_max - json_pos, "{\"channels\":[");
    
    for (int i = 0; i < CONFIG_CHANNELS; ++i) {
        json_pos += snprintf(json + json_pos, json_max - json_pos,
            "%s{\"name\":\"%s\",\"pullup_ohms\":%lu,\"type\":%u,\"filtering\":%d,\"params\":{",
            i ? "," : "",
            cfg.channels[i].name,
            (unsigned long)cfg.channels[i].pullup_ohms,
            cfg.channels[i].type,
            cfg.channels[i].filtering);
            
        if (cfg.channels[i].type == SENSOR_NTC) {
            const ntc_table_def_t* table = ntc_get_table(cfg.channels[i].params.ntc.table_id);
            json_pos += snprintf(json + json_pos, json_max - json_pos, 
                "\"ntc\":{\"table_id\":%u,\"table_name\":\"%s\"}}",
                cfg.channels[i].params.ntc.table_id,
                table ? table->name : "Unknown");
        } else if (cfg.channels[i].type == SENSOR_PRESSURE) {
            json_pos += snprintf(json + json_pos, json_max - json_pos, 
                "\"pressure\":{\"min_mv\":%u,\"max_mv\":%u,\"min_kpa\":%.2f,\"max_kpa\":%.2f}}",
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
 * Accepts JSON configuration and validates all channel settings before persisting to SPIFFS.
 * @param req HTTP request context
 * @return ESP_OK on successful save, ESP_FAIL on JSON parse error or config validation failure
 */
esp_err_t config_post_handler(httpd_req_t *req) {
    char buf[2048];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive request body");
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
        
        if (!cJSON_IsString(name) || !cJSON_IsNumber(pullup) || !cJSON_IsNumber(type)) {
            ESP_LOGW(TAG, "Invalid channel fields at index %d", i);
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel fields");
            return ESP_FAIL;
        }
        
        strncpy(cfg.channels[i].name, name->valuestring, CONFIG_NAME_LEN - 1);
        cfg.channels[i].name[CONFIG_NAME_LEN - 1] = 0;
        cfg.channels[i].pullup_ohms = (uint32_t)pullup->valueint;
        cfg.channels[i].type = (sensor_type_t)type->valueint;
        cfg.channels[i].filtering = filtering && cJSON_IsNumber(filtering) ? filtering->valueint : 0;
        
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
    
    cJSON_Delete(root);
    
    if (!config_save(&cfg)) {
        ESP_LOGE(TAG, "Failed to save configuration");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save config");
        return ESP_FAIL;
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
    esp_restart();
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
        .format_if_mount_failed = true
    });
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPIFFS mounted successfully");
    
    // Start HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    
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
    httpd_register_uri_handler(server, &index_uri);
    
    httpd_uri_t config_get_uri = { 
        .uri = "/api/config", 
        .method = HTTP_GET, 
        .handler = config_get_handler, 
        .user_ctx = NULL 
    };
    httpd_register_uri_handler(server, &config_get_uri);
    
    httpd_uri_t config_post_uri = { 
        .uri = "/api/config", 
        .method = HTTP_POST, 
        .handler = config_post_handler, 
        .user_ctx = NULL 
    };
    httpd_register_uri_handler(server, &config_post_uri);
    
    httpd_uri_t reboot_post_uri = { 
        .uri = "/api/reboot", 
        .method = HTTP_POST, 
        .handler = reboot_post_handler, 
        .user_ctx = NULL 
    };
    httpd_register_uri_handler(server, &reboot_post_uri);
    
    httpd_uri_t ntc_tables_uri = { 
        .uri = "/api/ntc_tables", 
        .method = HTTP_GET, 
        .handler = ntc_tables_get_handler, 
        .user_ctx = NULL 
    };
    httpd_register_uri_handler(server, &ntc_tables_uri);
    
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
