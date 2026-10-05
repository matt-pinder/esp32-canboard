#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "inc/config.h"
#include "inc/espnow_transport.h"
#include "nvs_flash.h"

#include "inc/http_server.h"
#include "inc/wifi_config.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

#ifndef KNOWN_WIFI_NETWORKS
#define KNOWN_WIFI_NETWORKS(X)
#endif

#define WIFI_SSID              "ESP32-CanBoard" ///< WiFi access point SSID
#define WIFI_PASS              "canconfig"      ///< WiFi access point password
#define WIFI_MAX_CONN          3                ///< Maximum number of simultaneous WiFi connections
#define KNOWN_WIFI_SEARCH_MS   1000
#define KNOWN_WIFI_CONNECT_MS  1000
#define WIFI_GOT_IP_BIT        BIT0

typedef struct {
    const char *ssid;
    const char *password;
} known_wifi_t;

#define WIFI_KNOWN_ENTRY(ssid_value, password_value) {.ssid = (ssid_value), .password = (password_value)},
static const known_wifi_t KNOWN_WIFI[] = {
    KNOWN_WIFI_NETWORKS(WIFI_KNOWN_ENTRY){.ssid = NULL, .password = NULL},
};
#undef WIFI_KNOWN_ENTRY

/**
 * @brief Log tag for this module
 */
static const char *TAG = "WIFI_CFG";
extern board_config_t board_cfg;
static EventGroupHandle_t wifi_events = NULL;
static uint8_t ap_station_count       = 0;

/**
 * @brief Scan for configured networks for up to KNOWN_WIFI_SEARCH_MS.
 */
static bool scan_for_known_ap(const known_wifi_t **selected, wifi_ap_record_t *selected_record) {
    if (selected == NULL || selected_record == NULL || KNOWN_WIFI[0].ssid == NULL) {
        return false;
    }
    *selected = NULL;

    const int64_t deadline_us = esp_timer_get_time() + (KNOWN_WIFI_SEARCH_MS * 1000LL);
    ESP_LOGI(TAG, "Looking for configured infrastructure WiFi for up to %d seconds", KNOWN_WIFI_SEARCH_MS / 1000);
    do {
        esp_err_t ret = esp_wifi_scan_start(NULL, true);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WiFi scan failed: %s", esp_err_to_name(ret));
            return false;
        }

        uint16_t ap_count = 0;
        ret               = esp_wifi_scan_get_ap_num(&ap_count);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Could not read WiFi scan results: %s", esp_err_to_name(ret));
            esp_wifi_clear_ap_list();
            return false;
        }
        if (ap_count == 0) {
            esp_wifi_clear_ap_list();
            continue;
        }

        wifi_ap_record_t *records = calloc(ap_count, sizeof(*records));
        if (records == NULL) {
            esp_wifi_clear_ap_list();
            return false;
        }
        uint16_t fetched = ap_count;
        ret              = esp_wifi_scan_get_ap_records(&fetched, records);
        if (ret == ESP_OK) {
            for (size_t known = 0; KNOWN_WIFI[known].ssid != NULL; known++) {
                for (uint16_t i = 0; i < fetched; i++) {
                    if (strcmp((const char *)records[i].ssid, KNOWN_WIFI[known].ssid) == 0) {
                        *selected        = &KNOWN_WIFI[known];
                        *selected_record = records[i];
                        ESP_LOGI(TAG, "Found configured network %s (RSSI %d)", KNOWN_WIFI[known].ssid, records[i].rssi);
                        break;
                    }
                }
                if (*selected != NULL) {
                    break;
                }
            }
        }
        free(records);
        esp_wifi_clear_ap_list();
        if (*selected != NULL) {
            return true;
        }
    } while (esp_timer_get_time() < deadline_us);

    ESP_LOGI(TAG, "No configured infrastructure WiFi was found");
    return false;
}

/**
 * @brief Try configured networks in list order while leaving the configuration AP active.
 */
static bool try_known_station(void) {
    const known_wifi_t *selected = NULL;
    wifi_ap_record_t record      = {0};
    if (!scan_for_known_ap(&selected, &record)) {
        return false;
    }

    const char *password = selected->password != NULL ? selected->password : "";
    size_t ssid_len      = strlen(selected->ssid);
    size_t password_len  = strlen(password);
    wifi_config_t sta_config = {0};
    if (ssid_len > sizeof(sta_config.sta.ssid) || password_len > sizeof(sta_config.sta.password)) {
        ESP_LOGW(TAG, "Configured infrastructure WiFi credentials are too long");
        return false;
    }
    memcpy(sta_config.sta.ssid, selected->ssid, ssid_len);
    memcpy(sta_config.sta.password, password, password_len);
    memcpy(sta_config.sta.bssid, record.bssid, sizeof(sta_config.sta.bssid));
    sta_config.sta.bssid_set          = true;
    sta_config.sta.channel            = record.primary;
    sta_config.sta.threshold.authmode = record.authmode;
    sta_config.sta.scan_method        = WIFI_FAST_SCAN;
    sta_config.sta.pmf_cfg.capable    = true;
    sta_config.sta.pmf_cfg.required   = false;

    xEventGroupClearBits(wifi_events, WIFI_GOT_IP_BIT);
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret == ESP_OK) {
        ret = esp_wifi_connect();
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not connect to %s: %s", selected->ssid, esp_err_to_name(ret));
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(wifi_events, WIFI_GOT_IP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(KNOWN_WIFI_CONNECT_MS));
    if ((bits & WIFI_GOT_IP_BIT) != 0) {
        ESP_LOGI(TAG, "Connected to %s", selected->ssid);
        return true;
    }

    ESP_LOGW(TAG, "Timed out connecting to %s; local AP remains available", selected->ssid);
    esp_wifi_disconnect();
    return false;
}

/**
 * @brief Start the local configuration AP without allocating the HTTP server yet.
 */
static void start_persistent_ap(void) {
    wifi_config_t ap_config = {.ap = {.ssid           = WIFI_SSID,
                                      .ssid_len       = strlen(WIFI_SSID),
                                      .password       = WIFI_PASS,
                                      .max_connection = WIFI_MAX_CONN,
                                      .channel        = ESPNOW_WIFI_CHANNEL,
                                      .authmode       = WIFI_AUTH_WPA2_PSK}};

    /* Keep the STA interface active for ESP-NOW so existing receivers continue
     * to see the canboard's established STA source MAC.  The SoftAP remains
     * active independently for configuration. */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Persistent WiFi AP started: %s on channel %d", WIFI_SSID, ESPNOW_WIFI_CHANNEL);
}

/**
 * @brief WiFi event handler to log station connect/disconnect and reasons
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_events, WIFI_GOT_IP_BIT);
        return;
    }

    if (event_base != WIFI_EVENT) {
        return;
    }

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
        if (ap_station_count < WIFI_MAX_CONN) {
            ap_station_count++;
        }
        start_http_server();
        ESP_LOGI(TAG,
                 "station: %02x:%02x:%02x:%02x:%02x:%02x join, AID=%d, bgn, 40U",
                 evt->mac[0],
                 evt->mac[1],
                 evt->mac[2],
                 evt->mac[3],
                 evt->mac[4],
                 evt->mac[5],
                 evt->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
        if (ap_station_count > 0) {
            ap_station_count--;
        }
        if (ap_station_count == 0) {
            stop_http_server();
        }
        ESP_LOGI(TAG,
                 "station: %02x:%02x:%02x:%02x:%02x:%02x leave, AID = %d, reason = %d, bss_flags is %u, bss:0x%p",
                 evt->mac[0],
                 evt->mac[1],
                 evt->mac[2],
                 evt->mac[3],
                 evt->mac[4],
                 evt->mac[5],
                 evt->aid,
                 evt->reason,
                 0,
                 event_data);
    }
}

/**
 * @brief Keep the configuration AP active and optionally connect to configured infrastructure WiFi.
 * The HTTP server is created only while a station is associated with the AP.
 */
void wifi_config_mode_start(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(wifi_events != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    start_persistent_ap();

    if (board_cfg.espnow_enabled) {
        ESP_LOGI(TAG, "ESP-NOW enabled; keeping WiFi on fixed channel %d", ESPNOW_WIFI_CHANNEL);
    }

    else {
        (void)try_known_station();
    }
}

/**
 * @brief Compatibility hook retained for existing HTTP handlers.
 */
void notify_client_connected(void) { /* AP association events now own HTTP server lifetime. */ }
