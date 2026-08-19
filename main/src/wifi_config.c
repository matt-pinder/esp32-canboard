#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "inc/config.h"
#include "inc/espnow_transport.h"

#include "inc/http_server.h"
#include "inc/wifi_config.h"
#include "secrets.h"

#define WIFI_SSID "ESP32-CanBoard" ///< WiFi access point SSID
#define WIFI_PASS "canconfig"    ///< WiFi access point password
#define WIFI_MAX_CONN 1            ///< Maximum number of simultaneous WiFi connections
#define CONFIG_TIMEOUT_MS 120000   ///< Configuration mode timeout in milliseconds (120 seconds)
#define PREFERRED_WIFI_SSID "Broomhall IoT"
#define PREFERRED_WIFI_SEARCH_MS 1000
#define PREFERRED_WIFI_CONNECT_MS 1000
#define WIFI_GOT_IP_BIT BIT0

/**
 * @brief Log tag for this module
 */
static const char *TAG = "WIFI_CFG";
extern board_config_t board_cfg;
/**
 * @brief Timer handle for configuration mode timeout
 */
static esp_timer_handle_t config_timer = NULL;
/**
 * @brief Flag indicating whether a client is currently connected to HTTP server
 */
static bool client_connected = false;
static EventGroupHandle_t wifi_events = NULL;
static void config_timeout_cb(void *arg);

/**
 * @brief Start the configuration HTTP server and its inactivity timer.
 */
static void start_config_server(void)
{
    start_http_server();

    const esp_timer_create_args_t timer_args = {
        .callback = &config_timeout_cb,
        .name = "cfg_timeout"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &config_timer));
    ESP_ERROR_CHECK(esp_timer_start_once(config_timer, CONFIG_TIMEOUT_MS * 1000));
}

/**
 * @brief Scan for the preferred network for up to PREFERRED_WIFI_SEARCH_MS.
 */
static bool scan_for_preferred_ap(void)
{
    const int64_t deadline_us = esp_timer_get_time() + (PREFERRED_WIFI_SEARCH_MS * 1000LL);

    ESP_LOGI(TAG, "Looking for %s for up to %d seconds",
             PREFERRED_WIFI_SSID, PREFERRED_WIFI_SEARCH_MS / 1000);
    do
    {
        esp_err_t ret = esp_wifi_scan_start(NULL, true);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "WiFi scan failed: %s", esp_err_to_name(ret));
            return false;
        }

        uint16_t ap_count = 0;
        ret = esp_wifi_scan_get_ap_num(&ap_count);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Could not read WiFi scan results: %s", esp_err_to_name(ret));
            esp_wifi_clear_ap_list();
            return false;
        }

        for (uint16_t i = 0; i < ap_count; i++)
        {
            wifi_ap_record_t ap_record = {0};
            ret = esp_wifi_scan_get_ap_record(&ap_record);
            if (ret != ESP_OK)
            {
                break;
            }
            if (strcmp((const char *)ap_record.ssid, PREFERRED_WIFI_SSID) == 0)
            {
                esp_wifi_clear_ap_list();
                ESP_LOGI(TAG, "Found %s (RSSI %d)", PREFERRED_WIFI_SSID, ap_record.rssi);
                return true;
            }
        }
        esp_wifi_clear_ap_list();
    } while (esp_timer_get_time() < deadline_us);

    ESP_LOGI(TAG, "%s was not found", PREFERRED_WIFI_SSID);
    return false;
}

/**
 * @brief Try to connect to the preferred network, leaving WiFi in STA mode on success.
 */
static bool try_preferred_station(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "Could not start station mode: %s", esp_err_to_name(ret));
        return false;
    }

    if (!scan_for_preferred_ap())
    {
        esp_wifi_stop();
        return false;
    }

    wifi_config_t sta_config = {
        .sta = {
            .ssid = PREFERRED_WIFI_SSID,
            .password = PREFERRED_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    xEventGroupClearBits(wifi_events, WIFI_GOT_IP_BIT);
    ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret == ESP_OK)
    {
        ret = esp_wifi_connect();
    }
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Could not connect to %s: %s", PREFERRED_WIFI_SSID, esp_err_to_name(ret));
        esp_wifi_stop();
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(wifi_events, WIFI_GOT_IP_BIT, pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(PREFERRED_WIFI_CONNECT_MS));
    if ((bits & WIFI_GOT_IP_BIT) != 0)
    {
        ESP_LOGI(TAG, "Connected to %s", PREFERRED_WIFI_SSID);
        return true;
    }

    ESP_LOGW(TAG, "Timed out connecting to %s; falling back to local AP", PREFERRED_WIFI_SSID);
    esp_wifi_disconnect();
    esp_wifi_stop();
    return false;
}

/**
 * @brief Start the existing local configuration AP and HTTP server.
 */
static void start_fallback_ap(void)
{
    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = WIFI_MAX_CONN,
            .channel = ESPNOW_WIFI_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK}};

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi AP+STA started: %s on channel %d", WIFI_SSID, ESPNOW_WIFI_CHANNEL);

    start_config_server();
}

/**
 * @brief Timer callback for configuration mode timeout
 * Disables WiFi AP and HTTP server if no client has connected within timeout period.
 * @param arg Callback argument (unused)
 */
static void config_timeout_cb(void *arg)
{
    if (!client_connected)
    {
        ESP_LOGI(TAG, "No client connected in 120s, disabling configuration HTTP server");
        stop_http_server();
        if (board_cfg.espnow_enabled)
        {
            esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to switch WiFi to STA mode for ESP-NOW: %s", esp_err_to_name(err));
            }
        }
        else
        {
            esp_wifi_stop();
        }
    }
}

/**
 * @brief WiFi event handler to log station connect/disconnect and reasons
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(wifi_events, WIFI_GOT_IP_BIT);
        return;
    }

    if (event_base != WIFI_EVENT)
        return;

    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station: %02x:%02x:%02x:%02x:%02x:%02x join, AID=%d, bgn, 40U",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5], evt->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station: %02x:%02x:%02x:%02x:%02x:%02x leave, AID = %d, reason = %d, bss_flags is %u, bss:0x%p",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5], evt->aid, evt->reason, 0, event_data);
    }
}

/**
 * @brief Connect to the preferred WiFi network, or start the configuration AP.
 * Searches for Broomhall IoT for 10 seconds. If it cannot be used, starts the
 * ESP32-CanBoard access point and HTTP configuration server for 120 seconds.
 */
void wifi_config_mode_start(void)
{
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

    if (!board_cfg.espnow_enabled && try_preferred_station())
    {
        start_config_server();
        return;
    }

    if (board_cfg.espnow_enabled)
    {
        ESP_LOGI(TAG, "ESP-NOW enabled; keeping WiFi on fixed channel %d", ESPNOW_WIFI_CHANNEL);
    }

    start_fallback_ap();
}

/**
 * @brief Notify that a client has connected and reset inactivity timer
 * Call this from HTTP server connection handler to prevent timeout during active configuration.
 */
void notify_client_connected(void)
{
    client_connected = true;
    if (config_timer)
    {
        esp_timer_stop(config_timer);
    }
}
