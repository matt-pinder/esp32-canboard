#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "inc/config.h"
#include "inc/espnow_transport.h"

#include "inc/http_server.h"
#include "inc/wifi_config.h"
#include "secrets.h"

#define WIFI_SSID "ESP32-CanBoard" ///< WiFi access point SSID
#define WIFI_PASS "canconfig"    ///< WiFi access point password
#define WIFI_MAX_CONN 1            ///< Maximum number of simultaneous WiFi connections
#define PREFERRED_WIFI_SSID "Broomhall IoTs"
#define PREFERRED_WIFI_SEARCH_MS 1000
#define PREFERRED_WIFI_CONNECT_MS 1000
#define WIFI_GOT_IP_BIT BIT0
#define USB_PRESENCE_DETECT GPIO_NUM_38
#define WIFI_DIAG_MAGIC 0x57444941U
#define WIFI_DIAG_HISTORY_MAGIC 0x57444849U
#define WIFI_DIAG_VERSION 2U
#define WIFI_DIAG_NAMESPACE "wifi_diag"
#define WIFI_DIAG_KEY "boot_history"
#define WIFI_DIAG_HISTORY_COUNT 4U
#define WIFI_DIAG_RESULT_NOT_RUN INT32_MIN
#define WIFI_DIAG_EVENT_AP_START BIT0
#define WIFI_DIAG_EVENT_AP_STOP BIT1

/**
 * @brief Log tag for this module
 */
static const char *TAG = "WIFI_CFG";
extern board_config_t board_cfg;
static EventGroupHandle_t wifi_events = NULL;
static SemaphoreHandle_t wifi_diag_mutex = NULL;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t boot_number;
    uint32_t reset_reason;
    uint32_t event_bits;
    uint32_t free_heap;
    int32_t set_mode_result;
    int32_t set_ap_config_result;
    int32_t wifi_start_result;
    int32_t get_mode_result;
    int32_t get_channel_result;
    int32_t get_ap_config_result;
    uint8_t espnow_enabled;
    uint8_t observed_mode;
    uint8_t primary_channel;
    uint8_t ap_ssid_len;
    uint8_t usb_vbus_present;
    uint8_t usb_phy_enabled;
    char ap_ssid[33];
} wifi_boot_diagnostic_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t next_slot;
    wifi_boot_diagnostic_t records[WIFI_DIAG_HISTORY_COUNT];
} wifi_boot_diagnostic_history_t;

static wifi_boot_diagnostic_t wifi_diag;
static wifi_boot_diagnostic_history_t wifi_diag_history;
static uint32_t wifi_diag_slot;
static bool usb_vbus_present = true;
static bool usb_phy_enabled = false;

static bool wifi_diag_valid(const wifi_boot_diagnostic_t *diag)
{
    return diag->magic == WIFI_DIAG_MAGIC && diag->version == WIFI_DIAG_VERSION;
}

static bool wifi_diag_history_valid(const wifi_boot_diagnostic_history_t *history)
{
    return history->magic == WIFI_DIAG_HISTORY_MAGIC && history->version == WIFI_DIAG_VERSION &&
           history->next_slot < WIFI_DIAG_HISTORY_COUNT;
}

static void wifi_diag_log_previous(const wifi_boot_diagnostic_t *diag)
{
    ESP_LOGW(TAG,
             "PREVIOUS BOOT WIFI: boot=%lu reset=%lu espnow=%u events=0x%02lx "
             "set_mode=%ld set_ap_config=%ld wifi_start=%ld get_mode=%ld mode=%u "
             "get_channel=%ld channel=%u get_ap_config=%ld ssid='%s' usb_vbus=%u "
             "usb_phy=%u heap=%lu",
             (unsigned long)diag->boot_number,
             (unsigned long)diag->reset_reason,
             diag->espnow_enabled,
             (unsigned long)diag->event_bits,
             (long)diag->set_mode_result,
             (long)diag->set_ap_config_result,
             (long)diag->wifi_start_result,
             (long)diag->get_mode_result,
             diag->observed_mode,
             (long)diag->get_channel_result,
             diag->primary_channel,
             (long)diag->get_ap_config_result,
             diag->ap_ssid,
             diag->usb_vbus_present,
             diag->usb_phy_enabled,
             (unsigned long)diag->free_heap);
}

static void wifi_diag_save_locked(void)
{
    wifi_diag_history.records[wifi_diag_slot] = wifi_diag;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_DIAG_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        err = nvs_set_blob(handle, WIFI_DIAG_KEY, &wifi_diag_history, sizeof(wifi_diag_history));
        if (err == ESP_OK)
            err = nvs_commit(handle);
        nvs_close(handle);
    }
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Could not save WiFi boot diagnostic: %s", esp_err_to_name(err));
}

static void wifi_diag_begin(void)
{
    wifi_diag_mutex = xSemaphoreCreateMutex();
    if (wifi_diag_mutex == NULL)
    {
        ESP_LOGW(TAG, "Could not allocate WiFi diagnostic mutex");
        return;
    }

    size_t length = sizeof(wifi_diag_history);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_DIAG_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        err = nvs_get_blob(handle, WIFI_DIAG_KEY, &wifi_diag_history, &length);
        nvs_close(handle);
    }
    if (err == ESP_OK && length == sizeof(wifi_diag_history) && wifi_diag_history_valid(&wifi_diag_history))
    {
        for (unsigned i = 0; i < WIFI_DIAG_HISTORY_COUNT; ++i)
        {
            if (wifi_diag_valid(&wifi_diag_history.records[i]))
                wifi_diag_log_previous(&wifi_diag_history.records[i]);
        }
    }
    else
    {
        if (err != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGW(TAG, "Could not read previous WiFi boot diagnostics: %s", esp_err_to_name(err));
        memset(&wifi_diag_history, 0, sizeof(wifi_diag_history));
        wifi_diag_history.magic = WIFI_DIAG_HISTORY_MAGIC;
        wifi_diag_history.version = WIFI_DIAG_VERSION;
    }

    uint32_t highest_boot_number = 0U;
    for (unsigned i = 0; i < WIFI_DIAG_HISTORY_COUNT; ++i)
    {
        if (wifi_diag_valid(&wifi_diag_history.records[i]) &&
            wifi_diag_history.records[i].boot_number > highest_boot_number)
            highest_boot_number = wifi_diag_history.records[i].boot_number;
    }
    wifi_diag_slot = wifi_diag_history.next_slot;
    wifi_diag_history.next_slot = (wifi_diag_slot + 1U) % WIFI_DIAG_HISTORY_COUNT;

    memset(&wifi_diag, 0, sizeof(wifi_diag));
    wifi_diag.magic = WIFI_DIAG_MAGIC;
    wifi_diag.version = WIFI_DIAG_VERSION;
    wifi_diag.boot_number = highest_boot_number + 1U;
    wifi_diag.reset_reason = (uint32_t)esp_reset_reason();
    wifi_diag.espnow_enabled = board_cfg.espnow_enabled;
    wifi_diag.usb_vbus_present = usb_vbus_present;
    wifi_diag.usb_phy_enabled = usb_phy_enabled;
    wifi_diag.set_mode_result = WIFI_DIAG_RESULT_NOT_RUN;
    wifi_diag.set_ap_config_result = WIFI_DIAG_RESULT_NOT_RUN;
    wifi_diag.wifi_start_result = WIFI_DIAG_RESULT_NOT_RUN;
    wifi_diag.get_mode_result = WIFI_DIAG_RESULT_NOT_RUN;
    wifi_diag.get_channel_result = WIFI_DIAG_RESULT_NOT_RUN;
    wifi_diag.get_ap_config_result = WIFI_DIAG_RESULT_NOT_RUN;
    wifi_diag_save_locked();
}

static void wifi_diag_record_start(int32_t mode_result, int32_t config_result,
                                   int32_t start_result)
{
    if (wifi_diag_mutex == NULL || xSemaphoreTake(wifi_diag_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return;

    wifi_diag.set_mode_result = mode_result;
    wifi_diag.set_ap_config_result = config_result;
    wifi_diag.wifi_start_result = start_result;
    wifi_diag.free_heap = esp_get_free_heap_size();
    wifi_diag.usb_vbus_present = usb_vbus_present;
    wifi_diag.usb_phy_enabled = usb_phy_enabled;

    wifi_mode_t mode = WIFI_MODE_NULL;
    wifi_diag.get_mode_result = esp_wifi_get_mode(&mode);
    if (wifi_diag.get_mode_result == ESP_OK)
        wifi_diag.observed_mode = (uint8_t)mode;

    wifi_second_chan_t secondary_channel = WIFI_SECOND_CHAN_NONE;
    wifi_diag.get_channel_result = esp_wifi_get_channel(&wifi_diag.primary_channel, &secondary_channel);

    wifi_config_t ap_config = {0};
    wifi_diag.get_ap_config_result = esp_wifi_get_config(WIFI_IF_AP, &ap_config);
    if (wifi_diag.get_ap_config_result == ESP_OK)
    {
        wifi_diag.ap_ssid_len = ap_config.ap.ssid_len;
        if (wifi_diag.ap_ssid_len > 32U)
            wifi_diag.ap_ssid_len = 32U;
        memcpy(wifi_diag.ap_ssid, ap_config.ap.ssid, wifi_diag.ap_ssid_len);
        wifi_diag.ap_ssid[wifi_diag.ap_ssid_len] = '\0';
    }
    wifi_diag_save_locked();
    xSemaphoreGive(wifi_diag_mutex);
}

static void wifi_diag_record_event(uint32_t event_bit)
{
    if (wifi_diag_mutex == NULL || xSemaphoreTake(wifi_diag_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return;
    wifi_diag.event_bits |= event_bit;
    wifi_diag.free_heap = esp_get_free_heap_size();
    wifi_diag_save_locked();
    xSemaphoreGive(wifi_diag_mutex);
}

static void configure_usb_phy_policy(void)
{
    const gpio_config_t detect_config = {
        .pin_bit_mask = 1ULL << USB_PRESENCE_DETECT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&detect_config);
    if (err != ESP_OK)
    {
        /* VBUS is diagnostic only; USB PHY is disabled at build time so WiFi
         * calibration never shares its BBPLL with an unused USB peripheral. */
        usb_vbus_present = true;
        ESP_LOGW(TAG, "Could not configure USB VBUS detection: %s",
                 esp_err_to_name(err));
        return;
    }

    usb_vbus_present = gpio_get_level(USB_PRESENCE_DETECT) != 0;
    ESP_LOGI(TAG, "USB VBUS %s on GPIO %d", usb_vbus_present ? "present" : "absent",
             USB_PRESENCE_DETECT);
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
 * @brief Try to connect the STA interface while leaving the configuration AP active.
 */
static bool try_preferred_station(void)
{
    if (!scan_for_preferred_ap())
    {
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
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret == ESP_OK)
    {
        ret = esp_wifi_connect();
    }
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Could not connect to %s: %s", PREFERRED_WIFI_SSID, esp_err_to_name(ret));
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
    return false;
}

/**
 * @brief Start the local configuration AP without allocating the HTTP server.
 */
static void start_persistent_ap(void)
{
    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = WIFI_MAX_CONN,
            .channel = ESPNOW_WIFI_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK}};

    esp_err_t mode_result = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (mode_result != ESP_OK)
    {
        wifi_diag_record_start(mode_result, WIFI_DIAG_RESULT_NOT_RUN, WIFI_DIAG_RESULT_NOT_RUN);
        ESP_ERROR_CHECK(mode_result);
    }

    esp_err_t config_result = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (config_result != ESP_OK)
    {
        wifi_diag_record_start(mode_result, config_result, WIFI_DIAG_RESULT_NOT_RUN);
        ESP_ERROR_CHECK(config_result);
    }

    esp_err_t start_result = esp_wifi_start();
    wifi_diag_record_start(mode_result, config_result, start_result);
    ESP_ERROR_CHECK(start_result);
    ESP_LOGI(TAG, "Persistent WiFi AP+STA started: %s on channel %d", WIFI_SSID, ESPNOW_WIFI_CHANNEL);
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

    if (event_id == WIFI_EVENT_AP_START)
    {
        ESP_LOGI(TAG, "Configuration AP start event received");
        wifi_diag_record_event(WIFI_DIAG_EVENT_AP_START);
    }
    else if (event_id == WIFI_EVENT_AP_STOP)
    {
        ESP_LOGW(TAG, "Configuration AP stop event received");
        wifi_diag_record_event(WIFI_DIAG_EVENT_AP_STOP);
    }
    else if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station: %02x:%02x:%02x:%02x:%02x:%02x join, AID=%d, bgn, 40U",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5], evt->aid);
        start_http_server();
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station: %02x:%02x:%02x:%02x:%02x:%02x leave, AID = %d, reason = %d, bss_flags is %u, bss:0x%p",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5], evt->aid, evt->reason, 0, event_data);
        stop_http_server();
    }
}

/**
 * @brief Keep the configuration AP active and optionally connect to preferred WiFi.
 * The HTTP server is allocated only while a station is associated with the AP.
 */
void wifi_config_mode_start(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    configure_usb_phy_policy();
    wifi_diag_begin();
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

    if (board_cfg.espnow_enabled)
    {
        ESP_LOGI(TAG, "ESP-NOW enabled; keeping WiFi on fixed channel %d", ESPNOW_WIFI_CHANNEL);
    }
    else if (try_preferred_station())
    {
        ESP_LOGI(TAG, "Configuration AP remains available while connected to %s", PREFERRED_WIFI_SSID);
    }
}

/**
 * @brief Compatibility hook retained for existing HTTP handlers.
 */
void notify_client_connected(void)
{
    /* HTTPD lifetime now follows AP station association. */
}
