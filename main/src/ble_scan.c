#include "inc/ble_scan.h"
#include "sdkconfig.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#endif

#define TAG "BLE_SCAN"
#define DRAGY_SERVICE_UUID 0xFD00
#define SCAN_DONE_BIT BIT0
#define SCAN_READY_BIT BIT1

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
static SemaphoreHandle_t scan_mutex;
static EventGroupHandle_t scan_events;
static ble_scan_result_t scan_results[BLE_SCAN_MAX_RESULTS];
static size_t scan_result_count;
static bool nimble_started;
static bool target_scan_active;
static bool target_scan_found;
static uint8_t target_scan_mac[ESP_NOW_ETH_ALEN];
static ble_scan_target_t target_scan_result;

static void mac_from_nimble(const uint8_t in[ESP_NOW_ETH_ALEN], uint8_t out[ESP_NOW_ETH_ALEN])
{
    for (int i = 0; i < ESP_NOW_ETH_ALEN; ++i) {
        out[i] = in[ESP_NOW_ETH_ALEN - 1 - i];
    }
}

static bool name_matches_dragy(const char *name)
{
    if (name == NULL || name[0] == '\0') {
        return false;
    }

    return (strncasecmp(name, "DRG", 3) == 0) ||
           (strncasecmp(name, "DRAGY", 5) == 0);
}

static bool fields_have_fd00(const struct ble_hs_adv_fields *fields)
{
    for (int i = 0; i < fields->num_uuids16; ++i) {
        if (ble_uuid_u16(&fields->uuids16[i].u) == DRAGY_SERVICE_UUID) {
            return true;
        }
    }

    if (fields->svc_data_uuid16_len >= 2 &&
        fields->svc_data_uuid16[0] == 0x00 &&
        fields->svc_data_uuid16[1] == 0xFD) {
        return true;
    }

    return false;
}

static ble_scan_result_t *find_or_add_result(const uint8_t mac[ESP_NOW_ETH_ALEN])
{
    for (size_t i = 0; i < scan_result_count; ++i) {
        if (memcmp(scan_results[i].mac, mac, ESP_NOW_ETH_ALEN) == 0) {
            return &scan_results[i];
        }
    }

    if (scan_result_count >= BLE_SCAN_MAX_RESULTS) {
        return NULL;
    }

    ble_scan_result_t *result = &scan_results[scan_result_count++];
    memset(result, 0, sizeof(*result));
    memcpy(result->mac, mac, ESP_NOW_ETH_ALEN);
    result->rssi = -127;
    return result;
}

static void record_disc_result(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields = {0};
    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0) {
        return;
    }

    uint8_t mac[ESP_NOW_ETH_ALEN];
    mac_from_nimble(disc->addr.val, mac);

    if (target_scan_active && memcmp(mac, target_scan_mac, ESP_NOW_ETH_ALEN) == 0) {
        memcpy(target_scan_result.mac, mac, ESP_NOW_ETH_ALEN);
        target_scan_result.addr_type = disc->addr.type;
        target_scan_found = true;
    }

    ble_scan_result_t *result = find_or_add_result(mac);
    if (result == NULL) {
        return;
    }

    result->rssi = disc->rssi;
    result->addr_type = disc->addr.type;

    if (fields.name != NULL && fields.name_len > 0) {
        size_t copy_len = fields.name_len;
        if (copy_len >= sizeof(result->name)) {
            copy_len = sizeof(result->name) - 1;
        }
        memcpy(result->name, fields.name, copy_len);
        for (size_t i = 0; i < copy_len; ++i) {
            if ((uint8_t)result->name[i] < 0x20 || result->name[i] == '"' || result->name[i] == '\\') {
                result->name[i] = ' ';
            }
        }
        result->name[copy_len] = '\0';
    }

    result->has_dragy_name = name_matches_dragy(result->name);
    result->has_fd00_service = fields_have_fd00(&fields);
    result->potential_dragy = result->has_dragy_name || result->has_fd00_service;
}

static int scan_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        record_disc_result(&event->disc);
        return 0;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        xEventGroupSetBits(scan_events, SCAN_DONE_BIT);
        return 0;
    default:
        return 0;
    }
}

static void ble_on_sync(void)
{
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer BLE own address type: rc=%d", rc);
        return;
    }
    xEventGroupSetBits(scan_events, SCAN_READY_BIT);
    ESP_LOGI(TAG, "NimBLE ready for BLE scans");
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset: reason=%d", reason);
    if (scan_events != NULL) {
        xEventGroupClearBits(scan_events, SCAN_READY_BIT);
        xEventGroupSetBits(scan_events, SCAN_DONE_BIT);
    }
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_scan_init(void)
{
    if (nimble_started) {
        return ESP_OK;
    }

    scan_mutex = xSemaphoreCreateMutex();
    scan_events = xEventGroupCreate();
    if (scan_mutex == NULL || scan_events == NULL) {
        ESP_LOGE(TAG, "Failed to allocate BLE scan synchronization objects");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    ble_svc_gap_init();
    int rc = ble_svc_gap_device_name_set("esp32-canboard");
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to set BLE GAP name: rc=%d", rc);
    }
#endif

    nimble_port_freertos_init(ble_host_task);
    nimble_started = true;
    return ESP_OK;
}

esp_err_t ble_scan_dragy(uint32_t duration_ms,
                         ble_scan_result_t *results,
                         size_t max_results,
                         size_t *result_count)
{
    if (results == NULL || result_count == NULL || max_results == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *result_count = 0;

    esp_err_t err = ble_scan_init();
    if (err != ESP_OK) {
        return err;
    }

    EventBits_t ready = xEventGroupWaitBits(scan_events, SCAN_READY_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(1000));
    if ((ready & SCAN_READY_BIT) == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memset(scan_results, 0, sizeof(scan_results));
    scan_result_count = 0;
    target_scan_active = false;
    xEventGroupClearBits(scan_events, SCAN_DONE_BIT);

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        xSemaphoreGive(scan_mutex);
        return ESP_FAIL;
    }

    struct ble_gap_disc_params disc_params = {0};
    disc_params.filter_duplicates = 1;
    disc_params.passive = 0;

    rc = ble_gap_disc(own_addr_type, duration_ms, &disc_params, scan_gap_event, NULL);
    if (rc != 0) {
        xSemaphoreGive(scan_mutex);
        ESP_LOGW(TAG, "Failed to start BLE discovery: rc=%d", rc);
        return ESP_FAIL;
    }

    EventBits_t bits = xEventGroupWaitBits(scan_events, SCAN_DONE_BIT, pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(duration_ms + 1000));
    if ((bits & SCAN_DONE_BIT) == 0) {
        ble_gap_disc_cancel();
        xEventGroupWaitBits(scan_events, SCAN_DONE_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(250));
    }

    size_t count = scan_result_count;
    if (count > max_results) {
        count = max_results;
    }
    memcpy(results, scan_results, count * sizeof(results[0]));
    *result_count = count;

    xSemaphoreGive(scan_mutex);
    return ESP_OK;
}

esp_err_t ble_scan_find_target(const uint8_t mac[ESP_NOW_ETH_ALEN],
                               uint32_t duration_ms,
                               ble_scan_target_t *target)
{
    if (mac == NULL || target == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ble_scan_init();
    if (err != ESP_OK) {
        return err;
    }

    EventBits_t ready = xEventGroupWaitBits(scan_events, SCAN_READY_BIT, pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(1000));
    if ((ready & SCAN_READY_BIT) == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memset(scan_results, 0, sizeof(scan_results));
    scan_result_count = 0;
    memcpy(target_scan_mac, mac, ESP_NOW_ETH_ALEN);
    memset(&target_scan_result, 0, sizeof(target_scan_result));
    target_scan_found = false;
    target_scan_active = true;
    xEventGroupClearBits(scan_events, SCAN_DONE_BIT);

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        target_scan_active = false;
        xSemaphoreGive(scan_mutex);
        return ESP_FAIL;
    }

    struct ble_gap_disc_params disc_params = {0};
    disc_params.filter_duplicates = 1;
    disc_params.passive = 0;

    rc = ble_gap_disc(own_addr_type, duration_ms, &disc_params, scan_gap_event, NULL);
    if (rc != 0) {
        target_scan_active = false;
        xSemaphoreGive(scan_mutex);
        ESP_LOGW(TAG, "Failed to start target discovery: rc=%d", rc);
        return ESP_FAIL;
    }

    EventBits_t bits = xEventGroupWaitBits(scan_events, SCAN_DONE_BIT, pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(duration_ms + 1000));
    if ((bits & SCAN_DONE_BIT) == 0) {
        ble_gap_disc_cancel();
        xEventGroupWaitBits(scan_events, SCAN_DONE_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(250));
    }

    target_scan_active = false;
    if (target_scan_found) {
        *target = target_scan_result;
        err = ESP_OK;
    } else {
        err = ESP_ERR_NOT_FOUND;
    }

    xSemaphoreGive(scan_mutex);
    return err;
}
#else
esp_err_t ble_scan_init(void)
{
    ESP_LOGW(TAG, "BLE scan requested but NimBLE is not enabled");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ble_scan_dragy(uint32_t duration_ms,
                         ble_scan_result_t *results,
                         size_t max_results,
                         size_t *result_count)
{
    (void)duration_ms;
    (void)results;
    (void)max_results;
    if (result_count != NULL) {
        *result_count = 0;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ble_scan_find_target(const uint8_t mac[ESP_NOW_ETH_ALEN],
                               uint32_t duration_ms,
                               ble_scan_target_t *target)
{
    (void)mac;
    (void)duration_ms;
    (void)target;
    return ESP_ERR_NOT_SUPPORTED;
}
#endif
