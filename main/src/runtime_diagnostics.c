#include "inc/runtime_diagnostics.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_crc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "inc/espnow_transport.h"
#include "nvs.h"

#define DIAGNOSTICS_NVS_PARTITION "config"
#define DIAGNOSTICS_NVS_NAMESPACE "diagnostics"
#define DIAGNOSTICS_NVS_KEY       "runtime"
#define DIAGNOSTICS_MAGIC         0x43424447UL
#define DIAGNOSTICS_VERSION       1U
#define SNAPSHOT_INTERVAL_MS      1000U
#define INCIDENT_WRITE_GUARD_MS   60000U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t boot_count;
    uint32_t uptime_ms;
    uint32_t free_heap_bytes;
    uint32_t min_free_heap_bytes;
    uint32_t espnow_queue_depth;
    uint32_t espnow_queue_drops;
    uint32_t espnow_send_failures;
    uint32_t espnow_callback_timeouts;
    uint32_t crc32;
} diagnostics_rtc_snapshot_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t boot_count;
    uint32_t fault_reset_count;
    uint32_t last_reset_reason;
    bool last_fault_available;
    uint8_t fault_reserved[3];
    uint32_t last_fault_boot_count;
    uint32_t last_fault_reset_reason;
    bool previous_snapshot_available;
    diagnostics_rtc_snapshot_t previous_snapshot;
    bool incident_available;
    char incident[64];
    uint32_t incident_boot_count;
    uint32_t incident_uptime_ms;
    uint8_t incident_client_index;
    uint8_t reserved[3];
    uint32_t incident_pending_ms;
    uint32_t incident_queue_depth;
    uint32_t incident_queue_drops;
    uint32_t incident_send_failures;
} diagnostics_record_t;

static const char *TAG = "DIAGNOSTICS";
static RTC_NOINIT_ATTR diagnostics_rtc_snapshot_t rtc_snapshot;
static diagnostics_record_t record;
static SemaphoreHandle_t record_mutex;
static int64_t last_incident_write_us;

static uint32_t snapshot_crc(const diagnostics_rtc_snapshot_t *snapshot)
{
    return esp_crc32_le(0, (const uint8_t *)snapshot,
                        offsetof(diagnostics_rtc_snapshot_t, crc32));
}

static bool snapshot_valid(const diagnostics_rtc_snapshot_t *snapshot)
{
    return snapshot->magic == DIAGNOSTICS_MAGIC &&
           snapshot->version == DIAGNOSTICS_VERSION &&
           snapshot->size == sizeof(*snapshot) &&
           snapshot->crc32 == snapshot_crc(snapshot);
}

static bool reset_reason_is_fault(esp_reset_reason_t reason)
{
    return reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
           reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT ||
           reason == ESP_RST_BROWNOUT || reason == ESP_RST_PWR_GLITCH ||
           reason == ESP_RST_CPU_LOCKUP;
}

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON: return "power_on";
        case ESP_RST_EXT: return "external_reset";
        case ESP_RST_SW: return "software_restart";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt_watchdog";
        case ESP_RST_TASK_WDT: return "task_watchdog";
        case ESP_RST_WDT: return "other_watchdog";
        case ESP_RST_DEEPSLEEP: return "deep_sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        case ESP_RST_USB: return "usb";
        case ESP_RST_JTAG: return "jtag";
        case ESP_RST_EFUSE: return "efuse";
        case ESP_RST_PWR_GLITCH: return "power_glitch";
        case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
        default: return "unknown";
    }
}

static esp_err_t save_record(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open_from_partition(DIAGNOSTICS_NVS_PARTITION,
                                             DIAGNOSTICS_NVS_NAMESPACE,
                                             NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, DIAGNOSTICS_NVS_KEY, &record, sizeof(record));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static void load_record(void)
{
    diagnostics_record_t stored = {0};
    nvs_handle_t handle;
    esp_err_t err = nvs_open_from_partition(DIAGNOSTICS_NVS_PARTITION,
                                             DIAGNOSTICS_NVS_NAMESPACE,
                                             NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t size = sizeof(stored);
        err = nvs_get_blob(handle, DIAGNOSTICS_NVS_KEY, &stored, &size);
        nvs_close(handle);
        if (err == ESP_OK && size == sizeof(stored) &&
            stored.magic == DIAGNOSTICS_MAGIC &&
            stored.version == DIAGNOSTICS_VERSION &&
            stored.size == sizeof(stored)) {
            record = stored;
            return;
        }
    }
    memset(&record, 0, sizeof(record));
    record.magic = DIAGNOSTICS_MAGIC;
    record.version = DIAGNOSTICS_VERSION;
    record.size = sizeof(record);
}

static void update_rtc_snapshot(void)
{
    espnow_transport_status_t transport = {0};
    espnow_transport_get_status(&transport);
    diagnostics_rtc_snapshot_t next = {
        .magic = DIAGNOSTICS_MAGIC,
        .version = DIAGNOSTICS_VERSION,
        .size = sizeof(next),
        .boot_count = record.boot_count,
        .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
        .free_heap_bytes = esp_get_free_heap_size(),
        .min_free_heap_bytes = esp_get_minimum_free_heap_size(),
        .espnow_queue_depth = transport.queue_depth,
        .espnow_queue_drops = transport.queue_drops,
        .espnow_send_failures = transport.send_failures,
        .espnow_callback_timeouts = transport.callback_timeouts,
    };
    next.crc32 = snapshot_crc(&next);
    rtc_snapshot = next;
}

static void diagnostics_task(void *arg)
{
    (void)arg;
    while (true) {
        update_rtc_snapshot();
        vTaskDelay(pdMS_TO_TICKS(SNAPSHOT_INTERVAL_MS));
    }
}

esp_err_t runtime_diagnostics_init(void)
{
    const esp_reset_reason_t reason = esp_reset_reason();
    const bool have_previous_snapshot = snapshot_valid(&rtc_snapshot);
    diagnostics_rtc_snapshot_t previous_snapshot = rtc_snapshot;

    record_mutex = xSemaphoreCreateMutex();
    if (record_mutex == NULL) return ESP_ERR_NO_MEM;
    load_record();
    record.boot_count++;
    record.last_reset_reason = (uint32_t)reason;
    if (reset_reason_is_fault(reason)) {
        record.fault_reset_count++;
        record.last_fault_available = true;
        record.last_fault_boot_count = record.boot_count;
        record.last_fault_reset_reason = (uint32_t)reason;
    }
    if (have_previous_snapshot && reset_reason_is_fault(reason)) {
        record.previous_snapshot_available = true;
        record.previous_snapshot = previous_snapshot;
    }

    esp_err_t err = save_record();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not persist boot diagnostics: %s", esp_err_to_name(err));
    }
    update_rtc_snapshot();
    if (xTaskCreate(diagnostics_task, "diagnostics", 3072, NULL, 2, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Boot %lu reset_reason=%s previous_snapshot=%d",
             (unsigned long)record.boot_count, reset_reason_name(reason),
             have_previous_snapshot);
    return ESP_OK;
}

void runtime_diagnostics_get_status(runtime_diagnostics_status_t *status)
{
    if (status == NULL) return;
    memset(status, 0, sizeof(*status));
    if (record_mutex != NULL) xSemaphoreTake(record_mutex, portMAX_DELAY);
    status->boot_count = record.boot_count;
    status->fault_reset_count = record.fault_reset_count;
    status->current_reset_reason = (esp_reset_reason_t)record.last_reset_reason;
    status->current_reset_reason_name = reset_reason_name(status->current_reset_reason);
    status->last_fault_available = record.last_fault_available;
    status->last_fault_boot_count = record.last_fault_boot_count;
    status->last_fault_reset_reason = (esp_reset_reason_t)record.last_fault_reset_reason;
    status->last_fault_reset_reason_name = reset_reason_name(status->last_fault_reset_reason);
    status->previous_snapshot_available = record.previous_snapshot_available;
    if (record.previous_snapshot_available) {
        status->previous_uptime_ms = record.previous_snapshot.uptime_ms;
        status->previous_free_heap_bytes = record.previous_snapshot.free_heap_bytes;
        status->previous_min_free_heap_bytes = record.previous_snapshot.min_free_heap_bytes;
        status->previous_espnow_queue_depth = record.previous_snapshot.espnow_queue_depth;
        status->previous_espnow_queue_drops = record.previous_snapshot.espnow_queue_drops;
        status->previous_espnow_send_failures = record.previous_snapshot.espnow_send_failures;
        status->previous_espnow_callback_timeouts = record.previous_snapshot.espnow_callback_timeouts;
    }
    status->incident_available = record.incident_available;
    memcpy(status->incident, record.incident, sizeof(status->incident));
    status->incident[sizeof(status->incident) - 1U] = '\0';
    status->incident_boot_count = record.incident_boot_count;
    status->incident_uptime_ms = record.incident_uptime_ms;
    status->incident_client_index = record.incident_client_index;
    status->incident_pending_ms = record.incident_pending_ms;
    status->incident_queue_depth = record.incident_queue_depth;
    status->incident_queue_drops = record.incident_queue_drops;
    status->incident_send_failures = record.incident_send_failures;
    if (record_mutex != NULL) xSemaphoreGive(record_mutex);
}

void runtime_diagnostics_record_espnow_timeout(uint8_t client_index,
                                              uint32_t pending_ms,
                                              uint32_t queue_depth,
                                              uint32_t queue_drops,
                                              uint32_t send_failures)
{
    const int64_t now_us = esp_timer_get_time();
    if (last_incident_write_us != 0 &&
        (now_us - last_incident_write_us) < (int64_t)INCIDENT_WRITE_GUARD_MS * 1000LL) {
        return;
    }
    if (record_mutex != NULL) xSemaphoreTake(record_mutex, portMAX_DELAY);
    record.incident_available = true;
    snprintf(record.incident, sizeof(record.incident), "ESP-NOW send callback timeout");
    record.incident_boot_count = record.boot_count;
    record.incident_uptime_ms = (uint32_t)(now_us / 1000ULL);
    record.incident_client_index = client_index;
    record.incident_pending_ms = pending_ms;
    record.incident_queue_depth = queue_depth;
    record.incident_queue_drops = queue_drops;
    record.incident_send_failures = send_failures;
    esp_err_t err = save_record();
    if (record_mutex != NULL) xSemaphoreGive(record_mutex);
    if (err == ESP_OK) {
        last_incident_write_us = now_us;
    } else {
        ESP_LOGW(TAG, "Could not persist ESP-NOW timeout: %s", esp_err_to_name(err));
    }
}
