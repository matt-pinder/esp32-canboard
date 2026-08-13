#include "inc/dragy_gps.h"
#include "sdkconfig.h"

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "inc/ble_scan.h"
#include "inc/can.h"
#include "inc/config.h"

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED && CONFIG_BT_NIMBLE_ROLE_CENTRAL
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"
#endif

#define TAG "DRAGY_GPS"
#define DRAGY_SCAN_MS 3000
#define DRAGY_RETRY_MIN_MS 10000
#define DRAGY_RETRY_MAX_MS 60000
#define DRAGY_CONNECT_TIMEOUT_MS 10000
#define DRAGY_STREAM_START_TIMEOUT_MS 8000
#define DRAGY_STREAM_STALL_MS 4000
#define DRAGY_BATTERY_READ_MS 60000
#define UBX_NAV_PVT_FRAME_LEN 100
#define UBX_BUFFER_LEN 256

extern board_config_t board_cfg;

typedef struct {
    bool enabled;
    uint32_t can_start_id;
    uint8_t target_mac[ESP_NOW_ETH_ALEN];
} dragy_config_t;

typedef struct {
    uint32_t i_tow_ms;
    int32_t longitude_1e7_deg;
    int32_t latitude_1e7_deg;
    int32_t altitude_msl_mm;
    uint32_t horizontal_accuracy_mm;
    uint32_t ground_speed_mm_s;
    int32_t heading_1e5_deg;
    uint8_t fix_type;
    uint8_t satellites;
    uint8_t ubx_flags;
    uint8_t battery_percent;
    bool fix_ok;
} dragy_sample_t;

static SemaphoreHandle_t config_mutex;
static QueueHandle_t sample_queue;
static TaskHandle_t dragy_task_handle;
static dragy_config_t runtime_config;
static volatile uint32_t config_revision;

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED && CONFIG_BT_NIMBLE_ROLE_CENTRAL
static const ble_uuid16_t dragy_service_uuid = BLE_UUID16_INIT(0xFD00);

static volatile bool ble_connected;
static volatile bool ble_connecting;
static volatile bool stream_ready;
static volatile bool battery_read_pending;
static volatile uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile TickType_t connected_at_tick;
static volatile TickType_t last_packet_tick;
static volatile TickType_t last_battery_read_tick;
static volatile uint8_t battery_percent = 0xFF;

static uint16_t service_start_handle;
static uint16_t service_end_handle;
static uint16_t fd02_def_handle;
static uint16_t fd02_value_handle;
static uint16_t fd02_cccd_handle;
static uint16_t fd03_value_handle;
static uint16_t fd04_value_handle;
static uint16_t next_def_after_fd02;

static uint8_t ubx_buffer[UBX_BUFFER_LEN];
static size_t ubx_buffer_used;

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int32_t read_i32_le(const uint8_t *data)
{
    return (int32_t)read_u32_le(data);
}

static bool ubx_checksum_valid(const uint8_t *frame, size_t length)
{
    if (length < 8) {
        return false;
    }

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < length - 2; ++i) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    return ck_a == frame[length - 2] && ck_b == frame[length - 1];
}

static bool parse_nav_pvt(const uint8_t frame[UBX_NAV_PVT_FRAME_LEN], dragy_sample_t *sample)
{
    if (frame[0] != 0xB5 || frame[1] != 0x62 || frame[2] != 0x01 || frame[3] != 0x07 ||
        read_u16_le(frame + 4) != 92 || !ubx_checksum_valid(frame, UBX_NAV_PVT_FRAME_LEN)) {
        return false;
    }

    const uint8_t *payload = frame + 6;
    int32_t signed_speed = read_i32_le(payload + 60);
    sample->i_tow_ms = read_u32_le(payload);
    sample->longitude_1e7_deg = read_i32_le(payload + 24);
    sample->latitude_1e7_deg = read_i32_le(payload + 28);
    sample->altitude_msl_mm = read_i32_le(payload + 36);
    sample->horizontal_accuracy_mm = read_u32_le(payload + 40);
    sample->ground_speed_mm_s = signed_speed > 0 ? (uint32_t)signed_speed : 0;
    sample->heading_1e5_deg = read_i32_le(payload + 64);
    sample->fix_type = payload[20];
    sample->ubx_flags = payload[21];
    sample->satellites = payload[23];
    sample->battery_percent = battery_percent;
    sample->fix_ok = (payload[21] & 0x01) != 0 && payload[20] >= 2 && payload[23] > 0;
    return true;
}

static void consume_ubx_bytes(const uint8_t *data, size_t length)
{
    if (length >= UBX_BUFFER_LEN) {
        data += length - UBX_BUFFER_LEN;
        length = UBX_BUFFER_LEN;
        ubx_buffer_used = 0;
    }

    if (ubx_buffer_used + length > UBX_BUFFER_LEN) {
        size_t discard = ubx_buffer_used + length - UBX_BUFFER_LEN;
        memmove(ubx_buffer, ubx_buffer + discard, ubx_buffer_used - discard);
        ubx_buffer_used -= discard;
    }
    memcpy(ubx_buffer + ubx_buffer_used, data, length);
    ubx_buffer_used += length;

    while (ubx_buffer_used >= 2) {
        size_t sync = 0;
        while (sync + 1 < ubx_buffer_used &&
               !(ubx_buffer[sync] == 0xB5 && ubx_buffer[sync + 1] == 0x62)) {
            ++sync;
        }
        if (sync > 0) {
            memmove(ubx_buffer, ubx_buffer + sync, ubx_buffer_used - sync);
            ubx_buffer_used -= sync;
        }
        if (ubx_buffer_used < 6) {
            return;
        }

        size_t frame_length = 6U + read_u16_le(ubx_buffer + 4) + 2U;
        if (frame_length > UBX_BUFFER_LEN) {
            memmove(ubx_buffer, ubx_buffer + 2, ubx_buffer_used - 2);
            ubx_buffer_used -= 2;
            continue;
        }
        if (ubx_buffer_used < frame_length) {
            return;
        }

        if (frame_length == UBX_NAV_PVT_FRAME_LEN) {
            dragy_sample_t sample = {0};
            if (parse_nav_pvt(ubx_buffer, &sample)) {
                last_packet_tick = xTaskGetTickCount();
                if (sample_queue != NULL) {
                    xQueueOverwrite(sample_queue, &sample);
                }
            }
        }

        memmove(ubx_buffer, ubx_buffer + frame_length, ubx_buffer_used - frame_length);
        ubx_buffer_used -= frame_length;
    }
}

static void reset_gatt_state(void)
{
    stream_ready = false;
    battery_read_pending = false;
    service_start_handle = 0;
    service_end_handle = 0;
    fd02_def_handle = 0;
    fd02_value_handle = 0;
    fd02_cccd_handle = 0;
    fd03_value_handle = 0;
    fd04_value_handle = 0;
    next_def_after_fd02 = 0;
    connected_at_tick = 0;
    last_packet_tick = 0;
    ubx_buffer_used = 0;
}

static void wake_dragy_task(void)
{
    if (dragy_task_handle != NULL) {
        xTaskNotifyGive(dragy_task_handle);
    }
}

static void terminate_connection(const char *reason)
{
    uint16_t handle = connection_handle;
    if (handle != BLE_HS_CONN_HANDLE_NONE) {
        stream_ready = false;
        ESP_LOGW(TAG, "%s; reconnecting", reason);
        ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

static int battery_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg)
{
    (void)arg;
    battery_read_pending = false;
    last_battery_read_tick = xTaskGetTickCount();

    if (conn_handle != connection_handle || error->status != 0 || attr == NULL || attr->om == NULL) {
        return 0;
    }

    uint16_t length = OS_MBUF_PKTLEN(attr->om);
    uint8_t status[2];
    if (length >= sizeof(status) && os_mbuf_copydata(attr->om, 0, sizeof(status), status) == 0 &&
        status[1] <= 100) {
        battery_percent = status[1];
        ESP_LOGI(TAG, "Dragy battery: %u%%", battery_percent);
    }
    return 0;
}

static void read_battery(void)
{
    if (fd04_value_handle == 0 || connection_handle == BLE_HS_CONN_HANDLE_NONE || battery_read_pending) {
        return;
    }
    battery_read_pending = true;
    int rc = ble_gattc_read(connection_handle, fd04_value_handle, battery_read_cb, NULL);
    if (rc != 0) {
        battery_read_pending = false;
        ESP_LOGD(TAG, "FD04 read could not start: rc=%d", rc);
    }
}

static int handshake_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    (void)arg;
    if (conn_handle != connection_handle) {
        return 0;
    }
    if (error->status != 0) {
        terminate_connection("FD03 handshake write failed");
        return 0;
    }

    stream_ready = true;
    connected_at_tick = xTaskGetTickCount();
    ESP_LOGI(TAG, "Dragy handshake complete; waiting for NAV-PVT data");
    read_battery();
    wake_dragy_task();
    return 0;
}

static int challenge_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr, void *arg)
{
    (void)arg;
    if (conn_handle != connection_handle) {
        return 0;
    }
    if (error->status != 0 || attr == NULL || attr->om == NULL || OS_MBUF_PKTLEN(attr->om) < 2) {
        terminate_connection("FD03 challenge read failed");
        return 0;
    }

    uint8_t challenge[2];
    if (os_mbuf_copydata(attr->om, 0, sizeof(challenge), challenge) != 0) {
        terminate_connection("FD03 challenge copy failed");
        return 0;
    }

    uint8_t response[4] = {
        challenge[0],
        challenge[1],
        (uint8_t)(challenge[0] ^ challenge[1]),
        (uint8_t)(challenge[0] & challenge[1]),
    };
    ESP_LOGI(TAG, "FD03 challenge %02X %02X; sending response", challenge[0], challenge[1]);
    int rc = ble_gattc_write_flat(conn_handle, fd03_value_handle, response, sizeof(response),
                                  handshake_write_cb, NULL);
    if (rc != 0) {
        terminate_connection("FD03 handshake write could not start");
    }
    return 0;
}

static int subscribe_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    (void)arg;
    if (conn_handle != connection_handle) {
        return 0;
    }
    if (error->status != 0) {
        terminate_connection("FD02 notification subscription failed");
        return 0;
    }

    int rc = ble_gattc_read(conn_handle, fd03_value_handle, challenge_read_cb, NULL);
    if (rc != 0) {
        terminate_connection("FD03 challenge read could not start");
    }
    return 0;
}

static int descriptor_discovery_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                   uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                                   void *arg)
{
    (void)chr_val_handle;
    (void)arg;
    if (conn_handle != connection_handle) {
        return 0;
    }

    if (error->status == 0 && dsc != NULL) {
        if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
            fd02_cccd_handle = dsc->handle;
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE || fd02_cccd_handle == 0) {
        terminate_connection("FD02 CCCD discovery failed");
        return 0;
    }

    const uint8_t notify_enable[2] = {0x01, 0x00};
    int rc = ble_gattc_write_flat(conn_handle, fd02_cccd_handle, notify_enable,
                                  sizeof(notify_enable), subscribe_write_cb, NULL);
    if (rc != 0) {
        terminate_connection("FD02 subscription write could not start");
    }
    return 0;
}

static int characteristic_discovery_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (conn_handle != connection_handle) {
        return 0;
    }

    if (error->status == 0 && chr != NULL) {
        uint16_t uuid = ble_uuid_u16(&chr->uuid.u);
        if (uuid == 0xFD02) {
            fd02_def_handle = chr->def_handle;
            fd02_value_handle = chr->val_handle;
        } else if (uuid == 0xFD03) {
            fd03_value_handle = chr->val_handle;
        } else if (uuid == 0xFD04) {
            fd04_value_handle = chr->val_handle;
        }
        if (fd02_def_handle != 0 && chr->def_handle > fd02_def_handle &&
            (next_def_after_fd02 == 0 || chr->def_handle < next_def_after_fd02)) {
            next_def_after_fd02 = chr->def_handle;
        }
        return 0;
    }

    if (error->status != BLE_HS_EDONE || fd02_value_handle == 0 || fd03_value_handle == 0) {
        terminate_connection("Required FD02/FD03 characteristics were not found");
        return 0;
    }

    uint16_t descriptor_end = next_def_after_fd02 > fd02_value_handle
                                ? (uint16_t)(next_def_after_fd02 - 1)
                                : service_end_handle;
    int rc = ble_gattc_disc_all_dscs(conn_handle, fd02_value_handle, descriptor_end,
                                     descriptor_discovery_cb, NULL);
    if (rc != 0) {
        terminate_connection("FD02 descriptor discovery could not start");
    }
    return 0;
}

static int service_discovery_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                const struct ble_gatt_svc *service, void *arg)
{
    (void)arg;
    if (conn_handle != connection_handle) {
        return 0;
    }

    if (error->status == 0 && service != NULL) {
        service_start_handle = service->start_handle;
        service_end_handle = service->end_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE || service_start_handle == 0) {
        terminate_connection("Dragy FD00 service was not found");
        return 0;
    }

    int rc = ble_gattc_disc_all_chrs(conn_handle, service_start_handle, service_end_handle,
                                     characteristic_discovery_cb, NULL);
    if (rc != 0) {
        terminate_connection("Dragy characteristic discovery could not start");
    }
    return 0;
}

static int dragy_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ble_connecting = false;
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "Dragy connection failed: status=%d", event->connect.status);
            ble_connected = false;
            connection_handle = BLE_HS_CONN_HANDLE_NONE;
            reset_gatt_state();
            wake_dragy_task();
            return 0;
        }

        connection_handle = event->connect.conn_handle;
        ble_connected = true;
        reset_gatt_state();
        ESP_LOGI(TAG, "Connected to Dragy; discovering FD00 service");
        if (ble_gattc_disc_svc_by_uuid(connection_handle, &dragy_service_uuid.u,
                                      service_discovery_cb, NULL) != 0) {
            terminate_connection("FD00 service discovery could not start");
        }
        wake_dragy_task();
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Dragy disconnected: reason=%d", event->disconnect.reason);
        ble_connected = false;
        ble_connecting = false;
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        reset_gatt_state();
        if (sample_queue != NULL) {
            xQueueReset(sample_queue);
        }
        wake_dragy_task();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.conn_handle == connection_handle &&
            event->notify_rx.attr_handle == fd02_value_handle && event->notify_rx.om != NULL) {
            uint16_t length = OS_MBUF_PKTLEN(event->notify_rx.om);
            uint8_t fragment[256];
            if (length <= sizeof(fragment) &&
                os_mbuf_copydata(event->notify_rx.om, 0, length, fragment) == 0) {
                consume_ubx_bytes(fragment, length);
            }
        }
        return 0;

    default:
        return 0;
    }
}

static void make_nimble_address(const ble_scan_target_t *target, ble_addr_t *address)
{
    address->type = target->addr_type;
    for (int i = 0; i < ESP_NOW_ETH_ALEN; ++i) {
        address->val[i] = target->mac[ESP_NOW_ETH_ALEN - 1 - i];
    }
}
#endif

static void get_runtime_config(dragy_config_t *config)
{
    if (config_mutex != NULL && xSemaphoreTake(config_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *config = runtime_config;
        xSemaphoreGive(config_mutex);
    } else {
        memset(config, 0, sizeof(*config));
    }
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 24) & 0xFF;
}

static void publish_status_frame(const dragy_sample_t *sample, uint32_t base_id)
{
    twai_message_t status = init_twai_message(base_id);
    status.data[0] = sample->fix_type;
    status.data[1] = sample->ubx_flags;
    status.data[2] = sample->satellites;
    status.data[3] = sample->battery_percent;
    write_u32_le(&status.data[4], sample->horizontal_accuracy_mm);
    can_transmit_frame(&status, "Dragy GPS status");
}

static void publish_sample(const dragy_sample_t *sample, uint32_t base_id)
{
    publish_status_frame(sample, base_id);

    twai_message_t motion = init_twai_message(base_id + 1);
    write_u32_le(&motion.data[0], sample->ground_speed_mm_s);
    write_u32_le(&motion.data[4], (uint32_t)sample->heading_1e5_deg);
    can_transmit_frame(&motion, "Dragy GPS motion");

    twai_message_t position = init_twai_message(base_id + 2);
    write_u32_le(&position.data[0], (uint32_t)sample->latitude_1e7_deg);
    write_u32_le(&position.data[4], (uint32_t)sample->longitude_1e7_deg);
    can_transmit_frame(&position, "Dragy GPS position");

    twai_message_t altitude_time = init_twai_message(base_id + 3);
    write_u32_le(&altitude_time.data[0], (uint32_t)sample->altitude_msl_mm);
    write_u32_le(&altitude_time.data[4], sample->i_tow_ms);
    can_transmit_frame(&altitude_time, "Dragy GPS altitude/time");
}

static void dragy_task(void *arg)
{
    (void)arg;
#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED && CONFIG_BT_NIMBLE_ROLE_CENTRAL
    dragy_config_t active_connection_config = {0};
    uint32_t active_config_revision = 0;
    TickType_t next_connect_attempt = 0;
    uint32_t retry_delay_ms = DRAGY_RETRY_MIN_MS;
    TickType_t last_no_fix_status = 0;

    while (true) {
        dragy_config_t config;
        get_runtime_config(&config);
        TickType_t now = xTaskGetTickCount();
        uint32_t current_config_revision = config_revision;
        if (active_config_revision != current_config_revision) {
            active_config_revision = current_config_revision;
            next_connect_attempt = 0;
            retry_delay_ms = DRAGY_RETRY_MIN_MS;
        }

        bool connection_config_changed =
            active_connection_config.enabled != config.enabled ||
            memcmp(active_connection_config.target_mac, config.target_mac, ESP_NOW_ETH_ALEN) != 0;
        if (connection_config_changed && (ble_connected || ble_connecting)) {
            if (ble_connected) {
                terminate_connection("GPS configuration changed");
            } else {
                ble_gap_conn_cancel();
                ble_connecting = false;
            }
            active_connection_config = config;
        }

        if (!config.enabled) {
            active_connection_config = config;
            xQueueReset(sample_queue);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            continue;
        }

        if (!ble_connected && !ble_connecting && now >= next_connect_attempt) {
            ble_scan_target_t target;
            esp_err_t scan_err = ble_scan_find_target(config.target_mac, DRAGY_SCAN_MS, &target);
            get_runtime_config(&config);
            if (!config.enabled) {
                continue;
            }
            if (scan_err == ESP_OK && memcmp(target.mac, config.target_mac, ESP_NOW_ETH_ALEN) == 0) {
                uint8_t own_addr_type;
                ble_addr_t address;
                make_nimble_address(&target, &address);
                int rc = ble_hs_id_infer_auto(0, &own_addr_type);
                if (rc == 0) {
                    reset_gatt_state();
                    battery_percent = 0xFF;
                    ble_connecting = true;
                    active_connection_config = config;
                    ESP_LOGI(TAG, "Connecting to Dragy %02X:%02X:%02X:%02X:%02X:%02X",
                             config.target_mac[0], config.target_mac[1], config.target_mac[2],
                             config.target_mac[3], config.target_mac[4], config.target_mac[5]);
                    rc = ble_gap_connect(own_addr_type, &address, DRAGY_CONNECT_TIMEOUT_MS,
                                         NULL, dragy_gap_event, NULL);
                    if (rc != 0) {
                        ble_connecting = false;
                        ESP_LOGW(TAG, "Dragy connection could not start: rc=%d", rc);
                    } else {
                        retry_delay_ms = DRAGY_RETRY_MIN_MS;
                    }
                }
            } else if (scan_err != ESP_ERR_NOT_FOUND && scan_err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "Dragy target scan failed: %s", esp_err_to_name(scan_err));
            }
            next_connect_attempt = xTaskGetTickCount() + pdMS_TO_TICKS(retry_delay_ms);
            if (scan_err != ESP_OK && retry_delay_ms < DRAGY_RETRY_MAX_MS) {
                retry_delay_ms *= 2;
                if (retry_delay_ms > DRAGY_RETRY_MAX_MS) {
                    retry_delay_ms = DRAGY_RETRY_MAX_MS;
                }
            }
        }

        if (ble_connected && stream_ready) {
            now = xTaskGetTickCount();
            if (last_packet_tick == 0 && connected_at_tick != 0 &&
                now - connected_at_tick > pdMS_TO_TICKS(DRAGY_STREAM_START_TIMEOUT_MS)) {
                terminate_connection("Dragy telemetry did not start");
            } else if (last_packet_tick != 0 &&
                       now - last_packet_tick > pdMS_TO_TICKS(DRAGY_STREAM_STALL_MS)) {
                terminate_connection("Dragy telemetry stalled");
            } else if (fd04_value_handle != 0 && !battery_read_pending &&
                       now - last_battery_read_tick > pdMS_TO_TICKS(DRAGY_BATTERY_READ_MS)) {
                read_battery();
            }
        }

        dragy_sample_t sample;
        if (xQueueReceive(sample_queue, &sample, pdMS_TO_TICKS(100)) == pdTRUE) {
            get_runtime_config(&config);
            if (config.enabled) {
                // ESP_LOGI("dragy", "i_tow_ms: %"PRIu32",longitude_1e7_deg: %"PRIu32",
                //     latitude_1e7_deg: %"PRIu32",
                //     altitude_msl_mm: %"PRIu32",
                //     horizontal_accuracy_mm: %"PRIu32",
                //     ground_speed_mm_s: %"PRIu32",
                //     heading_1e5_deg: %"PRIu32",
                //     fix_type: %d,
                //     ubx_flags: %d,
                //     satellites: %d,
                //     battery_percent: %d,
                //     fix_ok: %s", sample->i_tow_ms, sample->longitude_1e7_deg, sample->latitude_1e7_deg, sample->altitude_msl_mm, sample->horizontal_accuracy_mm, sample->ground_speed_mm_s, sample->heading_1e5_deg, sample->fix_type, sample->ubx_flags, sample->satellites, sample->battery_percent, sample->fix_ok);
                if (sample.fix_ok)
                {
                    publish_sample(&sample, config.can_start_id);
                }
                else
                {
                    now = xTaskGetTickCount();
                    if (last_no_fix_status == 0 || now - last_no_fix_status >= pdMS_TO_TICKS(1000)) {
                        publish_status_frame(&sample, config.can_start_id);
                        last_no_fix_status = now;
                    }
                }
            }
        }
    }
#else
    ESP_LOGW(TAG, "Dragy GPS requires NimBLE central support");
    vTaskDelete(NULL);
#endif
}

void dragy_gps_apply_config(void)
{
    if (config_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        runtime_config.enabled = board_cfg.gps_enabled;
        runtime_config.can_start_id = board_cfg.gps_can_start_id;
        memcpy(runtime_config.target_mac, board_cfg.gps_target_mac, ESP_NOW_ETH_ALEN);
        ++config_revision;
        xSemaphoreGive(config_mutex);
    }

    if (dragy_task_handle != NULL) {
        xTaskNotifyGive(dragy_task_handle);
    }
}

esp_err_t dragy_gps_start(void)
{
    if (dragy_task_handle != NULL) {
        dragy_gps_apply_config();
        return ESP_OK;
    }

    config_mutex = xSemaphoreCreateMutex();
    sample_queue = xQueueCreate(1, sizeof(dragy_sample_t));
    if (config_mutex == NULL || sample_queue == NULL) {
        ESP_LOGE(TAG, "Failed to allocate Dragy GPS synchronization objects");
        return ESP_ERR_NO_MEM;
    }

    dragy_gps_apply_config();
    if (xTaskCreatePinnedToCore(dragy_task, "dragyGps", 7168, NULL, 6,
                                &dragy_task_handle, 0) != pdPASS) {
        dragy_task_handle = NULL;
        ESP_LOGE(TAG, "Failed to create Dragy GPS task");
        return ESP_ERR_NO_MEM;
    }

    dragy_gps_apply_config();
    ESP_LOGI(TAG, "Dragy GPS task started");
    return ESP_OK;
}
