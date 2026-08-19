#include "inc/espnow_transport.h"

#include <string.h>

#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "inc/config.h"

extern board_config_t board_cfg;

static const char *TAG = "ESPNOW";
static bool espnow_started = false;
static uint8_t active_peer[ESP_NOW_ETH_ALEN] = {0};
static SemaphoreHandle_t send_done_sem = NULL;

#define ESPNOW_SEND_WAIT_MS 10
#define ESPNOW_SEND_RETRY_COUNT 3

static bool mac_is_empty(const uint8_t mac[ESP_NOW_ETH_ALEN])
{
    uint8_t zero[ESP_NOW_ETH_ALEN] = {0};
    return memcmp(mac, zero, ESP_NOW_ETH_ALEN) == 0;
}

static void espnow_receive_callback(const esp_now_recv_info_t *info, const uint8_t *data, int data_len)
{
    if (data_len == (int)sizeof(twai_message_t))
    {
        const uint8_t *src = (info && info->src_addr) ? info->src_addr : NULL;
        if (src)
        {
            ESP_LOGD(TAG, "Received TWAI-sized ESP-NOW packet from %02X:%02X:%02X:%02X:%02X:%02X",
                     src[0], src[1], src[2], src[3], src[4], src[5]);
        }
    }
    else
    {
        ESP_LOGD(TAG, "Ignored ESP-NOW packet length %d", data_len);
    }
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;

    if (send_done_sem != NULL)
    {
        xSemaphoreGive(send_done_sem);
    }
}

esp_err_t espnow_transport_start(void)
{
    if (!board_cfg.espnow_enabled)
    {
        return ESP_OK;
    }

    if (mac_is_empty(board_cfg.espnow_target_mac))
    {
        ESP_LOGW(TAG, "ESP-NOW enabled but target MAC is empty");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set ESP-NOW WiFi channel %d: %s",
                 ESPNOW_WIFI_CHANNEL, esp_err_to_name(err));
        return err;
    }

    uint8_t primary_channel = 0;
    wifi_second_chan_t secondary_channel = WIFI_SECOND_CHAN_NONE;
    err = esp_wifi_get_channel(&primary_channel, &secondary_channel);
    if (err != ESP_OK || primary_channel != ESPNOW_WIFI_CHANNEL)
    {
        ESP_LOGE(TAG, "ESP-NOW WiFi channel verification failed: expected=%d actual=%u error=%s",
                 ESPNOW_WIFI_CHANNEL, (unsigned)primary_channel, esp_err_to_name(err));
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }
    ESP_LOGI(TAG, "ESP-NOW WiFi channel fixed to %u", (unsigned)primary_channel);

    if (!espnow_started)
    {
        if (send_done_sem == NULL)
        {
            send_done_sem = xSemaphoreCreateBinary();
            if (send_done_sem == NULL)
            {
                ESP_LOGE(TAG, "Failed to create ESP-NOW send semaphore");
                return ESP_ERR_NO_MEM;
            }
            xSemaphoreGive(send_done_sem);
        }

        err = esp_now_init();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(err));
            return err;
        }

        err = esp_now_register_send_cb(espnow_send_callback);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register ESP-NOW send callback: %s", esp_err_to_name(err));
            esp_now_unregister_recv_cb();
            esp_now_deinit();
            return err;
        }

        err = esp_now_register_recv_cb(espnow_receive_callback);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register ESP-NOW receive callback: %s", esp_err_to_name(err));
            esp_now_unregister_send_cb();
            esp_now_deinit();
            return err;
        }

        espnow_started = true;
    }

    if (!mac_is_empty(active_peer) && memcmp(active_peer, board_cfg.espnow_target_mac, ESP_NOW_ETH_ALEN) != 0)
    {
        esp_now_del_peer(active_peer);
        memset(active_peer, 0, sizeof(active_peer));
    }

    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, board_cfg.espnow_target_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    err = esp_now_add_peer(&peer_info);
    if (err == ESP_ERR_ESPNOW_EXIST)
    {
        err = esp_now_mod_peer(&peer_info);
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add ESP-NOW peer %02X:%02X:%02X:%02X:%02X:%02X: %s",
                 peer_info.peer_addr[0], peer_info.peer_addr[1], peer_info.peer_addr[2],
                 peer_info.peer_addr[3], peer_info.peer_addr[4], peer_info.peer_addr[5],
                 esp_err_to_name(err));
        return err;
    }

    memcpy(active_peer, board_cfg.espnow_target_mac, sizeof(active_peer));
    ESP_LOGI(TAG, "ESP-NOW peer active: %02X:%02X:%02X:%02X:%02X:%02X",
             active_peer[0], active_peer[1], active_peer[2], active_peer[3], active_peer[4], active_peer[5]);
    return ESP_OK;
}

esp_err_t espnow_transport_stop(void)
{
    if (!espnow_started)
    {
        return ESP_OK;
    }

    if (!mac_is_empty(active_peer))
    {
        esp_now_del_peer(active_peer);
        memset(active_peer, 0, sizeof(active_peer));
    }

    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_err_t err = esp_now_deinit();
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to deinitialize ESP-NOW: %s", esp_err_to_name(err));
        return err;
    }

    espnow_started = false;
    if (send_done_sem != NULL)
    {
        xSemaphoreGive(send_done_sem);
    }
    ESP_LOGI(TAG, "ESP-NOW stopped");
    return ESP_OK;
}

esp_err_t espnow_transport_apply_config(void)
{
    if (board_cfg.espnow_enabled)
    {
        return espnow_transport_start();
    }
    return espnow_transport_stop();
}

esp_err_t espnow_transport_send_twai(const twai_message_t *message)
{
    if (!board_cfg.espnow_enabled || !espnow_started || message == NULL)
    {
        return ESP_OK;
    }

    if (send_done_sem == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(send_done_sem, pdMS_TO_TICKS(ESPNOW_SEND_WAIT_MS)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;
    for (int attempt = 0; attempt < ESPNOW_SEND_RETRY_COUNT; ++attempt)
    {
        err = esp_now_send(board_cfg.espnow_target_mac, (const uint8_t *)message, sizeof(*message));
        if (err == ESP_OK)
        {
            return ESP_OK;
        }
        if (err != ESP_ERR_ESPNOW_NO_MEM)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    xSemaphoreGive(send_done_sem);
    return err;
}

esp_err_t espnow_transport_try_relay_twai(const twai_message_t *message)
{
    if (!board_cfg.espnow_enabled || !board_cfg.can_relay_espnow_enabled ||
        !espnow_started || message == NULL || send_done_sem == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(send_done_sem, 0) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = esp_now_send(board_cfg.espnow_target_mac,
                                 (const uint8_t *)message,
                                 sizeof(*message));
    if (err != ESP_OK)
    {
        xSemaphoreGive(send_done_sem);
    }
    return err;
}
