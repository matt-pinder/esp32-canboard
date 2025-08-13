#include <string.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "inc/espnow.h"
#include "driver/twai.h"
#include "inc/can.h"
#include "inc/inputs.h"

void espnow_receive_callback(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    twai_message_t *rx_msg = (twai_message_t *)data;
}

void espnow_start(){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wifiConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifiConfig));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_now_init());
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, sensorboard_mac, ESP_NOW_ETH_ALEN);
    peerInfo.channel = 0;
    peerInfo.ifidx = ESP_IF_WIFI_STA;
    peerInfo.encrypt = false;
    
    if(esp_now_add_peer(&peerInfo) == ESP_OK)
        ESP_LOGI(espnow_log, "Peer Added: %s", peerInfo.peer_addr);

    esp_now_register_recv_cb(espnow_receive_callback);
}

void espnow_transmit(void *arg){
    ESP_LOGI(espnow_log, "ESPNOW Transmit Task Started");

    // Message Setup - Forward CAN Packets
    twai_message_t espnow_76c = { .identifier = 0x76c, .data_length_code = 8 }; 
    memset(espnow_76c.data, 0, sizeof(espnow_76c.data));

    while(1){
        
        uint16_t voltages_copy[NUM_ADC_CHANNELS];
        if (xSemaphoreTake(filtered_voltages_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(voltages_copy, filtered_voltages, sizeof(voltages_copy));
            xSemaphoreGive(filtered_voltages_mutex);
        }

        uint16_t pressures_copy[4];
        if (xSemaphoreTake(scaled_pressures_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(pressures_copy, scaled_pressures, sizeof(pressures_copy));
            xSemaphoreGive(scaled_pressures_mutex);
        }

        espnow_76c.data[0] = voltages_copy[0] & 0xFF;
        espnow_76c.data[1] = (voltages_copy[0] >> 8) & 0xFF;
        espnow_76c.data[2] = 0;
        espnow_76c.data[3] = 0;
        espnow_76c.data[4] = 0;
        espnow_76c.data[5] = 0;
        espnow_76c.data[6] = 0;
        espnow_76c.data[7] = 0;

        esp_now_send(sensorboard_mac, (uint8_t*)&espnow_76c, sizeof(espnow_76c));
        
        vTaskDelay(25);
    }
    vTaskDelete(NULL);
}