#pragma once
// Nige Screen e4:b0:63:ba:9e:f8
// Nige sensors  dc:da:0c:3b:b2:0c
static uint8_t nigescreen_mac[ESP_NOW_ETH_ALEN] = {0xE4, 0xB0, 0x63, 0xBA, 0x9E, 0xF8};
static uint8_t sensorboard_mac[ESP_NOW_ETH_ALEN] = {0xDC, 0xDA, 0x0C, 0x3B, 0xB2, 0x0C};

static const char* espnow_log = "espnow";

void espnow_start();
void espnow_transmit(void *arg);
void espnow_receive_callback(const esp_now_recv_info_t *info, const uint8_t *data, int len);