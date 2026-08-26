#include <inttypes.h>
#include <stdio.h>

#include "driver/twai.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define CAN_RX_GPIO GPIO_NUM_4
#define CAN_TX_GPIO GPIO_NUM_5
#define STATS_INTERVAL_US 1000000LL

static void print_frame(const twai_message_t *message)
{
    printf("{\"type\":\"frame\",\"timestamp_us\":%" PRId64
           ",\"id\":%" PRIu32 ",\"extended\":%s,\"rtr\":%s,\"dlc\":%u,\"data\":[",
           esp_timer_get_time(), message->identifier,
           message->extd ? "true" : "false",
           message->rtr ? "true" : "false",
           (unsigned)message->data_length_code);
    for (unsigned index = 0; index < 8; ++index) {
        printf("%s%u", index == 0 ? "" : ",", (unsigned)message->data[index]);
    }
    printf("]}\n");
}

static void print_stats(uint32_t frames)
{
    twai_status_info_t status = {0};
    esp_err_t result = twai_get_status_info(&status);
    if (result != ESP_OK) {
        printf("{\"type\":\"stats\",\"timestamp_us\":%" PRId64
               ",\"frames\":%" PRIu32 ",\"status_error\":true}\n",
               esp_timer_get_time(), frames);
        return;
    }

    printf("{\"type\":\"stats\",\"timestamp_us\":%" PRId64
           ",\"frames\":%" PRIu32 ",\"queued\":%" PRIu32
           ",\"rx_missed\":%" PRIu32 ",\"rx_overrun\":%" PRIu32
           ",\"bus_errors\":%" PRIu32 ",\"state\":%d}\n",
           esp_timer_get_time(), frames, status.msgs_to_rx,
           status.rx_missed_count, status.rx_overrun_count,
           status.bus_error_count, (int)status.state);
}

void app_main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);

    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
    general.rx_queue_len = 256;
    general.tx_queue_len = 0;
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&general, &timing, &filter));
    ESP_ERROR_CHECK(twai_start());

    printf("{\"type\":\"start\",\"timestamp_us\":%" PRId64
           ",\"bitrate_kbps\":500,\"mode\":\"listen-only\","
           "\"rx_gpio\":4,\"tx_gpio\":5,\"rx_queue_depth\":256,"
           "\"filter\":\"accept-all\",\"driver\":\"legacy-twai\"}\n",
           esp_timer_get_time());

    uint32_t frames = 0;
    int64_t next_stats_us = esp_timer_get_time() + STATS_INTERVAL_US;
    while (true) {
        twai_message_t message = {0};
        if (twai_receive(&message, pdMS_TO_TICKS(20)) == ESP_OK) {
            print_frame(&message);
            frames++;
        }
        int64_t now_us = esp_timer_get_time();
        if (now_us >= next_stats_us) {
            print_stats(frames);
            next_stats_us = now_us + STATS_INTERVAL_US;
        }
    }
}
