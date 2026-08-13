idf_component_register(SRCS "main.c"
                        "src/can.c"
                        "src/inputs.c"
                        "src/config.c"
                        "src/wifi_config.c"
                        "src/http_server.c"
                        "src/ble_scan.c"
                        "src/dragy_gps.c"
                        "src/espnow_transport.c"
                    INCLUDE_DIRS "."
                    "./src"
                    "./inc"
                    REQUIRES bt
                             driver
                             esp_adc
                             esp_driver_tsens
                             esp_http_server
                             esp_wifi
                             esp_event
                             esp_netif
                             esp_timer
                             nvs_flash
                             spiffs
                             espressif__cjson)

# Create SPIFFS image from spiffs folder
spiffs_create_partition_image(spiffs ./spiffs FLASH_IN_PROJECT)
