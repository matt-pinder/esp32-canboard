idf_component_register(SRCS "main.c"
                        "src/can.c"
                        "src/inputs.c"
                        "src/config.c"
                        "src/wifi_config.c"
                        "src/http_server.c"
                        "src/ble_scan.c"
                        "src/dragy_gps.c"
                        "src/espnow_transport.c"
                        "src/espnow_can_protocol.c"
                        "src/relay_rule_engine.c"
                        "src/relay_rule_http.c"
                        "src/mk60_emulator.c"
                        "src/mk60_emulator_protocol.c"
                        "src/can_receive_dispatch.c"
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
                             esp_partition
                             esp_timer
                             nvs_flash
                             spiffs
                             espressif__cjson)

# Web assets remain part of the normal project flash. Persistent board settings
# live in `config` NVS and relay rules in the raw `rules` partition; neither
# data partition has a FLASH_IN_PROJECT image.
spiffs_create_partition_image(spiffs ./spiffs FLASH_IN_PROJECT)
