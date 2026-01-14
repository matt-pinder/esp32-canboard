idf_component_register(SRCS "main.c"
                        "src/can.c"
                        "src/inputs.c"
                        "src/config.c"
                        "src/wifi_config.c"
                        "src/http_server.c"
                    INCLUDE_DIRS "."
                    "./src"
                    "./inc")
