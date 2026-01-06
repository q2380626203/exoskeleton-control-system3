idf_component_register(SRCS "main.cpp"
                            "unitree_motor_driver/unitree_motor.cpp" "unitree_motor_driver/crc_utils.cpp"
                            "external_modules/voice_module.c" "external_modules/button_detector.cpp"
                            "core_algorithm/position_buffer.c" "core_algorithm/speed_follow_mode.cpp"
                            "host_comm/tcp_data_upload.c" "host_comm/tcp_replay_receive.c" "host_comm/wifi_webserver.c" "host_comm/web_handlers.cpp"
                    INCLUDE_DIRS "." "unitree_motor_driver" "external_modules" "core_algorithm" "host_comm"
                    # ESP-IDF 组件依赖
                    PRIV_REQUIRES driver log esp_timer esp_wifi esp_netif esp_event esp_http_server nvs_flash)
                    