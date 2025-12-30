idf_component_register(SRCS "main.cpp"
                            "unitree_motor_driver/unitree_motor.cpp" "unitree_motor_driver/crc_utils.cpp"
                            "external_modules/bt_imu.c" "external_modules/voice_module.c" "external_modules/button_detector.cpp"
                            "core_algorithm/position_buffer.c" "core_algorithm/speed_follow_mode.cpp"
                            "host_comm/tcp_data_upload.c" "host_comm/wifi_webserver.c" "host_comm/web_handlers.cpp"
                    INCLUDE_DIRS "." "unitree_motor_driver" "external_modules" "core_algorithm" "host_comm"
                    # 嵌入AI模型文件（二进制格式，节省Flash空间）
                    # EMBED_FILES "model/motion_ai_model_quant.tflite"
                    # ESP-IDF 组件依赖
                    # driver: 提供 uart.h 等硬件驱动
                    # log: 提供 ESP_LOGI 等日志功能
                    # esp_timer: 提供定时器功能
                    # esp_wifi, esp_netif, esp_event: WiFi网络功能
                    # esp_http_server: HTTP服务器功能
                    # nvs_flash: NVS存储功能（WiFi需要）
                    # esp-tflite-micro: TensorFlow Lite Micro 推理引擎（已禁用）
                    # bt: 蓝牙协议栈（NimBLE，用于蓝牙IMU连接）
                    PRIV_REQUIRES driver log esp_timer esp_wifi esp_netif esp_event esp_http_server nvs_flash bt)
                    