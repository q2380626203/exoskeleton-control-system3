idf_component_register(SRCS "main.cpp" "unitree_motor.cpp" "crc_utils.cpp" "speed_follow_mode.cpp" "position_buffer.c"
                    INCLUDE_DIRS "."
                    # ESP-IDF 组件依赖
                    # driver: 提供 uart.h 等硬件驱动
                    # log: 提供 ESP_LOGI 等日志功能
                    # esp_timer: 提供定时器功能
                    PRIV_REQUIRES driver log esp_timer)
                    