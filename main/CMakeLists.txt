idf_component_register(SRCS "main.cpp" "unitree_motor.cpp" "crc_utils.cpp" "speed_follow_mode.cpp"
                    INCLUDE_DIRS "."                    
                    # 关键修正：添加主程序所依赖的所有 ESP-IDF 元件
                    # driver: 提供 uart.h 等硬件驱动
                    # log: 提供 ESP_LOGI 等日志功能
                    # nvs_flash: WiFi 初始化需要
                    # esp_netif: TCP/IP 网络接口
                    # esp_wifi: WiFi 驱动
                    # esp_event: 事件循环框架
                    # esp_http_server: HTTP 和 WebSocket 服务器
                    PRIV_REQUIRES driver log nvs_flash esp_netif esp_wifi esp_event esp_http_server esp_timer
                    EMBED_TXTFILES "index.html")
                    