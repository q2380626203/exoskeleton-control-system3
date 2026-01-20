#ifndef WIFI_WEBSERVER_H
#define WIFI_WEBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_http_server.h"

/* WiFi AP 和 Web服务器配置 */
#define WIFI_AP_SSID            "ESP32_AP"
#define WIFI_AP_PASSWORD        "12345678"
#define WIFI_AP_CHANNEL         1
#define WIFI_AP_MAX_CONNECTIONS 4
#define WEBSERVER_PORT          80

/**
 * @brief 初始化WiFi热点（AP模式）
 *
 * @return esp_err_t
 *         - ESP_OK: 成功
 *         - ESP_FAIL: 失败
 */
esp_err_t wifi_init_softap(void);

/**
 * @brief 启动Web服务器
 *
 * @return httpd_handle_t 服务器句柄，失败返回NULL
 */
httpd_handle_t start_webserver(void);

/**
 * @brief 停止Web服务器
 *
 * @param server 服务器句柄
 */
void stop_webserver(httpd_handle_t server);

/**
 * @brief 获取系统运行时间（秒）
 *
 * @return uint32_t 运行时间
 */
uint32_t get_system_uptime(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_WEBSERVER_H
