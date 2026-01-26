#ifndef WIFI_WEBSERVER_H
#define WIFI_WEBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* WiFi AP 和 Web服务器配置 */
#define WIFI_AP_SSID            "登山外骨骼1"
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
 * @brief 设置外部电机参数和互斥锁的访问接口
 * @param motor1_pos 电机1位置指针
 * @param motor2_pos 电机2位置指针
 * @param mutex 互斥锁句柄
 * @param speed_follow_ptr SpeedFollowMode实例指针
 */
void webserver_set_motor_access(float* motor1_pos, float* motor2_pos,
                                 SemaphoreHandle_t mutex, void* speed_follow_ptr);

#ifdef __cplusplus
}
#endif

#endif // WIFI_WEBSERVER_H
