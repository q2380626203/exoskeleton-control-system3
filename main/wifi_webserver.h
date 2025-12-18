#ifndef WIFI_WEBSERVER_H
#define WIFI_WEBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_http_server.h"

/* WiFi热点配置 */
#define WIFI_AP_SSID            "ESP32_3"
#define WIFI_AP_PASSWORD        "12345678"
#define WIFI_AP_CHANNEL         1
#define WIFI_AP_MAX_CONNECTIONS 4

/* Web服务器配置 */
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

/**
 * @brief 命令处理回调函数类型
 *
 * @param cmd 命令字符串
 * @return esp_err_t 处理结果
 */
typedef esp_err_t (*command_handler_t)(const char *cmd);

/**
 * @brief 参数设置回调函数类型
 *
 * @param param_name 参数名称
 * @param value 参数值
 * @return esp_err_t 处理结果
 */
typedef esp_err_t (*param_handler_t)(const char *param_name, float value);

/**
 * @brief 电机参数设置回调函数类型
 *
 * @param motor 电机编号 (1或2)
 * @param param_name 参数名称
 * @param value 参数值
 * @return esp_err_t 处理结果
 */
typedef esp_err_t (*motor_param_handler_t)(int motor, const char *param_name, float value);

/**
 * @brief 电机参数读取回调函数类型
 *
 * @param motor 电机编号 (1或2)
 * @param param_name 参数名称
 * @param value 输出参数值的指针
 * @return esp_err_t 处理结果
 */
typedef esp_err_t (*motor_param_getter_t)(int motor, const char *param_name, float *value);

/**
 * @brief 状态获取回调函数类型
 *
 * @param state_json 输出JSON字符串的缓冲区
 * @param max_len 缓冲区最大长度
 * @return esp_err_t 处理结果
 */
typedef esp_err_t (*state_getter_t)(char *state_json, size_t max_len);

/**
 * @brief 注册命令处理回调函数
 *
 * @param handler 命令处理函数指针
 */
void register_command_handler(command_handler_t handler);

/**
 * @brief 注册参数设置回调函数
 *
 * @param handler 参数设置函数指针
 */
void register_param_handler(param_handler_t handler);

/**
 * @brief 注册电机参数设置回调函数
 *
 * @param handler 电机参数设置函数指针
 */
void register_motor_param_handler(motor_param_handler_t handler);

/**
 * @brief 注册电机参数读取回调函数
 *
 * @param getter 电机参数读取函数指针
 */
void register_motor_param_getter(motor_param_getter_t getter);

/**
 * @brief 注册状态获取回调函数
 *
 * @param getter 状态获取函数指针
 */
void register_state_getter(state_getter_t getter);

#ifdef __cplusplus
}
#endif

#endif // WIFI_WEBSERVER_H
