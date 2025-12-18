#ifndef TCP_DATA_UPLOAD_H
#define TCP_DATA_UPLOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* 服务器配置 */
#define TCP_SERVER_HOST         "frp-any.com"
#define TCP_SERVER_PORT         18214

/* WiFi STA配置 */
#define WIFI_STA_SSID           "123"
#define WIFI_STA_PASSWORD       "12345678"
#define WIFI_STA_MAX_RETRY      10

/* NTP时间同步配置 */
#define NTP_SERVER_1            "ntp.aliyun.com"
#define NTP_SERVER_2            "cn.ntp.org.cn"
#define NTP_SERVER_3            "pool.ntp.org"
#define BEIJING_TIMEZONE        "CST-8"         // 北京时间 UTC+8

/* 设备ID配置 - 修改此值区分不同ESP32设备 */
#define DEVICE_ID               1       // 设备编号: 1, 2, 3...

/* 数据上传配置 */
#define TCP_SAMPLES_PER_PACKET  100     // 每包100条数据
#define TCP_DATA_CHANNELS       6       // 6个电机数据通道 (pos/vel/torque x 2)

/* 协议版本5: 整数编码
 * - 时间戳: int64毫秒时间戳 (如1734567890123)
 * - 电机数据: int16 (原值×100, 精度0.01, 范围±327)
 * - 每条: 8 + 12 + 2 = 22字节
 * - 每包: 8 + 100×22 = 2208字节 (节省45%)
 */

/* 单条数据格式 - 用于网络传输 (整数编码) */
#pragma pack(push, 1)
typedef struct {
    int64_t timestamp_ms;               // 毫秒时间戳 (如1734567890123)
    int16_t channels[TCP_DATA_CHANNELS]; // 6个电机数据通道 (原值×100, 精度0.01)
    int8_t m1_state;                    // 电机1状态标签: 0=空闲, 1=抬腿, 2=压腿, 3=检测速度触发
    int8_t m2_state;                    // 电机2状态标签
} tcp_sample_t;  // 8 + 12 + 2 = 22字节
#pragma pack(pop)

/* 数据包格式 - 协议版本5 */
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;                     // 帧头标识 0xAA55
    uint8_t device_id;                  // 设备ID (1, 2, 3...)
    uint8_t version;                    // 协议版本 (5)
    uint16_t seq;                       // 序列号
    uint16_t count;                     // 本包数据条数
    tcp_sample_t samples[TCP_SAMPLES_PER_PACKET];  // 100条数据，每条22字节
} tcp_data_packet_t;  // 8 + 2200 = 2208字节
#pragma pack(pop)

/* 单条数据结构 - 内部使用 (浮点格式) */
typedef struct {
    int64_t timestamp_ms;       // 毫秒时间戳 (如1734567890123)
    float motor1_pos;
    float motor1_vel;
    float motor1_torque;
    float motor2_pos;
    float motor2_vel;
    float motor2_torque;
    int8_t m1_state_label;      // 电机1状态标签: 0=空闲, 1=抬腿, 2=压腿, 3=检测速度触发
    int8_t m2_state_label;      // 电机2状态标签
} motor_data_record_t;

/**
 * @brief 初始化WiFi STA模式
 * @return ESP_OK成功, ESP_FAIL失败, ESP_ERR_TIMEOUT超时
 */
esp_err_t wifi_init_sta(void);

/**
 * @brief 在后台任务中初始化WiFi和TCP上传（非阻塞）
 * @note 此函数立即返回，WiFi/TCP初始化在后台进行
 */
void wifi_tcp_init_background(void);

/**
 * @brief 初始化SNTP时间同步（在WiFi连接后调用）
 */
void ntp_time_sync_init(void);

/**
 * @brief 获取当前北京时间的毫秒时间戳
 * @return 毫秒时间戳（从1970年1月1日起），如果时间未同步返回0
 */
uint64_t get_beijing_timestamp_ms(void);

/**
 * @brief 获取当前时间戳（double格式：秒.毫秒）
 * @return 时间戳，如1734567890.123，时间未同步时使用系统运行时间
 */
double get_timestamp_double(void);

/**
 * @brief 检查时间是否已同步
 * @return true已同步，false未同步
 */
bool is_time_synced(void);

/**
 * @brief 初始化TCP数据上传模块
 * @return ESP_OK成功
 */
esp_err_t tcp_data_upload_init(void);

/**
 * @brief 添加一条数据到发送缓冲区
 * @param record 数据记录
 * @return ESP_OK成功，ESP_ERR_NO_MEM缓冲区满
 */
esp_err_t tcp_data_add_record(const motor_data_record_t *record);

/**
 * @brief 启动TCP数据上传任务
 * @return ESP_OK成功
 */
esp_err_t tcp_data_upload_start(void);

/**
 * @brief 停止TCP数据上传任务
 */
void tcp_data_upload_stop(void);

/**
 * @brief 获取TCP连接状态
 * @return true已连接，false未连接
 */
bool tcp_data_is_connected(void);

/**
 * @brief 获取上传统计信息
 * @param packets_sent 已发送包数
 * @param packets_failed 发送失败包数
 * @param bytes_sent 已发送字节数
 */
void tcp_data_get_stats(uint32_t *packets_sent, uint32_t *packets_failed, uint32_t *bytes_sent);

#ifdef __cplusplus
}
#endif

#endif // TCP_DATA_UPLOAD_H
