#ifndef TCP_DATA_UPLOAD_H
#define TCP_DATA_UPLOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/uart.h"
#include <stdint.h>
#include <stdbool.h>

/* 服务器配置 - 公网 */
#define TCP_SERVER_HOST         "frp-any.com"
#define TCP_SERVER_PORT         18214

/* 服务器配置 - 局域网（自动获取网关地址） */
#define TCP_LAN_SERVER_PORT     8888    // 局域网服务器端口（手机上运行的服务端口）
#define TCP_ENABLE_LAN_UPLOAD   1       // 1=启用局域网上传, 0=禁用

/* 串口透传配置 - 通过4G模块透传到远程服务器 */
#define SERIAL_PASSTHROUGH_ENABLE   1       // 1=启用串口透传, 0=禁用
// 注意: 串口透传使用 fwrite(stdout)，波特率由 ESP-IDF menuconfig 控制台配置决定
// 以下两个宏未使用，保留作为参考：
// #define SERIAL_PASSTHROUGH_UART     UART_NUM_0  // 使用串口0（4G模块连接）
// #define SERIAL_PASSTHROUGH_BAUD     115200      // 串口波特率

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
#define DEVICE_ID               3       // 设备编号: 1, 2, 3...

/* 数据上传配置 */
#define TCP_SAMPLES_PER_PACKET  100     // 每包100条数据
#define TCP_DATA_CHANNELS       6       // 6个电机数据通道 (pos/vel/torque x 2)

/* 协议版本5: 整数编码
 * - 时间戳: int64毫秒时间戳 (如1734567890123)
 * - 电机数据: int16 (原值×100, 精度0.01, 范围±327)
 * - IMU roll数据: int16 (原值×100, 精度0.01), 0x7FFF表示未连接
 * - 每条: 8 + 12 + 4 + 2 = 26字节
 * - 每包: 8 + 100×26 = 2608字节
 */

/* 蓝牙IMU未连接时的特殊标记值 */
#define BT_IMU_NOT_CONNECTED    0x7FFF  /* int16最大正值，表示未连接 */

/* 单条数据格式 - 用于网络传输 (整数编码) */
#pragma pack(push, 1)
typedef struct {
    int64_t timestamp_ms;               // 毫秒时间戳 (如1734567890123)
    int16_t channels[TCP_DATA_CHANNELS]; // 6个电机数据通道 (原值×100, 精度0.01)
    int16_t roll_left;                  // 蓝牙IMU左腿roll角度 (原值×100), 0x7FFF=未连接
    int16_t roll_right;                 // 蓝牙IMU右腿roll角度 (原值×100), 0x7FFF=未连接
    int8_t m1_state;                    // 电机1状态标签: 0=空闲, 1=抬腿, 2=压腿, 3=检测速度触发
    int8_t m2_state;                    // 电机2状态标签
} tcp_sample_t;  // 8 + 12 + 4 + 2 = 26字节
#pragma pack(pop)

/* 数据包格式 - 协议版本6 (添加蓝牙IMU数据) */
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;                     // 帧头标识 0xAA55
    uint8_t device_id;                  // 设备ID (1, 2, 3...)
    uint8_t version;                    // 协议版本 (6)
    uint16_t seq;                       // 序列号
    uint16_t count;                     // 本包数据条数
    tcp_sample_t samples[TCP_SAMPLES_PER_PACKET];  // 100条数据，每条26字节
} tcp_data_packet_t;  // 8 + 2600 = 2608字节
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
    float roll_left;            // 蓝牙IMU左腿roll角度, NAN表示未连接
    float roll_right;           // 蓝牙IMU右腿roll角度, NAN表示未连接
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
 * @brief 获取TCP连接状态（公网）
 * @return true已连接，false未连接
 */
bool tcp_data_is_connected(void);

/**
 * @brief 获取局域网TCP连接状态
 * @return true已连接，false未连接
 */
bool tcp_data_lan_is_connected(void);

/**
 * @brief 获取网关IP地址（用于局域网连接）
 * @param ip_str 输出缓冲区，至少16字节
 * @return true成功获取，false失败
 */
bool tcp_get_gateway_ip(char *ip_str);

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
