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
#define TCP_SERVER_HOST         "8.137.35.154"
#define TCP_SERVER_PORT         16384

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

/* 时区配置 */
#define BEIJING_TIMEZONE        "CST-8"         // 北京时间 UTC+8

/* 设备ID配置 - 修改此值区分不同ESP32设备 */
#define DEVICE_ID               3       // 设备编号: 1, 2, 3...

/* 数据上传配置 */
#define TCP_SAMPLES_PER_PACKET  100     // 每包100条数据
#define TCP_DATA_CHANNELS       6       // 6个电机数据通道 (pos/vel/torque x 2)

/* 协议版本10: 精确时间戳版
 *
 * 包头 (14字节):
 *   - magic: 2 (0xAA55)
 *   - device_id: 1
 *   - version: 1 (10)
 *   - seq: 1 (0-255循环)
 *   - flags: 1 (bit0=sync_flag, bit1=has_roll)
 *   - interval_ms: 1 (实际平均采样间隔，仅供参考)
 *   - reserved: 1
 *   - base_timestamp: 6 (48bit毫秒时间戳，第一条数据的真实时间)
 *
 * 每条数据 - 不含roll (15字节):
 *   - time_offset_ms: uint16 相对于base_timestamp的毫秒偏移 (0~65535ms)
 *   - pos1, pos2: int16×2 位置×100 (精度0.01)
 *   - vel1, vel2: int16×2 速度×100 (精度0.01)
 *   - torque1, torque2: int16×2 转矩×100 (精度0.01)
 *   - states: 1 (高4位m1_state, 低4位m2_state)
 *
 * 每条数据 - 含roll (19字节, has_roll=1时):
 *   - time_offset_ms: uint16 相对于base_timestamp的毫秒偏移 (0~65535ms)
 *   - pos1, pos2: int16×2 位置×100 (精度0.01)
 *   - vel1, vel2: int16×2 速度×100 (精度0.01)
 *   - torque1, torque2: int16×2 转矩×100 (精度0.01)
 *   - roll_left, roll_right: int16×2 陀螺仪roll角度×100
 *   - states: 1 (高4位m1_state, 低4位m2_state)
 *
 * CRC: 1字节 (CRC8)
 *
 * 不带roll: 14 + 100×15 + 1 = 1515字节
 * 带roll:   14 + 100×19 + 1 = 1915字节
 */

/* 蓝牙IMU未连接时的特殊标记值 */
#define BT_IMU_NOT_CONNECTED    0x7FFF  /* int16最大正值，表示未连接 */

/* 包头标志位定义 */
#define FLAG_SYNC       0x01    /* bit0: 时间已同步 */
#define FLAG_HAS_ROLL   0x02    /* bit1: 包含roll数据 */

/* ==================== 数据包响应协议（用于时间同步） ==================== */
/*
 * 服务端收到AA55数据包后，返回响应用于时间同步
 *
 * 响应格式 (12字节):
 *   Magic(2) + Type(1) + DeviceID(1) + ServerTime(8) = 12字节
 */
#define DATA_RESPONSE_TYPE_TIME_SYNC 0x01   /* 时间同步响应 */
#define DATA_RESPONSE_TYPE_COMMAND   0x02   /* 命令响应（预留） */
#define DATA_RESPONSE_SIZE           12     /* 响应包大小 */

/* 数据包响应结构 (服务端发送) */
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;         /* 0xAA55 */
    uint8_t type;           /* 0x01 = 时间同步响应 */
    uint8_t device_id;      /* 设备ID */
    int64_t server_time_ms; /* 服务器当前时间戳（毫秒） */
} data_response_t;          /* 12字节 */
#pragma pack(pop)

/* 单条数据格式 - 不含roll (协议v10: 精确时间戳版) */
#pragma pack(push, 1)
typedef struct {
    uint16_t time_offset_ms;            // 相对于base_timestamp的毫秒偏移 (0~65535ms)
    int16_t pos1;                       // 电机1位置×100 (精度0.01)
    int16_t pos2;                       // 电机2位置×100 (精度0.01)
    int16_t vel1;                       // 电机1速度×100 (精度0.01)
    int16_t vel2;                       // 电机2速度×100 (精度0.01)
    int16_t torque1;                    // 电机1转矩×100 (精度0.01)
    int16_t torque2;                    // 电机2转矩×100 (精度0.01)
    uint8_t states;                     // 高4位m1_state, 低4位m2_state
} tcp_sample_t;  // 2 + 12 + 1 = 15字节
#pragma pack(pop)

/* 单条数据格式 - 含roll (协议v10: 精确时间戳版, has_roll=1时使用) */
#pragma pack(push, 1)
typedef struct {
    uint16_t time_offset_ms;            // 相对于base_timestamp的毫秒偏移 (0~65535ms)
    int16_t pos1;                       // 电机1位置×100 (精度0.01)
    int16_t pos2;                       // 电机2位置×100 (精度0.01)
    int16_t vel1;                       // 电机1速度×100 (精度0.01)
    int16_t vel2;                       // 电机2速度×100 (精度0.01)
    int16_t torque1;                    // 电机1转矩×100 (精度0.01)
    int16_t torque2;                    // 电机2转矩×100 (精度0.01)
    int16_t roll_left;                  // 蓝牙IMU左腿roll角度×100
    int16_t roll_right;                 // 蓝牙IMU右腿roll角度×100
    uint8_t states;                     // 高4位m1_state, 低4位m2_state
} tcp_sample_with_roll_t;  // 2 + 12 + 4 + 1 = 19字节
#pragma pack(pop)

/* 数据包包头格式 - 协议版本10 (精确时间戳+CRC8校验) */
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;                     // 帧头标识 0xAA55
    uint8_t device_id;                  // 设备ID (1, 2, 3...)
    uint8_t version;                    // 协议版本 (10)
    uint8_t seq;                        // 序列号 (0-255循环)
    uint8_t flags;                      // 标志位: bit0=sync, bit1=has_roll
    uint8_t interval_ms;                // 实际平均采样间隔(毫秒)，仅供参考
    uint8_t reserved;                   // 保留字节
    uint8_t base_time[6];               // 基准时间戳 (48bit毫秒, 小端序, 第一条数据的真实时间)
    // 注意: samples数据和crc8在发送时动态填充到发送缓冲区
} tcp_packet_header_t;  // 14字节
#pragma pack(pop)

/* 完整数据包大小 (含crc) */
#define TCP_SAMPLE_SIZE_NO_ROLL   15    // 每条数据15字节 (不含roll)
#define TCP_SAMPLE_SIZE_WITH_ROLL 19    // 每条数据19字节 (含roll)
#define TCP_PACKET_SIZE_NO_ROLL   (14 + TCP_SAMPLES_PER_PACKET * TCP_SAMPLE_SIZE_NO_ROLL + 1)      // 14 + 100×15 + 1 = 1515字节
#define TCP_PACKET_SIZE_WITH_ROLL (14 + TCP_SAMPLES_PER_PACKET * TCP_SAMPLE_SIZE_WITH_ROLL + 1)    // 14 + 100×19 + 1 = 1915字节

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
 * @brief 设置采样间隔（供main.cpp调用）
 * @param interval_ms 采样间隔（毫秒）
 */
void tcp_data_set_interval(uint8_t interval_ms);

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
