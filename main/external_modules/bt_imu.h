/**
 * @file bt_imu.h
 * @brief BLE IMU 连接器模块 - 使用HC系列蓝牙模块的IMU传感器接口
 *
 * 基于参考Python实现，使用不同的UUID和数据格式
 * 服务UUID: 49535343-fe7d-4ae5-8fa9-9fafd205e455
 * 数据包格式: 11字节，以0x55开头，带校验和
 *
 * 支持同时连接多个IMU设备
 */

#ifndef BT_IMU_H
#define BT_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_BT_IMU_DEVICES 2  // 最大支持设备数量

// IMU数据结构
typedef struct {
    // 欧拉角 (°)
    float roll;       // 滚转角X (AngleX)
    float pitch;      // 俯仰角Y (AngleY)
    float yaw;        // 偏航角Z (AngleZ)

    // 加速度 (g)
    float acc_x, acc_y, acc_z;

    // 角速度 (°/s)
    float gyro_x, gyro_y, gyro_z;

    // 磁场
    float mag_x, mag_y, mag_z;

    // 时间信息
    char time_str[32];  // 格式: "YYYY-MM-DD HH:MM:SS:MS"

    bool is_valid;      // 数据有效性
    uint32_t packet_count;  // 包计数器
    uint32_t timestamp;     // 时间戳
} bt_imu_data_t;

// BT IMU句柄类型
typedef struct bt_imu_handle bt_imu_handle_t;

/**
 * @brief 初始化BT IMU模块（支持多设备）
 * @return 初始化成功返回0，失败返回-1
 */
int bt_imu_init_multi(void);

/**
 * @brief 获取指定设备的IMU数据（线程安全）
 * @param device_index 设备索引 (0 或 1)
 * @param data 输出数据结构
 * @return true 数据有效，false 数据无效
 */
bool bt_imu_get_data_multi(int device_index, bt_imu_data_t* data);

/**
 * @brief 获取指定设备的PITCH角度
 * @param device_index 设备索引 (0 或 1)
 * @return PITCH角度值（度）
 */
float bt_imu_get_pitch_multi(int device_index);

/**
 * @brief 检查指定设备的连接状态
 * @param device_index 设备索引 (0 或 1)
 * @return true 已连接，false 未连接
 */
bool bt_imu_is_connected_multi(int device_index);

/**
 * @brief 获取指定设备的接收字节数统计
 * @param device_index 设备索引 (0 或 1)
 * @return 累计接收字节数
 */
uint32_t bt_imu_get_bytes_received_multi(int device_index);

/**
 * @brief 获取指定设备的校验错误计数
 * @param device_index 设备索引 (0 或 1)
 * @return 校验和错误次数
 */
uint32_t bt_imu_get_checksum_errors_multi(int device_index);

/**
 * @brief 获取已连接的设备数量
 * @return 已连接设备数量
 */
int bt_imu_get_connected_count(void);

// ========== 兼容旧接口（使用第一个设备） ==========

/**
 * @brief 初始化BT IMU模块
 * @return BT IMU句柄，失败返回NULL
 */
bt_imu_handle_t* bt_imu_init(void);

/**
 * @brief 销毁BT IMU模块
 * @param handle BT IMU句柄
 */
void bt_imu_destroy(bt_imu_handle_t* handle);

/**
 * @brief 获取BT IMU数据（线程安全）
 * @param handle BT IMU句柄
 * @param data 输出数据结构
 * @return true 数据有效，false 数据无效
 */
bool bt_imu_get_data(bt_imu_handle_t* handle, bt_imu_data_t* data);

/**
 * @brief 获取ROLL角度
 * @param handle BT IMU句柄
 * @return ROLL角度值（度）
 */
float bt_imu_get_roll(bt_imu_handle_t* handle);

/**
 * @brief 获取PITCH角度
 * @param handle BT IMU句柄
 * @return PITCH角度值（度）
 */
float bt_imu_get_pitch(bt_imu_handle_t* handle);

/**
 * @brief 获取YAW角度
 * @param handle BT IMU句柄
 * @return YAW角度值（度）
 */
float bt_imu_get_yaw(bt_imu_handle_t* handle);

/**
 * @brief 检查BLE连接状态
 * @param handle BT IMU句柄
 * @return true 已连接，false 未连接
 */
bool bt_imu_is_connected(bt_imu_handle_t* handle);

/**
 * @brief 获取接收字节数统计
 * @param handle BT IMU句柄
 * @return 累计接收字节数
 */
uint32_t bt_imu_get_bytes_received(bt_imu_handle_t* handle);

/**
 * @brief 获取校验错误计数
 * @param handle BT IMU句柄
 * @return 校验和错误次数
 */
uint32_t bt_imu_get_checksum_errors(bt_imu_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif // BT_IMU_H
