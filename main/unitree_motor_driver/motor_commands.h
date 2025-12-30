#ifndef MOTOR_COMMANDS_H
#define MOTOR_COMMANDS_H

#include <stdint.h>

// 定义 GO-M8010-6 电机的通信协议参数
// 所有的浮点数到整数的转换都需要乘以一个比例因子，并进行舍入
// 参考宇树科技文档中心中 GO-M8010-6 的通信协议部分

// 命令帧头和数据长度
#define GO_M8010_6_CMD_HEADER_BYTE1 0xFE // 命令帧头第一字节
#define GO_M8010_6_CMD_HEADER_BYTE2 0xEE // 命令帧头第二字节
#define GO_M8010_6_CMD_LENGTH       17   // 发送命令总长度 (字节)

// 反馈帧头和数据长度
#define GO_M8010_6_DATA_HEADER_BYTE1 0xFD // 反馈帧头第一字节
#define GO_M8010_6_DATA_HEADER_BYTE2 0xEE // 反馈帧头第二字节
#define GO_M8010_6_DATA_LENGTH       16   // 接收反馈总长度 (字节)

// 各参数的打包比例因子和范围 (GO-M8010-6 协议)
// 力矩 (T): 1N.m -> 256 (signed short int)
#define GO_M8010_6_TAU_RATIO        256.0f
#define GO_M8010_6_TAU_MAX_ABS      127.99f // |tau_ff| < (2^15-1)/256

// 速度 (W): rad/s -> packed_int16. omega_set = W / (2pi) * 256
// (signed short int)
#define GO_M8010_6_OMEGA_RAD_PER_UNIT  ( 6.28318f / 256.0f ) // 1 unit 对应多少 rad/s
#define GO_M8010_6_OMEGA_UNIT_PER_RAD  ( 256.0f / 6.28318f ) // 1 rad/s 对应多少 unit
#define GO_M8010_6_OMEGA_MAX_ABS    804.00f // |omega_des| <= 804.0 rad/s

// 位置 (Pos): rad -> packed_int32. pos_des = Pos / (2pi) * 32768
// (signed int)
#define GO_M8010_6_THETA_RAD_PER_UNIT  ( 6.28318f / 32768.0f ) // 1 unit 对应多少 rad
#define GO_M8010_6_THETA_UNIT_PER_RAD  ( 32768.0f / 6.28318f ) // 1 rad 对应多少 unit
#define GO_M8010_6_THETA_MAX_ABS    411774.0f // |theta_des| <= 411774 rad (65535 circle)

// Kp (K_P): kp_packed = K_P / 25.6f * 32768.0f;  (unsigned short int)
#define GO_M8010_6_KP_RATIO_INV_25_6  ( 32768.0f / 25.6f ) // 1 unit 对应多少 K_P
#define GO_M8010_6_KP_MAX           25.599f // 0 <= kp <= 25.599

// Kd (K_W): k_spd = K_W / 25.6f * 32768.0f; (unsigned short int)
#define GO_M8010_6_KD_RATIO_INV_25_6  ( 32768.0f / 25.6f ) // 1 unit 对应多少 K_W
#define GO_M8010_6_KD_MAX           25.599f // 0 <= kw <= 25.599

// GO-M8010-6 电机指令数据包结构 (17字节)
// 注意：以下结构体仅为内部字节偏移和大小参考，不直接用于打包，而是手动组装字节数组
// 因为存在位字段和非标准对齐，直接使用C结构体会引入填充字节。
// 实际打包将在unitree_motor.cpp中手动完成。
typedef struct __attribute__((packed)) // 使用__attribute__((packed)) 尝试紧凑排列
{
    uint8_t head[2];        // 0xFE 0xEE
    uint8_t mode_id;        // Bit 0-3: ID, Bit 4-6: STATUS, Bit 7: reserve
    int16_t tor_des;        // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des;        // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des;        // 期望关节输出位置 unit: rad      (q15)
    uint16_t k_pos;         // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    uint16_t k_spd;         // 期望关节阻尼系数 unit: -1.0-1.0 (q15)
    uint16_t CRC16;         // CRC
} RIS_ControlData_t; // 主机控制命令     17Byte

// 电机反馈数据包格式
typedef struct __attribute__((packed)) // 使用__attribute__((packed)) 尝试紧凑排列
{
    uint8_t head[2];        // 0xFD 0xEE
    uint8_t mode_id;        // Bit 0-3: ID, Bit 4-6: STATUS, Bit 7: Reserved
    int16_t torque_fbk;     // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed_fbk;      // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos_fbk;        // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;            // 电机温度: -128~127°C
    uint8_t MError_force_low; // Bit 0-2: MError, Bit 3-7: force (low 5 bits)
    uint8_t force_high_none;  // Bit 0-6: force (high 7 bits), Bit 7: none (reserved)
    uint16_t CRC16;         // CRC
} RIS_MotorData_t; // 电机返回数据     16Byte


// 为了保持与 main.cpp 中的兼容性，保留旧的结构体定义，但其数据在
// UnitreeMotorDriver::sendRecv 中转换为 GO-M8010-6 协议，并从 GO-M8010-6 反馈转换回来
// 应用程序应主要关注这些浮点数接口
typedef struct {
    uint8_t id;         // 电机ID
    uint8_t mode;       // 模式 (对应 GO-M8010-6 的 STATUS 字段)
    float pos;          // 位置 (rad)
    float vel;          // 速度 (rad/s)
    float t;            // 力矩 (N.m)
    float kp;           // 位置刚度系数
    float kd;           // 速度阻尼系数
} MotorCmdA1; // 此处命名仍为A1，但实际应转换为GO-M8010-6协议

typedef struct {
    uint8_t id;         // 电机ID
    uint8_t mode;       // 当前模式 (对应 GO-M8010-6 的 STATUS 字段)
    float pos;          // 当前位置 (rad)
    float vel;          // 当前速度 (rad/s)
    float t;            // 当前力矩 (N.m)
    int16_t temp;       // 温度 (°C)
    uint8_t MError;     // 错误状态
    uint16_t footForce; // 足端力传感器原始数值
} MotorDataA1; // 此处命名仍为A1，但实际应转换为GO-M8010-6协议

#endif // MOTOR_COMMANDS_H
