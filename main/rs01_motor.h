#ifndef _MI_MOTOR_H__
#define _MI_MOTOR_H__

#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_twai.h"          // TWAI驱动核心API
#include "esp_twai_onchip.h"   // TWAI片上控制器配置
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"  // FreeRTOS事件组，用于数据同步
#include <math.h>
#include <string.h>
#include "motor_commands.h"  // 引入宇树电机数据结构

#ifdef __cplusplus
extern "C" {
#endif

// TWAI (CAN) Configuration
#define RS01_TWAI_TX_PIN    GPIO_NUM_11   // TWAI TX引脚
#define RS01_TWAI_RX_PIN    GPIO_NUM_10   // TWAI RX引脚
#define RS01_TWAI_BITRATE   1000000      // CAN总线波特率 1Mbps
#define RS01_TWAI_TX_QUEUE_DEPTH 10      // 发送队列深度

// Motor Data Synchronization (数据同步配置)
#define MOTOR1_DATA_READY_BIT (1 << 0)   // 电机1数据就绪事件位
#define MOTOR2_DATA_READY_BIT (1 << 1)   // 电机2数据就绪事件位
#define MOTOR_DATA_TIMEOUT_MS 10         // 等待电机反馈数据超时时间（毫秒）

// Motor and Master IDs
#define MASTER_ID        0xFD
#define MOTER_1_ID       1
#define MOTER_2_ID       2

// Motor Parameter Ranges (from RS01 manual)
#define P_MIN           -12.57f
#define P_MAX           12.57f
#define V_MIN           -44.0f
#define V_MAX           44.0f
#define KP_MIN          0.0f
#define KP_MAX          500.0f
#define KD_MIN          0.0f
#define KD_MAX          5.0f
#define T_MIN           -17.0f
#define T_MAX           17.0f

// Control Mode and Parameter Indices (from RS01 manual)
#define RUN_MODE        0x7005    // Run Mode: 0-Motion, 1-Position(PP), 2-Speed, 3-Current, 5-Position(CSP)
#define CTRL_MODE       0         // Motion Control Mode
#define POS_MODE_PP     1         // Position Mode (PP)
#define SPEED_MODE      2         // Speed Mode
#define CUR_MODE        3         // Current Mode
#define POS_MODE_CSP    5         // Position Mode (CSP)

#define IQ_REF          0x7006    // Current Mode Iq Command (float, -23~23A)
#define SPD_REF         0x700A    // Speed Mode Speed Command (float, -44~44rad/s)
#define LIMIT_TORQUE    0x700B    // Torque Limit (float, 0~17Nm)
#define LOC_REF         0x7016    // Position Mode Angle Command (float, rad)
#define LIMIT_SPD       0x7017    // Position Mode (CSP) Speed Limit (float, 0~44rad/s)
#define LIMIT_CUR       0x7018    // Speed/Position Mode Current Limit (float, 0~23A)
#define VELOCITY_FILTER 0x7021    // Velocity Filter Value (float, filter coefficient)
#define REPORT_TIME     0x7026    // Report Time Setting (int, 1 for enable reporting)

// Structure for outgoing CAN frames (based on RS01 protocol)
typedef struct {
    uint8_t type;       // Communication type (5 bits)
    uint16_t data;      // Data field (16 bits)
    uint8_t target_id;  // Target motor ID (8 bits)
    uint8_t payload[8]; // 8-byte data payload
} can_frame_t;

// ========== RS01电机数据结构：统一使用宇树格式 ==========
// MI_Motor 直接使用 MotorDataA1 类型，实现完全统一
typedef MotorDataA1 MI_Motor;

// 错误标志位掩码定义（通过位操作从MError提取）
#define MI_ERROR_UNCALIBRATED       (1 << 0)  // Bit 0: 未校准
#define MI_ERROR_OVERLOAD           (1 << 1)  // Bit 1: 过载
#define MI_ERROR_MAGNETIC_ENCODER   (1 << 2)  // Bit 2: 磁编码器错误
#define MI_ERROR_OVER_TEMPERATURE   (1 << 3)  // Bit 3: 过温
#define MI_ERROR_DRIVER_FAULT       (1 << 4)  // Bit 4: 驱动器故障
#define MI_ERROR_UNDERVOLTAGE       (1 << 5)  // Bit 5: 欠压

// 错误检查宏（替代原来的bool字段）
#define MI_HAS_ERROR(motor, flag) (((motor)->MError & (flag)) != 0)

// Callback function type for motor data updates
typedef void (*MotorDataCallback)(MI_Motor*);

// Global variables for TWAI communication
extern twai_node_handle_t twai_node_handle;  // TWAI节点句柄
extern EventGroupHandle_t motor_data_event_group;  // 电机数据同步事件组
extern MI_Motor motors[2];
extern MotorDataCallback data_callback;

// Function Prototypes
void TWAI_Init(MotorDataCallback callback);
void TWAI_Send_Frame(const can_frame_t* frame);

void Motor_Enable(MI_Motor* motor);
void Motor_Reset(MI_Motor* motor, uint8_t clear_error);
void Motor_Set_Zero(MI_Motor* motor);
void Motor_ControlMode(MI_Motor* motor, float torque, float position, float speed, float kp, float kd);
void Set_SingleParameter(MI_Motor* motor, uint16_t parameter, float value);
void Set_CurMode(MI_Motor* motor, float current);
void Set_SpeedMode(MI_Motor* motor, float speed, float current_limit);
void Change_Mode(MI_Motor* motor, uint8_t mode);
void Motor_SetReporting(MI_Motor* motor, bool enable);

// 新增：同步发送并接收反馈数据（类似Unitree的sendRecv）
int Motor_ControlMode_SendRecv(MI_Motor* motor, float torque, float position, float speed, float kp, float kd);

// Helper function for data conversion
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

#ifdef __cplusplus
}
#endif

#endif // _MI_MOTOR_H__
