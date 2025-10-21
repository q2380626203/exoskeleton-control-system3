#ifndef _MI_MOTOR_H__
#define _MI_MOTOR_H__

#include <stdint.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <math.h>
#include <string.h>
#include "motor_commands.h"  // 引入宇树电机数据结构

#ifdef __cplusplus
extern "C" {
#endif

// UART Configuration
#define RS01_UART_NUM       UART_NUM_1  // ESP-IDF UART port for RS01 motor communication
#define RS01_UART_TX_PIN    GPIO_NUM_10 // UART transmit pin
#define RS01_UART_RX_PIN    GPIO_NUM_11 // UART receive pin
#define RS01_UART_BAUDRATE  115200      // UART baud rate

// CAN-to-Serial Module Protocol
#define CAN_RAW_FRAME_LENGTH 12

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

// Global variables for UART communication.
extern uart_config_t motor_uart_config;
extern MI_Motor motors[2];
extern MotorDataCallback data_callback;

// Function Prototypes
void UART_Rx_Init(MotorDataCallback callback);
void UART_Send_Frame(const can_frame_t* frame);

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
