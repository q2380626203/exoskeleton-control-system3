#include "rs01_motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RS01_MOTOR";

// Global variables
uart_config_t motor_uart_config;
MI_Motor motors[2];
MotorDataCallback data_callback = NULL;

// 调试打印时间控制（每2秒打印一次）
static TickType_t lastPrintTime[2] = {0, 0};
static const TickType_t PRINT_INTERVAL = pdMS_TO_TICKS(2000);

/* ========================================================================
 * 静态函数声明 (Static Function Declarations)
 * ======================================================================== */
static void parse_can_frame(const uint8_t* can_payload);

/* ========================================================================
 * 辅助函数 (Helper Functions)
 * ======================================================================== */

/**
 * @brief 浮点数线性映射到16位无符号整数
 * @details 将浮点数映射到[0, 65535]范围，中心点0x8000(32768)对应中间值
 *          适用于RS01电机控制模式的参数转换
 * @param value 待转换的浮点数值
 * @param min_val 浮点数最小值
 * @param max_val 浮点数最大值
 * @return uint16_t 转换后的16位无符号整数
 */
uint16_t float_to_uint16_linear(float value, float min_val, float max_val) {
    // 对称映射：0值映射到0x8000(32768)
    if (value < min_val) value = min_val;
    if (value > max_val) value = max_val;

    // 计算相对于中点的偏移
    float center = (min_val + max_val) / 2.0f;
    float half_range = (max_val - min_val) / 2.0f;

    // 将[-half_range, half_range]映射到[0, 65535]，中心点0x8000
    float normalized = (value - center) / half_range;  // [-1, 1]
    return (uint16_t)(32768 + normalized * 32768);     // 0x8000 ± 32768
}

/**
 * @brief 浮点数转换为指定位数的无符号整数
 * @details 将给定范围的浮点数线性映射到指定位数的无符号整数范围
 *          常用于CAN通信中的数据压缩
 * @param x 待转换的浮点数
 * @param x_min 浮点数最小值
 * @param x_max 浮点数最大值
 * @param bits 目标整数的位数
 * @return int 转换后的整数值
 */
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 指定位数无符号整数转换为浮点数
 * @details 将指定位数的无符号整数线性映射回浮点数范围
 *          用于解析CAN通信中的压缩数据
 * @param x_int 待转换的整数值
 * @param x_min 目标浮点数最小值
 * @param x_max 目标浮点数最大值
 * @param bits 源整数的位数
 * @return float 转换后的浮点数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int * span / ((float)((1 << bits) - 1))) + offset;
}

/* ========================================================================
 * 外部函数 - UART通信 (External Functions - UART Communication)
 * ======================================================================== */

/**
 * @brief 通过UART发送12字节CAN原始帧
 * @details 将CAN帧结构体打包为12字节格式（4字节扩展ID + 8字节数据）并通过UART发送
 *          帧格式：[扩展CAN ID高字节][扩展CAN ID次高字节][扩展CAN ID次低字节][扩展CAN ID低字节][8字节数据负载]
 * @param frame 指向CAN帧结构体的指针
 */
void UART_Send_Frame(const can_frame_t* frame) {
    // Construct the 29-bit extended CAN ID directly from the frame structure
    uint32_t extended_id = ((uint32_t)frame->type << 24) |
                           ((uint32_t)frame->data << 8) |
                           frame->target_id;

    uint8_t packet[CAN_RAW_FRAME_LENGTH];

    // Pack the 4-byte extended CAN ID (big-endian)
    packet[0] = (extended_id >> 24) & 0xFF;
    packet[1] = (extended_id >> 16) & 0xFF;
    packet[2] = (extended_id >> 8) & 0xFF;
    packet[3] = extended_id & 0xFF;

    // Copy the 8-byte data payload
    memcpy(&packet[4], frame->payload, 8);

    // 打印原始发送数据（调试模式 - 仅在测试时启用）
    // ESP_LOGI(TAG, "发送[电机%d]: %02X %02X %02X %02X | %02X %02X %02X %02X %02X %02X %02X %02X",
    //          frame->target_id,
    //          packet[0], packet[1], packet[2], packet[3],
    //          packet[4], packet[5], packet[6], packet[7],
    //          packet[8], packet[9], packet[10], packet[11]);

    // Send via UART
    int written = uart_write_bytes(RS01_UART_NUM, packet, CAN_RAW_FRAME_LENGTH);
    if (written != CAN_RAW_FRAME_LENGTH) {
        ESP_LOGE(TAG, "UART write failed, expected %d, wrote %d", CAN_RAW_FRAME_LENGTH, written);
    }
}

/**
 * @brief 初始化RS01电机UART通信
 * @details 配置UART参数（115200bps, 8N1），安装UART驱动，并初始化电机结构体数组
 *          配置内容：
 *          - 波特率：115200
 *          - 数据位：8
 *          - 校验位：无
 *          - 停止位：1
 *          - UART端口：UART_NUM_1
 *          - TX引脚：GPIO10, RX引脚：GPIO11
 * @param callback 电机数据更新回调函数，当接收到电机反馈时调用
 */
void UART_Rx_Init(MotorDataCallback callback) {
    data_callback = callback;

    // Configure UART parameters
    motor_uart_config.baud_rate = RS01_UART_BAUDRATE;
    motor_uart_config.data_bits = UART_DATA_8_BITS;
    motor_uart_config.parity = UART_PARITY_DISABLE;
    motor_uart_config.stop_bits = UART_STOP_BITS_1;
    motor_uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    motor_uart_config.source_clk = UART_SCLK_DEFAULT;

    // Delete existing UART driver if any (ignore error if not installed)
    uart_driver_delete(RS01_UART_NUM);

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(RS01_UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS01_UART_NUM, &motor_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS01_UART_NUM, RS01_UART_TX_PIN, RS01_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Initialize motor structures (统一为宇树格式)
    for (int i = 0; i < 2; i++) {
        motors[i].id = (i == 0) ? MOTER_1_ID : MOTER_2_ID;
        motors[i].mode = 0;         // 0:Reset, 1:Cali, 2:Motor
        motors[i].pos = 0.0f;       // 位置 (rad)
        motors[i].vel = 0.0f;       // 速度 (rad/s)
        motors[i].t = 0.0f;         // 力矩 (N.m)
        motors[i].temp = 0;         // 温度 (°C)
        motors[i].MError = 0;       // 错误状态码（位域）
        motors[i].footForce = 0;    // RS01无此传感器，固定为0
    }

    // 清空UART接收缓冲区，确保干净启动
    uart_flush_input(RS01_UART_NUM);

    ESP_LOGI(TAG, "UART initialized for motor communication");
}

/**
 * @brief [静态] 解析接收到的12字节CAN帧（参考宇树解析风格）
 * @details RS01 CAN帧格式（12字节）：
 *          字节0-3:  扩展CAN ID（29位）
 *                    - Bit 24-28: 帧类型 (0x02=反馈, 0x18=自动上报)
 *                    - Bit 16-23: 状态字段
 *                      * Bit 16:    欠压错误
 *                      * Bit 17:    驱动器故障
 *                      * Bit 18:    过温错误
 *                      * Bit 19:    磁编码器错误
 *                      * Bit 20:    过载错误
 *                      * Bit 21:    未校准错误
 *                      * Bit 22-23: 运行模式 (0:Reset, 1:Cali, 2:Motor)
 *                    - Bit 8-15:  电机ID
 *          字节4-5:  位置反馈 (uint16_t, 大端序)
 *          字节6-7:  速度反馈 (uint16_t, 大端序)
 *          字节8-9:  力矩反馈 (uint16_t, 大端序)
 *          字节10-11: 温度 (uint16_t, 大端序, 单位0.1°C)
 *
 *          解析完成后触发回调函数（如果已注册）
 *
 * @param can_payload 指向12字节CAN帧数据的指针
 */
static void parse_can_frame(const uint8_t* can_payload) {
    // ========== 解析扩展CAN ID（字节0-3） ==========
    uint32_t extended_id = ((uint32_t)can_payload[0] << 24) |
                           ((uint32_t)can_payload[1] << 16) |
                           ((uint32_t)can_payload[2] << 8) |
                           can_payload[3];

    uint8_t type = (extended_id >> 24) & 0x1F;  // 帧类型
    uint8_t motor_id = (extended_id >> 8) & 0xFF;  // 电机ID
    uint32_t status_part = extended_id >> 16;  // 状态字段

    // 只处理反馈帧(0x02)和自动上报帧(0x18)
    if (type != 0x02 && type != 0x18) {
        return;
    }

    // 检查电机ID是否有效
    if (motor_id < MOTER_1_ID || motor_id > MOTER_2_ID) {
        return;
    }

    // ========== 解析电机数据（字节4-11） ==========
    MI_Motor* motor = &motors[motor_id - 1];  // 电机1→索引0，电机2→索引1
    motor->id = motor_id;

    const uint8_t* data = &can_payload[4];  // 数据负载起始位置

    // 解析位置 - 字节4-5 (uint16_t, 大端序, 范围-12.57~12.57 rad)
    uint16_t raw_pos = (data[0] << 8) | data[1];
    motor->pos = uint_to_float(raw_pos, P_MIN, P_MAX, 16);

    // 解析速度 - 字节6-7 (uint16_t, 大端序, 范围-44~44 rad/s)
    uint16_t raw_vel = (data[2] << 8) | data[3];
    motor->vel = uint_to_float(raw_vel, V_MIN, V_MAX, 16);

    // 解析力矩 - 字节8-9 (uint16_t, 大端序, 范围-17~17 Nm)
    uint16_t raw_torque = (data[4] << 8) | data[5];
    motor->t = uint_to_float(raw_torque, T_MIN, T_MAX, 16);

    // 解析温度 - 字节10-11 (uint16_t, 大端序, 单位0.1°C)
    uint16_t raw_temp = (data[6] << 8) | data[7];
    motor->temp = (int16_t)(raw_temp / 10);

    // ========== 解析状态信息（从扩展ID提取） ==========

    // 构建MError位域（Bit0-5对应6种错误）
    motor->MError = 0;
    if (status_part & (1 << 0)) motor->MError |= MI_ERROR_UNDERVOLTAGE;      // Bit 16
    if (status_part & (1 << 1)) motor->MError |= MI_ERROR_DRIVER_FAULT;      // Bit 17
    if (status_part & (1 << 2)) motor->MError |= MI_ERROR_OVER_TEMPERATURE;  // Bit 18
    if (status_part & (1 << 3)) motor->MError |= MI_ERROR_MAGNETIC_ENCODER;  // Bit 19
    if (status_part & (1 << 4)) motor->MError |= MI_ERROR_OVERLOAD;          // Bit 20
    if (status_part & (1 << 5)) motor->MError |= MI_ERROR_UNCALIBRATED;      // Bit 21

    // 解析运行模式 - Bit 22-23
    motor->mode = (status_part >> 6) & 0x03;  // 0:Reset, 1:Cali, 2:Motor

    // RS01无足端力传感器，固定为0
    motor->footForce = 0;

    // ========== 调试打印（每2秒一次） ==========
    TickType_t currentTime = xTaskGetTickCount();
    int motorIndex = (motor->id == 1) ? 0 : 1;

    if (currentTime - lastPrintTime[motorIndex] >= PRINT_INTERVAL) {
        // ESP_LOGI(TAG, "[ID:%d] Pos:%.2f Vel:%.2f T:%.2f Temp:%d°C Mode:%d MError:0x%02X",
        //          motor->id, motor->pos, motor->vel, motor->t, motor->temp, motor->mode, motor->MError);
        lastPrintTime[motorIndex] = currentTime;
    }

    // 触发回调函数（如果已注册）
    if (data_callback) {
        data_callback(motor);
    }
}

/* ========================================================================
 * 外部函数 - 电机控制 (External Functions - Motor Control)
 * ======================================================================== */

/**
 * @brief 使能电机
 * @details 发送使能命令（Type 3）到指定电机，使电机进入运行状态
 * @param motor 指向电机结构体的指针
 */
void Motor_Enable(MI_Motor* motor) {
    can_frame_t frame;
    frame.type = 0x03;
    frame.target_id = motor->id;
    frame.data = MASTER_ID; // 使用MASTER_ID
    memset(frame.payload, 0, sizeof(frame.payload)); // Data payload is all zeros for enable command

    UART_Send_Frame(&frame);
    ESP_LOGI(TAG, "Motor %d enabled", motor->id);
}

/**
 * @brief 复位电机
 * @details 发送复位命令（Type 4）到指定电机
 * @param motor 指向电机结构体的指针
 * @param clear_error 是否清除错误标志（1：清除，0：不清除）
 */
void Motor_Reset(MI_Motor* motor, uint8_t clear_error) {
    can_frame_t frame;
    frame.type = 0x04;
    frame.target_id = motor->id;
    frame.data = MASTER_ID;
    memset(frame.payload, 0, sizeof(frame.payload));
    if (clear_error) {
        frame.payload[0] = 1;
    }

    UART_Send_Frame(&frame);
    ESP_LOGI(TAG, "Motor %d reset", motor->id);
}

/**
 * @brief 设置电机零点
 * @details 将电机当前位置设置为零点位置（Type 6命令）
 * @param motor 指向电机结构体的指针
 */
void Motor_Set_Zero(MI_Motor* motor) {
    can_frame_t frame;
    frame.type = 0x06;
    frame.target_id = motor->id;
    frame.data = MASTER_ID;
    memset(frame.payload, 0, sizeof(frame.payload));
    frame.payload[0] = 1;

    UART_Send_Frame(&frame);
    ESP_LOGI(TAG, "Motor %d set zero position", motor->id);
}

/**
 * @brief 设置电流模式
 * @details 将电机设置为电流控制模式并设置目标电流
 * @param motor 指向电机结构体的指针
 * @param current 目标电流值（A），范围：-23 ~ 23A
 */
void Set_CurMode(MI_Motor* motor, float current) {
    Set_SingleParameter(motor, IQ_REF, current);
}

/**
 * @brief 设置速度模式
 * @details 将电机设置为速度控制模式，并设置目标速度和电流限制
 * @param motor 指向电机结构体的指针
 * @param speed 目标速度（rad/s），范围：-44 ~ 44 rad/s
 * @param current_limit 电流限制（A），范围：0 ~ 23A
 */
void Set_SpeedMode(MI_Motor* motor, float speed, float current_limit) {
    // 设置速度指令
    Set_SingleParameter(motor, SPD_REF, speed);

    // 设置电流限制
    Set_SingleParameter(motor, LIMIT_CUR, current_limit);
}

/**
 * @brief 切换电机运行模式
 * @details 切换电机的运行模式（Type 18命令）
 * @param motor 指向电机结构体的指针
 * @param mode 运行模式：
 *             0 - 运控模式（Motion Control）
 *             1 - 位置模式PP（Position Profile）
 *             2 - 速度模式（Speed）
 *             3 - 电流模式（Current）
 *             5 - 位置模式CSP（Cyclic Synchronous Position）
 */
void Change_Mode(MI_Motor* motor, uint8_t mode) {
    can_frame_t frame;
    frame.type = 0x12; // Type 18
    frame.target_id = motor->id;
    frame.data = MASTER_ID;

    memset(frame.payload, 0, sizeof(frame.payload));

    uint16_t index = RUN_MODE;
    // Parameter index (little-endian in payload)
    frame.payload[0] = index & 0xFF;
    frame.payload[1] = (index >> 8) & 0xFF;

    // Mode value - 必须按float格式写入（4字节）
    float mode_float = (float)mode;
    memcpy(&frame.payload[4], &mode_float, sizeof(float));

    UART_Send_Frame(&frame);

    ESP_LOGI(TAG, "Change_Mode: 电机%d 设置为模式%d", motor->id, mode);
}

/**
 * @brief 设置单个参数
 * @details 向电机写入单个参数（Type 18命令）
 * @param motor 指向电机结构体的指针
 * @param parameter 参数索引（如IQ_REF, SPD_REF, LOC_REF等）
 * @param value 参数值（float类型）
 */
void Set_SingleParameter(MI_Motor* motor, uint16_t parameter, float value) {
    can_frame_t frame;
    frame.type = 0x12; // Type 18 - 正确的参数设置类型
    frame.data = MASTER_ID; // 使用MASTER_ID
    frame.target_id = motor->id;

    memset(frame.payload, 0, sizeof(frame.payload));

    // Parameter index (little-endian in payload)
    frame.payload[0] = parameter & 0xFF;
    frame.payload[1] = (parameter >> 8) & 0xFF;

    // Parameter value (float, little-endian)
    memcpy(&frame.payload[4], &value, sizeof(float));

    UART_Send_Frame(&frame);
}

/**
 * @brief 设置电机自动上报
 * @details 开启或关闭电机数据自动上报功能（Type 24命令）
 * @param motor 指向电机结构体的指针
 * @param enable true：开启自动上报，false：关闭自动上报
 */
void Motor_SetReporting(MI_Motor* motor, bool enable) {
    can_frame_t frame;
    frame.type = 0x18; // Type 24 - 上报控制类型
    frame.data = MASTER_ID; // 使用MASTER_ID
    frame.target_id = motor->id;

    // 设置数据区的8位固定序列
    frame.payload[0] = 0x01;
    frame.payload[1] = 0x02;
    frame.payload[2] = 0x03;
    frame.payload[3] = 0x04;
    frame.payload[4] = 0x05;
    frame.payload[5] = 0x06;
    frame.payload[6] = enable ? 0x01 : 0x00; // 0x00关闭, 0x01开启
    frame.payload[7] = 0x00; // 保留位

    UART_Send_Frame(&frame);
}

/**
 * @brief MIT运控模式控制
 * @details 使用MIT运控模式（Type 1）进行电机控制，支持位置/速度/力矩混合控制
 *          控制方程：τ = τ_ff + Kp(θ_des - θ) + Kd(ω_des - ω)
 *
 * @param motor 指向电机结构体的指针
 * @param torque 前馈力矩（Nm），范围：-17 ~ 17Nm
 * @param position 目标位置（rad），范围：-12.57 ~ 12.57rad
 * @param speed 目标速度（rad/s），范围：-44 ~ 44rad/s
 * @param kp 位置增益，范围：0 ~ 500
 * @param kd 阻尼增益，范围：0 ~ 5
 */
void Motor_ControlMode(MI_Motor* motor, float torque, float position, float speed, float kp, float kd) {
    can_frame_t frame;

    // 数据转换：使用线性映射
    uint16_t torque_cmd = float_to_uint16_linear(torque, T_MIN, T_MAX);       // 力矩 -17~17Nm
    uint16_t pos_cmd = float_to_uint16_linear(position, P_MIN, P_MAX);        // 角度 -12.57~12.57f
    uint16_t vel_cmd = float_to_uint16_linear(speed, V_MIN, V_MAX);           // 角速度 -44~44rad/s
    uint16_t kp_cmd = float_to_uint16_linear(kp, KP_MIN, KP_MAX);             // Kp 0.0~500.0
    uint16_t kd_cmd = float_to_uint16_linear(kd, KD_MIN, KD_MAX);             // Kd 0.0~5.0

    // 运控模式帧头构造
    frame.type = 0x01;                           // 运控模式类型
    frame.data = torque_cmd;                     // 力矩值直接放在data字段(16位)
    frame.target_id = motor->id;                 // 目标电机CAN_ID

    // 8字节数据区：目标角度 + 目标角速度 + Kp + Kd
    frame.payload[0] = (pos_cmd >> 8) & 0xFF;    // 目标角度高字节
    frame.payload[1] = pos_cmd & 0xFF;           // 目标角度低字节
    frame.payload[2] = (vel_cmd >> 8) & 0xFF;    // 目标角速度高字节
    frame.payload[3] = vel_cmd & 0xFF;           // 目标角速度低字节
    frame.payload[4] = (kp_cmd >> 8) & 0xFF;     // Kp高字节
    frame.payload[5] = kp_cmd & 0xFF;            // Kp低字节
    frame.payload[6] = (kd_cmd >> 8) & 0xFF;     // Kd高字节
    frame.payload[7] = kd_cmd & 0xFF;            // Kd低字节

    UART_Send_Frame(&frame);
}
/**
 * @brief MIT运控模式同步发送并接收反馈
 * @details 发送控制指令后，主动接收并解析反馈数据，类似Unitree的sendRecv
 *          工作流程：
 *          1. 发送MIT运控模式控制指令
 *          2. 等待UART TX完成
 *          3. 直接读取UART接收的12字节CAN帧
 *          4. 调用parse_can_frame解析数据并更新motors[]数组
 *
 * @param motor 指向电机结构体的指针
 * @param torque 前馈力矩（Nm），范围：-17 ~ 17Nm
 * @param position 目标位置（rad），范围：-12.57 ~ 12.57rad
 * @param speed 目标速度（rad/s），范围：-44 ~ 44rad/s
 * @param kp 位置增益，范围：0 ~ 500
 * @param kd 阻尼增益，范围：0 ~ 5
 * @return 0 成功, -1 失败
 */
int Motor_ControlMode_SendRecv(MI_Motor* motor, float torque, float position, float speed, float kp, float kd) {
    // 1. 发送控制指令
    Motor_ControlMode(motor, torque, position, speed, kp, kd);

    // 2. 等待TX FIFO清空
    uart_wait_tx_done(RS01_UART_NUM, pdMS_TO_TICKS(10));

    // 3. 直接读取UART数据（12字节CAN帧）
    uint8_t rx_buffer[CAN_RAW_FRAME_LENGTH];
    int rx_bytes = uart_read_bytes(RS01_UART_NUM, rx_buffer, CAN_RAW_FRAME_LENGTH, pdMS_TO_TICKS(10));

    if (rx_bytes == CAN_RAW_FRAME_LENGTH) {
        // 4. 直接调用parse_can_frame解析数据
        parse_can_frame(rx_buffer);
        return 0;
    } else {
        // 接收超时或数据不完整
        ESP_LOGW(TAG, "接收数据失败: 期望%d字节, 实际接收%d字节", CAN_RAW_FRAME_LENGTH, rx_bytes);
        return -1;
    }
}
