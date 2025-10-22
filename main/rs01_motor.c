#include "rs01_motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "RS01_MOTOR";

// Global variables
twai_node_handle_t twai_node_handle = NULL;  // TWAI节点句柄
EventGroupHandle_t motor_data_event_group = NULL;  // 电机数据同步事件组
QueueHandle_t twai_rx_queue = NULL;  // TWAI接收队列（用于ISR到任务的通信）
MI_Motor motors[2];
MotorDataCallback data_callback = NULL;

// 调试打印时间控制（每2秒打印一次）
static TickType_t lastPrintTime[2] = {0, 0};
static const TickType_t PRINT_INTERVAL = pdMS_TO_TICKS(2000);

// TWAI帧存储结构（用于队列传输，包含实际数据而非指针）
typedef struct {
    twai_frame_header_t header;
    uint8_t data[8];
    uint8_t data_len;
} twai_frame_storage_t;

/* ========================================================================
 * 静态函数声明 (Static Function Declarations)
 * ======================================================================== */
static void parse_can_frame(const twai_frame_t* rx_frame);
static bool IRAM_ATTR twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);

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
 * 外部函数 - TWAI通信 (External Functions - TWAI Communication)
 * ======================================================================== */

/**
 * @brief 通过TWAI发送CAN扩展帧
 * @details 将CAN帧结构体转换为TWAI帧格式并发送
 *          扩展帧ID格式：[帧类型5位][数据字段16位][目标ID 8位] = 29位扩展ID
 * @param frame 指向CAN帧结构体的指针
 */
void TWAI_Send_Frame(const can_frame_t* frame) {
    // 构造29位扩展CAN ID
    uint32_t extended_id = ((uint32_t)frame->type << 24) |
                           ((uint32_t)frame->data << 8) |
                           frame->target_id;

    // 构造TWAI帧
    twai_frame_t tx_msg = {
        .header.id = extended_id,
        .header.ide = true,  // 使用29位扩展ID
        .header.rtr = false, // 数据帧（非远程帧）
        .header.fdf = false, // 经典CAN格式（非FD）
        .header.brs = false, // 不使用位速率切换
        .buffer = (uint8_t*)frame->payload,
        .buffer_len = 8
    };

    // 发送CAN帧（超时0表示队列满时立即返回）
    esp_err_t ret = twai_node_transmit(twai_node_handle, &tx_msg, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI发送失败: %s (ID=0x%lx)", esp_err_to_name(ret), extended_id);
    }
}

/**
 * @brief TWAI接收中断回调函数（在ISR上下文中执行）
 * @details 当TWAI接收到CAN帧时触发，从ISR中读取帧并放入队列供任务处理
 *          注意：不在ISR中进行浮点运算，避免协处理器异常
 * @param handle TWAI节点句柄
 * @param edata 接收事件数据
 * @param user_ctx 用户上下文（未使用）
 * @return false表示不需要唤醒任务
 */
static bool IRAM_ATTR twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx) {
    twai_frame_t rx_frame = {0};
    uint8_t data_buffer[8];  // 为CAN数据分配缓冲区

    // 设置缓冲区指针，避免空指针解引用
    rx_frame.buffer = data_buffer;
    rx_frame.buffer_len = sizeof(data_buffer);

    // 从ISR中读取接收到的帧
    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
        // 统计接收到的帧数量（用于诊断）
        static uint32_t rx_count = 0;
        rx_count++;

        // 只处理扩展帧（29位ID）
        if (rx_frame.header.ide && twai_rx_queue != NULL) {
            // 创建存储结构，将数据复制进去（避免指针失效）
            twai_frame_storage_t frame_storage = {
                .header = rx_frame.header,
                .data_len = rx_frame.buffer_len
            };
            if (rx_frame.buffer_len <= sizeof(frame_storage.data)) {
                memcpy(frame_storage.data, rx_frame.buffer, rx_frame.buffer_len);
            }

            // 将帧放入队列，由任务处理（避免在ISR中进行浮点运算）
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(twai_rx_queue, &frame_storage, &xHigherPriorityTaskWoken);

            // 如果发送到队列导致更高优先级任务就绪，则进行上下文切换
            if (xHigherPriorityTaskWoken) {
                return true;  // 需要上下文切换
            }
        }
    }

    return false;  // 不需要上下文切换
}

/**
 * @brief TWAI帧处理任务（在任务上下文中执行）
 * @details 从队列中接收CAN帧并进行解析处理
 *          在任务上下文中执行浮点运算是安全的
 * @param arg 任务参数（未使用）
 */
static void twai_rx_task(void *arg) {
    twai_frame_storage_t frame_storage;
    twai_frame_t rx_frame;

    ESP_LOGI(TAG, "TWAI接收处理任务启动");

    while (1) {
        // 从队列中接收帧存储结构（阻塞等待）
        if (xQueueReceive(twai_rx_queue, &frame_storage, portMAX_DELAY) == pdTRUE) {
            // 重构twai_frame_t结构供解析使用
            rx_frame.header = frame_storage.header;
            rx_frame.buffer = frame_storage.data;
            rx_frame.buffer_len = frame_storage.data_len;

            // 在任务上下文中解析帧（可以安全地进行浮点运算）
            parse_can_frame(&rx_frame);
        }
    }
}

/**
 * @brief 初始化RS01电机TWAI通信
 * @details 配置TWAI参数（200kbps CAN总线），创建TWAI节点，注册接收回调，并初始化电机结构体数组
 *          配置内容：
 *          - 波特率：200kbps
 *          - TX引脚：GPIO10, RX引脚：GPIO11
 *          - 扩展帧格式（29位ID）
 *          - 事件驱动接收模式
 * @param callback 电机数据更新回调函数，当接收到电机反馈时调用
 */
void TWAI_Init(MotorDataCallback callback) {
    data_callback = callback;

    // 创建电机数据同步事件组
    motor_data_event_group = xEventGroupCreate();
    if (motor_data_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create motor_data_event_group!");
        return;
    }
    ESP_LOGI(TAG, "电机数据同步事件组创建成功");

    // 创建TWAI接收队列（用于ISR到任务的通信）
    twai_rx_queue = xQueueCreate(10, sizeof(twai_frame_storage_t));
    if (twai_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create twai_rx_queue!");
        return;
    }
    ESP_LOGI(TAG, "TWAI接收队列创建成功");

    // 配置TWAI节点参数
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = RS01_TWAI_TX_PIN,
        .io_cfg.rx = RS01_TWAI_RX_PIN,
        .bit_timing.bitrate = RS01_TWAI_BITRATE,  // 200kbps波特率
        .tx_queue_depth = RS01_TWAI_TX_QUEUE_DEPTH,
        .fail_retry_cnt = -1,  // 无限重试
        .intr_priority = 1,    // 中断优先级
    };

    // 创建TWAI控制器驱动实例
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &twai_node_handle));

    // 注册接收完成事件回调
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(twai_node_handle, &user_cbs, NULL));

    // 启动TWAI控制器
    ESP_ERROR_CHECK(twai_node_enable(twai_node_handle));

    // 初始化电机结构体（统一为宇树格式）
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

    // 创建TWAI接收处理任务
    BaseType_t ret = xTaskCreate(twai_rx_task, "twai_rx_task", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create twai_rx_task!");
        return;
    }

    ESP_LOGI(TAG, "TWAI初始化完成 (TX:%d, RX:%d, 波特率:%d)",
             RS01_TWAI_TX_PIN, RS01_TWAI_RX_PIN, RS01_TWAI_BITRATE);
}

/**
 * @brief [静态] 解析接收到的TWAI CAN扩展帧（参考宇树解析风格）
 * @details RS01 CAN帧格式（扩展帧29位ID + 8字节数据）：
 *          扩展CAN ID（29位）：
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
 *          数据负载（8字节）：
 *          字节0-1:  位置反馈 (uint16_t, 大端序)
 *          字节2-3:  速度反馈 (uint16_t, 大端序)
 *          字节4-5:  力矩反馈 (uint16_t, 大端序)
 *          字节6-7:  温度 (uint16_t, 大端序, 单位0.1°C)
 *
 *          解析完成后触发回调函数（如果已注册）
 *
 * @param rx_frame 指向TWAI接收帧的指针
 */
static void parse_can_frame(const twai_frame_t* rx_frame) {
    // ========== 解析扩展CAN ID ==========
    uint32_t extended_id = rx_frame->header.id;

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

    // ========== 解析电机数据（8字节负载） ==========
    MI_Motor* motor = &motors[motor_id - 1];  // 电机1→索引0，电机2→索引1
    motor->id = motor_id;

    const uint8_t* data = rx_frame->buffer;  // 数据负载起始位置

    // 解析位置 - 字节0-1 (uint16_t, 大端序, 范围-12.57~12.57 rad)
    uint16_t raw_pos = (data[0] << 8) | data[1];
    motor->pos = uint_to_float(raw_pos, P_MIN, P_MAX, 16);

    // 解析速度 - 字节2-3 (uint16_t, 大端序, 范围-44~44 rad/s)
    uint16_t raw_vel = (data[2] << 8) | data[3];
    motor->vel = uint_to_float(raw_vel, V_MIN, V_MAX, 16);

    // 解析力矩 - 字节4-5 (uint16_t, 大端序, 范围-17~17 Nm)
    uint16_t raw_torque = (data[4] << 8) | data[5];
    motor->t = uint_to_float(raw_torque, T_MIN, T_MAX, 16);

    // 解析温度 - 字节6-7 (uint16_t, 大端序, 单位0.1°C)
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

    // ========== 设置数据就绪事件位（用于同步等待） ==========
    // 根据电机ID设置对应的事件位，通知等待线程数据已就绪
    // 注意：此函数现在在任务上下文中执行，使用xEventGroupSetBits（非ISR版本）
    if (motor_data_event_group != NULL) {
        EventBits_t event_bit = (motor_id == MOTER_1_ID) ? MOTOR1_DATA_READY_BIT : MOTOR2_DATA_READY_BIT;
        xEventGroupSetBits(motor_data_event_group, event_bit);
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

    TWAI_Send_Frame(&frame);
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

    TWAI_Send_Frame(&frame);
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

    TWAI_Send_Frame(&frame);
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

    TWAI_Send_Frame(&frame);

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

    TWAI_Send_Frame(&frame);
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

    TWAI_Send_Frame(&frame);
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

    TWAI_Send_Frame(&frame);
}
/**
 * @brief MIT运控模式同步发送并接收反馈（TWAI事件同步模式）
 * @details 同步通信流程：
 *          1. 清除对应电机的数据就绪事件位
 *          2. 调用Motor_ControlMode发送控制指令
 *          3. 等待TWAI中断回调设置数据就绪事件位（超时10ms）
 *          4. 返回成功或超时错误
 *
 *          数据更新机制：
 *          - 电机反馈通过TWAI中断回调异步接收
 *          - 回调函数解析数据后更新motors[]数组并设置事件位
 *          - 本函数通过事件组同步等待，确保返回时数据已更新
 *
 * @param motor 指向电机结构体的指针
 * @param torque 前馈力矩（Nm），范围：-17 ~ 17Nm
 * @param position 目标位置（rad），范围：-12.57 ~ 12.57rad
 * @param speed 目标速度（rad/s），范围：-44 ~ 44rad/s
 * @param kp 位置增益，范围：0 ~ 500
 * @param kd 阻尼增益，范围：0 ~ 5
 * @return 0 成功接收到反馈数据，-1 超时或发送失败
 */
int Motor_ControlMode_SendRecv(MI_Motor* motor, float torque, float position, float speed, float kp, float kd) {
    if (motor_data_event_group == NULL) {
        ESP_LOGE(TAG, "Motor_ControlMode_SendRecv: 事件组未初始化！");
        return -1;
    }

    // 确定要等待的事件位（电机1或电机2）
    EventBits_t event_bit = (motor->id == MOTER_1_ID) ? MOTOR1_DATA_READY_BIT : MOTOR2_DATA_READY_BIT;

    // 清除对应电机的事件位，准备接收新数据
    xEventGroupClearBits(motor_data_event_group, event_bit);

    // 发送控制指令
    Motor_ControlMode(motor, torque, position, speed, kp, kd);

    // 等待数据就绪事件（超时时间由MOTOR_DATA_TIMEOUT_MS定义）
    EventBits_t bits = xEventGroupWaitBits(
        motor_data_event_group,           // 事件组句柄
        event_bit,                        // 要等待的事件位
        pdTRUE,                           // 返回前清除事件位
        pdFALSE,                          // 不需要等待所有位
        pdMS_TO_TICKS(MOTOR_DATA_TIMEOUT_MS)  // 超时时间10ms
    );

    // 检查是否成功接收到数据
    if (bits & event_bit) {
        // 数据已就绪，motors[]数组已被中断回调更新
        return 0;
    } else {
        // 超时，未收到电机反馈 - 限制打印频率为5秒一次
        static TickType_t lastTimeoutPrint[2] = {0, 0};  // 每个电机独立计时
        static const TickType_t TIMEOUT_PRINT_INTERVAL = pdMS_TO_TICKS(5000);  // 5秒
        TickType_t currentTime = xTaskGetTickCount();
        int motorIndex = (motor->id == MOTER_1_ID) ? 0 : 1;

        if (currentTime - lastTimeoutPrint[motorIndex] >= TIMEOUT_PRINT_INTERVAL) {
            ESP_LOGW(TAG, "Motor_ControlMode_SendRecv: 电机%d反馈超时", motor->id);
            lastTimeoutPrint[motorIndex] = currentTime;
        }
        return -1;
    }
}
