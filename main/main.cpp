#include <iostream>
#include <string.h> // For strtok
#include <stdlib.h> // For atof, atoi
#include <algorithm> // For std::min, resolves 'MIN' was not declared in this scope
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For Semaphore
#include "esp_log.h"
#include "esp_timer.h"
#include "unitree_motor.h"
#include "rs01_motor.h"          // RS01电机驱动
#include "motor_commands.h"
#include "speed_follow_mode.h"
#include "position_buffer.h"
#include "driver/gpio.h"
#include "wifi_webserver.h"
#include "voice_module.h"
#include "button_detector.h"

static const char *TAG = "MAIN";

// 语音模块实例（全局可见，供C和C++代码使用）
VoiceModule voice_module{};

// Web服务器句柄
static httpd_handle_t web_server = NULL;

// 定义电机ID和UART端口/引脚
#define MOTOR_ID_1      0x01
#define MOTOR_ID_2      0x02
#define UART_PORT_NUM   UART_NUM_2
#define UART_TX_PIN     GPIO_NUM_13
#define UART_RX_PIN     GPIO_NUM_12
// #define MAX485_RE_DE_PIN GPIO_NUM_6  // 纯串口模式不需要DE/RE控制引脚
#define UART_BAUD_RATE  4000000

UnitreeMotorDriver motor_driver;
SpeedFollowMode speed_follow; // 速度跟随模式实例
motor_position_buffers_t position_buffers; // 位置缓存区


// Motor command parameters for both motors, protected by a mutex
struct MotorParams {
    uint8_t motor_id;
    uint8_t motor_mode;
    float motor_pos;
    float motor_vel;
    float motor_t;
    float motor_kp;
    float motor_kd;
};

static MotorParams global_motor_1 = {MOTOR_ID_1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // FOC模式
static MotorParams global_motor_2 = {MOTOR_ID_2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // FOC模式

SemaphoreHandle_t motor_params_mutex; // 用于保护电机参数的互斥锁

// 速度跟随模式配置参数
static float global_speed_follow_threshold = 1.0f; // 自动激活阈值（可调整）

/**
 * @brief Web服务器命令处理回调
 * @param cmd 接收到的命令字符串 ("start" 或 "stop")
 * @return ESP_OK 成功, ESP_FAIL 未知命令
 */
extern "C" esp_err_t handle_web_command(const char *cmd) {
    if (strcmp(cmd, "start") == 0) {
        ESP_LOGI(TAG, "[WEB] 接收到启动命令");
        speed_follow.enable(true);
        return ESP_OK;
    }
    else if (strcmp(cmd, "stop") == 0) {
        ESP_LOGI(TAG, "[WEB] 接收到停止命令");
        speed_follow.enable(false);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "[WEB] 未知命令: %s", cmd);
    return ESP_FAIL;
}

/**
 * @brief Web服务器参数设置回调
 * @param param_name 参数名称 (如 "threshold")
 * @param value 参数值
 * @return ESP_OK 成功, ESP_FAIL 未知参数
 */
extern "C" esp_err_t handle_web_param(const char *param_name, float value) {
    if (strcmp(param_name, "threshold") == 0) {
        ESP_LOGI(TAG, "[WEB] 设置阈值: %.2f", value);
        global_speed_follow_threshold = value;
        speed_follow.setThreshold(value);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "[WEB] 未知参数: %s", param_name);
    return ESP_FAIL;
}

/**
 * @brief Web服务器电机参数设置回调
 * @param motor 电机编号 (1 或 2)
 * @param param_name 参数名称 (如 "trigger_speed", "p1_vel" 等)
 * @param value 参数值
 * @return ESP_OK 成功, ESP_FAIL 未知参数
 */
extern "C" esp_err_t handle_web_motor_param(int motor, const char *param_name, float value) {
    ESP_LOGI(TAG, "[WEB] 电机%d - %s: %.4f", motor, param_name, value);

    // 获取对应电机的配置
    speed_follow_config_t* config = speed_follow.getMotorConfig(motor);

    if (strcmp(param_name, "trigger_speed") == 0) {
        config->trigger_speed = value;
    }
    else if (strcmp(param_name, "phase1_duration") == 0) {
        config->phase1_duration_ms = (uint32_t)value;
    }
    else if (strcmp(param_name, "phase2_duration") == 0) {
        config->phase2_duration_ms = (uint32_t)value;
    }
    else if (strcmp(param_name, "waiting_duration") == 0) {
        config->waiting_duration_ms = (uint32_t)value;
    }
    else if (strcmp(param_name, "idle_duration") == 0) {
        config->idle_duration_ms = (uint32_t)value;
    }
    // Phase1参数
    else if (strcmp(param_name, "p1_vel") == 0) {
        config->phase1.vel = value;
    }
    else if (strcmp(param_name, "p1_torque") == 0) {
        config->phase1.torque = value;
    }
    else if (strcmp(param_name, "p1_kp") == 0) {
        config->phase1.kp = value;
    }
    else if (strcmp(param_name, "p1_kd") == 0) {
        config->phase1.kd = value;
    }
    // Phase2参数
    else if (strcmp(param_name, "p2_vel") == 0) {
        config->phase2.vel = value;
    }
    else if (strcmp(param_name, "p2_torque") == 0) {
        config->phase2.torque = value;
    }
    else if (strcmp(param_name, "p2_kp") == 0) {
        config->phase2.kp = value;
    }
    else if (strcmp(param_name, "p2_kd") == 0) {
        config->phase2.kd = value;
    }
    else {
        ESP_LOGW(TAG, "[WEB] 未知电机参数: %s", param_name);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Web服务器电机参数读取回调
 * @param motor 电机编号 (1 或 2)
 * @param param_name 参数名称
 * @param value 输出参数，用于返回参数值
 * @return ESP_OK 成功, ESP_FAIL 未知参数
 */
extern "C" esp_err_t handle_web_get_motor_param(int motor, const char *param_name, float *value) {
    // 获取对应电机的配置
    speed_follow_config_t* config = speed_follow.getMotorConfig(motor);

    if (strcmp(param_name, "trigger_speed") == 0) {
        *value = config->trigger_speed;
    }
    else if (strcmp(param_name, "phase1_duration") == 0) {
        *value = (float)config->phase1_duration_ms;
    }
    else if (strcmp(param_name, "phase2_duration") == 0) {
        *value = (float)config->phase2_duration_ms;
    }
    else if (strcmp(param_name, "waiting_duration") == 0) {
        *value = (float)config->waiting_duration_ms;
    }
    else if (strcmp(param_name, "idle_duration") == 0) {
        *value = (float)config->idle_duration_ms;
    }
    // Phase1参数
    else if (strcmp(param_name, "p1_vel") == 0) {
        *value = config->phase1.vel;
    }
    else if (strcmp(param_name, "p1_torque") == 0) {
        *value = config->phase1.torque;
    }
    else if (strcmp(param_name, "p1_kp") == 0) {
        *value = config->phase1.kp;
    }
    else if (strcmp(param_name, "p1_kd") == 0) {
        *value = config->phase1.kd;
    }
    // Phase2参数
    else if (strcmp(param_name, "p2_vel") == 0) {
        *value = config->phase2.vel;
    }
    else if (strcmp(param_name, "p2_torque") == 0) {
        *value = config->phase2.torque;
    }
    else if (strcmp(param_name, "p2_kp") == 0) {
        *value = config->phase2.kp;
    }
    else if (strcmp(param_name, "p2_kd") == 0) {
        *value = config->phase2.kd;
    }
    else {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief 电机控制任务，以500Hz频率执行双电机同步控制（RS01电机版本）
 * @param pvParameters FreeRTOS任务参数（未使用）
 * @details 执行流程：
 *          1. 读取全局电机参数（互斥锁保护）
 *          2. 使用Motor_ControlMode_SendRecv发送MIT控制指令并接收反馈
 *          3. 从motors[]全局数组读取电机反馈数据
 *          4. 更新位置缓存区并进行波形分析
 *          5. 调用速度跟随状态机 update()
 *          6. 打印电机数据（10通道格式化输出）
 */
void motor_control_task(void *pvParameters) {
    ESP_LOGI(TAG, "电机控制任务启动 - RS01电机同步模式 (500Hz)");

    // 初始化速度跟随模式
    speed_follow.init();

    // 启用自动开关功能
    speed_follow.enableAutoSwitch(true);
    speed_follow.setThreshold(global_speed_follow_threshold);

    // 初始化位置缓存区
    position_buffer_init(&position_buffers);

    // 设置双电机速度跟随模式参数访问
    speed_follow.setDualMotorParams(&global_motor_1.motor_id, &global_motor_1.motor_mode, &global_motor_1.motor_pos,
                                   &global_motor_1.motor_vel, &global_motor_1.motor_t, &global_motor_1.motor_kp, &global_motor_1.motor_kd,
                                   &global_motor_2.motor_id, &global_motor_2.motor_mode, &global_motor_2.motor_pos,
                                   &global_motor_2.motor_vel, &global_motor_2.motor_t, &global_motor_2.motor_kp, &global_motor_2.motor_kd,
                                   motor_params_mutex);

    // 设置差值缓存区访问（用于超时清空）
    speed_follow.setDiffBuffers(&position_buffers);

    // 统计计数器
    uint32_t loop_count = 0;

    // 电机数据变量（用于兼容现有代码）
    MotorDataA1 motor_data_1{};
    MotorDataA1 motor_data_2{};

    // Local variables to hold the current parameters for both motors
    MotorParams current_motor_1, current_motor_2;

    while (1) {
        // Safely read the global parameter values for both motors
        if (xSemaphoreTake(motor_params_mutex, portMAX_DELAY) == pdTRUE) {
            current_motor_1 = global_motor_1;
            current_motor_2 = global_motor_2;
            xSemaphoreGive(motor_params_mutex);

            // 🔍 诊断：每2秒打印一次读取到的参数
            static uint32_t last_print_time = 0;
            uint32_t now = esp_timer_get_time() / 1000;
            if (now - last_print_time > 2000) {
                // ESP_LOGI(TAG, "📖 控制循环读取: M1(vel=%.2f,t=%.2f) M2(vel=%.2f,t=%.2f)",
                //          current_motor_1.motor_vel, current_motor_1.motor_t,
                //          current_motor_2.motor_vel, current_motor_2.motor_t);
                last_print_time = now;
            }
        } else {
            ESP_LOGW(TAG, "Failed to take motor_params_mutex, using default parameters.");
            // Fallback to default values if mutex acquisition fails
            current_motor_1 = {MOTOR_ID_1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            current_motor_2 = {MOTOR_ID_2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }

        // ========== 控制电机1 - RS01 MIT运控模式 ==========
        int result1 = Motor_ControlMode_SendRecv(
            &motors[0],                    // 电机1（ID=1）
            current_motor_1.motor_t,       // torque: MIT力矩参数
            current_motor_1.motor_pos,     // position: MIT位置参数
            current_motor_1.motor_vel,     // speed: MIT速度参数
            current_motor_1.motor_kp,      // kp: MIT位置增益
            current_motor_1.motor_kd       // kd: MIT阻尼增益
        );

        if (result1 == 0) {
            // motors[0]已经是MotorDataA1格式，直接复制即可
            motor_data_1 = motors[0];

            // 添加位置数据到缓存区
            uint32_t timestamp = esp_timer_get_time() / 1000; // 转换为毫秒
            position_buffer_add_motor1(&position_buffers, motor_data_1.pos, timestamp);
        }

        // 确保两次发送指令间隔至少5ms（RS01电机硬件要求）
        vTaskDelay(pdMS_TO_TICKS(1));

        // ========== 控制电机2 - RS01 MIT运控模式 ==========
        int result2 = Motor_ControlMode_SendRecv(
            &motors[1],                    // 电机2（ID=2）
            current_motor_2.motor_t,       // torque: MIT力矩参数
            current_motor_2.motor_pos,     // position: MIT位置参数
            current_motor_2.motor_vel,     // speed: MIT速度参数
            current_motor_2.motor_kp,      // kp: MIT位置增益
            current_motor_2.motor_kd       // kd: MIT阻尼增益
        );

        if (result2 == 0) {
            // motors[1]已经是MotorDataA1格式，直接复制即可
            motor_data_2 = motors[1];

            // 添加位置数据到缓存区
            uint32_t timestamp = esp_timer_get_time() / 1000; // 转换为毫秒
            position_buffer_add_motor2(&position_buffers, motor_data_2.pos, timestamp);
        }

        // 合并打印两个电机的数据：电机1(ch0,ch1,ch2) + 电机2(ch3,ch4,ch5) + 波峰波谷差值最大值(ch6,ch7)
        if (result1 == 0 && result2 == 0) {
            // 分析波形并获取差值
            wave_analysis_result_t motor1_wave, motor2_wave;
            float motor1_diff = 0.0f, motor2_diff = 0.0f;
            uint32_t timestamp = esp_timer_get_time() / 1000;

            if (position_buffer_analyze_motor1_wave(&position_buffers, &motor1_wave)) {
                motor1_diff = motor1_wave.peak_valley_diff;
                // 将ch6差值存入缓存区
                diff_buffer_add_ch6(&position_buffers, motor1_diff, timestamp);
            }

            if (position_buffer_analyze_motor2_wave(&position_buffers, &motor2_wave)) {
                motor2_diff = motor2_wave.peak_valley_diff;
                // 将ch7差值存入缓存区
                diff_buffer_add_ch7(&position_buffers, motor2_diff, timestamp);
            }

            // 获取ch6和ch7缓存区的最大值
            float ch6_max = diff_buffer_get_ch6_max(&position_buffers);
            float ch7_max = diff_buffer_get_ch7_max(&position_buffers);

            // 获取周期高频RMS值作为输出
            float ch6_new = 1.0;
            float ch7_new = 1.0;

            // 检查阈值并可能激活速度跟随模式
            speed_follow.checkThresholdAndActivate(ch6_max, ch7_max);

            // 使用 ch6_max 和 ch7_max 更新速度跟随模式
            speed_follow.update(motor_data_1, ch6_max, ch7_max);
            speed_follow.update(motor_data_2, ch6_max, ch7_max);

            printf("motors:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   motor_data_1.pos, motor_data_1.vel, motor_data_1.t,
                   motor_data_2.pos, motor_data_2.vel, motor_data_2.t,
                   ch6_max, ch7_max, ch6_new, ch7_new);
        }

        loop_count++;

        // 高频率控制: 2ms延时 = 500Hz控制频率
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 应用程序主入口函数
 * @details 初始化顺序：
 *          1. 语音模块初始化
 *          2. 创建电机参数互斥锁
 *          3. WiFi热点初始化（AP模式）
 *          4. Web服务器启动
 *          5. 注册Web回调函数
 *          6. 电机驱动初始化（UART2, 4Mbps）
 *          7. 按键检测器初始化
 *          8. 创建按键检测任务和电机控制任务
 */
extern "C" void app_main() {
    ESP_LOGI(TAG, "ESP32 Unitree 电机驱动示例启动");

    // 初始化语音模块
    voice_module_init(&voice_module);
    voice_speak(&voice_module, "系统启动成功");

    // 为电机参数创建互斥锁
    motor_params_mutex = xSemaphoreCreateMutex();
    if (motor_params_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create motor_params_mutex!");
        return;
    }

    // 初始化WiFi热点
    ESP_LOGI(TAG, "正在初始化WiFi热点...");
    if (wifi_init_softap() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi热点初始化失败！");
        return;
    }

    // 启动Web服务器
    ESP_LOGI(TAG, "正在启动Web服务器...");
    web_server = start_webserver();
    if (web_server == NULL) {
        ESP_LOGE(TAG, "Web服务器启动失败！");
        return;
    }

    // 注册Web服务器回调函数
    register_command_handler(handle_web_command);
    register_param_handler(handle_web_param);
    register_motor_param_handler(handle_web_motor_param);
    register_motor_param_getter(handle_web_get_motor_param);
    ESP_LOGI(TAG, "Web服务器已启动，请连接WiFi: ESP32_Motor_Control, 访问: http://192.168.4.1");

    // ========== 初始化RS01电机驱动 ==========
    // 注意：RS01使用UART1，波特率115200，GPIO 10(TX)/11(RX)
    ESP_LOGI(TAG, "正在初始化RS01电机驱动...");

    // 初始化TWAI（不需要callback，使用事件驱动接收）
    TWAI_Init(NULL);

    // 设置电机1为运控模式并使能
    Change_Mode(&motors[0], CTRL_MODE);  // motors[0] = 电机ID 1
    vTaskDelay(pdMS_TO_TICKS(5));  // 确保指令间隔至少5ms

    Motor_Enable(&motors[0]);
    vTaskDelay(pdMS_TO_TICKS(5));  // 确保指令间隔至少5ms
    ESP_LOGI(TAG, "电机1已设置为运控模式并使能");

    // 设置电机2为运控模式并使能
    Change_Mode(&motors[1], CTRL_MODE);  // motors[1] = 电机ID 2
    vTaskDelay(pdMS_TO_TICKS(5));  // 确保指令间隔至少5ms

    Motor_Enable(&motors[1]);
    vTaskDelay(pdMS_TO_TICKS(5));  // 确保指令间隔至少5ms
    ESP_LOGI(TAG, "电机2已设置为运控模式并使能");

    ESP_LOGI(TAG, "RS01电机驱动初始化成功");

    // ========== 注释掉Unitree电机初始化 ==========
    // 初始化电机驱动
    // if (motor_driver.init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, GPIO_NUM_NC, UART_BAUD_RATE)) {
    //     ESP_LOGI(TAG, "电机驱动初始化成功 - 纯串口同步模式");
    // } else {
    //     ESP_LOGE(TAG, "电机驱动初始化失败！");
    //     return;
    // }

    // 初始化按键检测器
    ESP_LOGI(TAG, "正在初始化按键检测器...");
    button_detector_init();

    // 创建按键检测任务
    xTaskCreate(button_detector_task, "button_detector_task", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "按键检测任务已创建");

    // 创建电机控制任务
    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);
}
