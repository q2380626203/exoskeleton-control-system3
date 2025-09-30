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
#include "motor_commands.h"
#include "speed_follow_mode.h"
#include "position_buffer.h"
#include "driver/gpio.h"

static const char *TAG = "MAIN";

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








// 电机控制任务
void motor_control_task(void *pvParameters) {
    if (!motor_driver.isInitialized()) {
        ESP_LOGE(TAG, "电机驱动未初始化！");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "电机控制任务启动 - 纯串口同步模式 (500Hz)");

    // 初始化速度跟随模式
    speed_follow.init();

    // 启用自动开关功能
    speed_follow.enableAutoSwitch(true);
    speed_follow.setThreshold(6.0f);

    // 初始化位置缓存区
    position_buffer_init(&position_buffers);

    // 设置双电机速度跟随模式参数访问
    speed_follow.setDualMotorParams(&global_motor_1.motor_id, &global_motor_1.motor_mode, &global_motor_1.motor_pos,
                                   &global_motor_1.motor_vel, &global_motor_1.motor_t, &global_motor_1.motor_kp, &global_motor_1.motor_kd,
                                   &global_motor_2.motor_id, &global_motor_2.motor_mode, &global_motor_2.motor_pos,
                                   &global_motor_2.motor_vel, &global_motor_2.motor_t, &global_motor_2.motor_kp, &global_motor_2.motor_kd,
                                   motor_params_mutex);

    // 统计计数器
    uint32_t loop_count = 0;

    // 电机数据变量
    MotorDataA1 motor_data_1 = {0};
    MotorDataA1 motor_data_2 = {0};

    // Local variables to hold the current parameters for both motors
    MotorParams current_motor_1, current_motor_2;

    while (1) {
        // Safely read the global parameter values for both motors
        if (xSemaphoreTake(motor_params_mutex, portMAX_DELAY) == pdTRUE) {
            current_motor_1 = global_motor_1;
            current_motor_2 = global_motor_2;
            xSemaphoreGive(motor_params_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take motor_params_mutex, using default parameters.");
            // Fallback to default values if mutex acquisition fails
            current_motor_1 = {MOTOR_ID_1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // FOC模式
            current_motor_2 = {MOTOR_ID_2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // FOC模式
        }

        // 控制电机1 - FOC模式，通过MIT参数控制
        MotorCmdA1 control_cmd_1;
        control_cmd_1.id = current_motor_1.motor_id;
        control_cmd_1.mode = current_motor_1.motor_mode;  // FOC模式(1)
        control_cmd_1.pos = current_motor_1.motor_pos;    // MIT参数
        control_cmd_1.vel = current_motor_1.motor_vel;    // MIT参数
        control_cmd_1.t = current_motor_1.motor_t;        // MIT参数
        control_cmd_1.kp = current_motor_1.motor_kp;      // MIT参数
        control_cmd_1.kd = current_motor_1.motor_kd;      // MIT参数

        esp_err_t err1 = motor_driver.sendRecv(control_cmd_1, motor_data_1);
        if (err1 == ESP_OK) {
            speed_follow.update(motor_data_1);
            // 添加位置数据到缓存区
            uint32_t timestamp = esp_timer_get_time() / 1000; // 转换为毫秒
            position_buffer_add_motor1(&position_buffers, motor_data_1.pos, timestamp);
        }

        // 控制电机2 - FOC模式，通过MIT参数控制
        MotorCmdA1 control_cmd_2;
        control_cmd_2.id = current_motor_2.motor_id;
        control_cmd_2.mode = current_motor_2.motor_mode;  // FOC模式(1)
        control_cmd_2.pos = current_motor_2.motor_pos;    // MIT参数
        control_cmd_2.vel = current_motor_2.motor_vel;    // MIT参数
        control_cmd_2.t = current_motor_2.motor_t;        // MIT参数
        control_cmd_2.kp = current_motor_2.motor_kp;      // MIT参数
        control_cmd_2.kd = current_motor_2.motor_kd;      // MIT参数

        esp_err_t err2 = motor_driver.sendRecv(control_cmd_2, motor_data_2);
        if (err2 == ESP_OK) {
            speed_follow.update(motor_data_2);
            // 添加位置数据到缓存区
            uint32_t timestamp = esp_timer_get_time() / 1000; // 转换为毫秒
            position_buffer_add_motor2(&position_buffers, motor_data_2.pos, timestamp);
        }

        // 合并打印两个电机的数据：电机1(ch0,ch1,ch2) + 电机2(ch3,ch4,ch5) + 波峰波谷差值最大值(ch6,ch7)
        if (err1 == ESP_OK && err2 == ESP_OK) {
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

            // 检查阈值并可能激活速度跟随模式
            speed_follow.checkThresholdAndActivate(ch6_max, ch7_max);

            printf("motors:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   motor_data_1.pos, motor_data_1.vel, motor_data_1.t,
                   motor_data_2.pos, motor_data_2.vel, motor_data_2.t,
                   ch6_max, ch7_max);
        }

        loop_count++;

        // 关闭详细状态打印，只保留格式化的电机数据输出
        // 数据按 "motors:ch0,ch1,ch2,ch3,ch4,ch5" 格式输出
        // ch0-ch2: 电机1的位置,速度,力矩; ch3-ch5: 电机2的位置,速度,力矩

        // 高频率控制: 2ms延时 = 500Hz控制频率
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "ESP32 Unitree 电机驱动示例启动");

    // 为电机参数创建互斥锁
    motor_params_mutex = xSemaphoreCreateMutex();
    if (motor_params_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create motor_params_mutex!");
        return;
    }

    if (motor_driver.init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, GPIO_NUM_NC, UART_BAUD_RATE)) {
        ESP_LOGI(TAG, "电机驱动初始化成功 - 纯串口同步模式");
    } else {
        ESP_LOGE(TAG, "电机驱动初始化失败！");
        return;
    }

    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);
}
