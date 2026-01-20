/**
 * @file main.cpp
 * @brief ESP32外骨骼控制系统主程序
 */

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "unitree_motor.h"
#include "motor_commands.h"
#include "speed_follow_mode.h"
#include "position_buffer.h"
#include "wifi_webserver.h"
#include "voice_module.h"
#include "button_detector.h"
#include "tcp_data_upload.h"
#include "motor_config.h"

// Web服务器句柄
static httpd_handle_t webserver = NULL;

// ==================== 设备配置 ====================
#define DEVICE_ID              2               // 设备编号: 1, 2, 3...


// ==================== 硬件配置 ====================
// 电机驱动 (UART2 + RS485)
#define MOTOR_ID_1       0x01
#define MOTOR_ID_2       0x02
#define UART_PORT_NUM    UART_NUM_2
#define UART_TX_PIN      GPIO_NUM_12
#define UART_RX_PIN      GPIO_NUM_13
#define MAX485_RE_DE_PIN GPIO_NUM_11    //自动流控拉高自动流控，开启后当普通串口使用
#define UART_BAUD_RATE   4000000

// 4G模块 (UART0透传)
#define GPIO_4G_RESET    GPIO_NUM_10        // 复位引脚 (高电平正常工作)
// 警告: UART0引脚(GPIO43/44)为系统默认，不可修改，否则会导致透传和烧录异常

// 语音模块 (UART1)
#define VOICE_UART_NUM   UART_NUM_1
#define VOICE_TX_PIN     GPIO_NUM_2
#define VOICE_RX_PIN     GPIO_NUM_1
#define VOICE_BAUDRATE   9600

// 按键
#define BUTTON_ASSIST_UP_PIN      GPIO_NUM_3    // 助力增加按键
#define BUTTON_ASSIST_DOWN_PIN    GPIO_NUM_5    // 助力减少按键

// ==================== 功能开关 ====================
// 4G模块TCP透传功能在 tcp_data_upload.h 中定义

// ==================== 全局实例 ====================
VoiceModule voice_module{};
UnitreeMotorDriver motor_driver;
SpeedFollowMode speed_follow;
motor_position_buffers_t position_buffers;
SemaphoreHandle_t motor_params_mutex;
float global_speed_follow_threshold = 6.0f;

// ==================== 电机参数 ====================
struct MotorParams {
    uint8_t motor_id;
    uint8_t motor_mode;
    float motor_pos;
    float motor_vel;
    float motor_t;
    float motor_kp;
    float motor_kd;
};

static MotorParams global_motor_1 = {MOTOR_ID_1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static MotorParams global_motor_2 = {MOTOR_ID_2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// ==================== 电机反馈数据（用于Web显示） ====================
static float global_motor1_feedback_pos = 0.0f;
static float global_motor2_feedback_pos = 0.0f;

/**
 * @brief 电机控制任务
 */
void motor_control_task(void *pvParameters) {
    if (!motor_driver.isInitialized()) {
        vTaskDelete(NULL);
        return;
    }

    // 初始化速度跟随模式
    speed_follow.init();
    speed_follow.enableAutoSwitch(true);
    speed_follow.setThreshold(global_speed_follow_threshold);
    position_buffer_init(&position_buffers);

    // 设置双电机参数访问
    speed_follow.setDualMotorParams(
        &global_motor_1.motor_id, &global_motor_1.motor_mode, &global_motor_1.motor_pos,
        &global_motor_1.motor_vel, &global_motor_1.motor_t, &global_motor_1.motor_kp, &global_motor_1.motor_kd,
        &global_motor_2.motor_id, &global_motor_2.motor_mode, &global_motor_2.motor_pos,
        &global_motor_2.motor_vel, &global_motor_2.motor_t, &global_motor_2.motor_kp, &global_motor_2.motor_kd,
        motor_params_mutex);
    speed_follow.setDiffBuffers(&position_buffers);

    // 局部变量
    uint32_t loop_count = 0;
    MotorDataA1 motor_data_1{}, motor_data_2{};
    MotorParams current_motor_1, current_motor_2;
    float ch6_max = 0.0f, ch7_max = 0.0f;
    float roll_left = 0.0f, roll_right = 0.0f;

    while (1) {
        // 读取电机参数
        if (xSemaphoreTake(motor_params_mutex, portMAX_DELAY) == pdTRUE) {
            current_motor_1 = global_motor_1;
            current_motor_2 = global_motor_2;
            xSemaphoreGive(motor_params_mutex);
        } else {
            current_motor_1 = {MOTOR_ID_1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            current_motor_2 = {MOTOR_ID_2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }

        // 蓝牙IMU功能已禁用
        bool imu_left_valid = false;
        bool imu_right_valid = false;

        // ==================== 正常模式：使用真实电机反馈 ====================
        // 控制电机1
        MotorCmdA1 cmd1;
        cmd1.id = current_motor_1.motor_id;
        cmd1.mode = current_motor_1.motor_mode;
        cmd1.pos = current_motor_1.motor_pos;
        cmd1.vel = current_motor_1.motor_vel;
        cmd1.t = current_motor_1.motor_t;
        cmd1.kp = current_motor_1.motor_kp;
        cmd1.kd = current_motor_1.motor_kd;

        esp_err_t err1 = motor_driver.sendRecv(cmd1, motor_data_1);
        if (err1 == ESP_OK) {
            uint32_t timestamp = esp_timer_get_time() / 1000;
            position_buffer_add_motor1(&position_buffers, motor_data_1.pos, timestamp);
            // 更新全局反馈位置（用于Web显示）
            global_motor1_feedback_pos = motor_data_1.pos;
        }

        vTaskDelay(pdMS_TO_TICKS(1));

        // 控制电机2
        MotorCmdA1 cmd2;
        cmd2.id = current_motor_2.motor_id;
        cmd2.mode = current_motor_2.motor_mode;
        cmd2.pos = current_motor_2.motor_pos;
        cmd2.vel = current_motor_2.motor_vel;
        cmd2.t = current_motor_2.motor_t;
        cmd2.kp = current_motor_2.motor_kp;
        cmd2.kd = current_motor_2.motor_kd;

        esp_err_t err2 = motor_driver.sendRecv(cmd2, motor_data_2);
        if (err2 == ESP_OK) {
            uint32_t timestamp = esp_timer_get_time() / 1000;
            position_buffer_add_motor2(&position_buffers, motor_data_2.pos, timestamp);
            // 更新全局反馈位置（用于Web显示）
            global_motor2_feedback_pos = motor_data_2.pos;
        }

        // 波形分析与速度跟随
        wave_analysis_result_t motor1_wave, motor2_wave;
        float motor1_diff = 0.0f, motor2_diff = 0.0f;
        uint32_t timestamp = esp_timer_get_time() / 1000;

        if (position_buffer_analyze_motor1_wave(&position_buffers, &motor1_wave)) {
            motor1_diff = motor1_wave.peak_valley_diff;
            diff_buffer_add_ch6(&position_buffers, motor1_diff, timestamp);
        }
        if (position_buffer_analyze_motor2_wave(&position_buffers, &motor2_wave)) {
            motor2_diff = motor2_wave.peak_valley_diff;
            diff_buffer_add_ch7(&position_buffers, motor2_diff, timestamp);
        }

        ch6_max = diff_buffer_get_ch6_max(&position_buffers);
        ch7_max = diff_buffer_get_ch7_max(&position_buffers);

        speed_follow.checkThresholdAndActivate(ch6_max, ch7_max);
        speed_follow.update(motor_data_1, ch6_max, ch7_max, roll_left, roll_right, imu_left_valid, imu_right_valid);
        speed_follow.update(motor_data_2, ch6_max, ch7_max, roll_left, roll_right, imu_left_valid, imu_right_valid);

        // 计算采样间隔
        static int64_t hz_start_time = 0;
        static int hz_count = 0;
        if (hz_count == 0) {
            hz_start_time = esp_timer_get_time();
        }
        if (++hz_count >= 100) {
            int64_t elapsed_us = esp_timer_get_time() - hz_start_time;
            uint8_t interval_ms = (uint8_t)(elapsed_us / 100000);
            if (interval_ms < 1) interval_ms = 1;
            tcp_data_set_interval(interval_ms);
            hz_count = 0;
        }

        // TCP数据上传
        motor_data_record_t record = {
            .timestamp_ms = (int64_t)get_beijing_timestamp_ms(),
            .motor1_pos = motor_data_1.pos,
            .motor1_vel = motor_data_1.vel,
            .motor1_torque = motor_data_1.t,
            .motor2_pos = motor_data_2.pos,
            .motor2_vel = motor_data_2.vel,
            .motor2_torque = motor_data_2.t,
            .roll_left = imu_left_valid ? roll_left : NAN,
            .roll_right = imu_right_valid ? roll_right : NAN,
            .m1_state_label = (int8_t)speed_follow.getCurrentM1Phase(),
            .m2_state_label = (int8_t)speed_follow.getCurrentM2Phase()
        };
        tcp_data_add_record(&record);

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(3));
    }
}

/**
 * @brief 应用程序主入口
 */
extern "C" void app_main() {
    // 初始化语音模块
    voice_module_init(&voice_module, VOICE_UART_NUM, VOICE_TX_PIN, VOICE_RX_PIN, VOICE_BAUDRATE);

    // 创建互斥锁
    motor_params_mutex = xSemaphoreCreateMutex();
    if (motor_params_mutex == NULL) {
        return;
    }

    // 后台初始化4G模块TCP透传
    serial_4g_tcp_init_background();

    // 配置4G模块复位引脚 (GPIO10) - 高电平正常工作，拉低复位
    gpio_config_t io_conf_4g = {
        .pin_bit_mask = (1ULL << GPIO_4G_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_4g);
    gpio_set_level(GPIO_4G_RESET, 1);  // 保持高电平，4G模块正常工作

    // 配置MAX485 DE/RE引脚
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MAX485_RE_DE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(MAX485_RE_DE_PIN, 1);

    // 初始化电机驱动
    if (!motor_driver.init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, GPIO_NUM_NC, UART_BAUD_RATE)) {
        return;
    }

    // 初始化WiFi热点
    wifi_init_softap();

    // 设置Web服务器的电机参数访问接口
    webserver_set_motor_access(&global_motor1_feedback_pos, &global_motor2_feedback_pos,
                                motor_params_mutex, &speed_follow);

    // 启动Web服务器
    webserver = start_webserver();

    // 初始化按键检测
    button_detector_init(BUTTON_ASSIST_UP_PIN, BUTTON_ASSIST_DOWN_PIN);

    // 创建任务
    xTaskCreate(button_detector_task, "button_task", 4096, NULL, 4, NULL);
    xTaskCreate(motor_control_task, "motor_task", 4096, NULL, 5, NULL);
}
