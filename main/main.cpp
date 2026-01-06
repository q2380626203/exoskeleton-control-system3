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
#include "tcp_replay_receive.h"
#include "bt_imu.h"

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
#define BUTTON_POWER_SWITCH_PIN   GPIO_NUM_4    // 电源开关
#define BUTTON_ASSIST_DOWN_PIN    GPIO_NUM_5    // 助力减少按键

// ==================== 功能开关 ====================
// #define ENABLE_BT_IMU                       // 启用蓝牙IMU
// TCP相关开关在 tcp_data_upload.h 中定义

// ==================== 网络配置 ====================
// WiFi STA模式 (连接外部网络)
#define WIFI_STA_SSID           "123"
#define WIFI_STA_PASSWORD       "12345678"

// TCP服务器配置
#define DEVICE_ID              3               // 设备编号: 1, 2, 3...
#define TCP_SERVER_HOST         "8.137.35.154"  // 云服务器地址
#define TCP_SERVER_PORT         16385           // 云服务器端口
#define TCP_LAN_SERVER_PORT     8888            // 局域网服务器端口

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
    replay_sample_t replay_sample{};  // 回放数据
    bool was_replay_mode = false;     // 上一次是否处于回放模式
    speed_follow_mode_type_t saved_mode_type = SPEED_FOLLOW_MODE_PROGRAM;  // 保存原始模式

    while (1) {
        // 检查是否处于回放模式
        bool is_replay_mode = tcp_replay_is_active();

        // 回放模式切换处理
        if (is_replay_mode && !was_replay_mode) {
            // 进入回放模式：切换到AI模式并激活
            saved_mode_type = speed_follow.getModeType();
            speed_follow.setModeType(SPEED_FOLLOW_MODE_AI);
            speed_follow.enable(true);
            speed_follow.enableMotorControl(true);
            speed_follow.startAIRunning();  // 直接进入AI运行状态
        } else if (!is_replay_mode && was_replay_mode) {
            // 退出回放模式：停止AI运行并恢复原始模式
            speed_follow.stopAIRunning();
            speed_follow.setModeType(saved_mode_type);
        }
        was_replay_mode = is_replay_mode;

        // 读取电机参数
        if (xSemaphoreTake(motor_params_mutex, portMAX_DELAY) == pdTRUE) {
            current_motor_1 = global_motor_1;
            current_motor_2 = global_motor_2;
            xSemaphoreGive(motor_params_mutex);
        } else {
            current_motor_1 = {MOTOR_ID_1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            current_motor_2 = {MOTOR_ID_2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }

        // 获取蓝牙IMU数据
#ifdef ENABLE_BT_IMU
        bt_imu_data_t imu_data_left, imu_data_right;
        bool imu_left_valid = bt_imu_get_data_multi(0, &imu_data_left);
        bool imu_right_valid = bt_imu_get_data_multi(1, &imu_data_right);
        if (imu_left_valid) {
            roll_left = roundf(imu_data_left.roll * 100.0f) / 100.0f;
        }
        if (imu_right_valid) {
            roll_right = roundf(imu_data_right.roll * 100.0f) / 100.0f;
        }
#else
        bool imu_left_valid = false;
        bool imu_right_valid = false;
#endif

        // ==================== 回放模式：使用回放数据作为电机反馈 ====================
        if (is_replay_mode && tcp_replay_get_next_sample(&replay_sample)) {
            // 获取到下一条数据的时间间隔（用于精确时序控制）
            uint16_t next_interval_ms = tcp_replay_get_next_interval_ms();
            int64_t sample_start_time = esp_timer_get_time();  // 记录处理开始时间

            // 用回放数据填充电机反馈
            motor_data_1.id = MOTOR_ID_1;
            motor_data_1.pos = replay_sample.motor1_pos;
            motor_data_1.vel = replay_sample.motor1_vel;
            motor_data_1.t = replay_sample.motor1_torque;

            motor_data_2.id = MOTOR_ID_2;
            motor_data_2.pos = replay_sample.motor2_pos;
            motor_data_2.vel = replay_sample.motor2_vel;
            motor_data_2.t = replay_sample.motor2_torque;

            // 使用回放数据中的roll（如果有效）
            if (!isnan(replay_sample.roll_left)) {
                roll_left = replay_sample.roll_left;
                imu_left_valid = true;
            }
            if (!isnan(replay_sample.roll_right)) {
                roll_right = replay_sample.roll_right;
                imu_right_valid = true;
            }

            // 直接注入CSV中的状态标签（跳过波形分析和状态检测）
            speed_follow.updateAIPhase(replay_sample.m1_state_label, replay_sample.m2_state_label);

            // 调用update来根据注入的状态控制电机参数
            speed_follow.update(motor_data_1, 0, 0, roll_left, roll_right, imu_left_valid, imu_right_valid);
            speed_follow.update(motor_data_2, 0, 0, roll_left, roll_right, imu_left_valid, imu_right_valid);

            // 发送控制命令到真实电机（使用算法计算的参数）
            MotorCmdA1 cmd1;
            cmd1.id = current_motor_1.motor_id;
            cmd1.mode = current_motor_1.motor_mode;
            cmd1.pos = current_motor_1.motor_pos;
            cmd1.vel = current_motor_1.motor_vel;
            cmd1.t = current_motor_1.motor_t;
            cmd1.kp = current_motor_1.motor_kp;
            cmd1.kd = current_motor_1.motor_kd;
            MotorDataA1 dummy1{};
            motor_driver.sendRecv(cmd1, dummy1);

            vTaskDelay(pdMS_TO_TICKS(1));

            MotorCmdA1 cmd2;
            cmd2.id = current_motor_2.motor_id;
            cmd2.mode = current_motor_2.motor_mode;
            cmd2.pos = current_motor_2.motor_pos;
            cmd2.vel = current_motor_2.motor_vel;
            cmd2.t = current_motor_2.motor_t;
            cmd2.kp = current_motor_2.motor_kp;
            cmd2.kd = current_motor_2.motor_kd;
            MotorDataA1 dummy2{};
            motor_driver.sendRecv(cmd2, dummy2);

            // 精确时序控制：计算剩余需要等待的时间
            if (next_interval_ms > 0) {
                int64_t elapsed_us = esp_timer_get_time() - sample_start_time;
                int32_t remaining_ms = next_interval_ms - (int32_t)(elapsed_us / 1000);
                if (remaining_ms > 0) {
                    vTaskDelay(pdMS_TO_TICKS(remaining_ms));
                }
            }
            // 跳过循环末尾的固定延时
            continue;
        }
        // ==================== 正常模式：使用真实电机反馈 ====================
        else if (!is_replay_mode) {
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
            }
        } else {
            // 回放模式但没有数据，等待
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // 波形分析与速度跟随（仅正常模式执行，回放模式已在上面直接注入状态）
        if (!is_replay_mode) {
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
        }

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

    // 初始化蓝牙IMU
#ifdef ENABLE_BT_IMU
    bt_imu_init_multi();
#endif

    // 创建互斥锁
    motor_params_mutex = xSemaphoreCreateMutex();
    if (motor_params_mutex == NULL) {
        return;
    }

    // 设置网络配置（必须在wifi_tcp_init_background之前）
    tcp_network_config_t net_config = {
        .wifi_ssid = WIFI_STA_SSID,
        .wifi_password = WIFI_STA_PASSWORD,
        .tcp_server_host = TCP_SERVER_HOST,
        .tcp_server_port = TCP_SERVER_PORT,
        .tcp_lan_server_port = TCP_LAN_SERVER_PORT,
        .device_id = DEVICE_ID
    };
    tcp_set_network_config(&net_config);

    // 后台初始化WiFi和TCP
    wifi_tcp_init_background();

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

    // 初始化按键检测
    button_detector_init(BUTTON_ASSIST_UP_PIN, BUTTON_POWER_SWITCH_PIN, BUTTON_ASSIST_DOWN_PIN);

    // 创建任务
    xTaskCreate(button_detector_task, "button_task", 4096, NULL, 4, NULL);
    xTaskCreate(motor_control_task, "motor_task", 4096, NULL, 5, NULL);
}
