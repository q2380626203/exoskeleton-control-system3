/**
 * @file web_handlers.cpp
 * @brief Web服务器回调函数实现
 * @details 处理来自Web界面的命令、参数设置和状态查询
 */

#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "speed_follow_mode.h"
#include "voice_module.h"

// 外部变量声明
extern SpeedFollowMode speed_follow;
extern VoiceModule voice_module;

// 速度跟随模式配置参数（外部定义）
extern float global_speed_follow_threshold;

/**
 * @brief 将浮点数转换为中文数字字符串（用于语音播报）
 * @param value 浮点数值（范围 0.0 ~ 2.0）
 * @return 中文数字字符串
 * @note 将浮点数乘以10后转换为整数，然后转换为对应的中文数字
 *       例如：0.6 -> "六", 0.7 -> "七", 1.5 -> "十五", 2.0 -> "二十"
 */
static const char* float_to_chinese_number(float value) {
    // 将浮点数乘以10并四舍五入转换为整数
    int num = (int)(value * 10 + 0.5f);

    // 限制范围在 0-20
    if (num < 0) num = 0;
    if (num > 20) num = 20;

    // 中文数字映射表
    static const char* chinese_numbers[] = {
        "零",   // 0
        "一",   // 1
        "二",   // 2
        "三",   // 3
        "四",   // 4
        "五",   // 5
        "六",   // 6
        "七",   // 7
        "八",   // 8
        "九",   // 9
        "十",   // 10
        "十一", // 11
        "十二", // 12
        "十三", // 13
        "十四", // 14
        "十五", // 15
        "十六", // 16
        "十七", // 17
        "十八", // 18
        "十九", // 19
        "二十"  // 20
    };

    return chinese_numbers[num];
}

/**
 * @brief Web服务器命令处理回调
 * @param cmd 接收到的命令字符串 ("start", "stop", "assist_up", "assist_down")
 * @return ESP_OK 成功, ESP_FAIL 未知命令
 */
extern "C" esp_err_t handle_web_command(const char *cmd) {
    if (strcmp(cmd, "start") == 0) {
        speed_follow.enableMotorControl(true);
        return ESP_OK;
    }
    else if (strcmp(cmd, "stop") == 0) {
        speed_follow.enableMotorControl(false);
        return ESP_OK;
    }
    else if (strcmp(cmd, "assist_up") == 0) {
        float new_torque = speed_follow.adjustTorque(true);
        // 播放语音：助力值
        const char* torque_text = float_to_chinese_number(new_torque);
        voice_speak(&voice_module, torque_text);
        return ESP_OK;
    }
    else if (strcmp(cmd, "assist_down") == 0) {
        float new_torque = speed_follow.adjustTorque(false);
        // 播放语音：助力值
        const char* torque_text = float_to_chinese_number(new_torque);
        voice_speak(&voice_module, torque_text);
        return ESP_OK;
    }
    else if (strcmp(cmd, "mode_ai") == 0) {
        speed_follow.setModeType(SPEED_FOLLOW_MODE_AI);
        return ESP_OK;
    }
    else if (strcmp(cmd, "mode_program") == 0) {
        speed_follow.setModeType(SPEED_FOLLOW_MODE_PROGRAM);
        return ESP_OK;
    }
    else if (strcmp(cmd, "mode_imu") == 0) {
        speed_follow.setModeType(SPEED_FOLLOW_MODE_IMU);
        return ESP_OK;
    }

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
        global_speed_follow_threshold = value;
        speed_follow.setThreshold(value);
        return ESP_OK;
    }

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
    // 获取对应电机的配置
    speed_follow_config_t* config = speed_follow.getMotorConfig(motor);

    if (strcmp(param_name, "trigger_speed") == 0) {
        config->trigger_speed = value;
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
 * @brief Web服务器状态获取回调
 * @param state_json 输出JSON字符串的缓冲区
 * @param max_len 缓冲区最大长度
 * @return ESP_OK 成功, ESP_FAIL 失败
 */
extern "C" esp_err_t handle_web_get_state(char *state_json, size_t max_len) {
    // 状态机中文映射
    const char* state_names[] = {
        "空闲",           // SPEED_FOLLOW_IDLE
        "等待触发",        // SPEED_FOLLOW_WAITING
        "按键等待",        // SPEED_FOLLOW_BUTTON_WAITING
        "1号电机检测中",   // SPEED_FOLLOW_MOTOR1_WORKING
        "2号电机检测中",   // SPEED_FOLLOW_MOTOR2_WORKING
        "抬腿阶段",        // SPEED_FOLLOW_PHASE1
        "压腿阶段"         // SPEED_FOLLOW_PHASE2
    };

    // 阶段中文映射
    const char* phase_names[] = {
        "静止",    // 0
        "抬腿",    // 1
        "压腿"     // 2
    };

    speed_follow_state_t current_state = speed_follow.getState();
    uint8_t active_motor = speed_follow.getActiveMotor();
    uint8_t lifting_motor = speed_follow.getLiftingMotor();

    // 获取电机1的助力值（phase1.torque）
    speed_follow_config_t* motor1_config = speed_follow.getMotorConfig(1);
    float torque_value = motor1_config->phase1.torque;

    // 获取当前模式和阶段
    speed_follow_mode_type_t mode_type = speed_follow.getModeType();
    const char* mode_name = (mode_type == SPEED_FOLLOW_MODE_AI) ? "AI模式" :
                            (mode_type == SPEED_FOLLOW_MODE_IMU) ? "IMU模式" : "程序模式";

    int m1_phase = speed_follow.getCurrentM1Phase();
    int m2_phase = speed_follow.getCurrentM2Phase();

    // 确保阶段值在合法范围内
    if (m1_phase < 0 || m1_phase > 2) m1_phase = 0;
    if (m2_phase < 0 || m2_phase > 2) m2_phase = 0;

    int written = snprintf(state_json, max_len,
        "{\"state\":\"%s\",\"state_id\":%d,\"active_motor\":%d,\"lifting_motor\":%d,\"torque\":%.2f,\"mode\":\"%s\",\"m1_phase\":\"%s\",\"m2_phase\":\"%s\"}",
        state_names[current_state], current_state, active_motor, lifting_motor, torque_value,
        mode_name, phase_names[m1_phase], phase_names[m2_phase]);

    if (written < 0 || written >= (int)max_len) {
        return ESP_FAIL;
    }

    return ESP_OK;
}
