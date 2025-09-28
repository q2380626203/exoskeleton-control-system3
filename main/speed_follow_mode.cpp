#include "speed_follow_mode.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SpeedFollow";

SpeedFollowMode::SpeedFollowMode()
    : _state(SPEED_FOLLOW_IDLE), _phase_start_time(0),
      _global_motor_id(nullptr), _global_motor_mode(nullptr), _global_motor_pos(nullptr),
      _global_motor_vel(nullptr), _global_motor_t(nullptr), _global_motor_kp(nullptr),
      _global_motor_kd(nullptr), _global_mutex(nullptr) {
}

void SpeedFollowMode::init() {
    // 配置速度跟随模式参数
    _config.trigger_speed = -0.6f;  // 触发速度：-0.6 rad/s
    _config.idle_duration_ms = 300;    // 空闲状态持续300ms
    _config.phase1_duration_ms = 600;  // 第一阶段持续300ms
    _config.phase2_duration_ms = 600;  // 第二阶段持续300ms

    // 第一阶段参数：1 0.0 -10.0 -0.8 0.0 0.01
    _config.phase1.mode = 1;
    _config.phase1.pos = 0.0f;
    _config.phase1.vel = -15.0f;
    _config.phase1.torque = -0.9f;
    _config.phase1.kp = 0.0f;
    _config.phase1.kd = 0.04f;

    // 第二阶段参数：1 0.0 10 0.5 0.0 0.01
    _config.phase2.mode = 1;
    _config.phase2.pos = 0.0f;
    _config.phase2.vel = 10.0f;
    _config.phase2.torque = 0.5f;
    _config.phase2.kp = 0.0f;
    _config.phase2.kd = 0.03f;

    // 空闲状态参数：1 0.0 0.0 0.0 0.0 0.0
    _config.idle.mode = 1;
    _config.idle.pos = 0.0f;
    _config.idle.vel = 0.0f;
    _config.idle.torque = 0.0f;
    _config.idle.kp = 0.0f;
    _config.idle.kd = 0.0f;

    _state = SPEED_FOLLOW_IDLE;
    _phase_start_time = esp_timer_get_time() / 1000; // 设置初始空闲状态开始时间

    ESP_LOGI(TAG, "速度跟随模式已初始化");
    ESP_LOGI(TAG, "触发条件: 速度 < %.1f rad/s", _config.trigger_speed);
    ESP_LOGI(TAG, "空闲状态: 持续%d ms, 参数[%d, %.1f, %.1f, %.1f, %.2f, %.3f]",
             _config.idle_duration_ms, _config.idle.mode, _config.idle.pos, _config.idle.vel,
             _config.idle.torque, _config.idle.kp, _config.idle.kd);
    ESP_LOGI(TAG, "第一阶段: 持续%d ms, 参数[%d, %.1f, %.1f, %.1f, %.2f, %.3f]",
             _config.phase1_duration_ms, _config.phase1.mode, _config.phase1.pos,
             _config.phase1.vel, _config.phase1.torque, _config.phase1.kp, _config.phase1.kd);
    ESP_LOGI(TAG, "第二阶段: 持续%d ms, 参数[%d, %.1f, %.1f, %.1f, %.2f, %.3f]",
             _config.phase2_duration_ms, _config.phase2.mode, _config.phase2.pos,
             _config.phase2.vel, _config.phase2.torque, _config.phase2.kp, _config.phase2.kd);
}

void SpeedFollowMode::setGlobalParams(uint8_t* motor_id, uint8_t* motor_mode, float* motor_pos,
                                     float* motor_vel, float* motor_t, float* motor_kp, float* motor_kd,
                                     SemaphoreHandle_t mutex) {
    _global_motor_id = motor_id;
    _global_motor_mode = motor_mode;
    _global_motor_pos = motor_pos;
    _global_motor_vel = motor_vel;
    _global_motor_t = motor_t;
    _global_motor_kp = motor_kp;
    _global_motor_kd = motor_kd;
    _global_mutex = mutex;
}

void SpeedFollowMode::update(const MotorDataA1& motor_data) {
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒

    switch (_state) {
        case SPEED_FOLLOW_IDLE:
            // 空闲状态下应用空闲参数 (1 0.0 0.0 0.0 0.0 0.0)
            setGlobalMotorParams(_config.idle.mode, _config.idle.pos, _config.idle.vel,
                                _config.idle.torque, _config.idle.kp, _config.idle.kd);

            // 检查空闲状态是否达到300ms，达到后才检测触发条件
            if (current_time - _phase_start_time >= _config.idle_duration_ms) {
                if (checkTriggerCondition(motor_data)) {
                    _state = SPEED_FOLLOW_PHASE1;
                    _phase_start_time = current_time;
                    ESP_LOGI(TAG, "🚀 触发速度跟随模式! 检测到速度: %.3f rad/s", motor_data.vel);

                    // 应用第一阶段参数
                    setGlobalMotorParams(_config.phase1.mode, _config.phase1.pos, _config.phase1.vel,
                                        _config.phase1.torque, _config.phase1.kp, _config.phase1.kd);
                }
            }
            break;

        case SPEED_FOLLOW_PHASE1:
            // 检查第一阶段是否完成
            if (current_time - _phase_start_time >= _config.phase1_duration_ms) {
                _state = SPEED_FOLLOW_PHASE2;
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "📈 切换到第二阶段");

                // 应用第二阶段参数
                setGlobalMotorParams(_config.phase2.mode, _config.phase2.pos, _config.phase2.vel,
                                    _config.phase2.torque, _config.phase2.kp, _config.phase2.kd);
            } else {
                // 继续第一阶段
                setGlobalMotorParams(_config.phase1.mode, _config.phase1.pos, _config.phase1.vel,
                                    _config.phase1.torque, _config.phase1.kp, _config.phase1.kd);
            }
            break;

        case SPEED_FOLLOW_PHASE2:
            // 检查第二阶段是否完成
            if (current_time - _phase_start_time >= _config.phase2_duration_ms) {
                _state = SPEED_FOLLOW_IDLE;
                _phase_start_time = current_time; // 重新设置空闲状态开始时间
                ESP_LOGI(TAG, "✅ 速度跟随模式完成，回到空闲状态(300ms)");

                // 立即应用空闲状态参数
                setGlobalMotorParams(_config.idle.mode, _config.idle.pos, _config.idle.vel,
                                    _config.idle.torque, _config.idle.kp, _config.idle.kd);
            } else {
                // 继续第二阶段
                setGlobalMotorParams(_config.phase2.mode, _config.phase2.pos, _config.phase2.vel,
                                    _config.phase2.torque, _config.phase2.kp, _config.phase2.kd);
            }
            break;
    }
}

bool SpeedFollowMode::checkTriggerCondition(const MotorDataA1& motor_data) {
    // 检测2号电机速度方向为负且绝对值大于0.6
    return (motor_data.id == 2 && motor_data.vel < _config.trigger_speed);
}

void SpeedFollowMode::setGlobalMotorParams(uint8_t mode, float pos, float vel, float torque, float kp, float kd) {
    if (_global_mutex && _global_motor_mode && _global_motor_pos && _global_motor_vel &&
        _global_motor_t && _global_motor_kp && _global_motor_kd) {

        if (xSemaphoreTake(_global_mutex, portMAX_DELAY) == pdTRUE) {
            *_global_motor_mode = mode;
            *_global_motor_pos = pos;
            *_global_motor_vel = vel;
            *_global_motor_t = torque;
            *_global_motor_kp = kp;
            *_global_motor_kd = kd;
            xSemaphoreGive(_global_mutex);
        }
    }
}

void SpeedFollowMode::reset() {
    _state = SPEED_FOLLOW_IDLE;
    _phase_start_time = 0;
    ESP_LOGI(TAG, "速度跟随模式已重置");
}