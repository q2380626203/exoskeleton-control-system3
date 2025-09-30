#include "speed_follow_mode.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SpeedFollow";

SpeedFollowMode::SpeedFollowMode()
    : _state(SPEED_FOLLOW_IDLE), _phase_start_time(0),
      _motor1_id(nullptr), _motor1_mode(nullptr), _motor1_pos(nullptr),
      _motor1_vel(nullptr), _motor1_t(nullptr), _motor1_kp(nullptr), _motor1_kd(nullptr),
      _motor2_id(nullptr), _motor2_mode(nullptr), _motor2_pos(nullptr),
      _motor2_vel(nullptr), _motor2_t(nullptr), _motor2_kp(nullptr), _motor2_kd(nullptr),
      _global_mutex(nullptr) {
}

void SpeedFollowMode::init() {
    // 配置电机1速度跟随模式参数
    _config_motor1.trigger_speed = 0.6f;   // 触发速度：+0.6 rad/s (电机1)
    _config_motor1.idle_duration_ms = 300;
    _config_motor1.phase1_duration_ms = 600;
    _config_motor1.phase2_duration_ms = 600;

    // 电机1第一阶段参数：1 0.0 +15 +0.9 0.0 0.04
    _config_motor1.phase1.mode = 1;
    _config_motor1.phase1.pos = 0.0f;
    _config_motor1.phase1.vel = 15.0f;
    _config_motor1.phase1.torque = 0.9f;
    _config_motor1.phase1.kp = 0.0f;
    _config_motor1.phase1.kd = 0.04f;

    // 电机1第二阶段参数：1 0.0 -10 -0.5 0.0 0.03
    _config_motor1.phase2.mode = 1;
    _config_motor1.phase2.pos = 0.0f;
    _config_motor1.phase2.vel = -10.0f;
    _config_motor1.phase2.torque = -0.5f;
    _config_motor1.phase2.kp = 0.0f;
    _config_motor1.phase2.kd = 0.03f;

    // 电机1空闲状态参数：1 0.0 0.0 0.0 0.0 0.0
    _config_motor1.idle.mode = 1;
    _config_motor1.idle.pos = 0.0f;
    _config_motor1.idle.vel = 0.0f;
    _config_motor1.idle.torque = 0.0f;
    _config_motor1.idle.kp = 0.0f;
    _config_motor1.idle.kd = 0.0f;

    // 配置电机2速度跟随模式参数（保持原有逻辑）
    _config_motor2.trigger_speed = -0.6f;  // 触发速度：-0.6 rad/s (电机2)
    _config_motor2.idle_duration_ms = 300;
    _config_motor2.phase1_duration_ms = 600;
    _config_motor2.phase2_duration_ms = 600;

    // 电机2第一阶段参数：1 0.0 -15 -0.9 0.0 0.04
    _config_motor2.phase1.mode = 1;
    _config_motor2.phase1.pos = 0.0f;
    _config_motor2.phase1.vel = -15.0f;
    _config_motor2.phase1.torque = -0.9f;
    _config_motor2.phase1.kp = 0.0f;
    _config_motor2.phase1.kd = 0.04f;

    // 电机2第二阶段参数：1 0.0 +10 +0.5 0.0 0.03
    _config_motor2.phase2.mode = 1;
    _config_motor2.phase2.pos = 0.0f;
    _config_motor2.phase2.vel = 10.0f;
    _config_motor2.phase2.torque = 0.5f;
    _config_motor2.phase2.kp = 0.0f;
    _config_motor2.phase2.kd = 0.03f;

    // 电机2空闲状态参数：1 0.0 0.0 0.0 0.0 0.0
    _config_motor2.idle.mode = 1;
    _config_motor2.idle.pos = 0.0f;
    _config_motor2.idle.vel = 0.0f;
    _config_motor2.idle.torque = 0.0f;
    _config_motor2.idle.kp = 0.0f;
    _config_motor2.idle.kd = 0.0f;

    _state = SPEED_FOLLOW_IDLE;
    _phase_start_time = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "双电机速度跟随模式已初始化");
    ESP_LOGI(TAG, "电机1触发条件: 速度 > %.1f rad/s", _config_motor1.trigger_speed);
    ESP_LOGI(TAG, "电机2触发条件: 速度 < %.1f rad/s", _config_motor2.trigger_speed);
}

void SpeedFollowMode::setGlobalParams(uint8_t* motor_id, uint8_t* motor_mode, float* motor_pos,
                                     float* motor_vel, float* motor_t, float* motor_kp, float* motor_kd,
                                     SemaphoreHandle_t mutex) {
    // 兼容性函数，指向电机2参数
    _motor2_id = motor_id;
    _motor2_mode = motor_mode;
    _motor2_pos = motor_pos;
    _motor2_vel = motor_vel;
    _motor2_t = motor_t;
    _motor2_kp = motor_kp;
    _motor2_kd = motor_kd;
    _global_mutex = mutex;
}

void SpeedFollowMode::setDualMotorParams(uint8_t* motor1_id, uint8_t* motor1_mode, float* motor1_pos, float* motor1_vel,
                                         float* motor1_t, float* motor1_kp, float* motor1_kd,
                                         uint8_t* motor2_id, uint8_t* motor2_mode, float* motor2_pos, float* motor2_vel,
                                         float* motor2_t, float* motor2_kp, float* motor2_kd,
                                         SemaphoreHandle_t mutex) {
    // 电机1参数
    _motor1_id = motor1_id;
    _motor1_mode = motor1_mode;
    _motor1_pos = motor1_pos;
    _motor1_vel = motor1_vel;
    _motor1_t = motor1_t;
    _motor1_kp = motor1_kp;
    _motor1_kd = motor1_kd;

    // 电机2参数
    _motor2_id = motor2_id;
    _motor2_mode = motor2_mode;
    _motor2_pos = motor2_pos;
    _motor2_vel = motor2_vel;
    _motor2_t = motor2_t;
    _motor2_kp = motor2_kp;
    _motor2_kd = motor2_kd;

    _global_mutex = mutex;
}

void SpeedFollowMode::update(const MotorDataA1& motor_data) {
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒

    switch (_state) {
        case SPEED_FOLLOW_IDLE:
            // 空闲状态下应用空闲参数到对应电机
            if (motor_data.id == 1) {
                setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                              _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            } else if (motor_data.id == 2) {
                setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                              _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
            }

            // 检查空闲状态是否达到300ms，达到后才检测触发条件
            if (current_time - _phase_start_time >= _config_motor1.idle_duration_ms) {
                if (checkTriggerCondition(motor_data)) {
                    _state = SPEED_FOLLOW_PHASE1;
                    _phase_start_time = current_time;
                    ESP_LOGI(TAG, "🚀 电机%d触发速度跟随模式! 检测到速度: %.3f rad/s", motor_data.id, motor_data.vel);

                    // 根据触发的电机应用第一阶段参数
                    if (motor_data.id == 1) {
                        setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, _config_motor1.phase1.vel,
                                      _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                    } else if (motor_data.id == 2) {
                        setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, _config_motor2.phase1.vel,
                                      _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                    }
                }
            }
            break;

        case SPEED_FOLLOW_PHASE1:
            // 检查第一阶段是否完成
            if (current_time - _phase_start_time >= _config_motor1.phase1_duration_ms) {
                _state = SPEED_FOLLOW_PHASE2;
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "📈 切换到第二阶段");

                // 根据当前处理的电机应用第二阶段参数
                if (motor_data.id == 1) {
                    setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, _config_motor1.phase2.vel,
                                  _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                } else if (motor_data.id == 2) {
                    setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, _config_motor2.phase2.vel,
                                  _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                }
            } else {
                // 继续第一阶段
                if (motor_data.id == 1) {
                    setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, _config_motor1.phase1.vel,
                                  _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                } else if (motor_data.id == 2) {
                    setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, _config_motor2.phase1.vel,
                                  _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                }
            }
            break;

        case SPEED_FOLLOW_PHASE2:
            // 检查第二阶段是否完成
            if (current_time - _phase_start_time >= _config_motor1.phase2_duration_ms) {
                _state = SPEED_FOLLOW_IDLE;
                _phase_start_time = current_time; // 重新设置空闲状态开始时间
                ESP_LOGI(TAG, "✅ 速度跟随模式完成，回到空闲状态(300ms)");

                // 立即应用空闲状态参数
                if (motor_data.id == 1) {
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                } else if (motor_data.id == 2) {
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                }
            } else {
                // 继续第二阶段
                if (motor_data.id == 1) {
                    setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, _config_motor1.phase2.vel,
                                  _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                } else if (motor_data.id == 2) {
                    setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, _config_motor2.phase2.vel,
                                  _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                }
            }
            break;
    }
}

bool SpeedFollowMode::checkTriggerCondition(const MotorDataA1& motor_data) {
    // 检测电机1速度 > +0.6 或电机2速度 < -0.6
    if (motor_data.id == 1) {
        return motor_data.vel > _config_motor1.trigger_speed;
    } else if (motor_data.id == 2) {
        return motor_data.vel < _config_motor2.trigger_speed;
    }
    return false;
}

void SpeedFollowMode::setMotorParams(uint8_t motor_id, uint8_t mode, float pos, float vel, float torque, float kp, float kd) {
    if (_global_mutex) {
        if (xSemaphoreTake(_global_mutex, portMAX_DELAY) == pdTRUE) {
            if (motor_id == 1 && _motor1_mode && _motor1_pos && _motor1_vel && _motor1_t && _motor1_kp && _motor1_kd) {
                *_motor1_mode = mode;
                *_motor1_pos = pos;
                *_motor1_vel = vel;
                *_motor1_t = torque;
                *_motor1_kp = kp;
                *_motor1_kd = kd;
            } else if (motor_id == 2 && _motor2_mode && _motor2_pos && _motor2_vel && _motor2_t && _motor2_kp && _motor2_kd) {
                *_motor2_mode = mode;
                *_motor2_pos = pos;
                *_motor2_vel = vel;
                *_motor2_t = torque;
                *_motor2_kp = kp;
                *_motor2_kd = kd;
            }
            xSemaphoreGive(_global_mutex);
        }
    }
}

void SpeedFollowMode::reset() {
    _state = SPEED_FOLLOW_IDLE;
    _phase_start_time = 0;
    ESP_LOGI(TAG, "速度跟随模式已重置");
}