#include "speed_follow_mode.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SpeedFollow";

SpeedFollowMode::SpeedFollowMode()
    : _state(SPEED_FOLLOW_IDLE), _phase_start_time(0),
      _active_motor(1), _lifting_motor(0), _working_start_time(0),
      _auto_switch_enabled(false), _is_active(false), _threshold_value(10.0f), _first_trigger_detected(false),
      _triggered_channel(0), _waiting_start_time(0),
      _motor1_id(nullptr), _motor1_mode(nullptr), _motor1_pos(nullptr),
      _motor1_vel(nullptr), _motor1_t(nullptr), _motor1_kp(nullptr), _motor1_kd(nullptr),
      _motor2_id(nullptr), _motor2_mode(nullptr), _motor2_pos(nullptr),
      _motor2_vel(nullptr), _motor2_t(nullptr), _motor2_kp(nullptr), _motor2_kd(nullptr),
      _global_mutex(nullptr), _diff_buffers(nullptr) {
}

void SpeedFollowMode::init() {
    // 配置电机1速度跟随模式参数
    _config_motor1.trigger_speed = 0.75f;   // 触发速度：+0.75 rad/s (电机1)
    _config_motor1.phase1_duration_ms = 400;
    _config_motor1.phase2_duration_ms = 400;
    _config_motor1.waiting_duration_ms = 300;  // 等待时间：300ms
    _config_motor1.idle_duration_ms = 50;      // 空闲时间：50ms

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
    _config_motor2.trigger_speed = -0.75f;  // 触发速度：-0.75 rad/s (电机2)
    _config_motor2.phase1_duration_ms = 400;
    _config_motor2.phase2_duration_ms = 400;
    _config_motor2.waiting_duration_ms = 300;  // 等待时间：300ms
    _config_motor2.idle_duration_ms = 50;      // 空闲时间：50ms

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
    _active_motor = 0;
    _lifting_motor = 0;
    _phase_start_time = esp_timer_get_time() / 1000;
    _triggered_channel = 0;
    _waiting_start_time = 0;

    ESP_LOGI(TAG, "双电机协作速度跟随模式已初始化");
    ESP_LOGI(TAG, "新工作模式：ch6触发→等待300ms→检测2号-v，ch7触发→等待300ms→检测1号+v");
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

void SpeedFollowMode::setDiffBuffers(motor_position_buffers_t* buffers) {
    _diff_buffers = buffers;
}

void SpeedFollowMode::update(const MotorDataA1& motor_data) {
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒

    // 如果未激活，跳过速度跟随逻辑
    if (!_is_active) {
        return;
    }

    switch (_state) {
        case SPEED_FOLLOW_WAITING:
            // 等待状态：等待配置时间后检测对应电机速度
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);

            {
                // 根据触发通道选择对应电机的等待时间配置
                uint32_t waiting_duration = (_triggered_channel == 6) ? _config_motor2.waiting_duration_ms : _config_motor1.waiting_duration_ms;
                if (current_time - _waiting_start_time >= waiting_duration) {
                    // 等待完成，检测对应电机速度
                if (_triggered_channel == 6) {
                    // ch6触发，检测2号电机-v
                    if (motor_data.id == 2 && motor_data.vel < _config_motor2.trigger_speed) {
                        _state = SPEED_FOLLOW_PHASE1;
                        _lifting_motor = 2;
                        _active_motor = 1; // 下个周期1号工作
                        _phase_start_time = current_time;
                        ESP_LOGI(TAG, "🚀 ch6触发后检测到2号-v=%.3f，2号开始抬腿0.6s", motor_data.vel);

                        setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, _config_motor2.phase1.vel,
                                      _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                    }
                } else if (_triggered_channel == 7) {
                    // ch7触发，检测1号电机+v
                    if (motor_data.id == 1 && motor_data.vel > _config_motor1.trigger_speed) {
                        _state = SPEED_FOLLOW_PHASE1;
                        _lifting_motor = 1;
                        _active_motor = 2; // 下个周期2号工作
                        _phase_start_time = current_time;
                        ESP_LOGI(TAG, "🚀 ch7触发后检测到1号+v=%.3f，1号开始抬腿0.6s", motor_data.vel);

                        setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, _config_motor1.phase1.vel,
                                      _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                    }
                }
            }
            }
            break;

        case SPEED_FOLLOW_MOTOR1_WORKING:
            // 1号电机工作状态：立即检测+v触发抬腿
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);

            // 检测超时（1.2秒未触发）
            if (current_time - _working_start_time >= 1200) {
                ESP_LOGW(TAG, "⏱️ 1号电机工作超时1.2s未检测到速度触发，关闭速度跟随模式");
                _is_active = false;
                _state = SPEED_FOLLOW_IDLE;
                _first_trigger_detected = false;
                _triggered_channel = 0;
                _working_start_time = 0;
                // 清空所有缓存区
                if (_diff_buffers) {
                    diff_buffer_clear_all(_diff_buffers);
                    position_buffer_clear(position_buffer_get_motor1(_diff_buffers));
                    position_buffer_clear(position_buffer_get_motor2(_diff_buffers));
                    ESP_LOGI(TAG, "🗑️ 已清空位置缓存区和ch6/ch7差值缓存区，等待新的阈值触发");
                }
                break;
            }

            // 立即检测速度触发条件
            if (motor_data.id == 1 && motor_data.vel > _config_motor1.trigger_speed) {
                _state = SPEED_FOLLOW_PHASE1;
                _lifting_motor = 1; // 1号电机抬腿
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "🚀 1号电机检测到+v=%.3f，开始抬腿0.6s", motor_data.vel);

                // 1号电机开始抬腿动作
                setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, _config_motor1.phase1.vel,
                              _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
            }
            break;

        case SPEED_FOLLOW_MOTOR2_WORKING:
            // 2号电机工作状态：立即检测-v触发抬腿
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);

            // 检测超时（1.2秒未触发）
            if (current_time - _working_start_time >= 1200) {
                ESP_LOGW(TAG, "⏱️ 2号电机工作超时1.2s未检测到速度触发，关闭速度跟随模式");
                _is_active = false;
                _state = SPEED_FOLLOW_IDLE;
                _first_trigger_detected = false;
                _triggered_channel = 0;
                _working_start_time = 0;
                // 清空所有缓存区
                if (_diff_buffers) {
                    diff_buffer_clear_all(_diff_buffers);
                    position_buffer_clear(position_buffer_get_motor1(_diff_buffers));
                    position_buffer_clear(position_buffer_get_motor2(_diff_buffers));
                    ESP_LOGI(TAG, "🗑️ 已清空位置缓存区和ch6/ch7差值缓存区，等待新的阈值触发");
                }
                break;
            }

            // 立即检测速度触发条件
            if (motor_data.id == 2 && motor_data.vel < _config_motor2.trigger_speed) {
                _state = SPEED_FOLLOW_PHASE1;
                _lifting_motor = 2; // 2号电机抬腿
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "🚀 2号电机检测到-v=%.3f，开始抬腿0.6s", motor_data.vel);

                // 2号电机开始抬腿动作
                setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, _config_motor2.phase1.vel,
                              _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
            }
            break;

        case SPEED_FOLLOW_PHASE1:
            // 抬腿阶段 - 使用配置的持续时间
            {
                uint32_t phase1_duration = (_lifting_motor == 1) ? _config_motor1.phase1_duration_ms : _config_motor2.phase1_duration_ms;
                if (current_time - _phase_start_time >= phase1_duration) {
                    // 抬腿完成，切换工作电机并开始压腿
                _active_motor = (_lifting_motor == 1) ? 2 : 1; // 切换工作电机
                _state = SPEED_FOLLOW_PHASE2;
                _phase_start_time = current_time;

                ESP_LOGI(TAG, "🔄 抬腿完成，切换到%d号工作，%d号开始压腿0.6s", _active_motor, _lifting_motor);

                // 开始压腿动作
                if (_lifting_motor == 1) {
                    setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, _config_motor1.phase2.vel,
                                  _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                } else {
                    setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, _config_motor2.phase2.vel,
                                  _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                }
            } else {
                // 继续抬腿动作
                if (_lifting_motor == 1) {
                    setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, _config_motor1.phase1.vel,
                                  _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                } else {
                    setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, _config_motor2.phase1.vel,
                                  _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                }
            }
            }
            break;

        case SPEED_FOLLOW_PHASE2:
            // 压腿阶段 - 使用配置的持续时间
            {
                uint32_t phase2_duration = (_lifting_motor == 1) ? _config_motor1.phase2_duration_ms : _config_motor2.phase2_duration_ms;
                if (current_time - _phase_start_time >= phase2_duration) {
                    // 压腿完成，进入空闲周期
                _state = SPEED_FOLLOW_IDLE;
                _lifting_motor = 0;
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "✅ 压腿完成，进入空闲周期150ms");

                // 设置空闲状态
                setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                              _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                              _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
            } else {
                // 继续压腿动作
                if (_lifting_motor == 1) {
                    setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, _config_motor1.phase2.vel,
                                  _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                } else {
                    setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, _config_motor2.phase2.vel,
                                  _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                }
            }
            }
            break;

        case SPEED_FOLLOW_IDLE:
            // 空闲周期 - 使用配置的持续时间
            {
                // 根据即将工作的电机选择对应的空闲时间配置
                uint32_t idle_duration = (_active_motor == 1) ? _config_motor1.idle_duration_ms : _config_motor2.idle_duration_ms;
                if (current_time - _phase_start_time >= idle_duration) {
                    // 空闲完成，进入对应的工作状态
                if (_active_motor == 1) {
                    _state = SPEED_FOLLOW_MOTOR1_WORKING;
                } else {
                    _state = SPEED_FOLLOW_MOTOR2_WORKING;
                }
                _phase_start_time = current_time;
                _working_start_time = current_time; // 记录工作状态开始时间
                ESP_LOGI(TAG, "🔄 空闲完成，%d号电机开始工作检测", _active_motor);
            }

            // 保持空闲状态
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
            }
            break;

        default:
            break;
    }
}

bool SpeedFollowMode::checkTriggerCondition(const MotorDataA1& motor_data) {
    // 新逻辑中，触发检测已集成在update函数中
    // 保留此函数以维持接口兼容性
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
    _is_active = false;
    _first_trigger_detected = false;
    _triggered_channel = 0;
    _waiting_start_time = 0;
    _working_start_time = 0;
    ESP_LOGI(TAG, "速度跟随模式已重置");
}

void SpeedFollowMode::enableAutoSwitch(bool enable) {
    _auto_switch_enabled = enable;
    if (enable) {
        ESP_LOGI(TAG, "自动开关已启用，阈值: %.1f", _threshold_value);
    } else {
        ESP_LOGI(TAG, "自动开关已禁用");
        _is_active = false;
        _first_trigger_detected = false;
    }
}

void SpeedFollowMode::setThreshold(float threshold) {
    _threshold_value = threshold;
    ESP_LOGI(TAG, "触发阈值设置为: %.1f", _threshold_value);
}

void SpeedFollowMode::checkThresholdAndActivate(float ch6_max, float ch7_max) {
    if (!_auto_switch_enabled || _is_active) {
        return; // 自动开关未启用或已经激活
    }

    // 检查是否有任一通道超过阈值
    bool ch6_triggered = ch6_max > _threshold_value;
    bool ch7_triggered = ch7_max > _threshold_value;

    if (ch6_triggered || ch7_triggered) {
        if (!_first_trigger_detected) {
            // 首次触发，进入等待状态
            if (ch6_triggered && ch7_triggered) {
                // 两者都触发，选择数值更大的一方
                _triggered_channel = (ch6_max > ch7_max) ? 6 : 7;
                ESP_LOGI(TAG, "🚀 双通道触发！选择ch%d (ch6=%.1f, ch7=%.1f)，等待300ms检测对应电机",
                         _triggered_channel, ch6_max, ch7_max);
            } else if (ch6_triggered) {
                _triggered_channel = 6;
                ESP_LOGI(TAG, "🚀 ch6触发(%.1f)！等待300ms检测2号电机-v", ch6_max);
            } else {
                _triggered_channel = 7;
                ESP_LOGI(TAG, "🚀 ch7触发(%.1f)！等待300ms检测1号电机+v", ch7_max);
            }

            _first_trigger_detected = true;
        }

        // 激活速度跟随模式，进入等待状态
        _is_active = true;
        _state = SPEED_FOLLOW_WAITING;
        _waiting_start_time = esp_timer_get_time() / 1000;

        ESP_LOGI(TAG, "✅ 进入等待状态，300ms后检测%s电机速度",
                 _triggered_channel == 6 ? "2号" : "1号");
    }
}

void SpeedFollowMode::manualDeactivate() {
    if (_is_active) {
        _is_active = false;
        _state = SPEED_FOLLOW_IDLE;
        _first_trigger_detected = false;
        _triggered_channel = 0;
        _waiting_start_time = 0;
        _working_start_time = 0;
        ESP_LOGI(TAG, "❌ 速度跟随模式已手动关闭");
    }
}

