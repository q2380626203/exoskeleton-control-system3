#include "speed_follow_mode.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SpeedFollow";

// ============================================================================
// 构造函数
// ============================================================================

/**
 * @brief 速度跟随模式构造函数
 *
 * 初始化所有成员变量为默认值：
 * - 状态机初始状态为 IDLE
 * - 电机指针初始化为 nullptr
 * - 触发参数设置为默认值
 */
SpeedFollowMode::SpeedFollowMode()
    : _state(SPEED_FOLLOW_IDLE), _phase_start_time(0),
      _active_motor(1), _lifting_motor(0), _working_start_time(0),
      _auto_switch_enabled(false), _is_active(false), _is_stationary(false), _is_button_triggered(false), _is_buffer_triggered(false),
      _threshold_value(10.0f), _first_trigger_detected(false),
      _triggered_channel(0), _waiting_start_time(0),
      _captured_velocity(0.0f), _phase1_timeout_ms(500), _phase2_timeout_ms(350), _velocity_scale(0.8f), _phase2_vel_threshold(0.5f),
      _motor1_id(nullptr), _motor1_mode(nullptr), _motor1_pos(nullptr),
      _motor1_vel(nullptr), _motor1_t(nullptr), _motor1_kp(nullptr), _motor1_kd(nullptr),
      _motor2_id(nullptr), _motor2_mode(nullptr), _motor2_pos(nullptr),
      _motor2_vel(nullptr), _motor2_t(nullptr), _motor2_kp(nullptr), _motor2_kd(nullptr),
      _global_mutex(nullptr), _diff_buffers(nullptr) {
}

// ============================================================================
// 私有成员函数 (内部使用)
// ============================================================================

/**
 * @brief 设置指定电机的控制参数
 * @param motor_id 电机ID（1 或 2）
 * @param mode 控制模式（0=停止, 1=伺服, 10=电流）
 * @param pos 目标位置（弧度）
 * @param vel 目标速度（rad/s）
 * @param torque 前馈力矩（N·m）
 * @param kp 位置增益
 * @param kd 速度增益
 *
 * @note 使用互斥锁保护全局电机参数的修改
 * @note 只有在对应电机的指针有效时才会更新参数
 */
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

// ============================================================================
// 公共成员函数 (外部使用)
// ============================================================================

/**
 * @brief 初始化速度跟随模式配置
 *
 * 配置双电机速度跟随模式的所有参数，包括：
 * - 电机1/2的触发速度阈值
 * - PHASE1（抬腿）和PHASE2（压腿）的时间参数
 * - 各阶段的电机控制参数（mode, pos, vel, torque, kp, kd）
 * - 动态调整参数（超时时间、速度缩放因子等）
 *
 * @note 电机1触发条件：速度 > +2.0 rad/s
 * @note 电机2触发条件：速度 < -2.0 rad/s
 * @note ch6触发→检测2号电机，ch7触发→检测1号电机
 */
void SpeedFollowMode::init() {
    // 配置电机1速度跟随模式参数
    _config_motor1.trigger_speed = 2.0f;   // 触发速度：+0.75 rad/s (电机1)
    _config_motor1.phase1_duration_ms = 500;
    _config_motor1.phase2_duration_ms = 350;
    _config_motor1.waiting_duration_ms = 300;  // 等待时间
    _config_motor1.idle_duration_ms = 100;      // 空闲时间

    // 电机1第一阶段参数：1 0.0 +15 +0.9 0.0 0.04
    _config_motor1.phase1.mode = 1;
    _config_motor1.phase1.pos = 0.0f;
    _config_motor1.phase1.vel = 10.0f;
    _config_motor1.phase1.torque = 0.7f;
    _config_motor1.phase1.kp = 0.0f;
    _config_motor1.phase1.kd = 0.05f;

    // 电机1第二阶段参数：1 0.0 -10 -0.5 0.0 0.03
    _config_motor1.phase2.mode = 1;
    _config_motor1.phase2.pos = 0.0f;
    _config_motor1.phase2.vel = -10.0f;
    _config_motor1.phase2.torque = -0.3f;
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
    _config_motor2.trigger_speed = -2.0f;  // 触发速度：-0.75 rad/s (电机2)
    _config_motor2.phase1_duration_ms = 500;
    _config_motor2.phase2_duration_ms = 350;
    _config_motor2.waiting_duration_ms = 300;  // 等待时间：300ms
    _config_motor2.idle_duration_ms = 100;      // 空闲时间：50ms

    // 电机2第一阶段参数：1 0.0 -15 -0.9 0.0 0.04
    _config_motor2.phase1.mode = 1;
    _config_motor2.phase1.pos = 0.0f;
    _config_motor2.phase1.vel = -10.0f;
    _config_motor2.phase1.torque = -0.7f;
    _config_motor2.phase1.kp = 0.0f;
    _config_motor2.phase1.kd = 0.05f;

    // 电机2第二阶段参数：1 0.0 +10 +0.5 0.0 0.03
    _config_motor2.phase2.mode = 1;
    _config_motor2.phase2.pos = 0.0f;
    _config_motor2.phase2.vel = 10.0f;
    _config_motor2.phase2.torque = 0.3f;
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

    // 初始化动态参数
    _phase1_timeout_ms = 500;       // PHASE1超时时间（抬腿阶段）
    _phase2_timeout_ms = 350;       // PHASE2超时时间（压腿阶段）
    _velocity_scale = 0.8f;         // 速度缩放因子
    _phase2_vel_threshold = 0.5f;   // PHASE2完成速度阈值

    ESP_LOGI(TAG, "双电机协作速度跟随模式已初始化");
    ESP_LOGI(TAG, "首次触发模式：ch6触发→等待300ms→检测2号-v，ch7触发→等待300ms→检测1号+v");
    ESP_LOGI(TAG, "电机1触发条件: 速度 > %.1f rad/s", _config_motor1.trigger_speed);
    ESP_LOGI(TAG, "电机2触发条件: 速度 < %.1f rad/s", _config_motor2.trigger_speed);
    ESP_LOGI(TAG, "动态参数: PHASE1超时=%dms, PHASE2超时=%dms, 速度缩放=%.2f, PHASE2完成阈值=±%.2f rad/s",
             _phase1_timeout_ms, _phase2_timeout_ms, _velocity_scale, _phase2_vel_threshold);
}

/**
 * @brief 设置双电机控制参数的指针引用
 * @param motor1_id 电机1的ID指针
 * @param motor1_mode 电机1的控制模式指针
 * @param motor1_pos 电机1的目标位置指针
 * @param motor1_vel 电机1的目标速度指针
 * @param motor1_t 电机1的前馈力矩指针
 * @param motor1_kp 电机1的位置增益指针
 * @param motor1_kd 电机1的速度增益指针
 * @param motor2_id 电机2的ID指针
 * @param motor2_mode 电机2的控制模式指针
 * @param motor2_pos 电机2的目标位置指针
 * @param motor2_vel 电机2的目标速度指针
 * @param motor2_t 电机2的前馈力矩指针
 * @param motor2_kp 电机2的位置增益指针
 * @param motor2_kd 电机2的速度增益指针
 * @param mutex 保护全局电机参数的互斥锁句柄
 *
 * @note 这些指针指向全局电机参数，修改时需要使用互斥锁保护
 */
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

/**
 * @brief 设置位置差值缓存区指针
 * @param buffers 指向电机位置缓存区结构的指针
 *
 * @note 用于访问ch6/ch7差值数据，触发速度跟随模式
 */
void SpeedFollowMode::setDiffBuffers(motor_position_buffers_t* buffers) {
    _diff_buffers = buffers;
}

/**
 * @brief 启用或禁用自动触发开关
 * @param enable true=启用自动触发，false=禁用自动触发
 *
 * @note 启用后，当ch6/ch7差值超过阈值时会自动进入速度跟随模式
 * @note 禁用时会清除激活状态和首次触发标志
 */
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

/**
 * @brief 设置自动触发阈值
 * @param threshold ch6/ch7差值的触发阈值
 *
 * @note 当ch6_max或ch7_max超过该阈值时，会触发速度跟随模式
 */
void SpeedFollowMode::setThreshold(float threshold) {
    _threshold_value = threshold;
    ESP_LOGI(TAG, "触发阈值设置为: %.1f", _threshold_value);
}

/**
 * @brief 检查ch6/ch7差值是否超过阈值并激活速度跟随模式
 * @param ch6_max ch6通道的最大差值
 * @param ch7_max ch7通道的最大差值
 *
 * 触发逻辑：
 * - ch6触发 → 等待300ms → 检测2号电机-v
 * - ch7触发 → 等待300ms → 检测1号电机+v
 * - 双通道触发时选择数值更大的一方
 *
 * @note 只有在自动开关启用且未激活时才会检查
 * @note 触发后进入WAITING状态，等待对应电机速度触发
 */
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
        _is_buffer_triggered = true;  // 设置缓存区触发标志
        _state = SPEED_FOLLOW_WAITING;
        _waiting_start_time = esp_timer_get_time() / 1000;

        ESP_LOGI(TAG, "✅ 进入等待状态，300ms后检测%s电机速度",
                 _triggered_channel == 6 ? "2号" : "1号");
    }
}

/**
 * @brief 速度跟随状态机更新函数（每个控制周期调用两次）
 * @param motor_data 当前电机的反馈数据（包含id, vel等）
 * @param ch6_max ch6通道的最大差值（用于WAITING状态检查）
 * @param ch7_max ch7通道的最大差值（用于WAITING状态检查）
 *
 * 状态机流程：
 * 1. BUTTON_WAITING：按键触发等待，检测任意电机速度触发
 * 2. WAITING：ch6/ch7触发等待（300ms），检测对应电机速度
 * 3. MOTOR1_WORKING/MOTOR2_WORKING：检测对应电机速度触发（超时1.2s/4s）
 * 4. PHASE1：抬腿阶段，动态速度跟随（超时500ms或速度反转）
 * 5. PHASE2：压腿阶段，动态速度跟随（超时350ms或速度回零）
 * 6. IDLE：空闲周期（100ms），然后切换到下一个工作电机
 *
 * @note 此函数在motor_control_task中被调用两次（motor_data_1和motor_data_2）
 * @note 必须检查motor_data.id来判断当前处理的是哪个电机的数据
 * @note PHASE1/PHASE2使用实时速度*0.8动态更新电机参数
 */
void SpeedFollowMode::update(const MotorDataA1& motor_data, float ch6_max, float ch7_max) {
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒

    // 如果未激活，跳过速度跟随逻辑
    if (!_is_active) {
        return;
    }

    switch (_state) {
        case SPEED_FOLLOW_BUTTON_WAITING:
            // 按键触发等待状态：同时检测两个电机的速度，谁先触发就进入谁的PHASE1
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);

            // 检测电机1速度触发
            if (motor_data.id == 1 && motor_data.vel > _config_motor1.trigger_speed) {
                _captured_velocity = motor_data.vel;
                _state = SPEED_FOLLOW_PHASE1;
                _lifting_motor = 1;
                _active_motor = 2; // 下个周期2号工作
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "🎯 按键模式：1号电机速度触发(+v=%.3f)，开始抬腿", motor_data.vel);

                float scaled_vel = _captured_velocity * _velocity_scale;
                setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                              _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
            }
            // 检测电机2速度触发
            else if (motor_data.id == 2 && motor_data.vel < _config_motor2.trigger_speed) {
                _captured_velocity = motor_data.vel;
                _state = SPEED_FOLLOW_PHASE1;
                _lifting_motor = 2;
                _active_motor = 1; // 下个周期1号工作
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "🎯 按键模式：2号电机速度触发(-v=%.3f)，开始抬腿", motor_data.vel);

                float scaled_vel = _captured_velocity * _velocity_scale;
                setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                              _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
            }
            break;

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
                        // 捕获速度值
                        _captured_velocity = motor_data.vel;

                        _state = SPEED_FOLLOW_PHASE1;
                        _lifting_motor = 2;
                        _active_motor = 1; // 下个周期1号工作
                        _phase_start_time = current_time;
                        ESP_LOGI(TAG, "🚀 ch6触发后检测到2号-v=%.3f，捕获速度，2号开始抬腿(超时%dms)",
                                 motor_data.vel, _phase1_timeout_ms);

                        // 使用捕获速度的0.8倍设置电机参数
                        float scaled_vel = _captured_velocity * _velocity_scale;
                        setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                      _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                    }
                } else if (_triggered_channel == 7) {
                    // ch7触发，检测1号电机+v
                    if (motor_data.id == 1 && motor_data.vel > _config_motor1.trigger_speed) {
                        // 捕获速度值
                        _captured_velocity = motor_data.vel;

                        _state = SPEED_FOLLOW_PHASE1;
                        _lifting_motor = 1;
                        _active_motor = 2; // 下个周期2号工作
                        _phase_start_time = current_time;
                        ESP_LOGI(TAG, "🚀 ch7触发后检测到1号+v=%.3f，捕获速度，1号开始抬腿(超时%dms)",
                                 motor_data.vel, _phase1_timeout_ms);

                        // 使用捕获速度的0.8倍设置电机参数
                        float scaled_vel = _captured_velocity * _velocity_scale;
                        setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
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

            // 按键模式下超时时间为4秒，ch6/ch7触发模式为1.2秒
            {
                uint32_t timeout_ms = _is_button_triggered ? 4000 : 1200;

                // 检测超时
                if (current_time - _working_start_time >= timeout_ms) {
                    if (_is_button_triggered) {
                        ESP_LOGW(TAG, "⏱️ 1号电机工作超时4s未检测到速度触发，进入静止状态");
                        _is_button_triggered = false; // 清除按键触发标志
                    } else {
                        ESP_LOGW(TAG, "⏱️ 1号电机工作超时1.2s未检测到速度触发，进入静止状态");
                    }
                    _is_stationary = true; // 设置静止标志（按键和缓存区触发都会触发语音）
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
            }

            // 立即检测速度触发条件
            if (motor_data.id == 1 && motor_data.vel > _config_motor1.trigger_speed) {
                // 捕获速度值
                _captured_velocity = motor_data.vel;

                _state = SPEED_FOLLOW_PHASE1;
                _lifting_motor = 1; // 1号电机抬腿
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "🚀 1号电机检测到+v=%.3f，捕获速度，开始抬腿(超时%dms)",
                         motor_data.vel, _phase1_timeout_ms);

                // 1号电机开始抬腿动作，使用捕获速度的0.8倍
                float scaled_vel = _captured_velocity * _velocity_scale;
                setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                              _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
            }
            break;

        case SPEED_FOLLOW_MOTOR2_WORKING:
            // 2号电机工作状态：立即检测-v触发抬腿
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);

            // 按键模式下超时时间为4秒，ch6/ch7触发模式为1.2秒
            {
                uint32_t timeout_ms = _is_button_triggered ? 4000 : 1200;

                // 检测超时
                if (current_time - _working_start_time >= timeout_ms) {
                    if (_is_button_triggered) {
                        ESP_LOGW(TAG, "⏱️ 2号电机工作超时4s未检测到速度触发，进入静止状态");
                        _is_button_triggered = false; // 清除按键触发标志
                    } else {
                        ESP_LOGW(TAG, "⏱️ 2号电机工作超时1.2s未检测到速度触发，进入静止状态");
                    }
                    _is_stationary = true; // 设置静止标志（按键和缓存区触发都会触发语音）
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
            }

            // 立即检测速度触发条件
            if (motor_data.id == 2 && motor_data.vel < _config_motor2.trigger_speed) {
                // 捕获速度值
                _captured_velocity = motor_data.vel;

                _state = SPEED_FOLLOW_PHASE1;
                _lifting_motor = 2; // 2号电机抬腿
                _phase_start_time = current_time;
                ESP_LOGI(TAG, "🚀 2号电机检测到-v=%.3f，捕获速度，开始抬腿(超时%dms)",
                         motor_data.vel, _phase1_timeout_ms);

                // 2号电机开始抬腿动作，使用捕获速度的0.8倍
                float scaled_vel = _captured_velocity * _velocity_scale;
                setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                              _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
            }
            break;

        case SPEED_FOLLOW_PHASE1:
            // 抬腿阶段 - 使用超时机制 + 速度反转检测
            {
                bool should_transition = false;

                // 检查超时
                if (current_time - _phase_start_time >= _phase1_timeout_ms) {
                    should_transition = true;
                    ESP_LOGI(TAG, "⏱️ PHASE1超时(%dms)，强制进入PHASE2", _phase1_timeout_ms);
                }
                // 检查速度反转（提前触发）
                else if (_lifting_motor == 1 && motor_data.id == 1) {
                    // 电机1：从+v变为-v
                    if (_captured_velocity > 0 && motor_data.vel < 0) {
                        should_transition = true;
                        ESP_LOGI(TAG, "🔄 电机1速度反转(%.3f→%.3f)，提前进入PHASE2", _captured_velocity, motor_data.vel);
                    }
                } else if (_lifting_motor == 2 && motor_data.id == 2) {
                    // 电机2：从-v变为+v
                    if (_captured_velocity < 0 && motor_data.vel > 0) {
                        should_transition = true;
                        ESP_LOGI(TAG, "🔄 电机2速度反转(%.3f→%.3f)，提前进入PHASE2", _captured_velocity, motor_data.vel);
                    }
                }

                if (should_transition) {
                    // 抬腿完成，切换工作电机并开始压腿
                    _active_motor = (_lifting_motor == 1) ? 2 : 1; // 切换工作电机
                    _state = SPEED_FOLLOW_PHASE2;
                    _phase_start_time = current_time;

                    ESP_LOGI(TAG, "🔄 抬腿完成，切换到%d号工作，%d号开始压腿(超时%dms)",
                             _active_motor, _lifting_motor, _phase2_timeout_ms);

                    // 如果当前接收的就是抬腿电机的数据，立即设置PHASE2参数
                    if ((_lifting_motor == 1 && motor_data.id == 1) ||
                        (_lifting_motor == 2 && motor_data.id == 2)) {
                        float scaled_vel = motor_data.vel * _velocity_scale;
                        if (_lifting_motor == 1) {
                            setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                          _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                        } else {
                            setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                          _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                        }
                    }
            } else {
                // 继续抬腿动作 - 持续根据当前电机的实时速度*0.8更新参数
                if (_lifting_motor == 1 && motor_data.id == 1) {
                    // 电机1抬腿，使用电机1的实时速度*0.8
                    float scaled_vel = motor_data.vel * _velocity_scale;
                    setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                  _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                } else if (_lifting_motor == 2 && motor_data.id == 2) {
                    // 电机2抬腿，使用电机2的实时速度*0.8
                    float scaled_vel = motor_data.vel * _velocity_scale;
                    setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                  _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                }
                // 注意：如果收到的是另一个电机的数据，保持上一次设置的参数不变
            }
            }
            break;

        case SPEED_FOLLOW_PHASE2:
            // 压腿阶段 - 使用超时机制 + 速度反转检测 + 持续更新速度参数
            {
                bool should_transition = false;

                // 检查超时
                if (current_time - _phase_start_time >= _phase2_timeout_ms) {
                    should_transition = true;
                    ESP_LOGI(TAG, "⏱️ PHASE2超时(%dms)，强制进入IDLE", _phase2_timeout_ms);
                }
                // 检查速度是否回到低速区间（压腿完成）
                else if (_lifting_motor == 1 && motor_data.id == 1) {
                    // 电机1：速度降低到阈值范围内（-v到+v之间，接近零速）
                    if (motor_data.vel > -_phase2_vel_threshold && motor_data.vel < _phase2_vel_threshold) {
                        should_transition = true;
                        ESP_LOGI(TAG, "🔄 电机1速度降低到阈值内(%.3f)，完成压腿", motor_data.vel);
                    }
                } else if (_lifting_motor == 2 && motor_data.id == 2) {
                    // 电机2：速度降低到阈值范围内（-v到+v之间，接近零速）
                    if (motor_data.vel > -_phase2_vel_threshold && motor_data.vel < _phase2_vel_threshold) {
                        should_transition = true;
                        ESP_LOGI(TAG, "🔄 电机2速度降低到阈值内(%.3f)，完成压腿", motor_data.vel);
                    }
                }

                if (should_transition) {
                    // 压腿完成，进入空闲周期
                    _state = SPEED_FOLLOW_IDLE;
                    _lifting_motor = 0;
                    _phase_start_time = current_time;
                    ESP_LOGI(TAG, "✅ 压腿完成，进入空闲周期%dms",
                             (_active_motor == 1) ? _config_motor1.idle_duration_ms : _config_motor2.idle_duration_ms);

                    // 设置空闲状态
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                } else {
                    // 继续压腿动作 - 持续根据当前电机的实时速度*0.8更新参数
                    if (_lifting_motor == 1 && motor_data.id == 1) {
                        // 电机1压腿，使用电机1的实时速度*0.8
                        float scaled_vel = motor_data.vel * _velocity_scale;
                        setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                      _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                    } else if (_lifting_motor == 2 && motor_data.id == 2) {
                        // 电机2压腿，使用电机2的实时速度*0.8
                        float scaled_vel = motor_data.vel * _velocity_scale;
                        setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                      _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                    }
                    // 注意：如果收到的是另一个电机的数据，保持上一次设置的参数不变
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

/**
 * @brief 按键触发启动速度跟随模式
 *
 * 设置状态为BUTTON_WAITING，同时检测两个电机的速度：
 * - 1号电机速度 > +2.0 rad/s → 触发1号抬腿
 * - 2号电机速度 < -2.0 rad/s → 触发2号抬腿
 *
 * @note 按键模式超时时间为4秒（vs ch6/ch7触发的1.2秒）
 * @note 设置_is_button_triggered标志，用于区分按键和缓存区触发
 */
void SpeedFollowMode::startButtonWaiting() {
    // 按键触发进入按键等待状态
    _is_active = true;
    _is_button_triggered = true;
    _is_stationary = false;
    _state = SPEED_FOLLOW_BUTTON_WAITING;
    ESP_LOGI(TAG, "🔘 按键触发：进入BUTTON_WAITING状态，同时检测两个电机速度");
}
