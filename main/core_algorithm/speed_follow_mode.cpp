#include "speed_follow_mode.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ai_fixed_cycle.h"
#include "motor_config.h"

// static const char *TAG = "SpeedFollow";  // 运行时日志已禁用，TAG未使用

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
      _auto_switch_enabled(false), _is_active(false), _is_motor_control_enabled(true), _is_stationary(false), _is_button_triggered(false), _is_buffer_triggered(false),
      _threshold_value(10.0f), _first_trigger_detected(false),
      _triggered_channel(0), _waiting_data_count(0), _idle_data_count(0), _working_data_count(0),
      _captured_velocity(0.0f), _phase1_timeout_ms(500), _phase2_timeout_ms(350), _velocity_scale(0.8f), _velocity_limit(20.0f), _phase2_vel_threshold(0.5f),
      _phase2_peak_velocity(0.0f), _phase2_peak_time(0),
      _motor1_id(nullptr), _motor1_mode(nullptr), _motor1_pos(nullptr),
      _motor1_vel(nullptr), _motor1_t(nullptr), _motor1_kp(nullptr), _motor1_kd(nullptr),
      _motor2_id(nullptr), _motor2_mode(nullptr), _motor2_pos(nullptr),
      _motor2_vel(nullptr), _motor2_t(nullptr), _motor2_kp(nullptr), _motor2_kd(nullptr),
      _global_mutex(nullptr), _diff_buffers(nullptr),
      _mode_type(SPEED_FOLLOW_MODE_PROGRAM), _ai_m1_phase(0), _ai_m2_phase(0),
      _current_m1_phase(0), _current_m2_phase(0), _both_static_start_time(0) {
    // IMU滑动窗口功能已移除
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
    // 如果电机控制未启用，则不更新电机参数（状态机继续运行但不控制电机）
    if (!_is_motor_control_enabled) {
        return;
    }

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
    _config_motor1.trigger_speed = 3.0f;   // 触发速度阈值

    // 电机1第一阶段参数（抬腿）
    // vel: 正方向参考，实际速度由实时反馈动态计算
    _config_motor1.phase1.mode = 1;
    _config_motor1.phase1.pos = 0.0f;
    _config_motor1.phase1.vel = 10.0f;      // 方向参考：正
    _config_motor1.phase1.torque = 1.5f;
    _config_motor1.phase1.kp = 0.0f;
    _config_motor1.phase1.kd = 0.08f;

    // 电机1第二阶段参数（压腿）
    // vel: 负方向参考，实际速度由实时反馈动态计算
    _config_motor1.phase2.mode = 1;
    _config_motor1.phase2.pos = 0.0f;
    _config_motor1.phase2.vel = -10.0f;     // 方向参考：负
    _config_motor1.phase2.torque = -1.5f;
    _config_motor1.phase2.kp = 0.0f;
    _config_motor1.phase2.kd = 0.08f;

    // 电机1空闲状态参数
    _config_motor1.idle.mode = 1;
    _config_motor1.idle.pos = 0.0f;
    _config_motor1.idle.vel = 0.0f;
    _config_motor1.idle.torque = 0.0f;
    _config_motor1.idle.kp = 0.0f;
    _config_motor1.idle.kd = 0.0f;

    // 电机1被动状态参数
    _config_motor1.passive.mode = 1;
    _config_motor1.passive.pos = 0.0f;
    _config_motor1.passive.vel = -0.0f;
    _config_motor1.passive.torque = -0.0f;
    _config_motor1.passive.kp = 0.0f;
    _config_motor1.passive.kd = 0.05f;

    // 配置电机2速度跟随模式参数
    _config_motor2.trigger_speed = -3.0f;  // 触发速度阈值

    // 电机2第一阶段参数（抬腿）
    // vel: 负方向参考，实际速度由实时反馈动态计算
    _config_motor2.phase1.mode = 1;
    _config_motor2.phase1.pos = 0.0f;
    _config_motor2.phase1.vel = -10.0f;     // 方向参考：负
    _config_motor2.phase1.torque = -1.5f;
    _config_motor2.phase1.kp = 0.0f;
    _config_motor2.phase1.kd = 0.08f;

    // 电机2第二阶段参数（压腿）
    // vel: 正方向参考，实际速度由实时反馈动态计算
    _config_motor2.phase2.mode = 1;
    _config_motor2.phase2.pos = 0.0f;
    _config_motor2.phase2.vel = 10.0f;      // 方向参考：正
    _config_motor2.phase2.torque = 1.5f;
    _config_motor2.phase2.kp = 0.0f;
    _config_motor2.phase2.kd = 0.08f;

    // 电机2空闲状态参数
    _config_motor2.idle.mode = 1;
    _config_motor2.idle.pos = 0.0f;
    _config_motor2.idle.vel = 0.0f;
    _config_motor2.idle.torque = 0.0f;
    _config_motor2.idle.kp = 0.0f;
    _config_motor2.idle.kd = 0.0f;

    // 电机2被动状态参数
    _config_motor2.passive.mode = 1;
    _config_motor2.passive.pos = 0.0f;
    _config_motor2.passive.vel = 0.0f;
    _config_motor2.passive.torque = 0.0f;
    _config_motor2.passive.kp = 0.0f;
    _config_motor2.passive.kd = 0.05f;

    _state = SPEED_FOLLOW_IDLE;
    _active_motor = 0;
    _lifting_motor = 0;
    _phase_start_time = esp_timer_get_time() / 1000;
    _triggered_channel = 0;
    _waiting_data_count = 0;
    _idle_data_count = 0;
    _working_data_count = 0;

    // 初始化动态参数
    _phase1_timeout_ms = 1250;       // PHASE1超时时间（抬腿阶段）
    _phase2_timeout_ms = 1250;       // PHASE2超时时间（压腿阶段）
    _velocity_scale = 0.8f;         // 速度缩放因子
    _velocity_limit = 50.0f;        // 速度跟随限幅
    _phase2_vel_threshold = 0.5f;   // PHASE2完成速度阈值

    // 初始化AI固定周期模式参数
    _ai_cycle_duration_ms = 1000;   // 单腿周期1000ms
    _ai_cycle_start_time = 0;
    _ai_current_leg = 1;            // 从电机1开始
    _ai_peak_velocity = 40.0f;      // 峰值速度40 rad/s
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
        // ESP_LOGI(TAG, "自动开关已启用，阈值: %.1f", _threshold_value);  // 运行时日志已禁用
    } else {
        // ESP_LOGI(TAG, "自动开关已禁用");  // 运行时日志已禁用
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
    // ESP_LOGI(TAG, "触发阈值设置为: %.1f", _threshold_value);  // 运行时日志已禁用
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
                // ESP_LOGI(TAG, "🚀 双通道触发！选择ch%d (ch6=%.1f, ch7=%.1f)，等待300ms检测对应电机",
                //          _triggered_channel, ch6_max, ch7_max);
            } else if (ch6_triggered) {
                _triggered_channel = 6;
                // ESP_LOGI(TAG, "🚀 ch6触发(%.1f)！等待300ms检测2号电机-v", ch6_max);
            } else {
                _triggered_channel = 7;
                // ESP_LOGI(TAG, "🚀 ch7触发(%.1f)！等待300ms检测1号电机+v", ch7_max);
            }

            _first_trigger_detected = true;
        }

        // 激活速度跟随模式，进入等待状态
        _is_active = true;
        _is_buffer_triggered = true;  // 设置缓存区触发标志
        _state = SPEED_FOLLOW_WAITING;
        _waiting_data_count = 0;  // 初始化数据点计数器

        // ESP_LOGI(TAG, "✅ 进入等待状态，300ms后检测%s电机速度",
        //          _triggered_channel == 6 ? "2号" : "1号");
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
 * @note 每次更新时根据phase1.torque动态调整trigger_speed：
 *       - 力矩绝对值 <= 1.0：电机1触发速度 = 3.0，电机2触发速度 = -3.0
 *       - 力矩绝对值 > 1.0：电机1触发速度 = 7.0，电机2触发速度 = -7.0
 */
void SpeedFollowMode::update(const MotorDataA1& motor_data, float ch6_max, float ch7_max,
                             float roll_left, float roll_right, bool imu_left_valid, bool imu_right_valid) {
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒

    // 如果未激活，跳过速度跟随逻辑
    if (!_is_active) {
        return;
    }

    // 根据 phase1.torque 动态调整触发速度
    // 力矩范围：电机1 [0.0 ~ 1.7]，电机2 [-1.7 ~ 0.0]
    // 触发速度规则：
    //   - 力矩绝对值 < 1.0：电机1 = 3.0，电机2 = -3.0
    //   - 力矩绝对值 >= 1.0：电机1 = 7.0，电机2 = -7.0
    float motor1_torque_abs = (_config_motor1.phase1.torque < 0) ? -_config_motor1.phase1.torque : _config_motor1.phase1.torque;
    if (motor1_torque_abs >= 1.0f) {
        _config_motor1.trigger_speed = 7.0f;
        _config_motor2.trigger_speed = -7.0f;
    } else {
        _config_motor1.trigger_speed = 3.0f;
        _config_motor2.trigger_speed = -3.0f;
    }

    switch (_state) {
        case SPEED_FOLLOW_BUTTON_WAITING:
            // 按键触发等待状态（已废弃 - 不再使用）
            // 保留状态枚举以保持兼容性，但不执行任何操作
            break;

        case SPEED_FOLLOW_WAITING:
            // 等待状态：_idle_data_count数据点后检测对应电机速度，同步设置标签
            // 两个电机都使用空闲参数，根据触发通道设置标签表明检测哪条腿
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);

            // ch6触发检测M2，ch7触发检测M1
            if (_triggered_channel == 6) {
                _current_m1_phase = 0;  // M1空闲
                _current_m2_phase = 3;  // M2检测速度触发中
            } else {
                _current_m1_phase = 3;  // M1检测速度触发中
                _current_m2_phase = 0;  // M2空闲
            }

            {
                // 检测对应电机的数据点，每收到一个数据点计数+1
                if (_triggered_channel == 6) {
                    // ch6触发，检测2号电机-v
                    if (motor_data.id == 2) {
                        _waiting_data_count++;
                        if (_waiting_data_count >= 2 && motor_data.vel < _config_motor2.trigger_speed) {
                        // 捕获速度值 _waiting_data_count
                        _captured_velocity = motor_data.vel;
                        _phase_start_time = current_time;

                        // 程序模式：进入PHASE1状态机
                        _lifting_motor = 2;
                        _active_motor = 1;
                        _state = SPEED_FOLLOW_PHASE1;
                        float scaled_vel = _captured_velocity * _velocity_scale;

                        // M2设置抬腿参数，同步设置标签
                        setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                      _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                        _current_m2_phase = 1;  // M2抬腿中

                        // M1设置压腿参数，同步设置标签
                        setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                      _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                        _current_m1_phase = 2;  // M1压腿中
                        }
                    }
                } else if (_triggered_channel == 7) {
                    // ch7触发，检测1号电机+v
                    if (motor_data.id == 1) {
                        _waiting_data_count++;
                        if (_waiting_data_count >= 2 && motor_data.vel > _config_motor1.trigger_speed) {
                        // 捕获速度值
                        _captured_velocity = motor_data.vel;
                        _phase_start_time = current_time;

                        // 程序模式：进入PHASE1状态机
                        _lifting_motor = 1;
                        _active_motor = 2;
                        _state = SPEED_FOLLOW_PHASE1;
                        float scaled_vel = _captured_velocity * _velocity_scale;

                        // M1设置抬腿参数，同步设置标签
                        setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                      _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                        _current_m1_phase = 1;  // M1抬腿中

                        // M2设置压腿参数，同步设置标签
                        setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                      _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                        _current_m2_phase = 2;  // M2压腿中
                        }
                    }
                }
            }
            break;

        case SPEED_FOLLOW_MOTOR1_WORKING:
            // 1号电机工作状态：程序模式专用，检测速度触发
            {
                // 超时时间为2秒
                uint32_t timeout_ms = 2000;

                // 检测超时
                if (current_time - _working_start_time >= timeout_ms) {
                    // ESP_LOGW(TAG, "⏱️ 1号电机工作超时2s未检测到速度触发，进入静止状态");
                    _is_button_triggered = false; // 清除按键触发标志
                    _is_stationary = true; // 设置静止标志
                    _is_active = false;
                    _state = SPEED_FOLLOW_IDLE;
                    _idle_data_count = 0; // 初始化空闲计数器
                    _first_trigger_detected = false;
                    _triggered_channel = 0;
                    _working_start_time = 0;

                    // 超时退出时，两个电机都设置为空闲参数，同步设置标签
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    _current_m1_phase = 0;  // M1空闲

                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    _current_m2_phase = 0;  // M2空闲

                    // 清空所有缓存区
                    if (_diff_buffers) {
                        diff_buffer_clear_all(_diff_buffers);
                        position_buffer_clear(position_buffer_get_motor1(_diff_buffers));
                        position_buffer_clear(position_buffer_get_motor2(_diff_buffers));
                        // ESP_LOGI(TAG, "🗑️ 已清空位置缓存区和ch6/ch7差值缓存区，等待新的阈值触发");
                    }
                    break;
                }

                // 计数1号电机的数据点
                if (motor_data.id == 1) {
                    _working_data_count++;
                }

                // 根据模式选择触发条件
                bool motor1_should_trigger = false;

                // 程序模式：_working_data_count数据点后才检测速度阈值
                if (_working_data_count >= 3) {
                    motor1_should_trigger = (motor_data.id == 1 && motor_data.vel > _config_motor1.trigger_speed);
                }

                if (motor1_should_trigger) {
                    // 捕获速度值
                    _captured_velocity = motor_data.vel;
                    _state = SPEED_FOLLOW_PHASE1;
                    _lifting_motor = 1; // 1号电机抬腿
                    _phase_start_time = current_time;

                    // 1号电机开始抬腿动作，使用捕获速度的0.8倍，同步设置标签
                    float scaled_vel = _captured_velocity * _velocity_scale;
                    setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                  _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                    _current_m1_phase = 1;  // M1抬腿中

                    // M2压腿，使用捕获速度，同步设置标签
                    setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                  _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                    _current_m2_phase = 2;  // M2压腿中
                } else {
                    // M2设置被动参数，同步设置标签
                    setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                  _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                    _current_m2_phase = 4;  // M2被动状态

                    // M1使用空闲参数（检测状态），同步设置标签
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    _current_m1_phase = 3;  // M1检测速度触发中
                }
            }
            break;

        case SPEED_FOLLOW_MOTOR2_WORKING:
            // 2号电机工作状态：程序模式专用，检测速度触发
            {
                // 超时时间为2秒
                uint32_t timeout_ms = 2000;

                // 检测超时
                if (current_time - _working_start_time >= timeout_ms) {
                    // ESP_LOGW(TAG, "⏱️ 2号电机工作超时2s未检测到速度触发，进入静止状态");
                    _is_button_triggered = false; // 清除按键触发标志
                    _is_stationary = true; // 设置静止标志
                    _is_active = false;
                    _state = SPEED_FOLLOW_IDLE;
                    _idle_data_count = 0; // 初始化空闲计数器
                    _first_trigger_detected = false;
                    _triggered_channel = 0;
                    _working_start_time = 0;

                    // 超时退出时，两个电机都设置为空闲参数，同步设置标签
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    _current_m1_phase = 0;  // M1空闲

                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    _current_m2_phase = 0;  // M2空闲

                    // 清空所有缓存区
                    if (_diff_buffers) {
                        diff_buffer_clear_all(_diff_buffers);
                        position_buffer_clear(position_buffer_get_motor1(_diff_buffers));
                        position_buffer_clear(position_buffer_get_motor2(_diff_buffers));
                        // ESP_LOGI(TAG, "🗑️ 已清空位置缓存区和ch6/ch7差值缓存区，等待新的阈值触发");
                    }
                    break;
                }

                // 计数2号电机的数据点
                if (motor_data.id == 2) {
                    _working_data_count++;
                }

                // 根据模式选择触发条件
                bool motor2_should_trigger = false;

                // 程序模式：_working_data_count数据点后才检测速度阈值
                if (_working_data_count >= 3) {
                    motor2_should_trigger = (motor_data.id == 2 && motor_data.vel < _config_motor2.trigger_speed);
                }

                if (motor2_should_trigger) {
                    // 捕获速度值
                    _captured_velocity = motor_data.vel;
                    _state = SPEED_FOLLOW_PHASE1;
                    _lifting_motor = 2; // 2号电机抬腿
                    _phase_start_time = current_time;

                    // 2号电机开始抬腿动作，使用捕获速度的0.8倍，同步设置标签
                    float scaled_vel = _captured_velocity * _velocity_scale;
                    setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                  _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                    _current_m2_phase = 1;  // M2抬腿中

                    // M1压腿，使用捕获速度（注意M1压腿速度方向为负），同步设置标签
                    setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                  _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                    _current_m1_phase = 2;  // M1压腿中
                } else {
                    // M1设置被动参数，同步设置标签
                    setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                  _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                    _current_m1_phase = 4;  // M1被动状态

                    // M2使用空闲参数（检测状态），同步设置标签
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    _current_m2_phase = 3;  // M2检测速度触发中
                }
            }
            break;

        case SPEED_FOLLOW_PHASE1:
            // 抬腿阶段 - 程序模式专用：超时+速度反转检测
            {
                bool should_transition = false;

                // 检查超时
                if (current_time - _phase_start_time >= _phase1_timeout_ms) {
                    should_transition = true;
                    // ESP_LOGI(TAG, "⏱️ PHASE1超时(%dms)，强制进入PHASE2", _phase1_timeout_ms);
                }
                // 检查速度反转（提前触发）
                else if (_lifting_motor == 1 && motor_data.id == 1) {
                    // 电机1：从+v变为-v
                    if (_captured_velocity > 0 && motor_data.vel < 0) {
                        should_transition = true;
                        // ESP_LOGI(TAG, "🔄 电机1速度反转(%.3f→%.3f)，提前进入PHASE2", _captured_velocity, motor_data.vel);
                    }
                } else if (_lifting_motor == 2 && motor_data.id == 2) {
                    // 电机2：从-v变为+v
                    if (_captured_velocity < 0 && motor_data.vel > 0) {
                        should_transition = true;
                        // ESP_LOGI(TAG, "🔄 电机2速度反转(%.3f→%.3f)，提前进入PHASE2", _captured_velocity, motor_data.vel);
                    }
                }

                if (should_transition) {
                    // 抬腿完成，开始压腿
                    _state = SPEED_FOLLOW_PHASE2;
                    _phase_start_time = current_time;

                    // 重置PHASE2峰值检测参数
                    _phase2_peak_velocity = 0.0f;
                    _phase2_peak_time = 0;

                    // 抬腿腿使用捕获到的速度设置压腿参数，另一条腿设置被动参数
                    float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                    float scaled_vel = abs_vel * _velocity_scale;

                    if (_lifting_motor == 1) {
                        // 电机1从抬腿切换到压腿，速度应为负（-）
                        if (scaled_vel > 0) scaled_vel = -scaled_vel;
                        if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                        setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                      _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                        _current_m1_phase = 2;  // M1压腿中

                        // 电机2被动
                        setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                      _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                        _current_m2_phase = 4;  // M2被动状态
                    } else {
                        // 电机2从抬腿切换到压腿，速度应为正（+）
                        if (scaled_vel < 0) scaled_vel = -scaled_vel;
                        if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                        setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                      _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                        _current_m2_phase = 2;  // M2压腿中

                        // 电机1被动
                        setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                      _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                        _current_m1_phase = 4;  // M1被动状态
                    }
                } else {
                    // 继续抬腿动作 - 抬腿腿使用实时速度*0.8，速度>=7.0时另一条腿压腿，否则被动
                    if (_lifting_motor == 1 && motor_data.id == 1) {
                        // 电机1抬腿，速度应为正（+）
                        float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                        float scaled_vel = abs_vel * _velocity_scale;
                        if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;

                        // M1设置抬腿参数，同步设置标签
                        setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                      _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                        _current_m1_phase = 1;  // M1抬腿中

                        // 电机2始终压腿，同步设置标签
                        setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                      _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                        _current_m2_phase = 2;  // M2压腿中

                        // 速度>=7.0时电机2压腿，<7.0时被动，同步设置标签（已注释）
                        // if (abs_vel >= 7.0f) {
                        //     setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                        //                   _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                        //     _current_m2_phase = 2;  // M2压腿中
                        // } else {
                        //     setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                        //                   _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                        //     _current_m2_phase = 4;  // M2被动状态
                        // }
                    } else if (_lifting_motor == 2 && motor_data.id == 2) {
                        // 电机2抬腿，速度应为负（-）
                        float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                        float scaled_vel = -abs_vel * _velocity_scale;
                        if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;

                        // M2设置抬腿参数，同步设置标签
                        setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                      _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                        _current_m2_phase = 1;  // M2抬腿中

                        // 电机1始终压腿，同步设置标签
                        setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                      _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                        _current_m1_phase = 2;  // M1压腿中

                        // 速度>=7.0时电机1压腿，<7.0时被动，同步设置标签（已注释）
                        // if (abs_vel >= 7.0f) {
                        //     setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                        //                   _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                        //     _current_m1_phase = 2;  // M1压腿中
                        // } else {
                        //     setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                        //                   _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                        //     _current_m1_phase = 4;  // M1被动状态
                        // }
                    }
                    // 注意：如果收到的是另一个电机的数据，保持上一次设置的参数不变
                }
            }
            break;

        case SPEED_FOLLOW_PHASE2:
            // 压腿阶段 - 程序模式专用：超时+速度峰值检测
            // 峰值检测后进入IDLE，当前腿保持被动状态，另一条腿进入检测状态
            {
                bool peak_detected = false;

                // 检查超时 - 超时直接进入IDLE
                if (current_time - _phase_start_time >= _phase2_timeout_ms) {
                    // 超时，直接进入空闲
                    _state = SPEED_FOLLOW_IDLE;
                    _idle_data_count = 0; // 初始化空闲计数器
                    _lifting_motor = 0;
                    _phase_start_time = current_time;

                    // 设置空闲参数，同步设置标签
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    _current_m1_phase = 0;  // M1空闲

                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    _current_m2_phase = 0;  // M2空闲
                    break;
                }

                // 检查速度峰值（当速度开始下降时认为达到峰值）
                if (_lifting_motor == 1 && motor_data.id == 1) {
                    // 电机1压腿：只考虑负速度方向的值
                    if (motor_data.vel < 0) {
                        float abs_vel = -motor_data.vel;

                        // 检测峰值：速度低于已记录的峰值且峰值有效
                        if (_phase2_peak_velocity > _phase2_vel_threshold && abs_vel < _phase2_peak_velocity) {
                            peak_detected = true;
                        } else if (abs_vel > _phase2_peak_velocity) {
                            // 更新峰值
                            _phase2_peak_velocity = abs_vel;
                            _phase2_peak_time = current_time;
                        }
                    }
                } else if (_lifting_motor == 2 && motor_data.id == 2) {
                    // 电机2压腿：只考虑正速度方向的值
                    if (motor_data.vel > 0) {
                        float abs_vel = motor_data.vel;

                        // 检测峰值：速度低于已记录的峰值且峰值有效
                        if (_phase2_peak_velocity > _phase2_vel_threshold && abs_vel < _phase2_peak_velocity) {
                            peak_detected = true;
                        } else if (abs_vel > _phase2_peak_velocity) {
                            // 更新峰值
                            _phase2_peak_velocity = abs_vel;
                            _phase2_peak_time = current_time;
                        }
                    }
                }

                if (peak_detected) {
                    // 峰值检测到，状态机进入IDLE让另一条腿开始检测
                    _state = SPEED_FOLLOW_IDLE;
                    _idle_data_count = 0; // 初始化空闲计数器
                    _phase_start_time = current_time;
                    uint8_t pressing_motor = _lifting_motor;
                    _lifting_motor = 0;  // 清除抬腿电机标记

                    // 切换工作电机：压腿的腿完成后，另一条腿开始工作
                    _active_motor = (pressing_motor == 1) ? 2 : 1;
                } else {
                    // 继续压腿阶段 - 压腿电机使用实时速度跟随，另一条腿使用被动参数
                    // 只在收到压腿电机数据时更新速度，否则使用上次的速度
                    if ((_lifting_motor == 1 && motor_data.id == 1) || (_lifting_motor == 2 && motor_data.id == 2)) {
                        // 收到压腿电机数据，更新速度
                        float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                        float scaled_vel = abs_vel * _velocity_scale;

                        if (_lifting_motor == 1) {
                            // 电机1压腿，速度应为负（-）
                            if (scaled_vel > 0) scaled_vel = -scaled_vel;
                            if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                            setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                          _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                            _current_m1_phase = 2;  // M1压腿中

                            // 电机2被动
                            setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                          _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                            _current_m2_phase = 4;  // M2被动状态
                        } else {
                            // 电机2压腿，速度应为正（+）
                            if (scaled_vel < 0) scaled_vel = -scaled_vel;
                            if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                            setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                          _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                            _current_m2_phase = 2;  // M2压腿中

                            // 电机1被动
                            setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                          _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                            _current_m1_phase = 4;  // M1被动状态
                        }
                    }
                    // 注意：如果收到的是另一个电机的数据，保持上一次设置的参数不变
                }
            }
            break;

        case SPEED_FOLLOW_AI_RUNNING:
            // AI固定周期模式：使用ai_fixed_cycle模块计算速度
            {
                // 构建状态结构体
                ai_fixed_cycle_state_t cycle_state;
                cycle_state.cycle_duration_ms = _ai_cycle_duration_ms;
                cycle_state.cycle_start_time = _ai_cycle_start_time;
                cycle_state.current_leg = _ai_current_leg;
                cycle_state.peak_velocity = _ai_peak_velocity;

                // 调用更新函数
                ai_fixed_cycle_output_t output;
                ai_fixed_cycle_update(&cycle_state, current_time, &output);

                // 同步状态回来
                _ai_cycle_start_time = cycle_state.cycle_start_time;
                _ai_current_leg = cycle_state.current_leg;

                // 更新阶段信息
                _current_m1_phase = output.m1_phase;
                _current_m2_phase = output.m2_phase;
                _lifting_motor = output.lifting_motor;

                // 设置电机参数
                if (output.lifting_motor == 1) {
                    setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, output.motor1_vel,
                                  _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                    setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, output.motor2_vel,
                                  _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                } else {
                    setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, output.motor1_vel,
                                  _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                    setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, output.motor2_vel,
                                  _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                }
            }
            break;

        case SPEED_FOLLOW_IDLE:
            // 空闲周期 - 等待2个数据点后进入工作状态
            {
                // 计数对应电机的数据点
                if ((_active_motor == 1 && motor_data.id == 1) ||
                    (_active_motor == 2 && motor_data.id == 2)) {
                    _idle_data_count++;
                }

                if (_idle_data_count >= 13) {
                    // 收到_idle_data_count数据点，进入对应的工作状态
                    _idle_data_count = 0; // 清零空闲计数器
                    _working_data_count = 0; // 初始化工作计数器
                    if (_active_motor == 1) {
                        _state = SPEED_FOLLOW_MOTOR1_WORKING;
                    } else {
                        _state = SPEED_FOLLOW_MOTOR2_WORKING;
                    }
                    _phase_start_time = current_time;
                    _working_start_time = current_time; // 记录工作状态开始时间
                    // ESP_LOGI(TAG, "🔄 空闲完成，%d号电机开始工作检测", _active_motor);
                }

                // 设置电机为空闲状态，同步设置标签
                setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                              _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                _current_m1_phase = 0;  // M1空闲

                setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                              _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                _current_m2_phase = 0;  // M2空闲

            }
            break;

        default:
            break;
    }
}

/**
 * @brief 直接进入AI运行状态（用于回放模式）
 * @note 跳过触发检测，直接进入SPEED_FOLLOW_AI_RUNNING状态
 *       在此状态下，update()会根据_ai_m1_phase和_ai_m2_phase来控制电机
 */
void SpeedFollowMode::startAIRunning() {
    _is_active = true;
    _is_button_triggered = false;
    _is_stationary = false;
    _state = SPEED_FOLLOW_AI_RUNNING;
    _both_static_start_time = 0;

    // 初始化AI固定周期模式
    _ai_cycle_start_time = esp_timer_get_time() / 1000;
    _ai_current_leg = 1;  // 从电机1开始抬腿
}

/**
 * @brief 停止AI运行状态（用于回放模式退出）
 * @note 将状态恢复到IDLE，并设置电机为空闲参数
 */
void SpeedFollowMode::stopAIRunning() {
    _is_active = false;
    _state = SPEED_FOLLOW_IDLE;
    _idle_data_count = 0; // 初始化空闲计数器
    _ai_m1_phase = 0;
    _ai_m2_phase = 0;
    _both_static_start_time = 0;

    // 设置电机为空闲参数，同步设置标签
    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
    _current_m1_phase = 0;  // M1空闲

    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
    _current_m2_phase = 0;  // M2空闲
}

/**
 * @brief Web接口：调整助力（增加或减少phase1和phase2的torque）
 * @param increase true=增加助力，false=减少助力
 * @return 调整后的电机1助力值（用于语音播报）
 * @note 与GPIO3/GPIO5按键逻辑相同，同时调整电机1和电机2的phase1和phase2的torque
 *       电机1范围：phase1 [0 ~ 2.5], phase2 [-2.5 ~ 0]
 *       电机2范围：phase1 [-2.5 ~ 0], phase2 [0 ~ 2.5]
 *       步长：0.1
 */
float SpeedFollowMode::adjustTorque(bool increase) {
    if (increase) {
        // 增加助力 - phase1和phase2同时调整绝对值
        float new_torque1_phase1 = _config_motor1.phase1.torque + TORQUE_STEP;
        if (new_torque1_phase1 > MOTOR1_PHASE1_MAX) {
            new_torque1_phase1 = MOTOR1_PHASE1_MAX;
        }
        _config_motor1.phase1.torque = new_torque1_phase1;

        float new_torque1_phase2 = _config_motor1.phase2.torque - TORQUE_STEP;
        if (new_torque1_phase2 < MOTOR1_PHASE2_MIN) {
            new_torque1_phase2 = MOTOR1_PHASE2_MIN;
        }
        _config_motor1.phase2.torque = new_torque1_phase2;

        float new_torque2_phase1 = _config_motor2.phase1.torque - TORQUE_STEP;
        if (new_torque2_phase1 < MOTOR2_PHASE1_MIN) {
            new_torque2_phase1 = MOTOR2_PHASE1_MIN;
        }
        _config_motor2.phase1.torque = new_torque2_phase1;

        float new_torque2_phase2 = _config_motor2.phase2.torque + TORQUE_STEP;
        if (new_torque2_phase2 > MOTOR2_PHASE2_MAX) {
            new_torque2_phase2 = MOTOR2_PHASE2_MAX;
        }
        _config_motor2.phase2.torque = new_torque2_phase2;

        // ESP_LOGI(TAG, "[WEB] 助力增加 - M1_P1: %.2f, M1_P2: %.2f, M2_P1: %.2f, M2_P2: %.2f",
        //          new_torque1_phase1, new_torque1_phase2, new_torque2_phase1, new_torque2_phase2);  // 运行时日志已禁用
        return new_torque1_phase1;
    } else {
        // 减少助力 - phase1和phase2同时调整绝对值
        float new_torque1_phase1 = _config_motor1.phase1.torque - TORQUE_STEP;
        if (new_torque1_phase1 < MOTOR1_PHASE1_MIN) {
            new_torque1_phase1 = MOTOR1_PHASE1_MIN;
        }
        _config_motor1.phase1.torque = new_torque1_phase1;

        float new_torque1_phase2 = _config_motor1.phase2.torque + TORQUE_STEP;
        if (new_torque1_phase2 > MOTOR1_PHASE2_MAX) {
            new_torque1_phase2 = MOTOR1_PHASE2_MAX;
        }
        _config_motor1.phase2.torque = new_torque1_phase2;

        float new_torque2_phase1 = _config_motor2.phase1.torque + TORQUE_STEP;
        if (new_torque2_phase1 > MOTOR2_PHASE1_MAX) {
            new_torque2_phase1 = MOTOR2_PHASE1_MAX;
        }
        _config_motor2.phase1.torque = new_torque2_phase1;

        float new_torque2_phase2 = _config_motor2.phase2.torque - TORQUE_STEP;
        if (new_torque2_phase2 < MOTOR2_PHASE2_MIN) {
            new_torque2_phase2 = MOTOR2_PHASE2_MIN;
        }
        _config_motor2.phase2.torque = new_torque2_phase2;

        // ESP_LOGI(TAG, "[WEB] 助力减少 - M1_P1: %.2f, M1_P2: %.2f, M2_P1: %.2f, M2_P2: %.2f",
        //          new_torque1_phase1, new_torque1_phase2, new_torque2_phase1, new_torque2_phase2);  // 运行时日志已禁用
        return new_torque1_phase1;
    }
}

/**
 * @brief Web接口：调整Kd参数（增加或减少phase1和phase2的kd）
 * @param increase true=增加Kd，false=减少Kd
 * @return 调整后的电机1 phase1的kd值
 * @note 同时调整电机1和电机2的phase1和phase2的kd
 *       范围：[0.00 ~ 0.20]
 *       步长：0.01
 */
float SpeedFollowMode::adjustKd(bool increase) {
    if (increase) {
        // 增加Kd
        float new_kd = _config_motor1.phase1.kd + KD_STEP;
        if (new_kd > KD_MAX) {
            new_kd = KD_MAX;
        }
        _config_motor1.phase1.kd = new_kd;
        _config_motor1.phase2.kd = new_kd;
        _config_motor2.phase1.kd = new_kd;
        _config_motor2.phase2.kd = new_kd;

        return new_kd;
    } else {
        // 减少Kd
        float new_kd = _config_motor1.phase1.kd - KD_STEP;
        if (new_kd < KD_MIN) {
            new_kd = KD_MIN;
        }
        _config_motor1.phase1.kd = new_kd;
        _config_motor1.phase2.kd = new_kd;
        _config_motor2.phase1.kd = new_kd;
        _config_motor2.phase2.kd = new_kd;

        return new_kd;
    }
}

// ============================================================================
// 双模式切换相关接口实现
// ============================================================================

/**
 * @brief 获取当前m1阶段（用于TCP数据透传）
 * @return 阶段值（0:静止, 1:抬腿, 2:压腿, 3:检测中, 4:被动）
 */
int SpeedFollowMode::getCurrentM1Phase() const {
    return _current_m1_phase;
}

/**
 * @brief 获取当前m2阶段（用于TCP数据透传）
 * @return 阶段值（0:静止, 1:抬腿, 2:压腿, 3:检测中, 4:被动）
 */
int SpeedFollowMode::getCurrentM2Phase() const {
    return _current_m2_phase;
}
