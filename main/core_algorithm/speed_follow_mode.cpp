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
      _auto_switch_enabled(false), _is_active(false), _is_motor_control_enabled(true), _is_stationary(false), _is_button_triggered(false), _is_buffer_triggered(false),
      _threshold_value(10.0f), _first_trigger_detected(false),
      _triggered_channel(0), _waiting_start_time(0),
      _captured_velocity(0.0f), _phase1_timeout_ms(500), _phase2_timeout_ms(350), _velocity_scale(0.8f), _velocity_limit(20.0f), _phase2_vel_threshold(0.5f),
      _phase2_peak_velocity(0.0f), _phase2_peak_time(0),
      _phase3_decay_target(5.0f), _phase3_current_velocity(0.0f), _phase3_pressing_motor(0),
      _motor1_id(nullptr), _motor1_mode(nullptr), _motor1_pos(nullptr),
      _motor1_vel(nullptr), _motor1_t(nullptr), _motor1_kp(nullptr), _motor1_kd(nullptr),
      _motor2_id(nullptr), _motor2_mode(nullptr), _motor2_pos(nullptr),
      _motor2_vel(nullptr), _motor2_t(nullptr), _motor2_kp(nullptr), _motor2_kd(nullptr),
      _global_mutex(nullptr), _diff_buffers(nullptr),
      _mode_type(SPEED_FOLLOW_MODE_PROGRAM), _ai_m1_phase(0), _ai_m2_phase(0),
      _current_m1_phase(0), _current_m2_phase(0), _both_static_start_time(0),
      _imu_roll_threshold(1.0f) {
    // 初始化滑动窗口
    initRollWindow(_roll_left_window);
    initRollWindow(_roll_right_window);
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

/**
 * @brief 初始化IMU滑动窗口
 * @param window 要初始化的窗口结构
 */
void SpeedFollowMode::initRollWindow(imu_roll_window_t& window) {
    window.count = 0;
    window.head = 0;
    for (int i = 0; i < IMU_WINDOW_SIZE; i++) {
        window.data[i] = 0.0f;
    }
}

/**
 * @brief 向IMU滑动窗口添加新的roll值
 * @param window 滑动窗口结构
 * @param value 新的roll值
 */
void SpeedFollowMode::addRollValue(imu_roll_window_t& window, float value) {
    window.data[window.head] = value;
    window.head = (window.head + 1) % IMU_WINDOW_SIZE;
    if (window.count < IMU_WINDOW_SIZE) {
        window.count++;
    }
}

/**
 * @brief 在滑动窗口中查找是否存在roll值减小超过阈值的情况
 * @param window 滑动窗口结构
 * @param threshold roll值减小的阈值
 * @return true 如果找到减小超过阈值的情况，false 否则
 *
 * @note 检查窗口中相邻数据点之间的roll值减小（从旧到新）
 * @note 例如：数据 22.3 21.5 21.4 21.3 21.1，会检测到 22.3->21.5 的减小（符合条件）
 */
bool SpeedFollowMode::findRollDecrease(const imu_roll_window_t& window, float threshold) {
    if (window.count < 2) {
        return false; // 数据不足，无法判断
    }

    // 遍历窗口中的相邻数据对，从旧到新
    for (int i = 0; i < window.count - 1; i++) {
        // 计算当前数据点和下一个数据点的实际索引（环形缓冲区）
        int current_idx = (window.head - window.count + i + IMU_WINDOW_SIZE) % IMU_WINDOW_SIZE;
        int next_idx = (window.head - window.count + i + 1 + IMU_WINDOW_SIZE) % IMU_WINDOW_SIZE;

        // 计算相邻两点之间的roll值变化（旧值 - 新值）
        float roll_delta = window.data[current_idx] - window.data[next_idx];

        // 如果减小量超过阈值，返回true
        if (roll_delta > threshold) {
            return true;
        }
    }

    return false;
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
    _config_motor1.waiting_duration_ms = 5;  // 等待时间
    _config_motor1.idle_duration_ms = 5;      // 空闲时间

    // 电机1第一阶段参数（抬腿）
    // vel: 正方向参考，实际速度由实时反馈动态计算
    _config_motor1.phase1.mode = 1;
    _config_motor1.phase1.pos = 0.0f;
    _config_motor1.phase1.vel = 10.0f;      // 方向参考：正
    _config_motor1.phase1.torque = 0.7f;
    _config_motor1.phase1.kp = 0.0f;
    _config_motor1.phase1.kd = 0.08f;

    // 电机1第二阶段参数（压腿）
    // vel: 负方向参考，实际速度由实时反馈动态计算
    _config_motor1.phase2.mode = 1;
    _config_motor1.phase2.pos = 0.0f;
    _config_motor1.phase2.vel = -10.0f;     // 方向参考：负
    _config_motor1.phase2.torque = -1.0f;
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
    _config_motor1.passive.vel = 0.0f;
    _config_motor1.passive.torque = 0.0f;
    _config_motor1.passive.kp = 0.0f;
    _config_motor1.passive.kd = 0.05f;

    // 配置电机2速度跟随模式参数
    _config_motor2.trigger_speed = -3.0f;  // 触发速度阈值
    _config_motor2.waiting_duration_ms = 5;  // 等待时间
    _config_motor2.idle_duration_ms = 5;      // 空闲时间

    // 电机2第一阶段参数（抬腿）
    // vel: 负方向参考，实际速度由实时反馈动态计算
    _config_motor2.phase1.mode = 1;
    _config_motor2.phase1.pos = 0.0f;
    _config_motor2.phase1.vel = -10.0f;     // 方向参考：负
    _config_motor2.phase1.torque = -0.7f;
    _config_motor2.phase1.kp = 0.0f;
    _config_motor2.phase1.kd = 0.08f;

    // 电机2第二阶段参数（压腿）
    // vel: 正方向参考，实际速度由实时反馈动态计算
    _config_motor2.phase2.mode = 1;
    _config_motor2.phase2.pos = 0.0f;
    _config_motor2.phase2.vel = 10.0f;      // 方向参考：正
    _config_motor2.phase2.torque = 1.0f;
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
    _waiting_start_time = 0;

    // 初始化动态参数
    _phase1_timeout_ms = 600;       // PHASE1超时时间（抬腿阶段）
    _phase2_timeout_ms = 600;       // PHASE2超时时间（压腿阶段）
    _velocity_scale = 0.8f;         // 速度缩放因子
    _velocity_limit = 50.0f;        // 速度跟随限幅
    _phase2_vel_threshold = 0.5f;   // PHASE2完成速度阈值

    // PHASE3压腿衰减参数
    _phase3_decay_target = 5.0f;    // 衰减目标速度
    _phase3_current_velocity = 0.0f; // 当前衰减速度
    _phase3_pressing_motor = 0;     // 正在压腿的电机
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
        _waiting_start_time = esp_timer_get_time() / 1000;

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
        _config_motor1.trigger_speed = 5.0f;
        _config_motor2.trigger_speed = -5.0f;
    } else {
        _config_motor1.trigger_speed = 3.0f;
        _config_motor2.trigger_speed = -3.0f;
    }

    // 处理阶段5（压腿衰减）：在任何状态下都持续处理衰减
    // 衰减策略：根据实际电机速度反馈，逐步将目标速度降低到目标值
    // 每次更新时，目标速度 = 实际速度 * 衰减系数，直到达到最小目标速度
    if (_phase3_pressing_motor != 0) {
        // 只在对应电机的数据到来时更新衰减
        if ((_phase3_pressing_motor == 1 && motor_data.id == 1) ||
            (_phase3_pressing_motor == 2 && motor_data.id == 2)) {

            // 获取当前实际速度的绝对值
            float actual_vel_abs = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;

            // 衰减系数：每次将速度降低到当前值的 80%
            const float DECAY_RATIO = 0.85f;

            // 计算新的目标速度：取实际速度的衰减值
            float new_target_vel = actual_vel_abs * DECAY_RATIO;

            // 确保目标速度不低于最小值（_phase3_decay_target）
            if (new_target_vel < _phase3_decay_target) {
                new_target_vel = _phase3_decay_target;
            }

            // 更新当前衰减速度
            _phase3_current_velocity = new_target_vel;

            // 检查是否达到衰减完成条件：实际速度已经降到目标值以下
            bool decay_complete = (actual_vel_abs <= _phase3_decay_target + 0.5f);

            if (!decay_complete) {
                // 继续衰减：设置衰减中的电机速度
                if (_phase3_pressing_motor == 1) {
                    // 电机1衰减（负速度方向）
                    float scaled_vel = -_phase3_current_velocity * _velocity_scale;
                    if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                    setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                  _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                    _current_m1_phase = 5;  // M1衰减中
                } else {
                    // 电机2衰减（正速度方向）
                    float scaled_vel = _phase3_current_velocity * _velocity_scale;
                    if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                    setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                  _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                    _current_m2_phase = 5;  // M2衰减中
                }
            } else {
                // 衰减完成，该电机进入空闲
                if (_phase3_pressing_motor == 1) {
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    _current_m1_phase = 0;  // M1空闲
                } else {
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    _current_m2_phase = 0;  // M2空闲
                }
                _phase3_pressing_motor = 0;  // 清除衰减标记
            }
        }
    }

    switch (_state) {
        case SPEED_FOLLOW_BUTTON_WAITING:
            // 按键触发等待状态：同时检测两个电机的速度，谁先触发就进入谁的PHASE1
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);

            // 按键等待状态下，两个电机都处于检测状态
            _current_m1_phase = 3;  // 检测速度触发中
            _current_m2_phase = 3;  // 检测速度触发中

            {
                bool motor1_triggered = false;
                bool motor2_triggered = false;

                if (_mode_type == SPEED_FOLLOW_MODE_AI) {
                    // AI模式：使用AI推理结果判断抬腿触发
                    if (motor_data.id == 1) {
                        motor1_triggered = (_ai_m1_phase == 1);  // 1 = 抬腿
                        _current_m1_phase = _ai_m1_phase;  // 更新当前阶段（包括静止0、抬腿1、压腿2）
                    } else if (motor_data.id == 2) {
                        motor2_triggered = (_ai_m2_phase == 1);  // 1 = 抬腿
                        _current_m2_phase = _ai_m2_phase;  // 更新当前阶段（包括静止0、抬腿1、压腿2）
                    }
                } else if (_mode_type == SPEED_FOLLOW_MODE_IMU) {
                    // IMU模式：使用滑动窗口查找roll角度减小超过阈值的情况
                    if (motor_data.id == 1) {
                        motor1_triggered = findRollDecrease(_roll_left_window, _imu_roll_threshold);
                        _current_m1_phase = motor1_triggered ? 1 : 0;
                    } else if (motor_data.id == 2) {
                        motor2_triggered = findRollDecrease(_roll_right_window, _imu_roll_threshold);
                        _current_m2_phase = motor2_triggered ? 1 : 0;
                    }
                } else {
                    // 程序模式：使用速度阈值判断
                    if (motor_data.id == 1) {
                        motor1_triggered = (motor_data.vel > _config_motor1.trigger_speed);
                        _current_m1_phase = motor1_triggered ? 1 : 0;  // 触发为1，否则为0
                    } else if (motor_data.id == 2) {
                        motor2_triggered = (motor_data.vel < _config_motor2.trigger_speed);
                        _current_m2_phase = motor2_triggered ? 1 : 0;  // 触发为1，否则为0
                    }
                }

                // 检测电机1触发
                if (motor1_triggered) {
                    _captured_velocity = motor_data.vel;  // 捕获速度
                    _phase_start_time = current_time;

                    if (_mode_type == SPEED_FOLLOW_MODE_AI) {
                        // AI模式：直接进入AI_RUNNING状态，同时控制双腿
                        _state = SPEED_FOLLOW_AI_RUNNING;
                        _both_static_start_time = 0;  // 重置静止计时器
                    } else {
                        // 程序模式：进入PHASE1状态机
                        _active_motor = 2; // 下个周期2号工作
                        _lifting_motor = 1;
                        _state = SPEED_FOLLOW_PHASE1;
                        float scaled_vel = _captured_velocity * _velocity_scale;
                        setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                      _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                    }
                }
                // 检测电机2触发
                else if (motor2_triggered) {
                    _captured_velocity = motor_data.vel;  // 捕获速度
                    _phase_start_time = current_time;

                    if (_mode_type == SPEED_FOLLOW_MODE_AI) {
                        // AI模式：直接进入AI_RUNNING状态，同时控制双腿
                        _state = SPEED_FOLLOW_AI_RUNNING;
                        _both_static_start_time = 0;  // 重置静止计时器
                    } else {
                        // 程序模式：进入PHASE1状态机
                        _active_motor = 1; // 下个周期1号工作
                        _lifting_motor = 2;
                        _state = SPEED_FOLLOW_PHASE1;
                        float scaled_vel = _captured_velocity * _velocity_scale;
                        setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                      _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                    }
                }
                // AI模式下，即使不触发也要更新阶段显示（静止状态已由开头的setMotorParams处理）
            }
            break;

        case SPEED_FOLLOW_WAITING:
            // 等待状态：等待配置时间后检测对应电机速度
            setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                          _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
            setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                          _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);

            // 根据触发通道设置标签和电机参数：ch6触发检测M2，ch7触发检测M1
            if (_triggered_channel == 6) {
                _current_m1_phase = 4;  // M1被动状态
                _current_m2_phase = 3;  // M2检测速度触发中
                // 电机1使用被动状态参数
                setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                              _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
            } else {
                _current_m1_phase = 3;  // M1检测速度触发中
                _current_m2_phase = 4;  // M2被动状态
                // 电机2使用被动状态参数
                setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                              _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
            }

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
                        _phase_start_time = current_time;

                        if (_mode_type == SPEED_FOLLOW_MODE_AI) {
                            // AI模式：直接进入AI_RUNNING状态
                            _state = SPEED_FOLLOW_AI_RUNNING;
                            _both_static_start_time = 0;  // 重置静止计时器
                        } else {
                            // 程序模式：进入PHASE1状态机
                            _lifting_motor = 2;
                            _active_motor = 1;
                            _state = SPEED_FOLLOW_PHASE1;
                            float scaled_vel = _captured_velocity * _velocity_scale;
                            setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                          _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                        }
                    }
                } else if (_triggered_channel == 7) {
                    // ch7触发，检测1号电机+v
                    if (motor_data.id == 1 && motor_data.vel > _config_motor1.trigger_speed) {
                        // 捕获速度值
                        _captured_velocity = motor_data.vel;
                        _phase_start_time = current_time;

                        if (_mode_type == SPEED_FOLLOW_MODE_AI) {
                            // AI模式：直接进入AI_RUNNING状态
                            _state = SPEED_FOLLOW_AI_RUNNING;
                            _both_static_start_time = 0;  // 重置静止计时器
                        } else {
                            // 程序模式：进入PHASE1状态机
                            _lifting_motor = 1;
                            _active_motor = 2;
                            _state = SPEED_FOLLOW_PHASE1;
                            float scaled_vel = _captured_velocity * _velocity_scale;
                            setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                          _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                        }
                    }
                }
            }
            }
            break;

        case SPEED_FOLLOW_MOTOR1_WORKING:
            // 1号电机工作状态：程序模式专用，检测速度触发
            {
                // 按键模式下超时时间为4秒，ch6/ch7触发模式为1.2秒
                uint32_t timeout_ms = _is_button_triggered ? 4000 : 1200;

                // 检测超时
                if (current_time - _working_start_time >= timeout_ms) {
                    if (_is_button_triggered) {
                        // ESP_LOGW(TAG, "⏱️ 1号电机工作超时4s未检测到速度触发，进入静止状态");
                        _is_button_triggered = false; // 清除按键触发标志
                    } else {
                        // ESP_LOGW(TAG, "⏱️ 1号电机工作超时1.2s未检测到速度触发，进入静止状态");
                    }
                    _is_stationary = true; // 设置静止标志（按键和缓存区触发都会触发语音）
                    _is_active = false;
                    _state = SPEED_FOLLOW_IDLE;
                    _first_trigger_detected = false;
                    _triggered_channel = 0;
                    _working_start_time = 0;
                    _current_m1_phase = 0;  // 超时退出，设置为空闲
                    _current_m2_phase = 0;  // 超时退出，设置为空闲
                    _phase3_pressing_motor = 0;  // 清除衰减状态
                    // 超时退出时，两个电机都设置为空闲参数
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    // 清空所有缓存区
                    if (_diff_buffers) {
                        diff_buffer_clear_all(_diff_buffers);
                        position_buffer_clear(position_buffer_get_motor1(_diff_buffers));
                        position_buffer_clear(position_buffer_get_motor2(_diff_buffers));
                        // ESP_LOGI(TAG, "🗑️ 已清空位置缓存区和ch6/ch7差值缓存区，等待新的阈值触发");
                    }
                    break;
                }

                // 根据模式选择触发条件
                bool motor1_should_trigger = false;

                if (_mode_type == SPEED_FOLLOW_MODE_IMU) {
                    // IMU模式：在滑动窗口中查找roll值减小超过阈值的情况
                    motor1_should_trigger = findRollDecrease(_roll_left_window, _imu_roll_threshold);
                } else {
                    // 程序模式：检测速度阈值
                    motor1_should_trigger = (motor_data.id == 1 && motor_data.vel > _config_motor1.trigger_speed);
                }

                if (motor1_should_trigger) {
                    // 捕获速度值
                    _captured_velocity = motor_data.vel;
                    _state = SPEED_FOLLOW_PHASE1;
                    _lifting_motor = 1; // 1号电机抬腿
                    _phase_start_time = current_time;
                    _current_m1_phase = 1;  // 进入抬腿阶段
                    // M2：如果正在衰减保持阶段5，否则设为被动
                    if (_phase3_pressing_motor != 2) {
                        _current_m2_phase = 4;  // M2被动状态
                    }

                    // 1号电机开始抬腿动作，使用捕获速度的0.8倍
                    float scaled_vel = _captured_velocity * _velocity_scale;
                    setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                  _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                    // M2：如果不在衰减中，使用被动参数
                    if (_phase3_pressing_motor != 2) {
                        setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                      _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                    }
                } else {
                    _current_m1_phase = 3;  // 检测速度触发中
                    // M2：如果正在衰减保持阶段5，否则设为被动
                    if (_phase3_pressing_motor != 2) {
                        _current_m2_phase = 4;  // M2被动状态
                        setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                      _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                    }
                    // M1使用空闲参数（检测状态）
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                }
            }
            break;

        case SPEED_FOLLOW_MOTOR2_WORKING:
            // 2号电机工作状态：程序模式专用，检测速度触发
            {
                // 按键模式下超时时间为4秒，ch6/ch7触发模式为1.2秒
                uint32_t timeout_ms = _is_button_triggered ? 4000 : 1200;

                // 检测超时
                if (current_time - _working_start_time >= timeout_ms) {
                    if (_is_button_triggered) {
                        // ESP_LOGW(TAG, "⏱️ 2号电机工作超时4s未检测到速度触发，进入静止状态");
                        _is_button_triggered = false; // 清除按键触发标志
                    } else {
                        // ESP_LOGW(TAG, "⏱️ 2号电机工作超时1.2s未检测到速度触发，进入静止状态");
                    }
                    _is_stationary = true; // 设置静止标志（按键和缓存区触发都会触发语音）
                    _is_active = false;
                    _state = SPEED_FOLLOW_IDLE;
                    _first_trigger_detected = false;
                    _triggered_channel = 0;
                    _working_start_time = 0;
                    _current_m1_phase = 0;  // 超时退出，设置为空闲
                    _current_m2_phase = 0;  // 超时退出，设置为空闲
                    _phase3_pressing_motor = 0;  // 清除衰减状态
                    // 超时退出时，两个电机都设置为空闲参数
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    // 清空所有缓存区
                    if (_diff_buffers) {
                        diff_buffer_clear_all(_diff_buffers);
                        position_buffer_clear(position_buffer_get_motor1(_diff_buffers));
                        position_buffer_clear(position_buffer_get_motor2(_diff_buffers));
                        // ESP_LOGI(TAG, "🗑️ 已清空位置缓存区和ch6/ch7差值缓存区，等待新的阈值触发");
                    }
                    break;
                }

                // 根据模式选择触发条件
                bool motor2_should_trigger = false;

                if (_mode_type == SPEED_FOLLOW_MODE_IMU) {
                    // IMU模式：在滑动窗口中查找roll值减小超过阈值的情况
                    motor2_should_trigger = findRollDecrease(_roll_right_window, _imu_roll_threshold);
                } else {
                    // 程序模式：检测速度阈值
                    motor2_should_trigger = (motor_data.id == 2 && motor_data.vel < _config_motor2.trigger_speed);
                }

                if (motor2_should_trigger) {
                    // 捕获速度值
                    _captured_velocity = motor_data.vel;
                    _state = SPEED_FOLLOW_PHASE1;
                    _lifting_motor = 2; // 2号电机抬腿
                    _phase_start_time = current_time;
                    // M1：如果正在衰减保持阶段5，否则设为被动
                    if (_phase3_pressing_motor != 1) {
                        _current_m1_phase = 4;  // M1被动状态
                    }
                    _current_m2_phase = 1;  // 进入抬腿阶段

                    // 2号电机开始抬腿动作，使用捕获速度的0.8倍
                    float scaled_vel = _captured_velocity * _velocity_scale;
                    setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                  _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                    // M1：如果不在衰减中，使用被动参数
                    if (_phase3_pressing_motor != 1) {
                        setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                      _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                    }
                } else {
                    // M1：如果正在衰减保持阶段5，否则设为被动
                    if (_phase3_pressing_motor != 1) {
                        _current_m1_phase = 4;  // M1被动状态
                        setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                      _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                    }
                    _current_m2_phase = 3;  // 检测速度触发中
                    // M2使用空闲参数（检测状态）
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                }
            }
            break;

        case SPEED_FOLLOW_PHASE1:
            // 抬腿阶段 - 程序模式专用：超时+速度反转检测
            {
                bool should_transition = false;

                // 根据抬腿电机设置标签：抬腿的电机为1，另一个为被动状态4（但如果正在衰减则保持5）
                if (_lifting_motor == 1) {
                    _current_m1_phase = 1;  // M1抬腿中
                    if (_phase3_pressing_motor != 2) {
                        _current_m2_phase = 4;  // M2被动状态
                    }
                } else {
                    if (_phase3_pressing_motor != 1) {
                        _current_m1_phase = 4;  // M1被动状态
                    }
                    _current_m2_phase = 1;  // M2抬腿中
                }

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
                    // 抬腿完成，切换工作电机并开始压腿
                    _active_motor = (_lifting_motor == 1) ? 2 : 1; // 切换工作电机
                    _state = SPEED_FOLLOW_PHASE2;
                    _phase_start_time = current_time;

                    // 进入PHASE2时设置压腿标签，另一个电机为被动状态4（但如果正在衰减则保持5）
                    if (_lifting_motor == 1) {
                        _current_m1_phase = 2;  // M1压腿中
                        if (_phase3_pressing_motor != 2) {
                            _current_m2_phase = 4;  // M2被动状态
                        }
                    } else {
                        if (_phase3_pressing_motor != 1) {
                            _current_m1_phase = 4;  // M1被动状态
                        }
                        _current_m2_phase = 2;  // M2压腿中
                    }

                    // 重置PHASE2峰值检测参数
                    _phase2_peak_velocity = 0.0f;
                    _phase2_peak_time = 0;

                    // 如果当前接收的就是抬腿电机的数据，立即设置PHASE2参数（强制使用正确方向）
                    if ((_lifting_motor == 1 && motor_data.id == 1) ||
                        (_lifting_motor == 2 && motor_data.id == 2)) {
                        float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                        if (_lifting_motor == 1) {
                            // 电机1压腿，速度应为负（-）
                            float scaled_vel = -abs_vel * _velocity_scale;
                            if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                            setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                          _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                            // 电机2：如果不在衰减中，使用被动参数
                            if (_phase3_pressing_motor != 2) {
                                setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                              _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                            }
                        } else {
                            // 电机2压腿，速度应为正（+）
                            float scaled_vel = abs_vel * _velocity_scale;
                            if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                            setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                          _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                            // 电机1：如果不在衰减中，使用被动参数
                            if (_phase3_pressing_motor != 1) {
                                setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                              _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                            }
                        }
                    }
                } else {
                    // 继续抬腿动作 - 持续根据当前电机的实时速度绝对值*0.8更新参数，强制使用正确方向
                    if (_lifting_motor == 1 && motor_data.id == 1) {
                        // 电机1抬腿，速度应为正（+），取绝对值后使用正方向
                        float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                        float scaled_vel = abs_vel * _velocity_scale;
                        if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                        setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                      _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                        // 电机2：如果不在衰减中，使用被动参数
                        if (_phase3_pressing_motor != 2) {
                            setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                          _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                        }
                        _current_m1_phase = 1;  // M1抬腿中
                        if (_phase3_pressing_motor != 2) {
                            _current_m2_phase = 4;  // M2被动状态
                        }
                    } else if (_lifting_motor == 2 && motor_data.id == 2) {
                        // 电机2抬腿，速度应为负（-），取绝对值后使用负方向
                        float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                        float scaled_vel = -abs_vel * _velocity_scale;
                        if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                        setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                      _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                        // 电机1：如果不在衰减中，使用被动参数
                        if (_phase3_pressing_motor != 1) {
                            setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                          _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                        }
                        if (_phase3_pressing_motor != 1) {
                            _current_m1_phase = 4;  // M1被动状态
                        }
                        _current_m2_phase = 1;  // M2抬腿中
                    }
                    // 注意：如果收到的是另一个电机的数据，保持上一次设置的参数不变
                }
            }
            break;

        case SPEED_FOLLOW_PHASE2:
            // 压腿阶段 - 程序模式专用：超时+速度峰值检测
            // 峰值检测后：当前腿进入阶段5（衰减），另一条腿进入阶段3（检测）
            {
                bool peak_detected = false;

                // 检查超时 - 超时直接进入IDLE
                if (current_time - _phase_start_time >= _phase2_timeout_ms) {
                    // 超时，直接进入空闲
                    _state = SPEED_FOLLOW_IDLE;
                    _lifting_motor = 0;
                    _phase_start_time = current_time;
                    _current_m1_phase = 0;
                    _current_m2_phase = 0;
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
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
                    // 但当前压腿的腿进入阶段5（衰减），另一条腿进入阶段3（检测）
                    _state = SPEED_FOLLOW_IDLE;
                    _phase_start_time = current_time;
                    _phase3_pressing_motor = _lifting_motor;  // 记录正在衰减的电机
                    _phase3_current_velocity = _phase2_peak_velocity;  // 从峰值开始衰减
                    _lifting_motor = 0;  // 清除抬腿电机标记

                    // 设置阶段标签：压腿的腿=5（衰减），另一条腿=3（检测）
                    if (_phase3_pressing_motor == 1) {
                        _current_m1_phase = 5;  // M1压腿衰减中
                        _current_m2_phase = 3;  // M2检测状态
                    } else {
                        _current_m1_phase = 3;  // M1检测状态
                        _current_m2_phase = 5;  // M2压腿衰减中
                    }
                } else {
                    // 继续压腿动作 - 持续根据当前电机的实时速度绝对值*0.8更新参数
                    if (_lifting_motor == 1 && motor_data.id == 1) {
                        float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                        float scaled_vel = -abs_vel * _velocity_scale;
                        if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                        setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                      _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                        // 电机2：如果不在衰减中，使用被动参数
                        if (_phase3_pressing_motor != 2) {
                            setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                          _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                        }
                        _current_m1_phase = 2;
                        if (_phase3_pressing_motor != 2) {
                            _current_m2_phase = 4;
                        }
                    } else if (_lifting_motor == 2 && motor_data.id == 2) {
                        float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;
                        float scaled_vel = abs_vel * _velocity_scale;
                        if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                        setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                      _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                        // 电机1：如果不在衰减中，使用被动参数
                        if (_phase3_pressing_motor != 1) {
                            setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                          _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                        }
                        if (_phase3_pressing_motor != 1) {
                            _current_m1_phase = 4;
                        }
                        _current_m2_phase = 2;
                    }
                }
            }
            break;

        case SPEED_FOLLOW_AI_RUNNING:
            // AI模式运行状态：根据注入的状态标签控制电机
            // 状态标签: 0=静止, 1=抬腿, 2=压腿, 3=检测状态, 4=被动状态, 5=压腿衰减
            {
                // 衰减系数（与程序模式一致）
                const float DECAY_RATIO = 0.85f;

                // 更新m1阶段并设置参数
                if (motor_data.id == 1) {
                    _current_m1_phase = _ai_m1_phase;
                    // 计算速度绝对值，根据阶段确定正确的方向
                    float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;

                    if (_ai_m1_phase == 1) {
                        // 抬腿阶段：电机1速度应为正（+）
                        float scaled_vel = abs_vel * _velocity_scale;
                        if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                        setMotorParams(1, _config_motor1.phase1.mode, _config_motor1.phase1.pos, scaled_vel,
                                      _config_motor1.phase1.torque, _config_motor1.phase1.kp, _config_motor1.phase1.kd);
                    } else if (_ai_m1_phase == 2) {
                        // 压腿阶段：电机1速度应为负（-）
                        float scaled_vel = -abs_vel * _velocity_scale;
                        if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                        setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                      _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                    } else if (_ai_m1_phase == 3) {
                        // 检测状态（使用idle参数）
                        setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                      _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    } else if (_ai_m1_phase == 4) {
                        // 被动状态（使用passive参数）
                        setMotorParams(1, _config_motor1.passive.mode, _config_motor1.passive.pos, _config_motor1.passive.vel,
                                      _config_motor1.passive.torque, _config_motor1.passive.kp, _config_motor1.passive.kd);
                    } else if (_ai_m1_phase == 5) {
                        // 压腿衰减阶段：基于实际速度反馈的渐进衰减
                        // 目标速度 = 实际速度 * 衰减系数，但不低于最小值
                        float target_vel = abs_vel * DECAY_RATIO;
                        if (target_vel < _phase3_decay_target) {
                            target_vel = _phase3_decay_target;
                        }
                        // 电机1速度应为负（-）
                        float scaled_vel = -target_vel * _velocity_scale;
                        if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                        setMotorParams(1, _config_motor1.phase2.mode, _config_motor1.phase2.pos, scaled_vel,
                                      _config_motor1.phase2.torque, _config_motor1.phase2.kp, _config_motor1.phase2.kd);
                    } else {
                        // 静止阶段 (0或其他)
                        setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                      _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    }
                }
                // 更新m2阶段并设置参数
                else if (motor_data.id == 2) {
                    _current_m2_phase = _ai_m2_phase;
                    // 计算速度绝对值，根据阶段确定正确的方向
                    float abs_vel = (motor_data.vel >= 0) ? motor_data.vel : -motor_data.vel;

                    if (_ai_m2_phase == 1) {
                        // 抬腿阶段：电机2速度应为负（-）
                        float scaled_vel = -abs_vel * _velocity_scale;
                        if (scaled_vel < -_velocity_limit) scaled_vel = -_velocity_limit;
                        setMotorParams(2, _config_motor2.phase1.mode, _config_motor2.phase1.pos, scaled_vel,
                                      _config_motor2.phase1.torque, _config_motor2.phase1.kp, _config_motor2.phase1.kd);
                    } else if (_ai_m2_phase == 2) {
                        // 压腿阶段：电机2速度应为正（+）
                        float scaled_vel = abs_vel * _velocity_scale;
                        if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                        setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                      _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                    } else if (_ai_m2_phase == 3) {
                        // 检测状态（使用idle参数）
                        setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                      _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    } else if (_ai_m2_phase == 4) {
                        // 被动状态（使用passive参数）
                        setMotorParams(2, _config_motor2.passive.mode, _config_motor2.passive.pos, _config_motor2.passive.vel,
                                      _config_motor2.passive.torque, _config_motor2.passive.kp, _config_motor2.passive.kd);
                    } else if (_ai_m2_phase == 5) {
                        // 压腿衰减阶段：基于实际速度反馈的渐进衰减
                        // 目标速度 = 实际速度 * 衰减系数，但不低于最小值
                        float target_vel = abs_vel * DECAY_RATIO;
                        if (target_vel < _phase3_decay_target) {
                            target_vel = _phase3_decay_target;
                        }
                        // 电机2速度应为正（+）
                        float scaled_vel = target_vel * _velocity_scale;
                        if (scaled_vel > _velocity_limit) scaled_vel = _velocity_limit;
                        setMotorParams(2, _config_motor2.phase2.mode, _config_motor2.phase2.pos, scaled_vel,
                                      _config_motor2.phase2.torque, _config_motor2.phase2.kp, _config_motor2.phase2.kd);
                    } else {
                        // 静止阶段 (0或其他)
                        setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                      _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    }
                }
                // 注：AI模式不自动退出，需要外部调用stopAIRunning()来停止
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
                    // ESP_LOGI(TAG, "🔄 空闲完成，%d号电机开始工作检测", _active_motor);
                }

                // 对于没有在衰减的电机，保持空闲状态
                // 衰减中的电机已在switch之前处理，phase标签也在那里设置
                if (_phase3_pressing_motor != 1) {
                    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
                    // 即将检测的电机设为3，另一条腿设为0或保持衰减状态
                    if (_active_motor == 1) {
                        _current_m1_phase = 3;  // M1即将检测
                    } else {
                        _current_m1_phase = 0;  // M1空闲等待
                    }
                }
                if (_phase3_pressing_motor != 2) {
                    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
                    if (_active_motor == 2) {
                        _current_m2_phase = 3;  // M2即将检测
                    } else {
                        _current_m2_phase = 0;  // M2空闲等待
                    }
                }
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
    // ESP_LOGI(TAG, "🔘 按键触发：进入BUTTON_WAITING状态，同时检测两个电机速度");  // 运行时日志已禁用
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
}

/**
 * @brief 停止AI运行状态（用于回放模式退出）
 * @note 将状态恢复到IDLE，并设置电机为空闲参数
 */
void SpeedFollowMode::stopAIRunning() {
    _is_active = false;
    _state = SPEED_FOLLOW_IDLE;
    _ai_m1_phase = 0;
    _ai_m2_phase = 0;
    _current_m1_phase = 0;
    _current_m2_phase = 0;
    _both_static_start_time = 0;

    // 设置电机为空闲参数
    setMotorParams(1, _config_motor1.idle.mode, _config_motor1.idle.pos, _config_motor1.idle.vel,
                  _config_motor1.idle.torque, _config_motor1.idle.kp, _config_motor1.idle.kd);
    setMotorParams(2, _config_motor2.idle.mode, _config_motor2.idle.pos, _config_motor2.idle.vel,
                  _config_motor2.idle.torque, _config_motor2.idle.kp, _config_motor2.idle.kd);
}

/**
 * @brief Web接口：调整助力（增加或减少phase1.torque）
 * @param increase true=增加助力，false=减少助力
 * @return 调整后的电机1助力值（用于语音播报）
 * @note 与GPIO3/GPIO5按键逻辑相同，调整电机1和电机2的phase1.torque
 *       电机1范围：0 ~ 2.0，电机2范围：-2.0 ~ 0
 *       步长：0.1
 */
float SpeedFollowMode::adjustTorque(bool increase) {
    const float TORQUE_STEP = 0.1f;
    const float MOTOR1_TORQUE_MIN = 0.0f;
    const float MOTOR1_TORQUE_MAX = 2.0f;
    const float MOTOR2_TORQUE_MIN = -2.0f;
    const float MOTOR2_TORQUE_MAX = 0.0f;

    if (increase) {
        // 增加助力
        float new_torque1 = _config_motor1.phase1.torque + TORQUE_STEP;
        if (new_torque1 > MOTOR1_TORQUE_MAX) {
            new_torque1 = MOTOR1_TORQUE_MAX;
        }
        _config_motor1.phase1.torque = new_torque1;

        float new_torque2 = _config_motor2.phase1.torque - TORQUE_STEP;
        if (new_torque2 < MOTOR2_TORQUE_MIN) {
            new_torque2 = MOTOR2_TORQUE_MIN;
        }
        _config_motor2.phase1.torque = new_torque2;

        // ESP_LOGI(TAG, "[WEB] 助力增加 - 电机1: %.2f, 电机2: %.2f", new_torque1, new_torque2);  // 运行时日志已禁用
        return new_torque1;
    } else {
        // 减少助力
        float new_torque1 = _config_motor1.phase1.torque - TORQUE_STEP;
        if (new_torque1 < MOTOR1_TORQUE_MIN) {
            new_torque1 = MOTOR1_TORQUE_MIN;
        }
        _config_motor1.phase1.torque = new_torque1;

        float new_torque2 = _config_motor2.phase1.torque + TORQUE_STEP;
        if (new_torque2 > MOTOR2_TORQUE_MAX) {
            new_torque2 = MOTOR2_TORQUE_MAX;
        }
        _config_motor2.phase1.torque = new_torque2;

        // ESP_LOGI(TAG, "[WEB] 助力减少 - 电机1: %.2f, 电机2: %.2f", new_torque1, new_torque2);  // 运行时日志已禁用
        return new_torque1;
    }
}

// ============================================================================
// 双模式切换相关接口实现
// ============================================================================

/**
 * @brief 设置模式类型（AI模式或程序模式）
 * @param mode 模式类型
 * @note 仅在IDLE状态允许切换模式，避免运动中切换导致的不连贯
 */
void SpeedFollowMode::setModeType(speed_follow_mode_type_t mode) {
    // 仅在IDLE状态允许切换
    if (_state == SPEED_FOLLOW_IDLE) {
        _mode_type = mode;
        // const char* mode_name = (mode == SPEED_FOLLOW_MODE_AI) ? "AI模式" :
        //                         (mode == SPEED_FOLLOW_MODE_IMU) ? "IMU模式" : "程序模式";
        // ESP_LOGI(TAG, "[模式切换] 切换到%s", mode_name);  // 运行时日志已禁用

        // 切换到IMU模式时清空滑动窗口
        if (mode == SPEED_FOLLOW_MODE_IMU) {
            initRollWindow(_roll_left_window);
            initRollWindow(_roll_right_window);
        }
    } else {
        // ESP_LOGW(TAG, "[模式切换] 当前状态非IDLE，无法切换模式");  // 运行时日志已禁用
    }
}

/**
 * @brief 更新AI推理结果
 * @param m1_phase m1阶段（0:静止, 1:抬腿, 2:压腿）
 * @param m2_phase m2阶段（0:静止, 1:抬腿, 2:压腿）
 */
void SpeedFollowMode::updateAIPhase(int m1_phase, int m2_phase) {
    _ai_m1_phase = m1_phase;
    _ai_m2_phase = m2_phase;
}

/**
 * @brief 获取当前m1阶段（用于网页显示）
 * @return 阶段值（0:静止, 1:抬腿, 2:压腿）
 */
int SpeedFollowMode::getCurrentM1Phase() const {
    return _current_m1_phase;
}

/**
 * @brief 获取当前m2阶段（用于网页显示）
 * @return 阶段值（0:静止, 1:抬腿, 2:压腿）
 */
int SpeedFollowMode::getCurrentM2Phase() const {
    return _current_m2_phase;
}

/**
 * @brief 更新IMU滑动窗口的roll值
 * @param roll_left 左腿的roll值
 * @param roll_right 右腿的roll值
 * @param left_valid 左腿数据是否有效
 * @param right_valid 右腿数据是否有效
 *
 * @note 从main.cpp中IMU数据处理后调用
 */
void SpeedFollowMode::updateRollValue(float roll_left, float roll_right, bool left_valid, bool right_valid) {
    if (left_valid) {
        addRollValue(_roll_left_window, roll_left);
    }
    if (right_valid) {
        addRollValue(_roll_right_window, roll_right);
    }
}
