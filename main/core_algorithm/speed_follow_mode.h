#ifndef SPEED_FOLLOW_MODE_H
#define SPEED_FOLLOW_MODE_H

#include <cmath>
#include "motor_commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "position_buffer.h"

// 阶段判断模式
typedef enum {
    SPEED_FOLLOW_MODE_AI,        // AI模式
    SPEED_FOLLOW_MODE_PROGRAM,   // 程序硬编模式（默认）
    SPEED_FOLLOW_MODE_IMU        // IMU模式（roll角度触发）
} speed_follow_mode_type_t;

// 速度跟随模式状态
typedef enum {
    SPEED_FOLLOW_IDLE,              // 空闲周期
    SPEED_FOLLOW_WAITING,           // 等待状态：阈值触发后等待检测另一电机速度
    SPEED_FOLLOW_BUTTON_WAITING,    // 按键触发等待状态：同时检测两个电机速度，谁先触发就进入谁的工作状态
    SPEED_FOLLOW_AI_RUNNING,        // AI模式运行状态：同时控制双腿
    SPEED_FOLLOW_MOTOR1_WORKING,    // 1号电机工作（检测+v）
    SPEED_FOLLOW_MOTOR2_WORKING,    // 2号电机工作（检测-v）
    SPEED_FOLLOW_PHASE1,            // 第一阶段：抬腿动作
    SPEED_FOLLOW_PHASE2             // 第二阶段：压腿动作
} speed_follow_state_t;

// 速度跟随模式配置
typedef struct {
    float trigger_speed;    // 触发速度阈值

    // 第一阶段参数 (mode, pos, vel, torque, kp, kd)
    // vel: 用于指示正确的速度方向（正/负），实际速度由实时反馈动态计算
    struct {
        uint8_t mode;
        float pos;
        float vel;      // 方向参考：电机1为正，电机2为负
        float torque;
        float kp;
        float kd;
    } phase1;

    // 第二阶段参数
    // vel: 用于指示正确的速度方向（正/负），实际速度由实时反馈动态计算
    struct {
        uint8_t mode;
        float pos;
        float vel;      // 方向参考：电机1为负，电机2为正
        float torque;
        float kp;
        float kd;
    } phase2;

    // 空闲状态参数 (1 0.0 0.0 0.0 0.0 0.0)
    struct {
        uint8_t mode;
        float pos;
        float vel;
        float torque;
        float kp;
        float kd;
    } idle;

    // 被动状态参数（另一个电机处于抬腿/压腿时使用）
    struct {
        uint8_t mode;
        float pos;
        float vel;
        float torque;
        float kp;
        float kd;
    } passive;
} speed_follow_config_t;

// IMU滑动窗口数据结构（已废弃 - IMU功能已移除）
// 保留类型定义以保持 API 兼容性
typedef struct {
    float data[5];      // 滑动窗口数据
    int count;          // 当前窗口中的有效数据数量
    int head;           // 队列头部索引（最新数据插入位置）
} imu_roll_window_t;

class SpeedFollowMode {
private:
    // 内部使用的私有函数
    void setMotorParams(uint8_t motor_id, uint8_t mode, float pos, float vel, float torque, float kp, float kd);

public:
    SpeedFollowMode();

    // 初始化速度跟随模式
    void init();

    // 设置双电机参数的访问接口
    void setDualMotorParams(uint8_t* motor1_id, uint8_t* motor1_mode, float* motor1_pos, float* motor1_vel,
                           float* motor1_t, float* motor1_kp, float* motor1_kd,
                           uint8_t* motor2_id, uint8_t* motor2_mode, float* motor2_pos, float* motor2_vel,
                           float* motor2_t, float* motor2_kp, float* motor2_kd,
                           SemaphoreHandle_t mutex);

    // 设置差值缓存区指针（用于超时清空）
    void setDiffBuffers(motor_position_buffers_t* buffers);

    // 自动开关控制函数
    void enableAutoSwitch(bool enable = true);                    // 启用/禁用自动开关
    void setThreshold(float threshold);                           // 设置触发阈值
    void checkThresholdAndActivate(float ch6_max, float ch7_max); // 检查阈值并激活

    // 更新电机数据，检测触发条件，并修改全局参数
    void update(const MotorDataA1& motor_data, float ch6_max, float ch7_max,
                float roll_left, float roll_right, bool imu_left_valid, bool imu_right_valid);

    // 直接进入AI运行状态（用于回放模式）
    void startAIRunning();

    // 停止AI运行状态（用于回放模式退出）
    void stopAIRunning();

    // Web接口：获取电机配置
    speed_follow_config_t* getMotorConfig(int motor) {
        return (motor == 1) ? &_config_motor1 : &_config_motor2;
    }

    // Web接口：启用/禁用速度跟随
    void enable(bool enabled) {
        _is_active = enabled;
    }

    // Web接口：启用/禁用电机控制（关闭时状态机继续运行但不控制电机）
    void enableMotorControl(bool enabled) {
        _is_motor_control_enabled = enabled;
    }

    // Web接口：获取当前状态
    speed_follow_state_t getState() const {
        return _state;
    }

    // Web接口：获取当前活动电机（1或2）
    uint8_t getActiveMotor() const {
        return _active_motor;
    }

    // Web接口：获取当前抬腿电机（1或2，0表示无）
    uint8_t getLiftingMotor() const {
        return _lifting_motor;
    }

    // Web接口：调整助力（增加或减少phase1.torque）
    // 返回调整后的电机1助力值（用于语音播报）
    float adjustTorque(bool increase);

    // Web接口：调整Kd参数（增加或减少phase1和phase2的kd）
    // 返回调整后的电机1 phase1的kd值
    float adjustKd(bool increase);

    // Web接口：调整Phase2力矩（增加或减少phase2.torque的绝对值）
    // 返回调整后的电机1 phase2的torque绝对值（用于语音播报）
    float adjustPhase2Torque(bool increase);

    // Web接口：获取当前电机1 phase1的torque值
    float getCurrentTorque() const { return _config_motor1.phase1.torque; }

    // Web接口：获取当前电机1 phase1的kd值
    float getCurrentKd() const { return _config_motor1.phase1.kd; }

    // Web接口：获取当前电机1 phase2的torque绝对值
    float getCurrentPhase2Torque() const { return fabs(_config_motor1.phase2.torque); }

    // 检查是否处于静止状态（用于按键触发语音播放）
    bool isStationary() const { return _is_stationary; }

    // 检查是否缓存区触发激活（用于播放助力开启语音）
    bool isBufferTriggered() const { return _is_buffer_triggered; }

    // 清除静止标志（按键检测任务调用）
    void clearStationaryFlag() { _is_stationary = false; }

    // 清除缓存区触发标志（按键检测任务调用）
    void clearBufferTriggeredFlag() { _is_buffer_triggered = false; }

    // ===== 双模式切换接口 =====
    speed_follow_mode_type_t getModeType() const { return _mode_type; }

    // 获取当前判断的阶段（用于网页显示）
    int getCurrentM1Phase() const;
    int getCurrentM2Phase() const;

    // ===== 公共参数访问接口（用于TCP参数调整） =====
    // 速度缩放因子
    float getVelocityScale() const { return _velocity_scale; }
    void setVelocityScale(float scale) { _velocity_scale = scale; }

    // 速度限幅
    float getVelocityLimit() const { return _velocity_limit; }
    void setVelocityLimit(float limit) { _velocity_limit = limit; }

    // PHASE1超时时间（ms）
    uint32_t getPhase1Timeout() const { return _phase1_timeout_ms; }
    void setPhase1Timeout(uint32_t timeout_ms) { _phase1_timeout_ms = timeout_ms; }

    // PHASE2超时时间（ms）
    uint32_t getPhase2Timeout() const { return _phase2_timeout_ms; }
    void setPhase2Timeout(uint32_t timeout_ms) { _phase2_timeout_ms = timeout_ms; }

    // AI固定周期模式参数
    uint32_t getAICycleDuration() const { return _ai_cycle_duration_ms; }
    void setAICycleDuration(uint32_t duration_ms) { _ai_cycle_duration_ms = duration_ms; }
    float getAIPeakVelocity() const { return _ai_peak_velocity; }
    void setAIPeakVelocity(float velocity) { _ai_peak_velocity = velocity; }

private:
    speed_follow_state_t _state;
    speed_follow_config_t _config_motor1;  // 电机1配置
    speed_follow_config_t _config_motor2;  // 电机2配置
    uint32_t _phase_start_time;

    // 新增：工作电机管理
    uint8_t _active_motor;          // 当前工作电机（1或2）
    uint8_t _lifting_motor;         // 当前抬腿电机（1或2）
    uint32_t _working_start_time;   // 工作状态开始时间

    // 自动开关控制
    bool _auto_switch_enabled;      // 自动开关是否启用
    bool _is_active;                // 速度跟随是否激活
    bool _is_motor_control_enabled; // 电机控制是否启用（关闭时状态机运行但不控制电机）
    bool _is_stationary;            // 是否处于静止状态（用于按键触发语音播放）
    bool _is_button_triggered;      // 是否为按键触发模式
    bool _is_buffer_triggered;      // 是否为缓存区触发激活（用于播放助力开启语音）
    float _threshold_value;         // 触发阈值（默认10.0）
    bool _first_trigger_detected;   // 是否已检测到首次触发
    uint8_t _triggered_channel;     // 触发的通道（6或7）
    uint8_t _waiting_data_count;    // 等待状态下已接收的数据点计数
    uint8_t _idle_data_count;       // 空闲状态下已接收的数据点计数
    uint8_t _working_data_count;    // 工作状态下已接收的数据点计数

    // 速度捕获和动态参数
    float _captured_velocity;       // 捕获的触发速度值
    uint32_t _phase1_timeout_ms;    // PHASE1超时时间（默认500ms）
    uint32_t _phase2_timeout_ms;    // PHASE2超时时间（默认350ms）
    float _velocity_scale;          // 速度缩放因子（默认0.8）
    float _velocity_limit;          // 速度跟随限幅（默认20 rad/s）
    float _phase2_vel_threshold;    // PHASE2完成速度阈值（默认0.5 rad/s）

    // PHASE2峰值检测参数
    float _phase2_peak_velocity;    // PHASE2阶段的速度峰值（绝对值）
    uint32_t _phase2_peak_time;     // 速度峰值出现的时间

    // 电机1全局参数指针
    uint8_t* _motor1_id;
    uint8_t* _motor1_mode;
    float* _motor1_pos;
    float* _motor1_vel;
    float* _motor1_t;
    float* _motor1_kp;
    float* _motor1_kd;

    // 电机2全局参数指针（兼容性）
    uint8_t* _motor2_id;
    uint8_t* _motor2_mode;
    float* _motor2_pos;
    float* _motor2_vel;
    float* _motor2_t;
    float* _motor2_kp;
    float* _motor2_kd;

    SemaphoreHandle_t _global_mutex;

    // 差值缓存区指针（用于超时清空）
    motor_position_buffers_t* _diff_buffers;

    // ===== 双模式切换相关 =====
    speed_follow_mode_type_t _mode_type;  // 当前模式类型（AI/程序）
    int _ai_m1_phase;                     // AI预测的m1阶段（0:静止, 1:抬腿, 2:压腿）
    int _ai_m2_phase;                     // AI预测的m2阶段（0:静止, 1:抬腿, 2:压腿）
    int _current_m1_phase;                // 当前m1阶段（用于网页显示）
    int _current_m2_phase;                // 当前m2阶段（用于网页显示）

    // AI模式静止检测
    uint32_t _both_static_start_time;     // 双腿都静止的开始时间（用于4秒超时检测）

    // ===== AI固定周期模式相关 =====
    uint32_t _ai_cycle_duration_ms;       // AI模式单腿周期时长（默认1000ms）
    uint32_t _ai_cycle_start_time;        // 当前周期开始时间
    uint8_t _ai_current_leg;              // 当前抬腿的腿（1或2）
    float _ai_peak_velocity;              // AI模式峰值速度（默认40.0 rad/s）
};

#endif // SPEED_FOLLOW_MODE_H