#ifndef SPEED_FOLLOW_MODE_H
#define SPEED_FOLLOW_MODE_H

#include "motor_commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// 速度跟随模式状态
typedef enum {
    SPEED_FOLLOW_IDLE,      // 空闲状态（持续300ms）
    SPEED_FOLLOW_PHASE1,    // 第一阶段：负向运动
    SPEED_FOLLOW_PHASE2     // 第二阶段：正向运动
} speed_follow_state_t;

// 速度跟随模式配置
typedef struct {
    float trigger_speed;    // 触发速度阈值
    uint32_t idle_duration_ms;    // 空闲状态持续时间
    uint32_t phase1_duration_ms;  // 第一阶段持续时间
    uint32_t phase2_duration_ms;  // 第二阶段持续时间

    // 第一阶段参数 (mode, pos, vel, torque, kp, kd)
    struct {
        uint8_t mode;
        float pos;
        float vel;
        float torque;
        float kp;
        float kd;
    } phase1;

    // 第二阶段参数
    struct {
        uint8_t mode;
        float pos;
        float vel;
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
} speed_follow_config_t;

class SpeedFollowMode {
public:
    SpeedFollowMode();

    // 初始化速度跟随模式
    void init();

    // 更新电机数据，检测触发条件，并修改全局参数
    void update(const MotorDataA1& motor_data);

    // 设置全局参数的访问接口（兼容性，指向电机2）
    void setGlobalParams(uint8_t* motor_id, uint8_t* motor_mode, float* motor_pos,
                        float* motor_vel, float* motor_t, float* motor_kp, float* motor_kd,
                        SemaphoreHandle_t mutex);

    // 设置双电机参数的访问接口
    void setDualMotorParams(uint8_t* motor1_id, uint8_t* motor1_mode, float* motor1_pos, float* motor1_vel,
                           float* motor1_t, float* motor1_kp, float* motor1_kd,
                           uint8_t* motor2_id, uint8_t* motor2_mode, float* motor2_pos, float* motor2_vel,
                           float* motor2_t, float* motor2_kp, float* motor2_kd,
                           SemaphoreHandle_t mutex);

    // 获取当前状态
    speed_follow_state_t getState() const { return _state; }

    // 重置状态
    void reset();

private:
    speed_follow_state_t _state;
    speed_follow_config_t _config_motor1;  // 电机1配置
    speed_follow_config_t _config_motor2;  // 电机2配置
    uint32_t _phase_start_time;

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

    // 检测触发条件
    bool checkTriggerCondition(const MotorDataA1& motor_data);

    // 设置指定电机的全局参数
    void setMotorParams(uint8_t motor_id, uint8_t mode, float pos, float vel, float torque, float kp, float kd);
};

#endif // SPEED_FOLLOW_MODE_H