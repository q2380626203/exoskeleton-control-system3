#ifndef AI_FIXED_CYCLE_H
#define AI_FIXED_CYCLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 速度曲线查找表大小
#define AI_VELOCITY_CURVE_SIZE 20

// AI固定周期模式状态结构体
typedef struct {
    uint32_t cycle_duration_ms;    // 单腿周期时长（默认1000ms）
    uint32_t cycle_start_time;     // 当前周期开始时间
    uint8_t current_leg;           // 当前抬腿的腿（1或2）
    float peak_velocity;           // 峰值速度（默认40.0 rad/s）
} ai_fixed_cycle_state_t;

// AI固定周期模式输出结构体
typedef struct {
    float motor1_vel;              // 电机1速度
    float motor2_vel;              // 电机2速度
    int m1_phase;                  // 电机1阶段（1=抬腿, 2=压腿）
    int m2_phase;                  // 电机2阶段（1=抬腿, 2=压腿）
    uint8_t lifting_motor;         // 当前抬腿电机（1或2）
} ai_fixed_cycle_output_t;

/**
 * @brief 初始化AI固定周期模式状态
 * @param state 状态结构体指针
 */
void ai_fixed_cycle_init(ai_fixed_cycle_state_t* state);

/**
 * @brief 启动AI固定周期模式
 * @param state 状态结构体指针
 * @param current_time 当前时间（毫秒）
 */
void ai_fixed_cycle_start(ai_fixed_cycle_state_t* state, uint32_t current_time);

/**
 * @brief 更新AI固定周期模式，计算输出速度
 * @param state 状态结构体指针
 * @param current_time 当前时间（毫秒）
 * @param output 输出结构体指针
 */
void ai_fixed_cycle_update(ai_fixed_cycle_state_t* state, uint32_t current_time,
                           ai_fixed_cycle_output_t* output);

/**
 * @brief 获取速度曲线查找表
 * @return 速度曲线数组指针
 */
const float* ai_fixed_cycle_get_curve(void);

#ifdef __cplusplus
}
#endif

#endif // AI_FIXED_CYCLE_H
