#include "ai_fixed_cycle.h"

// 速度曲线查找表（20个点，从CSV数据分析得出）
static const float AI_VELOCITY_CURVE[AI_VELOCITY_CURVE_SIZE] = {
    0.318f, 0.717f, 0.953f, 0.899f, 0.762f, 0.580f, 0.439f, 0.375f, 0.374f, 0.414f,
    0.440f, 0.420f, 0.358f, 0.279f, 0.217f, 0.176f, 0.161f, 0.150f, 0.116f, 0.064f
};

void ai_fixed_cycle_init(ai_fixed_cycle_state_t* state) {
    state->cycle_duration_ms = 1000;
    state->cycle_start_time = 0;
    state->current_leg = 1;
    state->peak_velocity = 40.0f;
}

void ai_fixed_cycle_start(ai_fixed_cycle_state_t* state, uint32_t current_time) {
    state->cycle_start_time = current_time;
    state->current_leg = 1;
}

void ai_fixed_cycle_update(ai_fixed_cycle_state_t* state, uint32_t current_time,
                           ai_fixed_cycle_output_t* output) {
    // 计算当前周期内的时间进度
    uint32_t elapsed = current_time - state->cycle_start_time;

    // 检查是否需要切换到另一条腿
    if (elapsed >= state->cycle_duration_ms) {
        state->current_leg = (state->current_leg == 1) ? 2 : 1;
        state->cycle_start_time = current_time;
        elapsed = 0;
    }

    // 计算速度曲线索引
    int curve_index = (elapsed * AI_VELOCITY_CURVE_SIZE) / state->cycle_duration_ms;
    if (curve_index >= AI_VELOCITY_CURVE_SIZE) curve_index = AI_VELOCITY_CURVE_SIZE - 1;

    // 从查找表获取归一化速度，乘以峰值速度
    float curve_vel = AI_VELOCITY_CURVE[curve_index] * state->peak_velocity;

    // 根据当前抬腿的腿设置输出
    if (state->current_leg == 1) {
        output->motor1_vel = curve_vel;
        output->motor2_vel = curve_vel;
        output->m1_phase = 1;
        output->m2_phase = 2;
        output->lifting_motor = 1;
    } else {
        output->motor1_vel = -curve_vel;
        output->motor2_vel = -curve_vel;
        output->m1_phase = 2;
        output->m2_phase = 1;
        output->lifting_motor = 2;
    }
}

const float* ai_fixed_cycle_get_curve(void) {
    return AI_VELOCITY_CURVE;
}
