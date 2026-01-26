#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 力矩调整参数 ====================
// 助力调整步长
#define TORQUE_STEP 0.1f
// 电机1 Phase1 力矩范围：0 ~ 2.5
#define MOTOR1_PHASE1_MIN 0.0f
#define MOTOR1_PHASE1_MAX 2.0f
// 电机1 Phase2 力矩范围：-2.5 ~ 0
#define MOTOR1_PHASE2_MIN -2.0f
#define MOTOR1_PHASE2_MAX 0.0f
// 电机2 Phase1 力矩范围：-2.5 ~ 0
#define MOTOR2_PHASE1_MIN -2.0f
#define MOTOR2_PHASE1_MAX 0.0f
// 电机2 Phase2 力矩范围：0 ~ 2.5
#define MOTOR2_PHASE2_MIN 0.0f
#define MOTOR2_PHASE2_MAX 2.0f

// ==================== KD 调整参数 ====================
// KD 调整步长
#define KD_STEP 0.01f
// KD 范围：0.00 ~ 0.20
#define KD_MIN 0.0f
#define KD_MAX 0.20f

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONFIG_H
