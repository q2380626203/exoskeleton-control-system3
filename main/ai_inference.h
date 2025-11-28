#ifndef AI_INFERENCE_H
#define AI_INFERENCE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * AI推理结果结构体（双腿联合模型）
 */
typedef struct {
    // 场景识别 (0: 平地, 1: 爬楼)
    // 注意: 目前只训练了平地场景数据
    int scene;
    float scene_confidence;

    // m1阶段识别 (0: 静止, 1: 抬腿, 2: 压腿)
    int m1_phase;
    float m1_phase_confidence;

    // m2阶段识别 (0: 静止, 1: 抬腿, 2: 压腿)
    int m2_phase;
    float m2_phase_confidence;

    // 参数调整建议
    float delta_torque;   // 扭矩调整
    float delta_kd;       // 阻尼调整
    float delta_scale;    // 缩放调整
} ai_inference_result_t;

/**
 * 初始化AI模型
 *
 * 必须在使用推理功能前调用一次
 *
 * @return true: 成功, false: 失败
 */
bool ai_model_init(void);

/**
 * 运行AI推理（双腿联合模型，仅速度）
 *
 * @param m1_velocity_window m1速度窗口数据 (长度必须为50)
 * @param m2_velocity_window m2速度窗口数据 (长度必须为50)
 * @param result 推理结果输出（包含m1和m2的阶段预测）
 * @return true: 成功, false: 失败
 */
bool ai_run_inference(const float m1_velocity_window[50], const float m2_velocity_window[50],
                      ai_inference_result_t *result);

/**
 * 获取模型信息
 *
 * @param model_size 模型大小（字节）
 * @param arena_used 实际使用的Tensor Arena大小（字节）
 */
void ai_get_model_info(uint32_t *model_size, uint32_t *arena_used);

#ifdef __cplusplus
}
#endif

#endif // AI_INFERENCE_H
