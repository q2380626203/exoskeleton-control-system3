#ifndef POSITION_BUFFER_H
#define POSITION_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 环形缓存区大小
// 500Hz采样率 × 3.75秒 = 1875个数据点，用于分析两个周期的位置变化
#define POSITION_BUFFER_SIZE 1875

// ch6 ch7差值缓存区大小
// 500Hz采样率 × 1.2秒 = 600个数据点，用于存储波峰波谷差值
#define DIFF_BUFFER_SIZE 600

// 位置数据结构
typedef struct {
    float position;     // 位置值
    uint32_t timestamp; // 时间戳（毫秒）
} position_data_t;

// 差值数据结构（用于ch6 ch7）
typedef struct {
    float diff_value;   // 波峰波谷差值
    uint32_t timestamp; // 时间戳（毫秒）
} diff_data_t;

// 环形缓存区结构
typedef struct {
    position_data_t buffer[POSITION_BUFFER_SIZE];
    uint32_t head;      // 头指针（写入位置）
    uint32_t tail;      // 尾指针（读取位置）
    uint32_t count;     // 当前数据数量
    bool is_full;       // 是否已满
} position_buffer_t;

// 差值环形缓存区结构
typedef struct {
    diff_data_t buffer[DIFF_BUFFER_SIZE];
    uint32_t head;      // 头指针（写入位置）
    uint32_t tail;      // 尾指针（读取位置）
    uint32_t count;     // 当前数据数量
    bool is_full;       // 是否已满
} diff_buffer_t;

// 电机位置缓存区管理结构
typedef struct {
    position_buffer_t motor1_buffer;    // 1号电机缓存区
    position_buffer_t motor2_buffer;    // 2号电机缓存区
    diff_buffer_t ch6_buffer;           // ch6差值缓存区（1号电机波峰波谷差值）
    diff_buffer_t ch7_buffer;           // ch7差值缓存区（2号电机波峰波谷差值）
} motor_position_buffers_t;

// 函数声明

/**
 * @brief 初始化电机位置缓存区
 * @param buffers 电机位置缓存区管理结构指针
 */
void position_buffer_init(motor_position_buffers_t* buffers);

/**
 * @brief 向1号电机缓存区添加位置数据
 * @param buffers 电机位置缓存区管理结构指针
 * @param position 位置值
 * @param timestamp 时间戳
 * @return true 成功，false 失败
 */
bool position_buffer_add_motor1(motor_position_buffers_t* buffers, float position, uint32_t timestamp);

/**
 * @brief 向2号电机缓存区添加位置数据
 * @param buffers 电机位置缓存区管理结构指针
 * @param position 位置值
 * @param timestamp 时间戳
 * @return true 成功，false 失败
 */
bool position_buffer_add_motor2(motor_position_buffers_t* buffers, float position, uint32_t timestamp);

/**
 * @brief 清空缓存区
 * @param buffer 目标缓存区指针
 */
void position_buffer_clear(position_buffer_t* buffer);

/**
 * @brief 获取1号电机缓存区指针
 * @param buffers 电机位置缓存区管理结构指针
 * @return 1号电机缓存区指针
 */
position_buffer_t* position_buffer_get_motor1(motor_position_buffers_t* buffers);

/**
 * @brief 获取2号电机缓存区指针
 * @param buffers 电机位置缓存区管理结构指针
 * @return 2号电机缓存区指针
 */
position_buffer_t* position_buffer_get_motor2(motor_position_buffers_t* buffers);

// 波形分析结果结构
typedef struct {
    bool has_peak;          // 是否找到波峰
    bool has_valley;        // 是否找到波谷
    float peak_value;       // 波峰值
    float valley_value;     // 波谷值
    float peak_valley_diff; // 波峰波谷差值
    uint32_t peak_index;    // 波峰在缓存区中的索引
    uint32_t valley_index;  // 波谷在缓存区中的索引
} wave_analysis_result_t;

/**
 * @brief 分析1号电机最新的一个波峰波谷
 * @param buffers 电机位置缓存区管理结构指针
 * @param result 分析结果输出
 * @return true 分析成功，false 数据不足或分析失败
 */
bool position_buffer_analyze_motor1_wave(motor_position_buffers_t* buffers, wave_analysis_result_t* result);

/**
 * @brief 分析2号电机最新的一个波峰波谷
 * @param buffers 电机位置缓存区管理结构指针
 * @param result 分析结果输出
 * @return true 分析成功，false 数据不足或分析失败
 */
bool position_buffer_analyze_motor2_wave(motor_position_buffers_t* buffers, wave_analysis_result_t* result);

/**
 * @brief 向ch6缓存区添加差值数据
 * @param buffers 电机位置缓存区管理结构指针
 * @param diff_value 波峰波谷差值
 * @param timestamp 时间戳
 * @return true 成功，false 失败
 */
bool diff_buffer_add_ch6(motor_position_buffers_t* buffers, float diff_value, uint32_t timestamp);

/**
 * @brief 向ch7缓存区添加差值数据
 * @param buffers 电机位置缓存区管理结构指针
 * @param diff_value 波峰波谷差值
 * @param timestamp 时间戳
 * @return true 成功，false 失败
 */
bool diff_buffer_add_ch7(motor_position_buffers_t* buffers, float diff_value, uint32_t timestamp);

/**
 * @brief 获取ch6缓存区中的最大差值
 * @param buffers 电机位置缓存区管理结构指针
 * @return 最大差值，如果缓存区为空返回0.0f
 */
float diff_buffer_get_ch6_max(motor_position_buffers_t* buffers);

/**
 * @brief 获取ch7缓存区中的最大差值
 * @param buffers 电机位置缓存区管理结构指针
 * @return 最大差值，如果缓存区为空返回0.0f
 */
float diff_buffer_get_ch7_max(motor_position_buffers_t* buffers);

/**
 * @brief 清空ch6和ch7差值缓存区
 * @param buffers 电机位置缓存区管理结构指针
 */
void diff_buffer_clear_all(motor_position_buffers_t* buffers);

#ifdef __cplusplus
}
#endif

#endif // POSITION_BUFFER_H