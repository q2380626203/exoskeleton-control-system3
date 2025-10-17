#include "position_buffer.h"
#include <string.h>

// ============================================================================
// 静态函数(内部使用)
// ============================================================================

/**
 * @brief 向位置缓存区添加数据
 * @param buffer 位置缓存区指针
 * @param position 位置值
 * @param timestamp 时间戳
 * @return true 添加成功，false 失败
 */
static bool position_buffer_add(position_buffer_t* buffer, float position, uint32_t timestamp) {
    if (buffer == NULL) return false;

    // 在头部位置写入新数据
    buffer->buffer[buffer->head].position = position;
    buffer->buffer[buffer->head].timestamp = timestamp;

    // 更新头指针
    buffer->head = (buffer->head + 1) % POSITION_BUFFER_SIZE;

    // 如果缓存区已满，更新尾指针
    if (buffer->is_full) {
        buffer->tail = (buffer->tail + 1) % POSITION_BUFFER_SIZE;
    } else {
        buffer->count++;
        if (buffer->count == POSITION_BUFFER_SIZE) {
            buffer->is_full = true;
        }
    }

    return true;
}

/**
 * @brief 获取历史数据
 * @param buffer 位置缓存区指针
 * @param index 历史数据索引 (0表示最新数据)
 * @param data 输出的历史数据指针
 * @return true 获取成功，false 失败
 */
static bool position_buffer_get_history(position_buffer_t* buffer, uint32_t index, position_data_t* data) {
    if (buffer == NULL || data == NULL || index >= buffer->count) return false;

    // 计算历史数据的实际索引
    uint32_t actual_index = (buffer->head - 1 - index + POSITION_BUFFER_SIZE) % POSITION_BUFFER_SIZE;
    *data = buffer->buffer[actual_index];

    return true;
}

/**
 * @brief 分析缓存区中最新的一个波峰波谷
 * @param buffer 位置缓存区指针
 * @param result 波形分析结果输出指针
 * @return true 找到完整的波峰波谷对，false 未找到
 */
static bool position_buffer_analyze_latest_wave(position_buffer_t* buffer, wave_analysis_result_t* result) {
    if (buffer == NULL || result == NULL || buffer->count < 10) {
        return false; // 数据不足，至少需要10个点
    }

    // 初始化结果
    result->has_peak = false;
    result->has_valley = false;
    result->peak_value = 0.0f;
    result->valley_value = 0.0f;
    result->peak_valley_diff = 0.0f;
    result->peak_index = 0;
    result->valley_index = 0;

    // 分析参数
    const uint32_t min_distance = 5; // 波峰波谷之间的最小距离
    const float min_amplitude = 0.1f; // 最小幅度阈值

    // 从最新数据开始向前搜索
    uint32_t search_length = buffer->count > 500 ? 500 : buffer->count; // 限制搜索范围

    float current_peak = -1e6f;
    float current_valley = 1e6f;
    uint32_t peak_idx = 0;
    uint32_t valley_idx = 0;

    // 状态机：寻找波峰和波谷
    typedef enum {
        SEARCH_INIT,    // 初始状态
        FOUND_PEAK,     // 找到波峰
        FOUND_VALLEY    // 找到波谷
    } search_state_t;

    search_state_t state = SEARCH_INIT;

    for (uint32_t i = 0; i < search_length - 2; i++) {
        position_data_t current, prev, next;

        // 获取当前点和相邻点
        if (!position_buffer_get_history(buffer, i, &current) ||
            !position_buffer_get_history(buffer, i + 1, &prev) ||
            !position_buffer_get_history(buffer, i - 1, &next)) {
            continue;
        }

        // 检查是否为局部极值点
        bool is_peak = (current.position > prev.position) && (current.position > next.position);
        bool is_valley = (current.position < prev.position) && (current.position < next.position);

        switch (state) {
            case SEARCH_INIT:
                if (is_peak && current.position > current_peak) {
                    current_peak = current.position;
                    peak_idx = i;
                    state = FOUND_PEAK;
                } else if (is_valley && current.position < current_valley) {
                    current_valley = current.position;
                    valley_idx = i;
                    state = FOUND_VALLEY;
                }
                break;

            case FOUND_PEAK:
                if (is_valley && current.position < current_valley && i > peak_idx + min_distance) {
                    current_valley = current.position;
                    valley_idx = i;
                    // 检查幅度是否足够
                    if ((current_peak - current_valley) > min_amplitude) {
                        result->has_peak = true;
                        result->has_valley = true;
                        result->peak_value = current_peak;
                        result->valley_value = current_valley;
                        result->peak_valley_diff = current_peak - current_valley;
                        result->peak_index = peak_idx;
                        result->valley_index = valley_idx;
                        return true; // 找到了一个完整的波峰波谷对
                    }
                }
                break;

            case FOUND_VALLEY:
                if (is_peak && current.position > current_peak && i > valley_idx + min_distance) {
                    current_peak = current.position;
                    peak_idx = i;
                    // 检查幅度是否足够
                    if ((current_peak - current_valley) > min_amplitude) {
                        result->has_peak = true;
                        result->has_valley = true;
                        result->peak_value = current_peak;
                        result->valley_value = current_valley;
                        result->peak_valley_diff = current_peak - current_valley;
                        result->peak_index = peak_idx;
                        result->valley_index = valley_idx;
                        return true; // 找到了一个完整的波峰波谷对
                    }
                }
                break;
        }
    }

    return false; // 没有找到完整的波峰波谷对
}

/**
 * @brief 向差值缓存区添加数据
 * @param buffer 差值缓存区指针
 * @param diff_value 差值
 * @param timestamp 时间戳
 * @return true 添加成功，false 失败
 */
static bool diff_buffer_add(diff_buffer_t* buffer, float diff_value, uint32_t timestamp) {
    if (buffer == NULL) return false;

    // 在头部位置写入新数据
    buffer->buffer[buffer->head].diff_value = diff_value;
    buffer->buffer[buffer->head].timestamp = timestamp;

    // 更新头指针
    buffer->head = (buffer->head + 1) % DIFF_BUFFER_SIZE;

    // 如果缓存区已满，更新尾指针
    if (buffer->is_full) {
        buffer->tail = (buffer->tail + 1) % DIFF_BUFFER_SIZE;
    } else {
        buffer->count++;
        if (buffer->count == DIFF_BUFFER_SIZE) {
            buffer->is_full = true;
        }
    }

    return true;
}

/**
 * @brief 获取差值缓存区最大值
 * @param buffer 差值缓存区指针
 * @return 缓存区中的最大差值，缓存区为空时返回 0.0f
 */
static float diff_buffer_get_max(diff_buffer_t* buffer) {
    if (buffer == NULL || buffer->count == 0) return 0.0f;

    float max_value = buffer->buffer[0].diff_value;
    for (uint32_t i = 0; i < buffer->count; i++) {
        uint32_t index = (buffer->tail + i) % DIFF_BUFFER_SIZE;
        if (buffer->buffer[index].diff_value > max_value) {
            max_value = buffer->buffer[index].diff_value;
        }
    }

    return max_value;
}

// ============================================================================
// 外部函数(公开接口)
// ============================================================================

/**
 * @brief 初始化电机位置缓存区
 * @param buffers 电机位置缓存区结构体指针
 */
void position_buffer_init(motor_position_buffers_t* buffers) {
    if (buffers == NULL) return;

    // 初始化1号电机缓存区
    memset(&buffers->motor1_buffer, 0, sizeof(position_buffer_t));
    buffers->motor1_buffer.head = 0;
    buffers->motor1_buffer.tail = 0;
    buffers->motor1_buffer.count = 0;
    buffers->motor1_buffer.is_full = false;

    // 初始化2号电机缓存区
    memset(&buffers->motor2_buffer, 0, sizeof(position_buffer_t));
    buffers->motor2_buffer.head = 0;
    buffers->motor2_buffer.tail = 0;
    buffers->motor2_buffer.count = 0;
    buffers->motor2_buffer.is_full = false;

    // 初始化ch6差值缓存区
    memset(&buffers->ch6_buffer, 0, sizeof(diff_buffer_t));
    buffers->ch6_buffer.head = 0;
    buffers->ch6_buffer.tail = 0;
    buffers->ch6_buffer.count = 0;
    buffers->ch6_buffer.is_full = false;

    // 初始化ch7差值缓存区
    memset(&buffers->ch7_buffer, 0, sizeof(diff_buffer_t));
    buffers->ch7_buffer.head = 0;
    buffers->ch7_buffer.tail = 0;
    buffers->ch7_buffer.count = 0;
    buffers->ch7_buffer.is_full = false;
}

/**
 * @brief 向1号电机位置缓存区添加数据
 * @param buffers 电机位置缓存区结构体指针
 * @param position 位置值
 * @param timestamp 时间戳
 * @return true 添加成功，false 失败
 */
bool position_buffer_add_motor1(motor_position_buffers_t* buffers, float position, uint32_t timestamp) {
    if (buffers == NULL) return false;
    return position_buffer_add(&buffers->motor1_buffer, position, timestamp);
}

/**
 * @brief 向2号电机位置缓存区添加数据
 * @param buffers 电机位置缓存区结构体指针
 * @param position 位置值
 * @param timestamp 时间戳
 * @return true 添加成功，false 失败
 */
bool position_buffer_add_motor2(motor_position_buffers_t* buffers, float position, uint32_t timestamp) {
    if (buffers == NULL) return false;
    return position_buffer_add(&buffers->motor2_buffer, position, timestamp);
}

/**
 * @brief 清空位置缓存区
 * @param buffer 位置缓存区指针
 */
void position_buffer_clear(position_buffer_t* buffer) {
    if (buffer == NULL) return;

    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
    buffer->is_full = false;
    memset(buffer->buffer, 0, sizeof(buffer->buffer));
}

/**
 * @brief 获取1号电机位置缓存区指针
 * @param buffers 电机位置缓存区结构体指针
 * @return 1号电机位置缓存区指针，失败返回 NULL
 */
position_buffer_t* position_buffer_get_motor1(motor_position_buffers_t* buffers) {
    if (buffers == NULL) return NULL;
    return &buffers->motor1_buffer;
}

/**
 * @brief 获取2号电机位置缓存区指针
 * @param buffers 电机位置缓存区结构体指针
 * @return 2号电机位置缓存区指针，失败返回 NULL
 */
position_buffer_t* position_buffer_get_motor2(motor_position_buffers_t* buffers) {
    if (buffers == NULL) return NULL;
    return &buffers->motor2_buffer;
}

/**
 * @brief 分析1号电机位置缓存区的波形数据
 * @param buffers 电机位置缓存区结构体指针
 * @param result 波形分析结果输出指针
 * @return true 找到完整的波峰波谷对，false 未找到
 */
bool position_buffer_analyze_motor1_wave(motor_position_buffers_t* buffers, wave_analysis_result_t* result) {
    if (buffers == NULL) return false;
    return position_buffer_analyze_latest_wave(&buffers->motor1_buffer, result);
}

/**
 * @brief 分析2号电机位置缓存区的波形数据
 * @param buffers 电机位置缓存区结构体指针
 * @param result 波形分析结果输出指针
 * @return true 找到完整的波峰波谷对，false 未找到
 */
bool position_buffer_analyze_motor2_wave(motor_position_buffers_t* buffers, wave_analysis_result_t* result) {
    if (buffers == NULL) return false;
    return position_buffer_analyze_latest_wave(&buffers->motor2_buffer, result);
}

/**
 * @brief 向ch6差值缓存区添加数据
 * @param buffers 电机位置缓存区结构体指针
 * @param diff_value 差值
 * @param timestamp 时间戳
 * @return true 添加成功，false 失败
 */
bool diff_buffer_add_ch6(motor_position_buffers_t* buffers, float diff_value, uint32_t timestamp) {
    if (buffers == NULL) return false;
    return diff_buffer_add(&buffers->ch6_buffer, diff_value, timestamp);
}

/**
 * @brief 向ch7差值缓存区添加数据
 * @param buffers 电机位置缓存区结构体指针
 * @param diff_value 差值
 * @param timestamp 时间戳
 * @return true 添加成功，false 失败
 */
bool diff_buffer_add_ch7(motor_position_buffers_t* buffers, float diff_value, uint32_t timestamp) {
    if (buffers == NULL) return false;
    return diff_buffer_add(&buffers->ch7_buffer, diff_value, timestamp);
}

/**
 * @brief 获取ch6差值缓存区的最大值
 * @param buffers 电机位置缓存区结构体指针
 * @return ch6缓存区中的最大差值，缓存区为空时返回 0.0f
 */
float diff_buffer_get_ch6_max(motor_position_buffers_t* buffers) {
    if (buffers == NULL) return 0.0f;
    return diff_buffer_get_max(&buffers->ch6_buffer);
}

/**
 * @brief 获取ch7差值缓存区的最大值
 * @param buffers 电机位置缓存区结构体指针
 * @return ch7缓存区中的最大差值，缓存区为空时返回 0.0f
 */
float diff_buffer_get_ch7_max(motor_position_buffers_t* buffers) {
    if (buffers == NULL) return 0.0f;
    return diff_buffer_get_max(&buffers->ch7_buffer);
}

/**
 * @brief 清空所有差值缓存区(ch6和ch7)
 * @param buffers 电机位置缓存区结构体指针
 */
void diff_buffer_clear_all(motor_position_buffers_t* buffers) {
    if (buffers == NULL) return;

    // 清空ch6缓存区
    buffers->ch6_buffer.head = 0;
    buffers->ch6_buffer.tail = 0;
    buffers->ch6_buffer.count = 0;
    buffers->ch6_buffer.is_full = false;
    memset(buffers->ch6_buffer.buffer, 0, sizeof(buffers->ch6_buffer.buffer));

    // 清空ch7缓存区
    buffers->ch7_buffer.head = 0;
    buffers->ch7_buffer.tail = 0;
    buffers->ch7_buffer.count = 0;
    buffers->ch7_buffer.is_full = false;
    memset(buffers->ch7_buffer.buffer, 0, sizeof(buffers->ch7_buffer.buffer));
}
