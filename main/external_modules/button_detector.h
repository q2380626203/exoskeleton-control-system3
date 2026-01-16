#ifndef BUTTON_DETECTOR_H
#define BUTTON_DETECTOR_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 防抖参数
// ============================================================================

#define BUTTON_DEBOUNCE_MS        50           // 防抖延迟（毫秒）
#define BUTTON_SCAN_INTERVAL_MS   20           // 扫描间隔（毫秒）
#define DOUBLE_CLICK_INTERVAL_MS  400          // 双击间隔（毫秒）

// ============================================================================
// 类型定义
// ============================================================================

// 按键状态枚举
typedef enum {
    BUTTON_STATE_RELEASED = 0,
    BUTTON_STATE_PRESSED = 1
} ButtonState;

// 开关状态枚举
typedef enum {
    SWITCH_STATE_OFF = 0,
    SWITCH_STATE_ON = 1
} SwitchState;

// 按键检测器结构体
typedef struct {
    gpio_num_t gpio_pin;
    ButtonState current_state;
    ButtonState last_stable_state;
    uint32_t last_change_time;
    bool is_debouncing;
} ButtonDetector;

// 开关检测器结构体
typedef struct {
    gpio_num_t gpio_pin;
    SwitchState current_state;
    SwitchState last_state;
    uint32_t last_change_time;
    bool is_debouncing;
} SwitchDetector;

// 按键回调函数类型
typedef void (*button_press_callback_t)(gpio_num_t pin);
typedef void (*switch_change_callback_t)(gpio_num_t pin, SwitchState new_state);

// ============================================================================
// 外部公开函数声明
// ============================================================================

/**
 * @brief 初始化所有按键和开关
 * @param assist_up_pin 助力增加按键引脚
 * @param power_switch_pin 电源开关引脚
 * @param assist_down_pin 助力减少按键引脚
 */
void button_detector_init(gpio_num_t assist_up_pin, gpio_num_t power_switch_pin, gpio_num_t assist_down_pin);

/**
 * @brief 按键检测任务（FreeRTOS任务）
 * @param param 任务参数（未使用）
 */
void button_detector_task(void* param);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_DETECTOR_H
