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
// GPIO引脚定义
// ============================================================================

#define BUTTON_ASSIST_UP_PIN      GPIO_NUM_5   // 助力增加按键（自复位）
#define BUTTON_POWER_SWITCH_PIN   GPIO_NUM_4   // 助力开关（持续导通）
#define BUTTON_ASSIST_DOWN_PIN    GPIO_NUM_3   // 助力减少按键（自复位）

// ============================================================================
// 防抖参数
// ============================================================================

#define BUTTON_DEBOUNCE_MS        50           // 防抖延迟（毫秒）
#define BUTTON_SCAN_INTERVAL_MS   20           // 扫描间隔（毫秒）

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
 */
void button_detector_init(void);

/**
 * @brief 按键检测任务（FreeRTOS任务）
 * @param param 任务参数（未使用）
 */
void button_detector_task(void* param);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_DETECTOR_H
