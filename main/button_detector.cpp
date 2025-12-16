#include "button_detector.h"
#include "voice_module.h"
#include "speed_follow_mode.h"

// ============================================================================
// 静态常量和宏定义
// ============================================================================

static const char *TAG = "BUTTON_DETECTOR";

// 助力调整步长
#define TORQUE_STEP 0.1f
// 电机1 Phase1 力矩范围：0 ~ 1.7
#define MOTOR1_TORQUE_MIN 0.0f
#define MOTOR1_TORQUE_MAX 1.7f
// 电机2 Phase1 力矩范围：-1.7 ~ 0
#define MOTOR2_TORQUE_MIN -1.7f
#define MOTOR2_TORQUE_MAX 0.0f


// ============================================================================
// 静态全局变量
// ============================================================================

// 检测器实例
static ButtonDetector assist_up_button;
static ButtonDetector assist_down_button;
static SwitchDetector power_switch;

// 回调函数指针
static button_press_callback_t on_assist_up_pressed = NULL;
static button_press_callback_t on_assist_down_pressed = NULL;
static switch_change_callback_t on_power_switch_changed = NULL;


// ============================================================================
// 外部引用
// ============================================================================

// 外部语音模块引用（需要在main.cpp中定义并初始化）
extern "C" {
    extern VoiceModule voice_module;
}

// 外部速度跟随模式引用（需要在main.cpp中定义）
extern SpeedFollowMode speed_follow;

// ============================================================================
// 静态辅助函数声明
// ============================================================================

static void init_button_detector(ButtonDetector* detector, gpio_num_t pin);
static void init_switch_detector(SwitchDetector* detector, gpio_num_t pin);
static void configure_gpio_input(gpio_num_t pin);
static ButtonState button_detector_read(ButtonDetector* detector);
static SwitchState switch_detector_read(SwitchDetector* detector);
static void button_detector_update(ButtonDetector* detector);
static void switch_detector_update(SwitchDetector* detector);
static void default_assist_up_callback(gpio_num_t pin);
static void default_assist_down_callback(gpio_num_t pin);
static void default_power_switch_callback(gpio_num_t pin, SwitchState new_state);
static const char* float_to_chinese_number(float value);

// ============================================================================
// 静态辅助函数实现
// ============================================================================

/**
 * @brief 初始化按键检测器
 * @param detector 按键检测器结构体指针
 * @param pin GPIO引脚编号
 */
static void init_button_detector(ButtonDetector* detector, gpio_num_t pin) {
    detector->gpio_pin = pin;
    detector->current_state = BUTTON_STATE_RELEASED;
    detector->last_stable_state = BUTTON_STATE_RELEASED;
    detector->last_change_time = 0;
    detector->is_debouncing = false;
}

/**
 * @brief 初始化开关检测器
 * @param detector 开关检测器结构体指针
 * @param pin GPIO引脚编号
 */
static void init_switch_detector(SwitchDetector* detector, gpio_num_t pin) {
    detector->gpio_pin = pin;
    detector->current_state = SWITCH_STATE_OFF;
    detector->last_state = SWITCH_STATE_OFF;
    detector->last_change_time = 0;
    detector->is_debouncing = false;
}

/**
 * @brief 配置GPIO为输入模式，带内部下拉电阻
 * @param pin GPIO引脚编号
 * @note 未按下时悬空或弱接地（通过内部下拉确保低电平），按下时接3.3V（高电平）
 */
static void configure_gpio_input(gpio_num_t pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,    // 不需要内部上拉
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // 启用内部下拉，确保未按下时为低电平
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

/**
 * @brief 读取按键当前物理状态（带防抖）
 * @param detector 按键检测器结构体指针
 * @return 按键状态：BUTTON_STATE_PRESSED（按下）或 BUTTON_STATE_RELEASED（释放）
 * @note 按下时接3.3V，GPIO读取为高电平(1)
 */
static ButtonState button_detector_read(ButtonDetector* detector) {
    int level = gpio_get_level(detector->gpio_pin);
    // 按下时接3.3V，GPIO读取为高电平(1)
    return level ? BUTTON_STATE_PRESSED : BUTTON_STATE_RELEASED;
}

/**
 * @brief 读取开关当前物理状态（带防抖）
 * @param detector 开关检测器结构体指针
 * @return 开关状态：SWITCH_STATE_ON（接通）或 SWITCH_STATE_OFF（断开）
 * @note 接通时接3.3V，GPIO读取为高电平(1)
 */
static SwitchState switch_detector_read(SwitchDetector* detector) {
    int level = gpio_get_level(detector->gpio_pin);
    // 接通时接3.3V，GPIO读取为高电平(1)
    return level ? SWITCH_STATE_ON : SWITCH_STATE_OFF;
}

/**
 * @brief 更新按键状态（自复位按键检测）
 * @param detector 按键检测器结构体指针
 * @note 检测从释放到按下的边沿触发事件，防抖时间为 BUTTON_DEBOUNCE_MS
 */
static void button_detector_update(ButtonDetector* detector) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ButtonState current_reading = button_detector_read(detector);

    // 检测状态变化
    if (current_reading != detector->current_state) {
        detector->current_state = current_reading;
        detector->last_change_time = current_time;
        detector->is_debouncing = true;
    }

    // 防抖处理
    if (detector->is_debouncing) {
        if ((current_time - detector->last_change_time) >= BUTTON_DEBOUNCE_MS) {
            detector->is_debouncing = false;

            // 检测按下事件（从释放到按下的边沿）
            if (detector->last_stable_state == BUTTON_STATE_RELEASED &&
                detector->current_state == BUTTON_STATE_PRESSED) {

                // ESP_LOGI(TAG, "按键按下 GPIO%d", detector->gpio_pin);

                // 触发对应的回调
                if (detector->gpio_pin == BUTTON_ASSIST_UP_PIN && on_assist_up_pressed) {
                    on_assist_up_pressed(detector->gpio_pin);
                } else if (detector->gpio_pin == BUTTON_ASSIST_DOWN_PIN && on_assist_down_pressed) {
                    on_assist_down_pressed(detector->gpio_pin);
                }
            }

            detector->last_stable_state = detector->current_state;
        }
    }
}

/**
 * @brief 更新开关状态（持续导通开关检测）
 * @param detector 开关检测器结构体指针
 * @note 检测开关状态的切换变化，防抖时间为 BUTTON_DEBOUNCE_MS
 */
static void switch_detector_update(SwitchDetector* detector) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    SwitchState current_reading = switch_detector_read(detector);

    // 检测状态变化
    if (current_reading != detector->current_state) {
        detector->current_state = current_reading;
        detector->last_change_time = current_time;
        detector->is_debouncing = true;
    }

    // 防抖处理
    if (detector->is_debouncing) {
        if ((current_time - detector->last_change_time) >= BUTTON_DEBOUNCE_MS) {
            detector->is_debouncing = false;

            // 检测开关状态切换
            if (detector->current_state != detector->last_state) {
                // ESP_LOGI(TAG, "开关切换 GPIO%d: %s",
                //          detector->gpio_pin,
                //          detector->current_state == SWITCH_STATE_ON ? "ON" : "OFF");

                // 触发回调
                if (on_power_switch_changed) {
                    on_power_switch_changed(detector->gpio_pin, detector->current_state);
                }

                detector->last_state = detector->current_state;
            }
        }
    }
}

/**
 * @brief 将浮点数转换为中文数字字符串
 * @param value 浮点数值（范围 0.0 ~ 2.0）
 * @return 中文数字字符串
 * @note 将浮点数乘以10后转换为整数，然后转换为对应的中文数字
 *       例如：0.6 -> "六", 0.7 -> "七", 1.5 -> "十五", 2.0 -> "二十"
 */
static const char* float_to_chinese_number(float value) {
    // 将浮点数乘以10并四舍五入转换为整数
    int num = (int)(value * 10 + 0.5f);

    // 限制范围在 0-20
    if (num < 0) num = 0;
    if (num > 20) num = 20;

    // 中文数字映射表
    static const char* chinese_numbers[] = {
        "零",   // 0
        "一",   // 1
        "二",   // 2
        "三",   // 3
        "四",   // 4
        "五",   // 5
        "六",   // 6
        "七",   // 7
        "八",   // 8
        "九",   // 9
        "十",   // 10
        "十一", // 11
        "十二", // 12
        "十三", // 13
        "十四", // 14
        "十五", // 15
        "十六", // 16
        "十七", // 17
        "十八", // 18
        "十九", // 19
        "二十"  // 20
    };

    return chinese_numbers[num];
}

/**
 * @brief 助力增加按键回调的默认实现
 * @param pin 触发的GPIO引脚编号
 * @note 增加电机1和电机2的Phase1力矩**绝对值**
 *       - 电机1: 范围 0 ~ 1.7，每次增加 TORQUE_STEP
 *       - 电机2: 范围 -1.7 ~ 0，每次增加力矩绝对值（更大的负值）
 *       - 力矩 > 1.0 时：触发速度设置为 ±7.0
 *       - 力矩 <= 1.0 时：触发速度恢复为 ±3.0（默认值）
 */
static void default_assist_up_callback(gpio_num_t pin) {
    // 获取电机1和电机2的配置
    speed_follow_config_t* motor1_config = speed_follow.getMotorConfig(1);
    speed_follow_config_t* motor2_config = speed_follow.getMotorConfig(2);

    // 增加电机1的Phase1力矩（范围：0 ~ 1.7）
    float new_torque1 = motor1_config->phase1.torque + TORQUE_STEP;
    if (new_torque1 > MOTOR1_TORQUE_MAX) {
        new_torque1 = MOTOR1_TORQUE_MAX;
    }
    motor1_config->phase1.torque = new_torque1;

    // 增加电机2的Phase1力矩**绝对值**（范围：-1.7 ~ 0，负号是方向，增加助力意味着更大的负值）
    float new_torque2 = motor2_config->phase1.torque - TORQUE_STEP;  // 减去0.1使其更负
    if (new_torque2 < MOTOR2_TORQUE_MIN) {
        new_torque2 = MOTOR2_TORQUE_MIN;
    }
    motor2_config->phase1.torque = new_torque2;


    // ESP_LOGI(TAG, "助力增加 - 电机1: %.2f, 电机2: %.2f, 触发速度: %.1f",
    //          new_torque1, new_torque2, motor1_config->trigger_speed);

    // 播放电机1的力矩绝对值（中文数字）
    const char* torque_text = float_to_chinese_number(new_torque1);
    voice_speak(&voice_module, torque_text);
}

/**
 * @brief 助力减少按键回调的默认实现
 * @param pin 触发的GPIO引脚编号
 * @note 减少电机1和电机2的Phase1力矩**绝对值**
 *       - 电机1: 范围 0 ~ 1.7，每次减少 TORQUE_STEP
 *       - 电机2: 范围 -1.7 ~ 0，每次减少力矩绝对值（更小的负值）
 *       - 力矩 > 1.0 时：触发速度设置为 ±7.0
 *       - 力矩 <= 1.0 时：触发速度恢复为 ±3.0（默认值）
 */
static void default_assist_down_callback(gpio_num_t pin) {
    // 获取电机1和电机2的配置
    speed_follow_config_t* motor1_config = speed_follow.getMotorConfig(1);
    speed_follow_config_t* motor2_config = speed_follow.getMotorConfig(2);

    // 减少电机1的Phase1力矩（范围：0 ~ 1.7）
    float new_torque1 = motor1_config->phase1.torque - TORQUE_STEP;
    if (new_torque1 < MOTOR1_TORQUE_MIN) {
        new_torque1 = MOTOR1_TORQUE_MIN;
    }
    motor1_config->phase1.torque = new_torque1;

    // 减少电机2的Phase1力矩**绝对值**（范围：-1.7 ~ 0，负号是方向，减少助力意味着更小的负值）
    float new_torque2 = motor2_config->phase1.torque + TORQUE_STEP;  // 加上0.1使其更接近0
    if (new_torque2 > MOTOR2_TORQUE_MAX) {
        new_torque2 = MOTOR2_TORQUE_MAX;
    }
    motor2_config->phase1.torque = new_torque2;


    // ESP_LOGI(TAG, "助力减少 - 电机1: %.2f, 电机2: %.2f, 触发速度: %.1f",
    //          new_torque1, new_torque2, motor1_config->trigger_speed);

    // 播放电机1的力矩绝对值（中文数字）
    const char* torque_text = float_to_chinese_number(new_torque1);
    voice_speak(&voice_module, torque_text);
}

/**
 * @brief 电源开关回调的默认实现
 * @param pin 触发的GPIO引脚编号
 * @param new_state 新的开关状态：SWITCH_STATE_ON（开启）或 SWITCH_STATE_OFF（关闭）
 * @note 控制速度跟随模式的启用/禁用
 *       - ON: 启动按键触发模式，播放"助力启动"语音
 *       - OFF: 播放"助力关闭"语音
 */
static void default_power_switch_callback(gpio_num_t pin, SwitchState new_state) {
    if (new_state == SWITCH_STATE_ON) {
        // ESP_LOGI(TAG, "助力启动");
        voice_speak(&voice_module, "助力启动");
        // 启动按键触发模式
        speed_follow.startButtonWaiting();
    } else {
        // ESP_LOGI(TAG, "助力关闭");
        voice_speak(&voice_module, "助力关闭");
    }
}

// ============================================================================
// 外部公开函数实现
// ============================================================================

/**
 * @brief 初始化所有按键和开关
 * @note 配置GPIO引脚为输入模式，初始化三个检测器（助力增加、助力减少、电源开关）
 *       GPIO3: 助力增加按键
 *       GPIO4: 电源开关
 *       GPIO5: 助力减少按键
 */
void button_detector_init(void) {
    ESP_LOGI(TAG, "初始化按键检测器...");

    // 配置GPIO引脚
    configure_gpio_input(BUTTON_ASSIST_UP_PIN);
    configure_gpio_input(BUTTON_POWER_SWITCH_PIN);
    configure_gpio_input(BUTTON_ASSIST_DOWN_PIN);

    // 初始化检测器
    init_button_detector(&assist_up_button, BUTTON_ASSIST_UP_PIN);
    init_button_detector(&assist_down_button, BUTTON_ASSIST_DOWN_PIN);
    init_switch_detector(&power_switch, BUTTON_POWER_SWITCH_PIN);

    // 读取开关初始状态
    power_switch.current_state = gpio_get_level(BUTTON_POWER_SWITCH_PIN) ? SWITCH_STATE_ON : SWITCH_STATE_OFF;
    power_switch.last_state = power_switch.current_state;

    ESP_LOGI(TAG, "按键检测器初始化完成");
    ESP_LOGI(TAG, "GPIO3: 助力增加, GPIO4: 电源开关, GPIO5: 助力减少");
}

/**
 * @brief 按键检测任务（FreeRTOS任务）
 * @param param 任务参数（未使用）
 * @note 该任务循环执行以下操作：
 *       1. 更新所有按键和开关状态
 *       2. 检测缓存区触发标志并播放语音
 *       3. 检测静止状态并播放语音
 *       4. 等待 BUTTON_SCAN_INTERVAL_MS 后进行下次扫描
 */
void button_detector_task(void* param) {
    // 注册默认回调函数
    on_assist_up_pressed = default_assist_up_callback;
    on_assist_down_pressed = default_assist_down_callback;
    on_power_switch_changed = default_power_switch_callback;

    
    while (1) {
        // 更新所有按键和开关状态
        button_detector_update(&assist_up_button);
        button_detector_update(&assist_down_button);
        switch_detector_update(&power_switch);

        // 检测缓存区触发开启标志并播放语音
        if (speed_follow.isBufferTriggered()) {
            // ESP_LOGI(TAG, "检测到缓存区触发开启，播放语音提示");
            voice_speak(&voice_module, "助力开启");
            speed_follow.clearBufferTriggeredFlag(); // 清除标志，避免重复播放
        }

        // 检测静止状态并播放语音
        if (speed_follow.isStationary()) {
            // ESP_LOGI(TAG, "检测到静止状态，播放语音提示");
            voice_speak(&voice_module, "助力停止");
            speed_follow.clearStationaryFlag(); // 清除标志，避免重复播放
        }

        // 等待一段时间再进行下次扫描
        vTaskDelay(pdMS_TO_TICKS(BUTTON_SCAN_INTERVAL_MS));
    }
}
