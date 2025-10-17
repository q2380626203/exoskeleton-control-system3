#include "voice_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "VOICE_MODULE";

// UTF-8到GB2312的简化映射表
static const char* utf8_gb2312_map[][2] = {
    {"系统", "\xCF\xB5\xCD\xB3"},
    {"启动", "\xC6\xF4\xB6\xAF"},
    {"成功", "\xB3\xC9\xB9\xA6"},
    {"助力", "\xD6\xFA\xC1\xA6"},
    {"增加", "\xD4\xF6\xBC\xD3"},
    {"减少", "\xBC\xF5\xC9\xD9"},
    {"关闭", "\xB9\xD8\xB1\xD5"},
    {"开启", "\xBF\xAA\xC6\xF4"},
    {"停止", "\xCD\xA3\xD6\xB9"},
    {"当前", "\xB5\xB1\xC7\xB0"},
    {"状态", "\xD7\xB4\xCC\xAC"},
    {"正常", "\xD5\xFD\xB3\xA3"},
    {"零", "\xC1\xE3"}, {"一", "\xD2\xBB"}, {"二", "\xB6\xFE"}, {"三", "\xC8\xFD"},
    {"四", "\xCB\xC4"}, {"五", "\xCE\xE5"}, {"六", "\xC1\xF9"}, {"七", "\xC6\xDF"},
    {"八", "\xB0\xCB"}, {"九", "\xBE\xC5"}, {"十", "\xCA\xAE"},
    {"十一", "\xCA\xAE\xD2\xBB"}, {"十二", "\xCA\xAE\xB6\xFE"}, {"十三", "\xCA\xAE\xC8\xFD"},
    {"十四", "\xCA\xAE\xCB\xC4"}, {"十五", "\xCA\xAE\xCE\xE5"}, {"十六", "\xCA\xAE\xC1\xF9"},
    {"十七", "\xCA\xAE\xC6\xDF"}, {"十八", "\xCA\xAE\xB0\xCB"}, {"十九", "\xCA\xAE\xBE\xC5"},
    {"二十", "\xB6\xFE\xCA\xAE"}
};

// ============================================================================
// 静态内部函数（仅在本文件内使用）
// ============================================================================

/**
 * @brief 将 UTF-8 文本转换为 GB2312 编码并通过 UART 发送给语音模块
 * @param text 要发送的 UTF-8 格式文本
 * @note 使用预定义的映射表进行简单字符串替换转换
 */
static void voice_send_gb2312(const char* text) {
    char buffer[512];
    strcpy(buffer, text);

    // 简单的字符串替换
    int map_size = sizeof(utf8_gb2312_map) / sizeof(utf8_gb2312_map[0]);

    for (int i = 0; i < map_size; i++) {
        char* pos = strstr(buffer, utf8_gb2312_map[i][0]);
        if (pos != NULL) {
            // 简单替换 - 这里只处理第一个匹配
            int utf8_len = strlen(utf8_gb2312_map[i][0]);
            int gb2312_len = strlen(utf8_gb2312_map[i][1]);

            // 移动后续字符
            memmove(pos + gb2312_len, pos + utf8_len, strlen(pos + utf8_len) + 1);
            // 复制GB2312字符
            memcpy(pos, utf8_gb2312_map[i][1], gb2312_len);
        }
    }

    uart_write_bytes(VOICE_UART_NUM, buffer, strlen(buffer));
}

// ============================================================================
// 外部接口函数（供其他模块调用）
// ============================================================================

/**
 * @brief 初始化语音模块
 * @param module 语音模块结构体指针
 * @note 配置 UART0 参数(TX=GPIO1, RX=GPIO3, 9600bps)并等待模块稳定
 */
void voice_module_init(VoiceModule* module) {
    // 配置UART参数
    module->uart_config.baud_rate = VOICE_BAUDRATE;
    module->uart_config.data_bits = UART_DATA_8_BITS;
    module->uart_config.parity = UART_PARITY_DISABLE;
    module->uart_config.stop_bits = UART_STOP_BITS_1;
    module->uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    module->uart_config.source_clk = UART_SCLK_DEFAULT;

    // 安装UART驱动
    ESP_ERROR_CHECK(uart_driver_install(VOICE_UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(VOICE_UART_NUM, &module->uart_config));
    ESP_ERROR_CHECK(uart_set_pin(VOICE_UART_NUM, VOICE_TX_PIN, VOICE_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    module->initialized = true;

    // 等待模块稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "语音模块初始化完成");
}

/**
 * @brief 让语音模块播报指定文本
 * @param module 语音模块结构体指针
 * @param text 要播报的 UTF-8 文本内容
 * @note 自动转换为 GB2312 编码并添加结束符
 */
void voice_speak(VoiceModule* module, const char* text) {
    if (!module->initialized) return;

    voice_send_gb2312(text);
    uart_write_bytes(VOICE_UART_NUM, "\r\n\r\n\r\n", 6);
    vTaskDelay(pdMS_TO_TICKS(100));
}
