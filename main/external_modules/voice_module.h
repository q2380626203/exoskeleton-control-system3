#ifndef VOICE_MODULE_H
#define VOICE_MODULE_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t uart_num;
    uart_config_t uart_config;
    bool initialized;
} VoiceModule;

/**
 * @brief 初始化语音模块
 * @param module 语音模块结构体指针
 * @param uart_num UART端口号
 * @param tx_pin 发送引脚
 * @param rx_pin 接收引脚
 * @param baud_rate 波特率
 */
void voice_module_init(VoiceModule* module, uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate);

/**
 * @brief 让语音模块播报指定文本
 * @param module 语音模块结构体指针
 * @param text 要播报的 UTF-8 文本内容
 */
void voice_speak(VoiceModule* module, const char* text);

#ifdef __cplusplus
}
#endif

#endif
