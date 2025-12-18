#ifndef VOICE_MODULE_H
#define VOICE_MODULE_H

#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART配置
#define VOICE_UART_NUM       UART_NUM_1
#define VOICE_TX_PIN         1
#define VOICE_RX_PIN         2
#define VOICE_BAUDRATE       9600

typedef struct {
    uart_config_t uart_config;
    bool initialized;
} VoiceModule;

// 外部接口函数
void voice_module_init(VoiceModule* module);
void voice_speak(VoiceModule* module, const char* text);

#ifdef __cplusplus
}
#endif

#endif
