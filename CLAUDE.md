# ESP32 外骨骼控制系统 - 项目规则

## 重要约束

### 串口透传模式 (SERIAL_PASSTHROUGH_ENABLE=1)

当启用串口透传模式时，UART0 被用于与4G模块通信，**绝对禁止**在代码中使用任何会向UART0输出的函数：

- **禁止使用**: `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE`, `ESP_LOGD`, `ESP_LOGV`
- **禁止使用**: `printf`, `puts`, `putchar`, `fprintf(stdout, ...)`
- **禁止使用**: `esp_rom_printf`
- **原因**: 这些输出会干扰串口透传数据，导致通信协议失败

### 正确做法

所有ESP_LOG调用必须注释掉或使用条件编译：

```c
// 错误 - 会干扰串口透传
ESP_LOGI(TAG, "注册成功");

// 正确 - 已注释
// ESP_LOGI(TAG, "注册成功");  // 运行时日志已禁用

// 正确 - 条件编译
#if !SERIAL_PASSTHROUGH_ENABLE
ESP_LOGI(TAG, "调试信息");
#endif
```
