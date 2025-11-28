# main - ESP32固件源码

本目录包含ESP32主控固件的所有源代码，实现双腿机器人的运动控制和AI推理功能。

## 核心模块

### AI推理模块
- **ai_inference.cpp/h** - TensorFlow Lite Micro推理引擎
  - 加载量化模型进行实时运动阶段识别
  - 双腿联合推理（m1和m2）
  - 输出：场景分类、m1/m2阶段、参数调整建议
  - **关键修复**：TFLite输出顺序与H5模型不同，已修正索引映射

### 电机控制模块
- **unitree_motor.cpp/h** - 宇树电机通信协议
  - RS485总线通信
  - 位置、速度、扭矩控制
  - 电机状态读取

- **speed_follow_mode.cpp/h** - 速度跟随控制模式
  - 基于AI推理结果的自适应速度控制
  - 运动阶段切换逻辑

### 数据采集模块
- **button_detector.cpp/h** - 按键检测
  - 多通道PWM按键检测
  - 用于标注数据（静止/抬腿/压腿）

### 通信模块
- **wifi_webserver.c/h** - WiFi配置网页服务器
- **voice_module.c/h** - 语音模块控制

### 工具模块
- **crc_utils.cpp/h** - CRC校验工具
- **position_buffer.c/h** - 位置数据缓冲
- **motor_commands.h** - 电机命令定义
- **webpage.h** - 嵌入式网页资源

### 主程序
- **main.cpp** - 主程序入口
  - 系统初始化
  - 任务调度
  - AI推理循环
  - 数据记录

## 编译与烧录

```bash
# 配置项目
idf.py menuconfig

# 编译
idf.py build

# 烧录
idf.py flash

# 监控串口
idf.py monitor
```

## AI模型部署

模型文件位置：`main/model/motion_ai_model_quant.tflite`

### 重要说明：TFLite输出顺序

TFLite转换后的输出顺序与H5模型不同：

**H5模型**（训练时）:
```
输出0: scene (场景分类)
输出1: m1_phase (m1阶段)
输出2: m2_phase (m2阶段)
输出3: params (参数调整)
```

**TFLite模型**（ESP32部署）:
```
输出0: m1_phase (m1阶段) ← 修正
输出1: params (参数调整)  ← 修正
输出2: scene (场景分类)   ← 修正
输出3: m2_phase (m2阶段)  ← 修正
```

**ai_inference.cpp已修正此问题**（第204-207行）

## 配置参数

### Tensor Arena大小
- 当前配置：60KB
- 位置：`ai_inference.cpp:35`

### 窗口大小
- 速度窗口：50个采样点
- 更新频率：实时推理

### 输出格式
```c
typedef struct {
    int scene;                    // 场景 (0:平地, 1:爬楼)
    float scene_confidence;
    int m1_phase;                 // m1阶段 (0:静止, 1:抬腿, 2:压腿)
    float m1_phase_confidence;
    int m2_phase;                 // m2阶段 (0:静止, 1:抬腿, 2:压腿)
    float m2_phase_confidence;
    float delta_torque;           // 扭矩调整
    float delta_kd;               // 阻尼调整
    float delta_scale;            // 缩放调整
} ai_inference_result_t;
```

## 调试

### 启用推理日志
在 `ai_inference.cpp` 第254-269行取消注释：
```cpp
ESP_LOGI(TAG, "推理结果:");
ESP_LOGI(TAG, "  m1阶段: %s (%.1f%%)", ...);
ESP_LOGI(TAG, "  m2阶段: %s (%.1f%%)", ...);
```

### 串口数据采集
运行主程序时，数据会通过串口输出CSV格式：
```
timestamp,m1_pos,m1_vel,m1_torque,m2_pos,m2_vel,m2_torque,...
```

## 依赖组件
- ESP-IDF 5.x
- TensorFlow Lite Micro (通过组件管理器)
- FreeRTOS

## 性能指标
- 推理延迟：<50ms
- 内存占用：~60KB (Tensor Arena)
- 模型大小：~30-50KB
- CPU占用：单核约30%

## 故障排除

### 问题：m2一直输出0
**原因**：TFLite输出顺序错误
**解决**：已在ai_inference.cpp:204-207修正

### 问题：推理失败
1. 检查模型文件是否正确嵌入
2. 增加Tensor Arena大小
3. 检查输入数据范围

### 问题：CRC校验失败
检查RS485总线连接和波特率配置
