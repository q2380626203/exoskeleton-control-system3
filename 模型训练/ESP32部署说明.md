# ESP32 模型部署方案

## 问题：C头文件太大

- 量化模型：145.7 KB
- C头文件：920.4 KB（膨胀6倍）
- ESP32 Flash 空间有限，需要优化

---

## 推荐方案：使用 ESP-IDF 二进制嵌入

### 1. 文件结构

```
your_esp32_project/
├── main/
│   ├── CMakeLists.txt
│   ├── main.cpp
│   └── model/
│       └── motion_ai_model_quant.tflite  # 直接放二进制文件
└── CMakeLists.txt
```

### 2. 修改 main/CMakeLists.txt

```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    EMBED_FILES "model/motion_ai_model_quant.tflite"
)
```

### 3. C++ 代码中使用

```cpp
#include <stdio.h>
#include "esp_log.h"

// 声明嵌入的二进制数据
extern const uint8_t model_data_start[] asm("_binary_motion_ai_model_quant_tflite_start");
extern const uint8_t model_data_end[]   asm("_binary_motion_ai_model_quant_tflite_end");

void setup_tflite_model() {
    // 计算模型大小
    const uint32_t model_size = model_data_end - model_data_start;

    ESP_LOGI("AI", "Model size: %lu bytes", model_size);

    // 使用 model_data_start 指针初始化 TFLite 解释器
    // ... TFLite Micro 初始化代码 ...
}
```

### 4. 编译

```bash
cd your_esp32_project
idf.py build
idf.py flash
```

---

## 优势

✅ **节省 Flash 空间**：只占用 145.7 KB（vs 920.4 KB）
✅ **编译更快**：无需编译巨大的 C 数组
✅ **内存效率高**：直接从 Flash 读取，无需拷贝
✅ **官方推荐**：ESP-IDF 标准做法

---

## 替代方案：压缩 C 头文件（不推荐）

如果必须使用 C 头文件格式，可以优化格式：

### 优化后的格式

```c
// 每行 12 字节，减少换行符
const unsigned char model_data[] __attribute__((aligned(8))) = {
    0x1c,0x00,0x00,0x00,0x54,0x46,0x4c,0x33,0x00,0x00,0x00,0x00,
    0x00,0x00,0x12,0x00,0x1c,0x00,0x04,0x00,0x08,0x00,0x0c,0x00,
    // ... 更紧凑 ...
};
```

这样可以将文件大小从 920 KB 减少到约 600 KB，但仍然比二进制大。

---

## 部署步骤总结

### 步骤1：复制模型文件

```bash
# 将量化模型复制到 ESP32 项目
cp motion_ai_model_quant.tflite /path/to/esp32_project/main/model/
```

### 步骤2：配置 CMakeLists.txt

在 `main/CMakeLists.txt` 中添加：

```cmake
EMBED_FILES "model/motion_ai_model_quant.tflite"
```

### 步骤3：编写推理代码

```cpp
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

// 声明嵌入的模型
extern const uint8_t model_data_start[] asm("_binary_motion_ai_model_quant_tflite_start");
extern const uint8_t model_data_end[]   asm("_binary_motion_ai_model_quant_tflite_end");

// Tensor Arena 大小（根据模型调整）
constexpr int kTensorArenaSize = 50 * 1024; // 50KB
uint8_t tensor_arena[kTensorArenaSize];

void setup_model() {
    // 加载模型
    const tflite::Model* model = tflite::GetModel(model_data_start);

    // 检查模型版本
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE("AI", "Model version mismatch!");
        return;
    }

    // 配置 Op Resolver（添加模型需要的操作）
    static tflite::MicroMutableOpResolver<10> resolver;
    resolver.AddConv2D();
    resolver.AddMaxPool2D();
    resolver.AddReshape();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    resolver.AddDequantize();

    // 创建解释器
    static tflite::MicroInterpreter interpreter(
        model, resolver, tensor_arena, kTensorArenaSize
    );

    // 分配张量
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        ESP_LOGE("AI", "AllocateTensors failed!");
        return;
    }

    ESP_LOGI("AI", "Model loaded successfully");
    ESP_LOGI("AI", "Arena used: %d bytes", interpreter.arena_used_bytes());
}

void run_inference(float velocity_window[50]) {
    // 获取输入张量
    TfLiteTensor* input = interpreter.input(0);

    // 填充输入数据 (shape: [1, 50, 1])
    for (int i = 0; i < 50; i++) {
        input->data.f[i] = velocity_window[i];
    }

    // 执行推理
    if (interpreter.Invoke() != kTfLiteOk) {
        ESP_LOGE("AI", "Invoke failed!");
        return;
    }

    // 获取输出
    TfLiteTensor* output_scene = interpreter.output(0);  // 场景分类 [1, 2]
    TfLiteTensor* output_phase = interpreter.output(1);  // 阶段分类 [1, 3]
    TfLiteTensor* output_params = interpreter.output(2); // 参数调整 [1, 3]

    // 解析结果
    int scene = (output_scene->data.f[0] > output_scene->data.f[1]) ? 0 : 1;

    int phase = 0;
    float max_prob = output_phase->data.f[0];
    for (int i = 1; i < 3; i++) {
        if (output_phase->data.f[i] > max_prob) {
            max_prob = output_phase->data.f[i];
            phase = i;
        }
    }

    float delta_torque = output_params->data.f[0];
    float delta_kd = output_params->data.f[1];
    float delta_scale = output_params->data.f[2];

    ESP_LOGI("AI", "Scene: %d, Phase: %d", scene, phase);
    ESP_LOGI("AI", "Params: torque=%.3f, kd=%.3f, scale=%.3f",
             delta_torque, delta_kd, delta_scale);
}
```

### 步骤4：集成到主循环

```cpp
void app_main() {
    // 初始化模型
    setup_model();

    // 主循环
    while (1) {
        // 收集50个速度数据点
        float velocity_window[50];
        collect_velocity_data(velocity_window);

        // 运行推理
        run_inference(velocity_window);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

---

## 性能参考

根据转换结果：
- **推理时间**：0.03 ms（PC 端测试）
- **推理频率**：34,880 Hz
- **ESP32 预估**：5-10 ms（取决于 CPU 频率）

---

## 常见问题

### Q: Tensor Arena 大小如何确定？

A: 从转换输出看，模型需要的内存约 50KB。可以从小到大尝试：
```cpp
constexpr int kTensorArenaSize = 30 * 1024; // 30KB
// 如果报错 "AllocateTensors failed"，逐步增大
```

### Q: 如何知道需要添加哪些 Op？

A: 查看模型转换日志，或使用 TFLite 可视化工具（Netron）查看模型结构。

### Q: 量化模型在 ESP32 上精度会下降吗？

A: 动态范围量化通常精度损失 < 1%，完全可接受。

---

## 参考资料

- [ESP-IDF 嵌入二进制文件](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#embedding-binary-data)
- [TFLite Micro for ESP32](https://github.com/espressif/esp-tflite-micro)
- [本项目模型训练文档](./README.md)
