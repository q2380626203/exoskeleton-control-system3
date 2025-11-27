# AI模型训练 - 运动阶段识别与参数调整

本目录包含AI模型训练和部署的完整工具链，实现从训练数据到ESP32可部署模型的转换。

## 目录结构

```
模型训练/
├── 1_模型训练.py          # 多任务学习模型训练
├── 2_模型转换.py          # TFLite转换与量化
├── README.md              # 本文档
└── requirements.txt       # Python依赖
```

## 工作流程

```
数据收集与标注/5_训练数据生成.py
    ↓ 生成 训练数据_平地_m1.npz
模型训练/1_模型训练.py
    ↓ 生成 motion_ai_model.h5
模型训练/2_模型转换.py
    ↓ 生成 motion_ai_model_quant.tflite 和 model_data.h
ESP32部署
```

---

## 1. 模型训练 (1_模型训练.py)

### 功能

训练多任务学习模型，同时完成三个任务：
1. **场景分类**：平地/爬楼 (2类)
2. **阶段分类**：静止/抬腿/压腿 (3类)
3. **参数预测**：delta_torque, delta_kd, delta_scale (3个连续值)

### 模型架构

```
输入层 (窗口大小, 1)
    ↓
共享特征提取层:
├── Conv1D(32, kernel=5) + BatchNorm + MaxPool
├── Conv1D(64, kernel=3) + BatchNorm + MaxPool
├── Flatten
└── Dense(128) + Dropout(0.3)
    ↓
三个任务分支:
├── 场景分类: Dense(64) → Softmax(2)
├── 阶段分类: Dense(64) → Softmax(3)
└── 参数预测: Dense(64) → Tanh(3)
```

### 使用方法

```bash
# 基本用法
python 1_模型训练.py --input ../数据收集与标注/训练数据_平地_m1.npz --scene 平地

# 完整参数
python 1_模型训练.py \
    --input ../数据收集与标注/训练数据_平地_m1.npz \
    --scene 平地 \
    --epochs 50 \
    --batch-size 32 \
    --val-split 0.2 \
    --output motion_ai_model.h5
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--input` | 训练数据NPZ文件路径 | 必需 |
| `--scene` | 场景名称 (平地/爬楼) | 必需 |
| `--epochs` | 训练轮数 | 50 |
| `--batch-size` | 批大小 | 32 |
| `--val-split` | 验证集比例 | 0.2 |
| `--output` | 输出模型文件名 | motion_ai_model.h5 |

### 输出文件

- `motion_ai_model.h5` - 完整Keras模型
- `motion_ai_model.weights.h5` - 模型权重
- `training_history.png` - 训练曲线可视化
- `model_evaluation.txt` - 模型评估报告
- `best_model_checkpoint.h5` - 最佳模型检查点

### 训练策略

- **优化器**: Adam (learning_rate=0.001)
- **损失函数**:
  - 场景分类: sparse_categorical_crossentropy (权重=1.0)
  - 阶段分类: sparse_categorical_crossentropy (权重=2.0)
  - 参数预测: MSE (权重=1.0)
- **回调函数**:
  - EarlyStopping: 监控val_loss，patience=10
  - ReduceLROnPlateau: 自动降低学习率，patience=5
  - ModelCheckpoint: 保存最佳模型

### 评估指标

- 场景分类准确率
- 阶段分类准确率（总体 + 各阶段）
- 参数预测MAE（delta_torque, delta_kd, delta_scale）

---

## 2. 模型转换 (2_模型转换.py)

### 功能

将训练好的Keras模型转换为TensorFlow Lite格式，并进行量化优化：
1. **浮点TFLite转换** - 标准优化
2. **INT8量化** - 大幅减小模型大小（4-10倍）
3. **C头文件生成** - 用于ESP32部署
4. **推理性能测试** - 评估推理速度

### 使用方法

```bash
# 基本用法（仅浮点转换）
python 2_模型转换.py --input motion_ai_model.h5

# 完整用法（包含INT8量化）
python 2_模型转换.py \
    --input motion_ai_model.h5 \
    --data ../数据收集与标注/训练数据_平地_m1.npz \
    --quant int8 \
    --output-dir .
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--input` | Keras模型文件 (.h5) | 必需 |
| `--data` | 训练数据NPZ（用于量化校准） | None |
| `--quant` | 量化类型 (none/float16/int8) | int8 |
| `--output-dir` | 输出目录 | 当前目录 |

### 输出文件

- `motion_ai_model.tflite` - 浮点TFLite模型
- `motion_ai_model_quant.tflite` - INT8量化模型
- `model_data.h` - C语言头文件（用于ESP32）

### 量化说明

**INT8量化优势**:
- 模型大小减小 4-10倍
- 推理速度提升 2-4倍
- 适合资源受限的嵌入式设备

**代表性数据集**:
- 使用训练数据的随机样本（默认100个）
- 用于量化校准，确保精度损失最小

### C头文件格式

```c
#ifndef MODEL_DATA_H
#define MODEL_DATA_H

#include <stdint.h>

// TFLite模型数据 (xxx bytes)
const unsigned char model_data[] __attribute__((aligned(8))) = {
    0x1c, 0x00, 0x00, 0x00, 0x54, 0x46, 0x4c, 0x33, ...
};

const unsigned int model_data_len = xxx;

#endif // MODEL_DATA_H
```

### 性能测试

脚本会自动测试推理性能（100次推理取平均）：
- 平均推理时间
- 最小/最大推理时间
- 推理频率 (Hz)
- 浮点模型 vs 量化模型的性能对比

---

## 训练数据格式

训练数据由 `数据收集与标注/5_训练数据生成.py` 生成，NPZ文件包含：

```python
{
    'X': (N, 窗口大小, 1),      # 速度窗口特征
    'y_phase': (N,),            # 阶段标签 (0=静止, 1=抬腿, 2=压腿)
    'y_params': (N, 3),         # 参数调整标签 [delta_torque, delta_kd, delta_scale]
    'motor': 'm1' or 'm2',      # 电机标识
    'scene': '平地' or '爬楼',   # 场景名称
    'window_size': 50,          # 窗口大小
    'step': 5                   # 滑动步长
}
```

---

## ESP32部署

### 1. 复制文件

将生成的 `model_data.h` 复制到ESP32项目的 `components/` 目录。

### 2. 集成TensorFlow Lite Micro

在ESP32项目中添加TFLite Micro库：

```cmake
# CMakeLists.txt
idf_component_register(
    SRCS "main.cpp" "ai_motion_predictor.cpp"
    INCLUDE_DIRS "."
    REQUIRES tensorflow-lite
)
```

### 3. 推理代码示例

```cpp
#include "model_data.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"

// 初始化
tflite::MicroInterpreter* interpreter;
tflite::MicroMutableOpResolver<10> resolver;
// ... 配置resolver和interpreter ...

// 推理
float velocity_window[50] = { /* 速度数据 */ };
for (int i = 0; i < 50; i++) {
    interpreter->input(0)->data.f[i] = velocity_window[i];
}
interpreter->Invoke();

// 获取输出
float* scene_output = interpreter->output(0)->data.f;
float* phase_output = interpreter->output(1)->data.f;
float* params_output = interpreter->output(2)->data.f;
```

---

## 常见问题

### Q: 训练时显存不足怎么办？

A: 减小batch_size，例如从32降到16或8。

### Q: 模型过拟合怎么办？

A:
1. 增加Dropout比例（当前0.3）
2. 减少模型参数（减少Conv1D通道数）
3. 使用数据增强

### Q: INT8量化后精度下降严重？

A:
1. 增加代表性数据集样本数（当前100个）
2. 使用动态范围量化替代INT8
3. 检查训练数据分布是否均衡

### Q: ESP32推理速度慢？

A:
1. 确保使用INT8量化模型
2. 减小窗口大小（当前50）
3. 优化模型结构（减少层数/参数）

---

## 参考资料

- [TensorFlow Lite文档](https://www.tensorflow.org/lite)
- [TFLite Micro for ESP32](https://github.com/espressif/esp-tflite-micro)
- [方案三_AI阶段识别与参数调整.md](../方案三_AI阶段识别与参数调整.md)
