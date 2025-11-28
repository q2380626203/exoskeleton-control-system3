# 模型训练

本目录包含AI模型训练、转换和调试的核心工具。

## 文件列表

```
模型训练/
├── 1_模型训练.py              # 训练双腿联合模型
├── 2_模型转换.py              # 转换为TFLite量化模型
├── debug_model_outputs.py    # 调试输出顺序
├── requirements.txt          # Python依赖包
└── README.md                # 本文档
```

## 工作流程

```
训练数据_*.npz → 1_模型训练.py → 2_模型转换.py → 部署到ESP32
                  (H5模型)        (TFLite模型)      (嵌入固件)
                      ↓
                debug_model_outputs.py
                 (验证输出顺序)
```

---

## 1. 模型训练 (1_模型训练.py)

### 功能
训练双腿联合多任务学习模型，输出H5格式模型文件。

### 使用方法
```bash
python 1_模型训练.py \
  --input ../数据收集与标注/训练数据_平地_双腿.npz \
  --scene 平地 \
  --epochs 50 \
  --batch-size 32
```

### 参数说明
| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--input, -i` | 训练数据NPZ文件 | 必需 |
| `--scene, -s` | 场景名称（平地/爬楼） | 平地 |
| `--epochs, -e` | 训练轮数 | 50 |
| `--batch-size, -b` | 批大小 | 32 |
| `--val-split` | 验证集比例 | 0.2 |
| `--output, -o` | 输出模型文件名 | motion_ai_model.h5 |

### 模型架构

**输入**：`(batch, 50, 2)` - 50个时间步的双腿速度 `[m1_vel, m2_vel]`

**特征提取**（共享层）：
- Conv1D(32, kernel=5) + BatchNorm + MaxPool
- Conv1D(64, kernel=3) + BatchNorm + MaxPool
- Flatten + Dense(128) + Dropout(0.3)

**四个输出分支**：
1. **scene** - 场景分类 (2类): 平地/爬楼
2. **m1_phase** - m1阶段分类 (3类): 静止/抬腿/压腿
3. **m2_phase** - m2阶段分类 (3类): 静止/抬腿/压腿
4. **params** - 参数调整 (3个连续值): delta_torque/kd/scale

### 输出文件
- `motion_ai_model.h5` - 完整模型（H5格式）
- `motion_ai_model.weights.h5` - 模型权重
- `training_history.png` - 训练曲线（9张子图）
- `model_evaluation.txt` - 性能评估报告

### 训练效果
- m1阶段分类准确率：>94%
- m2阶段分类准确率：>96%
- 参数预测MAE：<0.1
- 总参数量：约100K
- 模型大小：约400KB (FP32)

---

## 2. 模型转换 (2_模型转换.py)

### 功能
将H5模型转换为TFLite量化模型（INT8），用于ESP32部署。

### 使用方法
```bash
python 2_模型转换.py \
  --input motion_ai_model.h5 \
  --data ../数据收集与标注/训练数据_平地_双腿.npz \
  --output motion_ai_model_quant.tflite
```

### 参数说明
| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--input, -i` | H5模型文件 | 必需 |
| `--data, -d` | 训练数据NPZ（用于量化校准） | 必需 |
| `--output, -o` | 输出TFLite文件名 | motion_ai_model_quant.tflite |
| `--quantize` | 启用INT8量化 | True |

### 量化配置
```python
# INT8量化，输入输出保持FP32
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen  # 使用真实数据校准
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.float32   # 输入FP32
converter.inference_output_type = tf.float32  # 输出FP32
```

### 量化效果
- 模型大小：400KB → 40KB（压缩10倍）
- 推理速度：提升2-3倍
- 精度损失：<2%

### 输出文件
- `motion_ai_model_quant.tflite` - 量化后的TFLite模型（约40KB）

### ⚠️ 重要警告：输出顺序变化

**TFLite转换后输出顺序与H5模型不同！**

| 输出索引 | H5模型（训练时） | TFLite模型（部署时） |
|---------|-----------------|-------------------|
| 输出0 | scene (场景) | **m1_phase** ⚠️ |
| 输出1 | m1_phase (m1阶段) | **params** ⚠️ |
| 输出2 | m2_phase (m2阶段) | **scene** ⚠️ |
| 输出3 | params (参数) | **m2_phase** ⚠️ |

**此问题已在 `main/ai_inference.cpp:204-207` 修正！**

---

## 3. 调试工具 (debug_model_outputs.py)

### 功能
验证H5和TFLite模型的输出顺序、形状和推理结果。

### 使用方法
```bash
python debug_model_outputs.py
```

### 前置条件
- `motion_ai_model.h5` - H5模型
- `motion_ai_model_quant.tflite` - TFLite模型
- `平地数据_已推理.csv` - ESP32推理数据（用于真实数据测试）

### 测试内容
1. **H5模型信息**
   - 输出层名称和形状
   - 随机数据推理测试

2. **TFLite模型信息**
   - 输入/输出张量详情（名称、形状、数据类型、量化参数）
   - 输出张量索引顺序

3. **真实数据对比**
   - 使用ESP32实际运行数据
   - 对比H5和TFLite推理结果
   - 验证输出顺序映射

### 输出示例
```
TFLite输出张量顺序:
  输出0: StatefulPartitionedCall_1:1, 形状=[1 3]  ← m1_phase
  输出1: StatefulPartitionedCall_1:3, 形状=[1 3]  ← params
  输出2: StatefulPartitionedCall_1:0, 形状=[1 2]  ← scene
  输出3: StatefulPartitionedCall_1:2, 形状=[1 3]  ← m2_phase

⚠ 关键检查点:
  1. TFLite的输出0应该对应m1_phase
  2. TFLite的输出3应该对应m2_phase
  3. 如果输出名称或顺序不同,ESP32代码需要相应调整!
```

### 用途
- 诊断ESP32部署问题（特别是m2输出异常）
- 验证TFLite转换正确性
- 确认量化精度影响

---

## 完整训练流程

### Step 1: 安装依赖
```bash
pip install -r requirements.txt
```

### Step 2: 训练模型
```bash
python 1_模型训练.py \
  --input ../数据收集与标注/训练数据_平地_双腿.npz \
  --scene 平地 \
  --epochs 50
```

**检查点**：
- ✅ 查看 `training_history.png` 验证收敛
- ✅ 查看 `model_evaluation.txt` 确认准确率>94%
- ✅ 无过拟合（训练和验证曲线接近）

### Step 3: 转换模型
```bash
python 2_模型转换.py \
  --input motion_ai_model.h5 \
  --data ../数据收集与标注/训练数据_平地_双腿.npz
```

**检查点**：
- ✅ 模型大小<100KB
- ✅ 无转换错误或警告

### Step 4: 验证输出顺序
```bash
python debug_model_outputs.py
```

**检查点**：
- ✅ TFLite输出0是m1_phase（形状[1,3]）
- ✅ TFLite输出3是m2_phase（形状[1,3]）
- ✅ TFLite输出2是scene（形状[1,2]）

### Step 5: 部署到ESP32
```bash
# 复制模型到固件目录
cp motion_ai_model_quant.tflite ../main/model/

# 编译烧录
cd ..
idf.py build flash
```

### Step 6: 验证部署（可选）
```bash
# 1. ESP32运行并采集推理数据 → 平地数据_已推理.csv
# 2. 再次运行调试工具，使用真实数据对比
python debug_model_outputs.py
```

---

## 性能指标

### 模型性能
| 指标 | 训练集 | 验证集 |
|------|--------|--------|
| m1阶段准确率 | 97% | 94.81% |
| m2阶段准确率 | 98% | 96.23% |
| 参数MAE | 0.04 | 0.05 |

### ESP32推理性能
- **推理延迟**：<50ms
- **内存占用**：60KB (Tensor Arena)
- **模型大小**：40KB (量化后)
- **CPU占用**：单核约30%

### 量化影响
- 模型大小：400KB → 40KB（压缩10倍）
- 精度损失：<2%
- 推理速度：提升2-3倍

---

## 常见问题

### Q1: m2推理一直输出0
**原因**：TFLite输出顺序与H5不同，ESP32代码错误地读取了scene输出（只有2个值）作为m2_phase（需要3个值）

**解决**：已在 `main/ai_inference.cpp:204-207` 修正输出索引映射
```cpp
// 修正后
TfLiteTensor* output_m1_phase = interpreter->output(0);  // TFLite输出0是m1
TfLiteTensor* output_params = interpreter->output(1);    // TFLite输出1是params
TfLiteTensor* output_scene = interpreter->output(2);     // TFLite输出2是scene
TfLiteTensor* output_m2_phase = interpreter->output(3);  // TFLite输出3是m2
```

### Q2: 训练不收敛
**可能原因**：
- 数据质量问题（标签错误、不平衡）
- 学习率过大
- 批大小不合适

**解决方案**：
- 检查数据标注质量
- 减小学习率（默认0.001）
- 调整批大小（默认32）
- 增加训练轮数

### Q3: 模型太大无法部署
**解决方案**：
- 确保启用INT8量化（`--quantize`）
- 减少模型层数或通道数
- 增加Tensor Arena大小（`main/ai_inference.cpp:35`）

### Q4: TFLite转换失败
**可能原因**：
- H5模型包含不支持的算子
- 量化配置错误

**解决方案**：
- 检查模型架构是否兼容TFLite
- 确保提供代表性数据集用于量化校准
- 查看转换日志中的具体错误

---

## 依赖说明

详见 `requirements.txt`：
- **TensorFlow >= 2.10.0** - 深度学习框架
- **Keras >= 2.10.0** - 高层API
- **NumPy >= 1.21.0** - 数值计算
- **Pandas >= 1.3.0** - 数据处理
- **scikit-learn >= 1.0.0** - 数据划分和评估
- **matplotlib >= 3.4.0** - 可视化

安装：
```bash
pip install -r requirements.txt
```

---

## 下一步

模型转换完成后，进入 `../main/` 目录部署到ESP32：

```bash
# 1. 复制模型文件
cp motion_ai_model_quant.tflite ../main/model/

# 2. 编译烧录
cd ..
idf.py build flash monitor
```

验证推理结果是否正常（m2不应该100%输出静止）。
