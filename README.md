# 双腿机器人AI控制系统 - 完整工具链

本项目实现基于深度学习的双腿机器人运动控制系统，包含数据采集、模型训练和ESP32部署的完整工具链。

## 项目结构

```
yswgg/
├── 数据收集与标注/          # 数据采集和标注工具链
│   ├── 1_串口数据采集.py
│   ├── 2_数据自动标注.py
│   ├── 3.2_双腿对比标注工具_pyqtgraph.py
│   ├── 3.3_AI推理数据对比标注工具.py
│   ├── 4_理想曲线提取.py
│   ├── 5_训练数据生成.py
│   └── README.md            # 详细工具说明
│
├── 模型训练/                # AI模型训练和转换
│   ├── 1_模型训练.py
│   ├── 2_模型转换.py
│   ├── debug_model_outputs.py
│   ├── requirements.txt
│   └── README.md            # 详细工具说明
│
├── main/                    # ESP32固件源码
│   ├── ai_inference.cpp/h   # TFLite推理引擎
│   ├── unitree_motor.cpp/h  # 电机控制
│   ├── main.cpp            # 主程序
│   ├── model/              # 嵌入的TFLite模型
│   └── README.md            # 详细模块说明
│
├── CMakeLists.txt          # ESP-IDF构建配置
└── README.md               # 本文档
```

---

## 完整工作流程

### 阶段1：数据收集与标注

**目标**：采集双腿运动数据并标注运动阶段

```
ESP32采集 → 自动标注 → 手动修正 → 生成训练数据
```

**详细步骤**：

```bash
cd 数据收集与标注

# 1. 连接ESP32，采集原始数据
python 1_串口数据采集.py --port COM3 --output 平地数据_原始.csv

# 2. 自动标注运动阶段
python 2_数据自动标注.py --input 平地数据_原始.csv --output 平地数据_已标注.csv

# 3. 手动修正错误标注（可选）
python 3.2_双腿对比标注工具_pyqtgraph.py 平地数据_已标注.csv
# 保存为 平地数据_已修正.csv

# 4. 生成训练数据集
python 5_训练数据生成.py \
  --input 平地数据_已修正.csv \
  --scene 平地 \
  --output 训练数据_平地_双腿.npz
```

**输出文件**：
- `训练数据_平地_双腿.npz` - 包含速度窗口和阶段标签的训练数据

**数据格式**：
- 输入：`(N, 50, 2)` - N个样本，每个50时间步，2通道 `[m1_vel, m2_vel]`
- 标签：m1阶段、m2阶段、参数调整

详见：[数据收集与标注/README.md](数据收集与标注/README.md)

---

### 阶段2：模型训练与转换

**目标**：训练双腿联合模型并转换为TFLite格式

```
训练H5模型 → 转换TFLite → 验证输出顺序
```

**详细步骤**：

```bash
cd 模型训练

# 1. 训练模型
python 1_模型训练.py \
  --input ../数据收集与标注/训练数据_平地_双腿.npz \
  --scene 平地 \
  --epochs 50

# 2. 转换为TFLite量化模型
python 2_模型转换.py \
  --input motion_ai_model.h5 \
  --data ../数据收集与标注/训练数据_平地_双腿.npz \
  --output motion_ai_model_quant.tflite

# 3. 验证输出顺序（重要！）
python debug_model_outputs.py
```

**输出文件**：
- `motion_ai_model.h5` - H5格式模型（400KB）
- `motion_ai_model_quant.tflite` - TFLite量化模型（40KB）
- `training_history.png` - 训练曲线
- `model_evaluation.txt` - 性能报告

**模型性能**：
- m1阶段分类准确率：94.81%
- m2阶段分类准确率：96.23%
- 模型大小：40KB（INT8量化）
- ESP32推理延迟：<50ms

详见：[模型训练/README.md](模型训练/README.md)

---

### 阶段3：ESP32部署

**目标**：将TFLite模型部署到ESP32并验证推理结果

```
复制模型 → 编译固件 → 烧录测试 → 验证结果
```

**详细步骤**：

```bash
# 1. 复制模型到固件目录
cp 模型训练/motion_ai_model_quant.tflite main/model/

# 2. 编译烧录固件
idf.py build flash

# 3. 监控运行
idf.py monitor
```

**验证推理结果**：
- m1和m2应该都有静止/抬腿/压腿三种状态
- m2不应该100%输出静止（已修正TFLite输出顺序问题）
- 推理延迟<50ms
- 无内存溢出

详见：[main/README.md](main/README.md)

---

## 核心技术

### 1. 双腿联合模型

**输入**：
- 双腿速度窗口：`(batch, 50, 2)` - `[m1_vel, m2_vel]`
- 50个时间步（约1秒历史数据）

**输出**：
1. **场景分类** (2类)：平地/爬楼
2. **m1阶段** (3类)：静止/抬腿/压腿
3. **m2阶段** (3类)：静止/抬腿/压腿
4. **参数调整** (3个连续值)：delta_torque/kd/scale

**架构**：
```
输入 → Conv1D → BatchNorm → MaxPool →
       Conv1D → BatchNorm → MaxPool →
       Flatten → Dense(128) → Dropout →
       ├─ scene分支
       ├─ m1_phase分支
       ├─ m2_phase分支
       └─ params分支
```

### 2. 数据标注策略

**自动标注规则**：
- **静止**（0）：`abs(velocity) < 0.1`
- **抬腿**（1）：`velocity > 0.1`（速度为正）
- **压腿**（2）：`velocity < -0.1`（速度为负）

**m2标签修正**：
- m2电机物理反向，代码已处理反转逻辑
- 确保m1和m2标签镜像协调

### 3. 模型量化与部署

**量化策略**：
- INT8量化权重和激活
- 输入/输出保持FP32（便于接口对接）
- 使用代表性数据集校准量化参数

**量化效果**：
- 模型大小：400KB → 40KB（10倍压缩）
- 推理速度：提升2-3倍
- 精度损失：<2%

**⚠️ 关键问题：TFLite输出顺序变化**

TFLite转换后输出顺序与H5模型不同！已在 `main/ai_inference.cpp:204-207` 修正：

| H5输出顺序 | TFLite输出顺序 |
|-----------|---------------|
| 0: scene | 0: m1_phase ⚠️ |
| 1: m1_phase | 1: params ⚠️ |
| 2: m2_phase | 2: scene ⚠️ |
| 3: params | 3: m2_phase ⚠️ |

---

## 系统架构

### 数据流

```
┌─────────────┐
│   ESP32     │ → 串口采集原始数据
│  (主程序)    │
└─────────────┘
      ↓
┌─────────────┐
│ 自动标注工具  │ → 基于速度阈值打标签
└─────────────┘
      ↓
┌─────────────┐
│ 手动修正工具  │ → 交互式修正错误
└─────────────┘
      ↓
┌─────────────┐
│ 训练数据生成  │ → 滑动窗口 + 标签
└─────────────┘
      ↓
┌─────────────┐
│  模型训练    │ → H5模型 (94-96%准确率)
└─────────────┘
      ↓
┌─────────────┐
│  模型转换    │ → TFLite模型 (40KB)
└─────────────┘
      ↓
┌─────────────┐
│  ESP32部署   │ → 实时推理 (<50ms)
└─────────────┘
```

### 控制循环

```
┌──────────────────────────────────────────┐
│              ESP32主循环                  │
├──────────────────────────────────────────┤
│                                          │
│  1. 读取电机状态 (位置、速度、扭矩)        │
│       ↓                                  │
│  2. 维护50点速度窗口                      │
│       ↓                                  │
│  3. AI推理 (TFLite)                      │
│       ├─ m1阶段识别                       │
│       ├─ m2阶段识别                       │
│       └─ 参数调整建议                     │
│       ↓                                  │
│  4. 速度跟随控制                          │
│       ├─ 根据阶段调整目标速度              │
│       └─ 应用参数调整                     │
│       ↓                                  │
│  5. 发送电机命令                          │
│                                          │
└──────────────────────────────────────────┘
     ↓ 每100ms循环一次
```

---

## 性能指标

### 数据采集
- 采样频率：10Hz
- 数据量：>5000行（约10分钟）
- 通道：7个（m1_pos/vel/torque, m2_pos/vel/torque, ch6/7）

### 模型性能
| 指标 | 训练集 | 验证集 |
|------|--------|--------|
| m1阶段准确率 | 97% | 94.81% |
| m2阶段准确率 | 98% | 96.23% |
| 参数预测MAE | 0.04 | 0.05 |

### ESP32性能
- 推理延迟：<50ms
- 内存占用：60KB (Tensor Arena)
- 模型大小：40KB
- CPU占用：单核约30%
- 功耗：正常（无明显增加）

---

## 环境配置

### Python环境（数据处理和训练）

```bash
# 安装依赖
cd 数据收集与标注
pip install pandas numpy matplotlib pyqtgraph pyserial

cd ../模型训练
pip install -r requirements.txt
```

**依赖清单**：
- TensorFlow >= 2.10.0
- Keras >= 2.10.0
- NumPy >= 1.21.0
- Pandas >= 1.3.0
- Matplotlib >= 3.4.0
- scikit-learn >= 1.0.0
- PyQtGraph（数据标注工具）
- PySerial（串口通信）

### ESP32环境（固件开发）

**ESP-IDF版本**：5.x

**组件依赖**：
- TensorFlow Lite Micro（通过组件管理器）
- FreeRTOS（ESP-IDF内置）

**配置**：
```bash
# 获取ESP-IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh

# 激活环境
. ./export.sh

# 配置项目
cd /path/to/yswgg
idf.py menuconfig
```

---

## 常见问题与故障排除

### Q1: m2推理一直输出0
**原因**：TFLite输出顺序错误

**解决**：已在 `main/ai_inference.cpp:204-207` 修正，重新编译烧录即可

### Q2: 训练不收敛
**检查**：
- 数据质量（标签准确性、平衡性）
- 学习率（默认0.001）
- 训练轮数（默认50）

### Q3: ESP32推理结果不准确
**诊断步骤**：
1. 运行 `模型训练/debug_model_outputs.py` 验证模型
2. 检查ESP32输入数据预处理
3. 对比PC和ESP32推理结果

### Q4: 串口数据采集失败
**检查**：
- 串口端口号（COM3、/dev/ttyUSB0等）
- 波特率（默认921600）
- 驱动程序（CH340、CP2102等）

### Q5: 模型太大无法烧录
**解决**：
- 确保使用量化模型（40KB）
- 检查分区表配置
- 增加Flash分区大小

---

## 项目特色

### 1. 端到端工具链
从数据采集到模型部署的完整自动化流程

### 2. 双腿联合推理
单个模型同时预测两条腿的状态，确保协调性

### 3. 多任务学习
同时完成场景识别、阶段分类和参数调整

### 4. 极致优化
INT8量化 + 嵌入式部署，40KB模型实现<50ms推理

### 5. 交互式标注
PyQtGraph可视化工具，高效修正标注错误

### 6. 完善的调试工具
- debug_model_outputs.py：验证输出顺序
- 3.3_AI推理数据对比标注工具.py：对比人工和AI标注

---

## 后续改进方向

### 数据增强
- [ ] 收集爬楼场景数据
- [ ] 增加不同地形数据
- [ ] 数据增强技术（噪声、平移）

### 模型优化
- [ ] 尝试LSTM/GRU架构
- [ ] 注意力机制
- [ ] 模型剪枝和蒸馏

### 功能扩展
- [ ] 实时参数调整应用
- [ ] 异常检测和安全保护
- [ ] 多模态融合（IMU + 视觉）

### 工具完善
- [ ] Web界面数据采集
- [ ] 自动化测试框架
- [ ] 模型版本管理

---

## 技术栈总结

| 层次 | 技术 |
|------|------|
| 硬件平台 | ESP32 + 宇树电机 |
| 通信协议 | RS485 + WiFi |
| 深度学习 | TensorFlow/Keras |
| 嵌入式推理 | TensorFlow Lite Micro |
| 数据处理 | Pandas + NumPy |
| 可视化 | Matplotlib + PyQtGraph |
| 实时系统 | FreeRTOS |
| 构建系统 | ESP-IDF + CMake |

---

## 开发团队与许可

本项目用于双腿机器人运动控制研究。

**关键模块**：
- 数据采集与标注工具链
- 双腿联合AI模型
- ESP32 TFLite部署

**参考文档**：
- [数据收集与标注/README.md](数据收集与标注/README.md)
- [模型训练/README.md](模型训练/README.md)
- [main/README.md](main/README.md)

---

## 快速开始

**第一次使用**：

```bash
# 1. 安装Python依赖
cd 数据收集与标注 && pip install pandas numpy matplotlib pyqtgraph pyserial
cd ../模型训练 && pip install -r requirements.txt

# 2. 配置ESP-IDF
# 参考ESP-IDF官方文档

# 3. 采集第一批数据
cd 数据收集与标注
python 1_串口数据采集.py --port COM3 --output 测试数据.csv

# 4. 标注数据
python 2_数据自动标注.py --input 测试数据.csv --output 测试数据_已标注.csv

# 5. 生成训练数据
python 5_训练数据生成.py --input 测试数据_已标注.csv --scene 平地

# 6. 训练模型
cd ../模型训练
python 1_模型训练.py --input ../数据收集与标注/训练数据_平地_双腿.npz

# 7. 转换和部署
python 2_模型转换.py --input motion_ai_model.h5 --data ../数据收集与标注/训练数据_平地_双腿.npz
cp motion_ai_model_quant.tflite ../main/model/
cd .. && idf.py build flash monitor
```

**已有模型，直接部署**：

```bash
# 复制模型
cp 模型训练/motion_ai_model_quant.tflite main/model/

# 编译烧录
idf.py build flash monitor
```

---

## 版本历史

### v1.0 - 初始版本
- ✅ 双腿联合模型架构
- ✅ 数据采集和标注工具链
- ✅ TFLite量化和ESP32部署
- ✅ 修复TFLite输出顺序问题（debug_model_outputs.py）

---

## 联系与支持

遇到问题请查阅各子目录的README文档，或检查常见问题部分。
