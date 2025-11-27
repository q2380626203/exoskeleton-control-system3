# 方案三：AI阶段识别 + 动态参数调整

## 1. 方案架构

```
┌──────────────────────────────────────────────────────────────────┐
│                        AI 模块                                    │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────┐   │
│  │ 场景识别    │    │ 阶段识别     │    │ 参数调整            │   │
│  │             │    │             │    │                     │   │
│  │ 平地/爬楼   │ →  │ 静止/抬腿/  │ →  │ torque, kd, scale  │   │
│  │             │    │ 过渡/压腿   │    │                     │   │
│  └─────────────┘    └─────────────┘    └─────────────────────┘   │
│         ↑                  ↑                     ↑                │
│         │                  │                     │                │
│    长期特征            短期特征            曲线偏差分析           │
│   (步态周期等)        (速度窗口)         (实时vs理想)            │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────────┐
│                      控制模块（保持现有）                          │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│   实时速度 × scale → 设置电机MIT参数                              │
│   ├── torque: AI动态调整 (0.3 ~ 1.5)                             │
│   ├── kd: AI动态调整 (0.03 ~ 0.15)                               │
│   └── scale: AI动态调整 (0.6 ~ 1.0)                              │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. 状态定义

### 2.1 场景状态

```cpp
typedef enum {
    SCENE_FLAT_GROUND,    // 平地行走
    SCENE_CLIMBING        // 爬楼梯
} scene_type_t;
```

### 2.2 运动阶段状态

```cpp
typedef enum {
    PHASE_IDLE,           // 静止（速度接近0）
    PHASE_LIFTING,        // 抬腿（速度上升）
    PHASE_TRANSITION,     // 过渡（速度达到峰值后下降到0附近）
    PHASE_PRESSING,       // 压腿（速度负向增加）
} motion_phase_t;
```

### 2.3 完整状态周期

```
平地行走（电机1）：
时间 →  ════════════════════════════════════════════════════►

位置：  ───────╱￣￣￣￣￣╲───────╱￣￣￣￣￣╲───────
              ↑    ↑    ↑
            抬腿 过渡 压腿

速度：      ╭─╮         ╭─╮
          ╱   ╲       ╱   ╲
    ─────╱     ╲─────╱     ╲─────
                ╲   ╱
                 ╰─╯
         ↑   ↑   ↑   ↑
       静止 抬腿 过渡 压腿

阶段：  [IDLE][LIFT][TRANS][PRESS][IDLE][LIFT][TRANS][PRESS]
```

---

## 3. 理想曲线的作用

### 3.1 什么是理想曲线

```
理想曲线 = 无助力状态下采集的用户自然运动曲线

特征：
├── 代表用户"想要"的运动模式
├── 不受电机干扰，是纯净的用户意图
├── 包含位置和速度两个维度
└── 针对不同场景（平地/爬楼）有不同曲线
```

### 3.2 理想曲线的用途

```
实时曲线 vs 理想曲线 对比：

情况1：实时速度 < 理想速度（用户跟不上）
       → 说明用户需要更多助力
       → AI增加 torque，增加 scale

情况2：实时速度 > 理想速度（用户超前）
       → 说明助力过大或用户在加速
       → AI减小 torque，减小 scale

情况3：实时速度 ≈ 理想速度（刚好跟上）
       → 参数合适，保持不变
```

### 3.3 理想曲线数据结构

```cpp
// 理想曲线采样点
typedef struct {
    float velocity;     // 理想速度 (rad/s)
    float position;     // 理想位置 (rad) - 相对起始点
} ideal_curve_point_t;

// 一个完整阶段的理想曲线
typedef struct {
    ideal_curve_point_t points[100];  // 采样点（约200ms @ 500Hz）
    uint16_t num_points;              // 实际点数
    float peak_velocity;              // 峰值速度
    float total_displacement;         // 总位移
    uint32_t duration_ms;             // 阶段持续时间
} ideal_phase_curve_t;

// 场景的完整理想曲线
typedef struct {
    ideal_phase_curve_t lifting;      // 抬腿阶段曲线
    ideal_phase_curve_t transition;   // 过渡阶段曲线
    ideal_phase_curve_t pressing;     // 压腿阶段曲线
} ideal_motion_curve_t;

// 全局理想曲线库
ideal_motion_curve_t ideal_curve_flat_ground;   // 平地
ideal_motion_curve_t ideal_curve_climbing;       // 爬楼
```

---

## 4. AI参数调整逻辑

### 4.1 参数调整范围

```cpp
// 可调整的参数及其范围
typedef struct {
    float torque;           // 力矩: 0.3 ~ 1.5 (当前默认0.7)
    float kd;               // 速度阻尼: 0.03 ~ 0.15 (当前默认0.08)
    float velocity_scale;   // 速度缩放: 0.6 ~ 1.0 (当前默认0.8)
} ai_control_params_t;

#define TORQUE_MIN      0.3f
#define TORQUE_MAX      1.5f
#define TORQUE_DEFAULT  0.7f

#define KD_MIN          0.03f
#define KD_MAX          0.15f
#define KD_DEFAULT      0.08f

#define SCALE_MIN       0.6f
#define SCALE_MAX       1.0f
#define SCALE_DEFAULT   0.8f
```

### 4.2 AI调整策略

```cpp
/**
 * AI参数调整函数
 *
 * @param current_vel     当前实时速度
 * @param ideal_vel       理想曲线对应位置的速度
 * @param phase           当前运动阶段
 * @param scene           当前场景
 * @param params          输出：调整后的参数
 */
void ai_adjust_params(float current_vel, float ideal_vel,
                      motion_phase_t phase, scene_type_t scene,
                      ai_control_params_t* params) {

    // 计算速度偏差
    float vel_error = ideal_vel - current_vel;
    float vel_error_ratio = vel_error / (fabs(ideal_vel) + 0.1f);  // 防止除0

    // 基础参数（根据场景选择）
    if (scene == SCENE_CLIMBING) {
        params->torque = 1.0f;      // 爬楼需要更大力矩
        params->kd = 0.06f;
        params->velocity_scale = 0.75f;
    } else {
        params->torque = TORQUE_DEFAULT;
        params->kd = KD_DEFAULT;
        params->velocity_scale = SCALE_DEFAULT;
    }

    // 根据偏差动态调整
    if (phase == PHASE_LIFTING) {
        // 抬腿阶段：如果速度跟不上理想曲线，增加助力
        if (vel_error_ratio > 0.1f) {
            // 用户速度慢于理想速度10%以上
            params->torque += 0.1f * vel_error_ratio;
            params->velocity_scale += 0.05f * vel_error_ratio;
        } else if (vel_error_ratio < -0.1f) {
            // 用户速度快于理想速度，减小助力
            params->torque += 0.1f * vel_error_ratio;  // vel_error_ratio是负的
        }
    }
    else if (phase == PHASE_PRESSING) {
        // 压腿阶段：调整kd使下压更平滑
        if (fabs(vel_error_ratio) > 0.15f) {
            params->kd += 0.01f * fabs(vel_error_ratio);
        }
    }

    // 限幅
    params->torque = fmaxf(TORQUE_MIN, fminf(TORQUE_MAX, params->torque));
    params->kd = fmaxf(KD_MIN, fminf(KD_MAX, params->kd));
    params->velocity_scale = fmaxf(SCALE_MIN, fminf(SCALE_MAX, params->velocity_scale));
}
```

---

## 5. 数据采集方案

### 5.1 需要采集的数据

```
采集场景：
├── 场景1：平地行走（无助力）
│   ├── 慢速行走 2分钟
│   ├── 正常速度 2分钟
│   └── 快速行走 2分钟
│
└── 场景2：爬楼梯（无助力）
    ├── 慢速爬楼 2分钟
    └── 正常速度 2分钟

每个采样点记录：
├── timestamp (ms)
├── motor1_vel (rad/s)
├── motor2_vel (rad/s)
├── motor1_pos (rad)
└── motor2_pos (rad)
```

### 5.2 数据采集代码

```cpp
// data_collector.h
#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 采集模式
typedef enum {
    COLLECT_OFF,              // 关闭
    COLLECT_FLAT_GROUND,      // 平地采集
    COLLECT_CLIMBING          // 爬楼采集
} collect_mode_t;

// 初始化
void data_collector_init(void);

// 设置采集模式
void data_collector_set_mode(collect_mode_t mode);

// 记录数据（在motor_control_task中调用）
void data_collector_record(uint32_t timestamp,
                           float m1_vel, float m2_vel,
                           float m1_pos, float m2_pos);

// 获取采集统计
uint32_t data_collector_get_count(void);

#ifdef __cplusplus
}
#endif

#endif
```

```cpp
// data_collector.c
#include "data_collector.h"
#include <stdio.h>
#include "esp_log.h"

static const char* TAG = "Collector";

static collect_mode_t _mode = COLLECT_OFF;
static uint32_t _count = 0;
static uint32_t _start_time = 0;

void data_collector_init(void) {
    _mode = COLLECT_OFF;
    _count = 0;
}

void data_collector_set_mode(collect_mode_t mode) {
    _mode = mode;
    _count = 0;

    if (mode != COLLECT_OFF) {
        // 输出CSV头
        const char* scene = (mode == COLLECT_FLAT_GROUND) ? "FLAT" : "CLIMB";
        printf("\n=== DATA_START [%s] ===\n", scene);
        printf("ts,m1_vel,m2_vel,m1_pos,m2_pos\n");
        ESP_LOGI(TAG, "开始采集: %s", scene);
    } else {
        printf("=== DATA_END [%lu samples] ===\n", _count);
        ESP_LOGI(TAG, "采集结束: %lu 样本", _count);
    }
}

void data_collector_record(uint32_t timestamp,
                           float m1_vel, float m2_vel,
                           float m1_pos, float m2_pos) {
    if (_mode == COLLECT_OFF) return;

    // 每个采样点输出一行（精简格式减少串口负载）
    printf("%lu,%.3f,%.3f,%.4f,%.4f\n",
           timestamp, m1_vel, m2_vel, m1_pos, m2_pos);

    _count++;

    // 每1000个样本输出进度
    if (_count % 1000 == 0) {
        ESP_LOGI(TAG, "已采集 %lu 样本", _count);
    }
}

uint32_t data_collector_get_count(void) {
    return _count;
}
```

### 5.3 集成到 main.cpp

```cpp
#include "data_collector.h"

// 在 handle_web_command 中添加
extern "C" esp_err_t handle_web_command(const char *cmd) {
    // ... 原有代码 ...

    // 数据采集控制
    else if (strcmp(cmd, "collect_flat") == 0) {
        ESP_LOGI(TAG, "[WEB] 开始平地数据采集");
        speed_follow.enableMotorControl(false);  // 关闭电机输出！
        data_collector_set_mode(COLLECT_FLAT_GROUND);
        return ESP_OK;
    }
    else if (strcmp(cmd, "collect_climb") == 0) {
        ESP_LOGI(TAG, "[WEB] 开始爬楼数据采集");
        speed_follow.enableMotorControl(false);  // 关闭电机输出！
        data_collector_set_mode(COLLECT_CLIMBING);
        return ESP_OK;
    }
    else if (strcmp(cmd, "collect_stop") == 0) {
        ESP_LOGI(TAG, "[WEB] 停止数据采集");
        data_collector_set_mode(COLLECT_OFF);
        return ESP_OK;
    }

    // ... 原有代码 ...
}

// 在 motor_control_task 中添加
void motor_control_task(void *pvParameters) {
    // ... 原有初始化 ...

    data_collector_init();  // 初始化数据采集器

    while (1) {
        // ... 原有电机通信代码 ...

        if (err1 == ESP_OK && err2 == ESP_OK) {
            uint32_t timestamp = esp_timer_get_time() / 1000;

            // ========== 数据采集 ==========
            data_collector_record(timestamp,
                                  motor_data_1.vel, motor_data_2.vel,
                                  motor_data_1.pos, motor_data_2.pos);
            // ==============================

            // ... 原有波形分析和速度跟随代码 ...
        }

        // ... 原有延时 ...
    }
}
```

---

## 6. 数据标注方案

### 6.1 自动标注脚本

```python
# label_motion_data.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_data(filename):
    """加载采集的数据"""
    df = pd.read_csv(filename)
    return df

def auto_label(df, motor_col='m1_vel'):
    """
    自动标注运动阶段

    阶段定义：
    0 = IDLE (静止)：速度绝对值 < 阈值
    1 = LIFTING (抬腿)：速度正向增加
    2 = TRANSITION (过渡)：速度从峰值下降到0附近
    3 = PRESSING (压腿)：速度负向增加
    """

    vel = df[motor_col].values
    labels = np.zeros(len(vel), dtype=int)

    # 参数（可根据实际数据调整）
    IDLE_THRESHOLD = 2.0       # 静止阈值
    SMOOTH_WINDOW = 5          # 平滑窗口

    # 计算速度变化率（加速度）
    vel_smooth = pd.Series(vel).rolling(SMOOTH_WINDOW, center=True).mean().values
    vel_diff = np.gradient(vel_smooth)

    for i in range(len(vel)):
        v = vel_smooth[i] if not np.isnan(vel_smooth[i]) else vel[i]
        dv = vel_diff[i] if not np.isnan(vel_diff[i]) else 0

        if abs(v) < IDLE_THRESHOLD:
            labels[i] = 0  # IDLE
        elif v > 0 and dv > 0:
            labels[i] = 1  # LIFTING (正速度且加速)
        elif v > 0 and dv <= 0:
            labels[i] = 2  # TRANSITION (正速度但减速)
        elif v < 0 and dv < 0:
            labels[i] = 3  # PRESSING (负速度且加速，绝对值增大)
        elif v < 0 and dv >= 0:
            labels[i] = 2  # TRANSITION (负速度但减速，回零)
        else:
            labels[i] = 0  # 默认静止

    df['label'] = labels
    return df

def visualize(df, motor_col='m1_vel'):
    """可视化标注结果"""
    fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)

    colors = {0: 'gray', 1: 'green', 2: 'orange', 3: 'red'}
    labels_text = {0: 'IDLE', 1: 'LIFTING', 2: 'TRANSITION', 3: 'PRESSING'}

    # 速度曲线
    ax1 = axes[0]
    ax1.plot(df['ts'], df[motor_col], 'b-', linewidth=0.5)
    ax1.set_ylabel('Velocity (rad/s)')
    ax1.set_title('Motor Velocity')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # 标注结果（用颜色区分）
    ax2 = axes[1]
    for label_val in range(4):
        mask = df['label'] == label_val
        ax2.scatter(df.loc[mask, 'ts'], df.loc[mask, motor_col],
                   c=colors[label_val], s=1, label=labels_text[label_val])
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.set_title('Labeled Phases')
    ax2.legend()

    # 标签时序
    ax3 = axes[2]
    ax3.plot(df['ts'], df['label'], 'k-', linewidth=0.5)
    ax3.set_ylabel('Phase Label')
    ax3.set_xlabel('Time (ms)')
    ax3.set_yticks([0, 1, 2, 3])
    ax3.set_yticklabels(['IDLE', 'LIFTING', 'TRANS', 'PRESSING'])

    plt.tight_layout()
    plt.savefig('labeled_visualization.png', dpi=150)
    plt.show()

def extract_ideal_curves(df, motor_col='m1_vel', pos_col='m1_pos'):
    """
    从标注数据中提取理想曲线
    找到多个完整周期，取平均值
    """
    # 找到所有 IDLE -> LIFTING 的转换点（周期起始）
    labels = df['label'].values
    cycle_starts = []

    for i in range(1, len(labels)):
        if labels[i-1] == 0 and labels[i] == 1:
            cycle_starts.append(i)

    print(f"找到 {len(cycle_starts)} 个运动周期")

    # 提取每个阶段的曲线
    curves = {
        'lifting': [],      # 抬腿阶段曲线
        'transition': [],   # 过渡阶段曲线
        'pressing': []      # 压腿阶段曲线
    }

    for start_idx in cycle_starts[:-1]:  # 跳过最后一个不完整周期
        # 找到这个周期的各阶段边界
        # ... (提取逻辑)
        pass

    # 计算平均曲线
    # ...

    return curves

if __name__ == '__main__':
    # 加载数据
    df = load_data('flat_ground_data.csv')

    # 自动标注
    df = auto_label(df)

    # 可视化
    visualize(df)

    # 保存标注结果
    df.to_csv('flat_ground_labeled.csv', index=False)
    print(f"标注完成，保存到 flat_ground_labeled.csv")

    # 标签统计
    print("\n标签分布:")
    print(df['label'].value_counts().sort_index())
```

### 6.2 人工校验和修正

```python
# manual_correction.py
"""
交互式标注修正工具
用matplotlib的交互功能，点击修正错误标注
"""
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons

class LabelCorrector:
    def __init__(self, df):
        self.df = df
        self.current_label = 0

        self.fig, self.ax = plt.subplots(figsize=(15, 6))
        self.line, = self.ax.plot(df['ts'], df['m1_vel'], 'b-', picker=5)

        # 标签选择器
        ax_radio = plt.axes([0.02, 0.5, 0.1, 0.3])
        self.radio = RadioButtons(ax_radio,
                                  ('IDLE', 'LIFTING', 'TRANS', 'PRESSING'))
        self.radio.on_clicked(self.on_radio)

        # 保存按钮
        ax_save = plt.axes([0.02, 0.3, 0.1, 0.1])
        self.btn_save = Button(ax_save, 'Save')
        self.btn_save.on_clicked(self.save)

        self.fig.canvas.mpl_connect('pick_event', self.on_pick)

    def on_radio(self, label):
        label_map = {'IDLE': 0, 'LIFTING': 1, 'TRANS': 2, 'PRESSING': 3}
        self.current_label = label_map[label]

    def on_pick(self, event):
        # 点击修正标签
        ind = event.ind[0]
        self.df.loc[ind, 'label'] = self.current_label
        self.update_plot()

    def update_plot(self):
        # 更新显示
        pass

    def save(self, event):
        self.df.to_csv('corrected_labels.csv', index=False)
        print("已保存修正后的标签")

    def run(self):
        plt.show()
```

---

## 7. 理想曲线提取与存储

### 7.1 Python提取脚本

```python
# extract_ideal_curve.py
import pandas as pd
import numpy as np

def extract_ideal_curve(df, motor_col='m1_vel'):
    """
    从标注数据提取理想曲线
    返回每个阶段的平均速度曲线
    """

    # 找所有完整周期
    cycles = find_complete_cycles(df)

    # 对每个阶段，收集所有周期的曲线
    lifting_curves = []
    transition_curves = []
    pressing_curves = []

    for cycle in cycles:
        lifting_curves.append(cycle['lifting'])
        transition_curves.append(cycle['transition'])
        pressing_curves.append(cycle['pressing'])

    # 插值对齐并取平均
    ideal_lifting = average_curves(lifting_curves, target_len=100)
    ideal_transition = average_curves(transition_curves, target_len=50)
    ideal_pressing = average_curves(pressing_curves, target_len=100)

    return {
        'lifting': ideal_lifting,
        'transition': ideal_transition,
        'pressing': ideal_pressing
    }

def export_to_c_header(curves, filename='ideal_curves.h'):
    """导出为C头文件"""

    with open(filename, 'w') as f:
        f.write("// 自动生成的理想曲线数据\n")
        f.write("#ifndef IDEAL_CURVES_H\n")
        f.write("#define IDEAL_CURVES_H\n\n")

        for phase_name, curve in curves.items():
            f.write(f"// {phase_name} 阶段理想速度曲线\n")
            f.write(f"static const float ideal_{phase_name}_vel[] = {{\n    ")

            for i, val in enumerate(curve):
                f.write(f"{val:.4f}f")
                if i < len(curve) - 1:
                    f.write(", ")
                if (i + 1) % 10 == 0:
                    f.write("\n    ")

            f.write("\n};\n")
            f.write(f"static const int ideal_{phase_name}_len = {len(curve)};\n\n")

        f.write("#endif // IDEAL_CURVES_H\n")

    print(f"已导出到 {filename}")
```

### 7.2 生成的C头文件示例

```c
// ideal_curves.h
#ifndef IDEAL_CURVES_H
#define IDEAL_CURVES_H

// 平地模式 - 抬腿阶段理想速度曲线 (100个点, 约200ms)
static const float ideal_flat_lifting_vel[] = {
    0.5000f, 0.8234f, 1.2456f, 1.7890f, 2.4532f, 3.1234f, 3.8901f, 4.5678f, 5.1234f, 5.5678f,
    5.8901f, 6.1234f, 6.2345f, 6.2890f, 6.2456f, 6.1234f, 5.9012f, 5.5678f, 5.1234f, 4.5678f,
    // ... 更多数据点 ...
};
static const int ideal_flat_lifting_len = 100;

// 平地模式 - 过渡阶段理想速度曲线 (50个点, 约100ms)
static const float ideal_flat_transition_vel[] = {
    4.5678f, 4.0123f, 3.4567f, 2.8901f, 2.3456f, 1.8901f, 1.4567f, 1.0234f, 0.6789f, 0.3456f,
    // ... 更多数据点 ...
};
static const int ideal_flat_transition_len = 50;

// 平地模式 - 压腿阶段理想速度曲线 (100个点, 约200ms)
static const float ideal_flat_pressing_vel[] = {
    -0.3456f, -0.7890f, -1.2345f, -1.7890f, -2.3456f, -2.8901f, -3.4567f, -3.9012f, -4.2345f, -4.4567f,
    // ... 更多数据点 ...
};
static const int ideal_flat_pressing_len = 100;

// 爬楼模式的曲线类似...

#endif // IDEAL_CURVES_H
```

---

## 8. AI模型设计

### 8.1 模型任务

```
输入：最近50-100个速度采样点（100-200ms窗口）
输出：
├── scene: 场景分类 (平地/爬楼)
├── phase: 阶段分类 (静止/抬腿/过渡/压腿)
├── phase_progress: 阶段进度 (0.0~1.0)
└── param_adjustment: 参数调整建议 (torque, kd, scale的delta值)
```

### 8.2 模型结构

```python
import tensorflow as tf
from tensorflow.keras import layers, models

def create_motion_ai_model(window_size=50):
    """
    多任务学习模型：
    - 场景分类
    - 阶段分类
    - 参数预测
    """

    # 输入层
    input_layer = layers.Input(shape=(window_size, 1), name='velocity_input')

    # 共享特征提取层
    x = layers.Conv1D(32, 5, activation='relu', padding='same')(input_layer)
    x = layers.MaxPooling1D(2)(x)
    x = layers.Conv1D(64, 3, activation='relu', padding='same')(x)
    x = layers.MaxPooling1D(2)(x)
    x = layers.Flatten()(x)
    shared_features = layers.Dense(64, activation='relu')(x)

    # 任务1：场景分类 (平地/爬楼)
    scene_branch = layers.Dense(32, activation='relu')(shared_features)
    scene_output = layers.Dense(2, activation='softmax', name='scene')(scene_branch)

    # 任务2：阶段分类 (静止/抬腿/过渡/压腿)
    phase_branch = layers.Dense(32, activation='relu')(shared_features)
    phase_output = layers.Dense(4, activation='softmax', name='phase')(phase_branch)

    # 任务3：参数调整预测 (delta_torque, delta_kd, delta_scale)
    param_branch = layers.Dense(32, activation='relu')(shared_features)
    param_output = layers.Dense(3, activation='tanh', name='params')(param_branch)
    # tanh输出范围[-1,1]，实际使用时乘以调整范围

    model = models.Model(
        inputs=input_layer,
        outputs=[scene_output, phase_output, param_output]
    )

    model.compile(
        optimizer='adam',
        loss={
            'scene': 'sparse_categorical_crossentropy',
            'phase': 'sparse_categorical_crossentropy',
            'params': 'mse'
        },
        loss_weights={'scene': 1.0, 'phase': 2.0, 'params': 1.0},
        metrics={'scene': 'accuracy', 'phase': 'accuracy'}
    )

    return model
```

### 8.3 参数调整标签生成

```python
def generate_param_labels(df, ideal_curves):
    """
    生成参数调整的训练标签

    思路：
    - 比较实时速度与理想速度的偏差
    - 偏差越大，需要的调整越大
    """

    param_labels = []

    for i, row in df.iterrows():
        phase = row['label']
        current_vel = row['m1_vel']

        # 获取对应阶段的理想速度
        if phase == 1:  # LIFTING
            progress = estimate_phase_progress(df, i, phase)
            ideal_vel = ideal_curves['lifting'][int(progress * 99)]
        elif phase == 2:  # TRANSITION
            progress = estimate_phase_progress(df, i, phase)
            ideal_vel = ideal_curves['transition'][int(progress * 49)]
        elif phase == 3:  # PRESSING
            progress = estimate_phase_progress(df, i, phase)
            ideal_vel = ideal_curves['pressing'][int(progress * 99)]
        else:
            ideal_vel = 0

        # 计算偏差并生成调整标签
        vel_error = ideal_vel - current_vel

        # 归一化到[-1, 1]
        delta_torque = np.clip(vel_error * 0.1, -1, 1)
        delta_kd = np.clip(abs(vel_error) * 0.05, -1, 1)
        delta_scale = np.clip(vel_error * 0.05, -1, 1)

        param_labels.append([delta_torque, delta_kd, delta_scale])

    return np.array(param_labels)
```

---

## 9. 部署到ESP32

### 9.1 推理代码框架

```cpp
// ai_motion_predictor.h
#ifndef AI_MOTION_PREDICTOR_H
#define AI_MOTION_PREDICTOR_H

#include <stdint.h>

typedef enum {
    SCENE_FLAT = 0,
    SCENE_CLIMB = 1
} ai_scene_t;

typedef enum {
    PHASE_IDLE = 0,
    PHASE_LIFTING = 1,
    PHASE_TRANSITION = 2,
    PHASE_PRESSING = 3
} ai_phase_t;

typedef struct {
    ai_scene_t scene;
    float scene_confidence;

    ai_phase_t phase;
    float phase_confidence;

    float delta_torque;    // 力矩调整量 [-0.3, +0.3]
    float delta_kd;        // kd调整量 [-0.03, +0.03]
    float delta_scale;     // scale调整量 [-0.1, +0.1]
} ai_prediction_t;

// 初始化AI模型
void ai_motion_init(void);

// 添加新的速度采样
void ai_motion_add_sample(float velocity);

// 执行预测
void ai_motion_predict(ai_prediction_t* prediction);

#endif
```

### 9.2 集成到状态机

```cpp
void SpeedFollowMode::update(const MotorDataA1& motor_data, float ch6_max, float ch7_max) {
    uint32_t current_time = esp_timer_get_time() / 1000;

    // ========== AI预测 ==========
    if (motor_data.id == 1) {
        ai_motion_add_sample(motor_data.vel);

        ai_prediction_t pred;
        ai_motion_predict(&pred);

        // 更新场景
        _current_scene = pred.scene;

        // 根据AI预测调整参数
        if (pred.phase_confidence > 0.7f) {
            float adjusted_torque = _config_motor1.phase1.torque + pred.delta_torque;
            float adjusted_kd = _config_motor1.phase1.kd + pred.delta_kd;
            float adjusted_scale = _velocity_scale + pred.delta_scale;

            // 限幅
            _config_motor1.phase1.torque = fmaxf(0.3f, fminf(1.5f, adjusted_torque));
            _config_motor1.phase1.kd = fmaxf(0.03f, fminf(0.15f, adjusted_kd));
            _velocity_scale = fmaxf(0.6f, fminf(1.0f, adjusted_scale));
        }

        // AI驱动的状态转换
        if (_ai_enabled) {
            handleAIPhaseTransition(pred);
        }
    }
    // ==============================

    // ... 原有状态机逻辑 ...
}

void SpeedFollowMode::handleAIPhaseTransition(const ai_prediction_t& pred) {
    // AI识别阶段转换
    if (_last_ai_phase == PHASE_IDLE && pred.phase == PHASE_LIFTING && pred.phase_confidence > 0.8f) {
        // 静止 → 抬腿
        _state = SPEED_FOLLOW_PHASE1;
        _lifting_motor = 1;  // 或根据速度方向判断
        ESP_LOGI(TAG, "AI: 检测到抬腿开始 (conf=%.2f)", pred.phase_confidence);
    }
    else if (_last_ai_phase == PHASE_LIFTING && pred.phase == PHASE_TRANSITION && pred.phase_confidence > 0.7f) {
        // 抬腿 → 过渡
        // 可以开始准备切换到压腿
    }
    else if (pred.phase == PHASE_PRESSING && pred.phase_confidence > 0.7f) {
        // 进入压腿
        _state = SPEED_FOLLOW_PHASE2;
        ESP_LOGI(TAG, "AI: 检测到压腿开始 (conf=%.2f)", pred.phase_confidence);
    }
    else if (pred.phase == PHASE_IDLE && pred.phase_confidence > 0.7f) {
        // 回到静止
        _state = SPEED_FOLLOW_IDLE;
    }

    _last_ai_phase = pred.phase;
}
```

---

## 10. 实施步骤总结

### 第一周：数据采集

```
Day 1-2: 代码修改
├── 添加 data_collector 模块
├── 添加 Web 控制接口
└── 编译烧录测试

Day 3-5: 数据采集
├── 平地行走采集（无助力，3-5分钟）
├── 爬楼梯采集（无助力，2-3分钟）
└── 保存串口输出到CSV文件

Day 6-7: 数据标注
├── 运行自动标注脚本
├── 可视化检查标注结果
└── 人工修正错误标注
```

### 第二周：模型训练

```
Day 1-2: 数据预处理
├── 提取理想曲线
├── 生成滑动窗口训练数据
└── 生成参数调整标签

Day 3-5: 模型训练
├── 训练多任务模型
├── 评估各任务准确率
└── 调优超参数

Day 6-7: 模型转换
├── TFLite转换和量化
├── 生成C头文件
└── 测试推理速度
```

### 第三周：部署集成

```
Day 1-3: ESP32集成
├── 添加TFLite推理代码
├── 集成到状态机
└── 编译烧录

Day 4-7: 测试优化
├── 功能测试
├── 对比测试（有/无AI）
├── 调整置信度阈值
└── 优化参数调整范围
```

---

## 11. 预期效果

| 指标 | 当前方案 | AI方案 |
|------|---------|--------|
| 触发准确率 | ~85% | >95% |
| 阶段识别 | 基于阈值/超时 | AI实时识别 |
| 参数适应性 | 固定参数 | 动态调整 |
| 场景适应 | 单一模式 | 自动识别平地/爬楼 |
| 用户体验 | 良好 | 更自然、更平滑 |
