#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI模型训练 - 第二周Day3-5

功能：
1. 加载训练数据（来自数据收集与标注/5_训练数据生成.py）
2. 构建多任务学习模型：
   - 任务1：场景分类（平地/爬楼）
   - 任务2：阶段分类（静止/抬腿/压腿）
   - 任务3：参数预测（delta_torque, delta_kd, delta_scale）
3. 训练模型并保存
4. 评估模型性能

模型架构：
- 输入：(batch, 50, 1) 速度窗口
- 共享特征提取：Conv1D + MaxPooling
- 三个任务分支输出

使用方法：
python 1_模型训练.py --input 训练数据_平地_m1.npz --epochs 50 --batch-size 32

输出：
- motion_ai_model.h5 (完整模型)
- motion_ai_model.weights.h5 (模型权重)
- training_history.png (训练曲线)
- model_evaluation.txt (评估报告)
"""

import numpy as np
import argparse
import matplotlib.pyplot as plt
from pathlib import Path
from sklearn.model_selection import train_test_split

# TensorFlow/Keras
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, models, callbacks

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


class 运动AI模型:
    """
    运动阶段识别与参数调整的多任务学习模型
    """

    def __init__(self, 窗口大小=50, 场景数=2, 阶段数=3):
        """
        初始化模型

        参数:
            窗口大小: 输入窗口大小（采样点数）
            场景数: 场景类别数（平地/爬楼）
            阶段数: 阶段类别数（静止/抬腿/压腿）
        """
        self.窗口大小 = 窗口大小
        self.场景数 = 场景数
        self.阶段数 = 阶段数
        self.model = None
        self.history = None

    def 构建模型(self):
        """
        构建多任务学习模型

        架构：
        - 输入层：(窗口大小, 1)
        - 共享特征提取：Conv1D + MaxPooling + Flatten + Dense
        - 场景分支：Dense -> Softmax (2类)
        - 阶段分支：Dense -> Softmax (3类)
        - 参数分支：Dense -> Tanh (3个连续值)
        """
        print("\n构建多任务学习模型...")

        # 输入层
        input_layer = layers.Input(shape=(self.窗口大小, 1), name='velocity_input')

        # 共享特征提取层
        x = layers.Conv1D(32, 5, activation='relu', padding='same', name='conv1')(input_layer)
        x = layers.BatchNormalization()(x)
        x = layers.MaxPooling1D(2, name='pool1')(x)

        x = layers.Conv1D(64, 3, activation='relu', padding='same', name='conv2')(x)
        x = layers.BatchNormalization()(x)
        x = layers.MaxPooling1D(2, name='pool2')(x)

        x = layers.Flatten(name='flatten')(x)
        shared_features = layers.Dense(128, activation='relu', name='shared_dense')(x)
        shared_features = layers.Dropout(0.3, name='dropout')(shared_features)

        # 任务1：场景分类（平地/爬楼）
        scene_branch = layers.Dense(64, activation='relu', name='scene_dense')(shared_features)
        scene_output = layers.Dense(self.场景数, activation='softmax', name='scene')(scene_branch)

        # 任务2：阶段分类（静止/抬腿/压腿）
        phase_branch = layers.Dense(64, activation='relu', name='phase_dense')(shared_features)
        phase_output = layers.Dense(self.阶段数, activation='softmax', name='phase')(phase_branch)

        # 任务3：参数调整预测（delta_torque, delta_kd, delta_scale）
        # 输出范围[-1, 1]，使用tanh激活
        param_branch = layers.Dense(64, activation='relu', name='param_dense')(shared_features)
        param_output = layers.Dense(3, activation='tanh', name='params')(param_branch)

        # 构建模型
        self.model = models.Model(
            inputs=input_layer,
            outputs=[scene_output, phase_output, param_output],
            name='motion_ai_model'
        )

        # 编译模型
        self.model.compile(
            optimizer=keras.optimizers.Adam(learning_rate=0.001),
            loss={
                'scene': keras.losses.SparseCategoricalCrossentropy(),
                'phase': keras.losses.SparseCategoricalCrossentropy(),
                'params': keras.losses.MeanSquaredError()
            },
            loss_weights={
                'scene': 1.0,   # 场景分类权重
                'phase': 2.0,   # 阶段分类权重（更重要）
                'params': 1.0   # 参数预测权重
            },
            metrics={
                'scene': ['accuracy'],
                'phase': ['accuracy'],
                'params': [keras.metrics.MeanAbsoluteError()]
            }
        )

        # 打印模型摘要
        print("\n模型摘要:")
        self.model.summary()

        # 计算参数量
        总参数 = self.model.count_params()
        print(f"\n总参数量: {总参数:,}")
        print(f"预计模型大小: ~{总参数*4/1024:.1f} KB (FP32)")

        return self.model

    def 训练模型(self, X_train, y_scene_train, y_phase_train, y_params_train,
                X_val, y_scene_val, y_phase_val, y_params_val,
                epochs=50, batch_size=32):
        """
        训练模型

        参数:
            X_train, y_*_train: 训练数据
            X_val, y_*_val: 验证数据
            epochs: 训练轮数
            batch_size: 批大小
        """
        print("\n开始训练模型...")
        print(f"训练集: {len(X_train)} 样本")
        print(f"验证集: {len(X_val)} 样本")
        print(f"Epochs: {epochs}, Batch size: {batch_size}")

        # 回调函数
        callback_list = [
            # 早停：如果验证损失不再下降，提前停止训练
            callbacks.EarlyStopping(
                monitor='val_loss',
                patience=10,
                restore_best_weights=True,
                verbose=1
            ),
            # 学习率衰减：如果验证损失plateau，降低学习率
            callbacks.ReduceLROnPlateau(
                monitor='val_loss',
                factor=0.5,
                patience=5,
                min_lr=1e-6,
                verbose=1
            ),
            # 模型检查点：保存最佳模型
            callbacks.ModelCheckpoint(
                'best_model_checkpoint.h5',
                monitor='val_loss',
                save_best_only=True,
                verbose=1
            )
        ]

        # 训练
        self.history = self.model.fit(
            X_train,
            {
                'scene': y_scene_train,
                'phase': y_phase_train,
                'params': y_params_train
            },
            validation_data=(
                X_val,
                {
                    'scene': y_scene_val,
                    'phase': y_phase_val,
                    'params': y_params_val
                }
            ),
            epochs=epochs,
            batch_size=batch_size,
            callbacks=callback_list,
            verbose=1
        )

        print("\n✓ 训练完成！")

        return self.history

    def 保存模型(self, 输出文件='motion_ai_model.h5'):
        """
        保存完整模型

        参数:
            输出文件: 输出H5文件名
        """
        self.model.save(输出文件)
        print(f"\n✓ 完整模型已保存: {输出文件}")
        print(f"  文件大小: {Path(输出文件).stat().st_size / 1024:.1f} KB")

        # 同时保存权重（Keras要求权重文件以.weights.h5结尾）
        权重文件 = 输出文件.replace('.h5', '.weights.h5')
        self.model.save_weights(权重文件)
        print(f"✓ 模型权重已保存: {权重文件}")

    def 可视化训练历史(self, 输出文件='training_history.png'):
        """
        可视化训练历史

        参数:
            输出文件: 输出图片文件名
        """
        if self.history is None:
            print("⚠ 没有训练历史记录")
            return

        history = self.history.history

        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        fig.suptitle('模型训练历史', fontsize=16, fontweight='bold')

        # 1. 总损失
        ax = axes[0, 0]
        ax.plot(history['loss'], label='训练损失', linewidth=2)
        ax.plot(history['val_loss'], label='验证损失', linewidth=2)
        ax.set_title('总损失', fontsize=12, fontweight='bold')
        ax.set_xlabel('Epoch')
        ax.set_ylabel('Loss')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 2. 场景分类损失
        ax = axes[0, 1]
        ax.plot(history['scene_loss'], label='训练损失', linewidth=2)
        ax.plot(history['val_scene_loss'], label='验证损失', linewidth=2)
        ax.set_title('场景分类损失', fontsize=12, fontweight='bold')
        ax.set_xlabel('Epoch')
        ax.set_ylabel('Loss')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 3. 阶段分类损失
        ax = axes[0, 2]
        ax.plot(history['phase_loss'], label='训练损失', linewidth=2)
        ax.plot(history['val_phase_loss'], label='验证损失', linewidth=2)
        ax.set_title('阶段分类损失', fontsize=12, fontweight='bold')
        ax.set_xlabel('Epoch')
        ax.set_ylabel('Loss')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 4. 场景分类准确率
        ax = axes[1, 0]
        ax.plot(history['scene_accuracy'], label='训练准确率', linewidth=2)
        ax.plot(history['val_scene_accuracy'], label='验证准确率', linewidth=2)
        ax.set_title('场景分类准确率', fontsize=12, fontweight='bold')
        ax.set_xlabel('Epoch')
        ax.set_ylabel('Accuracy')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 5. 阶段分类准确率
        ax = axes[1, 1]
        ax.plot(history['phase_accuracy'], label='训练准确率', linewidth=2)
        ax.plot(history['val_phase_accuracy'], label='验证准确率', linewidth=2)
        ax.set_title('阶段分类准确率', fontsize=12, fontweight='bold')
        ax.set_xlabel('Epoch')
        ax.set_ylabel('Accuracy')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 6. 参数预测MAE
        ax = axes[1, 2]
        ax.plot(history['params_mae'], label='训练MAE', linewidth=2)
        ax.plot(history['val_params_mae'], label='验证MAE', linewidth=2)
        ax.set_title('参数预测MAE', fontsize=12, fontweight='bold')
        ax.set_xlabel('Epoch')
        ax.set_ylabel('MAE')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(输出文件, dpi=150, bbox_inches='tight')
        print(f"\n✓ 训练历史可视化已保存: {输出文件}")

    def 评估模型(self, X_test, y_scene_test, y_phase_test, y_params_test, 输出文件='model_evaluation.txt'):
        """
        评估模型性能

        参数:
            X_test, y_*_test: 测试数据
            输出文件: 输出文本报告
        """
        print("\n评估模型性能...")

        # 预测
        predictions = self.model.predict(X_test, verbose=0)
        pred_scene, pred_phase, pred_params = predictions

        # 场景分类准确率
        scene_pred_labels = np.argmax(pred_scene, axis=1)
        scene_accuracy = np.mean(scene_pred_labels == y_scene_test)

        # 阶段分类准确率
        phase_pred_labels = np.argmax(pred_phase, axis=1)
        phase_accuracy = np.mean(phase_pred_labels == y_phase_test)

        # 参数预测MAE
        params_mae = np.mean(np.abs(pred_params - y_params_test), axis=0)

        # 打印评估结果
        print(f"\n测试集评估结果:")
        print(f"  场景分类准确率: {scene_accuracy*100:.2f}%")
        print(f"  阶段分类准确率: {phase_accuracy*100:.2f}%")
        print(f"  参数预测MAE:")
        print(f"    delta_torque: {params_mae[0]:.4f}")
        print(f"    delta_kd: {params_mae[1]:.4f}")
        print(f"    delta_scale: {params_mae[2]:.4f}")

        # 保存详细报告
        with open(输出文件, 'w', encoding='utf-8') as f:
            f.write("=" * 70 + "\n")
            f.write("模型评估报告\n")
            f.write("=" * 70 + "\n\n")

            f.write(f"测试集样本数: {len(X_test)}\n\n")

            f.write("-" * 70 + "\n")
            f.write("场景分类性能\n")
            f.write("-" * 70 + "\n")
            f.write(f"准确率: {scene_accuracy*100:.2f}%\n\n")

            f.write("-" * 70 + "\n")
            f.write("阶段分类性能\n")
            f.write("-" * 70 + "\n")
            f.write(f"准确率: {phase_accuracy*100:.2f}%\n")

            # 计算每个阶段的准确率
            for label in range(self.阶段数):
                mask = y_phase_test == label
                if np.any(mask):
                    acc = np.mean(phase_pred_labels[mask] == label)
                    f.write(f"  阶段{label}准确率: {acc*100:.2f}%\n")

            f.write("\n" + "-" * 70 + "\n")
            f.write("参数预测性能\n")
            f.write("-" * 70 + "\n")
            f.write(f"delta_torque MAE: {params_mae[0]:.4f}\n")
            f.write(f"delta_kd MAE: {params_mae[1]:.4f}\n")
            f.write(f"delta_scale MAE: {params_mae[2]:.4f}\n")
            f.write(f"平均MAE: {np.mean(params_mae):.4f}\n")

        print(f"✓ 评估报告已保存: {输出文件}")


def 加载训练数据(npz_file):
    """
    加载训练数据NPZ文件

    参数:
        npz_file: NPZ文件路径

    返回:
        X, y_phase, y_params: 数据数组
    """
    print(f"\n加载训练数据: {npz_file}")
    data = np.load(npz_file, allow_pickle=True)

    print(f"NPZ文件包含键: {list(data.keys())}")

    X = data['X']
    y_phase = data['y_phase']
    y_params = data['y_params']

    print(f"✓ 数据加载完成:")
    print(f"  X shape: {X.shape}")
    print(f"  y_phase shape: {y_phase.shape}")
    print(f"  y_params shape: {y_params.shape}")

    return X, y_phase, y_params


def main():
    parser = argparse.ArgumentParser(description='AI模型训练')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入训练数据NPZ文件')
    parser.add_argument('--scene', '-s', type=str, default='平地',
                        choices=['平地', '爬楼'], help='场景名称（用于生成场景标签）')
    parser.add_argument('--epochs', '-e', type=int, default=50,
                        help='训练轮数 (默认: 50)')
    parser.add_argument('--batch-size', '-b', type=int, default=32,
                        help='批大小 (默认: 32)')
    parser.add_argument('--val-split', type=float, default=0.2,
                        help='验证集比例 (默认: 0.2)')
    parser.add_argument('--output', '-o', type=str, default='motion_ai_model.h5',
                        help='输出模型文件名')

    args = parser.parse_args()

    print("=" * 70)
    print("AI模型训练 - 运动阶段识别与参数调整")
    print("=" * 70)

    # 1. 加载数据
    X, y_phase, y_params = 加载训练数据(args.input)

    # 2. 生成场景标签（当前数据集来自单一场景）
    场景映射 = {'平地': 0, '爬楼': 1}
    场景标签 = 场景映射[args.scene]
    y_scene = np.full(len(X), 场景标签, dtype=np.int32)
    print(f"\n生成场景标签: {args.scene} (标签={场景标签})")

    # 3. 划分训练集和验证集
    print(f"\n划分数据集 (验证集比例={args.val_split})...")
    X_train, X_val, y_scene_train, y_scene_val, y_phase_train, y_phase_val, y_params_train, y_params_val = train_test_split(
        X, y_scene, y_phase, y_params,
        test_size=args.val_split,
        random_state=42,
        stratify=y_phase  # 按阶段分层采样
    )

    print(f"训练集: {len(X_train)} 样本")
    print(f"验证集: {len(X_val)} 样本")

    # 4. 构建模型
    窗口大小 = X.shape[1]
    模型 = 运动AI模型(窗口大小=窗口大小)
    模型.构建模型()

    # 5. 训练模型
    模型.训练模型(
        X_train, y_scene_train, y_phase_train, y_params_train,
        X_val, y_scene_val, y_phase_val, y_params_val,
        epochs=args.epochs,
        batch_size=args.batch_size
    )

    # 6. 保存模型
    模型.保存模型(输出文件=args.output)

    # 7. 可视化训练历史
    模型.可视化训练历史()

    # 8. 评估模型（使用验证集作为测试集）
    模型.评估模型(X_val, y_scene_val, y_phase_val, y_params_val)

    print("\n" + "=" * 70)
    print("✓ 模型训练完成！")
    print("=" * 70)
    print(f"\n生成文件:")
    print(f"  - {args.output}")
    print(f"  - {args.output.replace('.h5', '.weights.h5')}")
    print(f"  - training_history.png")
    print(f"  - model_evaluation.txt")
    print(f"  - best_model_checkpoint.h5")


if __name__ == '__main__':
    main()
