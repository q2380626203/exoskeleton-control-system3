#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双腿联合训练数据生成工具 - 方案A

功能：
1. 加载已标注的数据（包含m1和m2）
2. 生成滑动窗口训练数据（双腿联合输入）
3. 生成双腿的阶段标签
4. 生成参数调整标签
5. 保存为训练数据集（NPZ格式）

输入格式：
- 输入：[50, 4] = [m1_vel, m1_pos, m2_vel, m2_pos] per timestep
- 输出：场景、m1阶段、m2阶段、参数调整

使用方法：
python 5_训练数据生成.py --input 平地数据_已修正.csv --scene 平地
"""

import pandas as pd
import numpy as np
import argparse
from pathlib import Path


class 双腿联合训练数据生成器:
    """
    生成双腿联合AI模型训练数据
    """

    IDLE = 0
    LIFTING = 1
    PRESSING = 2

    def __init__(self, df, 窗口大小=50, 步长=5):
        """
        初始化生成器

        参数:
            df: 已标注的DataFrame（包含m1和m2列）
            窗口大小: 滑动窗口大小（采样点数）
            步长: 滑动窗口步长
        """
        self.df = df
        self.窗口大小 = 窗口大小
        self.步长 = 步长

    def 生成滑动窗口(self):
        """
        生成双腿联合滑动窗口训练数据（仅使用速度）

        返回:
            X: 特征数组 (N, 窗口大小, 2) - [m1_vel, m2_vel]
            y_m1_phase: m1阶段标签 (N,)
            y_m2_phase: m2阶段标签 (N,)
        """
        m1_vel = self.df['m1_vel'].values
        m2_vel = self.df['m2_vel'].values
        m1_label = self.df['m1_label'].values
        m2_label = self.df['m2_label'].values

        X = []
        y_m1_phase = []
        y_m2_phase = []

        # 滑动窗口
        for i in range(self.窗口大小, len(m1_vel), self.步长):
            # 构建窗口: 每个时间步包含 [m1_vel, m2_vel]
            窗口 = np.zeros((self.窗口大小, 2), dtype=np.float32)
            for j in range(self.窗口大小):
                idx = i - self.窗口大小 + j
                窗口[j, 0] = m1_vel[idx]  # m1速度
                窗口[j, 1] = m2_vel[idx]  # m2速度

            # 使用窗口最后一个点的标签
            当前m1标签 = m1_label[i - 1]
            当前m2标签 = m2_label[i - 1]

            X.append(窗口)
            y_m1_phase.append(当前m1标签)
            y_m2_phase.append(当前m2标签)

        X = np.array(X, dtype=np.float32)
        y_m1_phase = np.array(y_m1_phase, dtype=np.int32)
        y_m2_phase = np.array(y_m2_phase, dtype=np.int32)

        print(f"\n生成双腿联合滑动窗口数据（仅速度）:")
        print(f"  窗口大小: {self.窗口大小}")
        print(f"  步长: {self.步长}")
        print(f"  样本数: {len(X)}")
        print(f"  特征维度: {X.shape} (窗口大小, 2通道)")
        print(f"  数据格式: [m1_vel, m2_vel]")
        print(f"  m1阶段标签: {y_m1_phase.shape}")
        print(f"  m2阶段标签: {y_m2_phase.shape}")

        # 打印阶段分布
        阶段名称 = {0: '静止', 1: '抬腿', 2: '压腿'}
        print(f"\nm1阶段分布:")
        for label in range(3):
            count = np.sum(y_m1_phase == label)
            percentage = count / len(y_m1_phase) * 100
            print(f"  {阶段名称[label]:8s}: {count:6d} ({percentage:5.1f}%)")

        print(f"\nm2阶段分布:")
        for label in range(3):
            count = np.sum(y_m2_phase == label)
            percentage = count / len(y_m2_phase) * 100
            print(f"  {阶段名称[label]:8s}: {count:6d} ({percentage:5.1f}%)")

        return X, y_m1_phase, y_m2_phase

    def 生成参数标签(self, 理想曲线文件=None):
        """
        生成参数调整标签

        思路：
        - 对于每个时间点，找到对应阶段的理想速度
        - 计算实时速度与理想速度的偏差
        - 根据偏差生成参数调整量

        参数:
            理想曲线文件: 理想曲线NPZ文件（可选）

        返回:
            y_params: 参数调整标签 (N, 3) [delta_torque, delta_kd, delta_scale]
        """
        m1_vel = self.df['m1_vel'].values
        m1_label = self.df['m1_label'].values

        # 如果有理想曲线文件，加载
        理想曲线 = None
        if 理想曲线文件 and Path(理想曲线文件).exists():
            理想曲线 = np.load(理想曲线文件, allow_pickle=True)
            print(f"\n加载理想曲线: {理想曲线文件}")

        y_params = []

        for i in range(self.窗口大小, len(m1_vel), self.步长):
            当前速度 = m1_vel[i - 1]
            当前阶段 = m1_label[i - 1]

            # 默认参数调整量
            delta_torque = 0.0
            delta_kd = 0.0
            delta_scale = 0.0

            # 如果有理想曲线，计算偏差
            if 理想曲线 is not None and 当前阶段 in [self.LIFTING, self.PRESSING]:
                # 获取理想速度（这里简化处理，实际需要根据阶段进度）
                阶段映射 = {
                    self.LIFTING: 'lifting',
                    self.PRESSING: 'pressing'
                }
                阶段名 = 阶段映射.get(当前阶段)

                if 阶段名 and 阶段名 in 理想曲线:
                    理想速度序列 = 理想曲线[阶段名]
                    # 简化：使用中点速度作为参考
                    理想速度 = 理想速度序列[len(理想速度序列) // 2]

                    # 计算偏差
                    速度偏差 = 理想速度 - 当前速度
                    偏差比例 = 速度偏差 / (abs(理想速度) + 0.1)  # 归一化

                    # 生成调整量（归一化到[-1, 1]）
                    delta_torque = np.clip(偏差比例 * 0.5, -1.0, 1.0)
                    delta_kd = np.clip(abs(偏差比例) * 0.3, -1.0, 1.0)
                    delta_scale = np.clip(偏差比例 * 0.3, -1.0, 1.0)

            y_params.append([delta_torque, delta_kd, delta_scale])

        y_params = np.array(y_params, dtype=np.float32)

        print(f"\n生成参数调整标签:")
        print(f"  样本数: {len(y_params)}")
        print(f"  维度: {y_params.shape}")
        print(f"  delta_torque 范围: [{y_params[:, 0].min():.3f}, {y_params[:, 0].max():.3f}]")
        print(f"  delta_kd 范围: [{y_params[:, 1].min():.3f}, {y_params[:, 1].max():.3f}]")
        print(f"  delta_scale 范围: [{y_params[:, 2].min():.3f}, {y_params[:, 2].max():.3f}]")

        return y_params

    def 保存训练数据(self, X, y_m1_phase, y_m2_phase, y_params, 场景名称, 输出文件=None):
        """
        保存训练数据为NPZ格式

        参数:
            X: 特征数组 (N, 窗口大小, 4)
            y_m1_phase: m1阶段标签
            y_m2_phase: m2阶段标签
            y_params: 参数标签
            场景名称: 场景名称
            输出文件: 输出文件名
        """
        if 输出文件 is None:
            输出文件 = f'训练数据_{场景名称}_双腿.npz'

        np.savez_compressed(
            输出文件,
            X=X,
            y_m1_phase=y_m1_phase,
            y_m2_phase=y_m2_phase,
            y_params=y_params,
            model_type='dual_leg_joint',
            scene=场景名称,
            window_size=self.窗口大小,
            step=self.步长
        )

        print(f"\n✓ 双腿联合训练数据已保存: {输出文件}")
        print(f"  文件大小: {Path(输出文件).stat().st_size / 1024:.1f} KB")
        print(f"  数据格式说明:")
        print(f"    X: {X.shape} - (样本数, 窗口大小, 2通道[m1_vel, m2_vel])")
        print(f"    y_m1_phase: {y_m1_phase.shape} - m1阶段标签")
        print(f"    y_m2_phase: {y_m2_phase.shape} - m2阶段标签")
        print(f"    y_params: {y_params.shape} - 参数调整标签")


def 生成统计报告(X, y_m1_phase, y_m2_phase, y_params, 输出文件='双腿训练数据统计.txt'):
    """
    生成双腿联合训练数据统计报告

    参数:
        X: 特征数组
        y_m1_phase: m1阶段标签
        y_m2_phase: m2阶段标签
        y_params: 参数标签
        输出文件: 输出文本文件
    """
    with open(输出文件, 'w', encoding='utf-8') as f:
        f.write("=" * 70 + "\n")
        f.write("双腿联合训练数据统计报告\n")
        f.write("=" * 70 + "\n\n")

        f.write(f"总样本数: {len(X)}\n")
        f.write(f"特征维度: {X.shape}\n")
        f.write(f"  窗口大小: {X.shape[1]}\n")
        f.write(f"  通道数: {X.shape[2]} (m1_vel, m1_pos, m2_vel, m2_pos)\n\n")

        # m1阶段分布
        f.write("-" * 70 + "\n")
        f.write("m1阶段标签分布\n")
        f.write("-" * 70 + "\n")
        阶段名称 = {0: '静止', 1: '抬腿', 2: '压腿'}
        for label in range(3):
            count = np.sum(y_m1_phase == label)
            percentage = count / len(y_m1_phase) * 100
            f.write(f"{阶段名称[label]:8s}: {count:6d} ({percentage:5.1f}%)\n")

        # m2阶段分布
        f.write("\n" + "-" * 70 + "\n")
        f.write("m2阶段标签分布\n")
        f.write("-" * 70 + "\n")
        for label in range(3):
            count = np.sum(y_m2_phase == label)
            percentage = count / len(y_m2_phase) * 100
            f.write(f"{阶段名称[label]:8s}: {count:6d} ({percentage:5.1f}%)\n")

        # 双腿联合状态分布
        f.write("\n" + "-" * 70 + "\n")
        f.write("双腿联合状态分布（验证腿部协调性）\n")
        f.write("-" * 70 + "\n")
        状态计数 = {}
        for m1, m2 in zip(y_m1_phase, y_m2_phase):
            状态 = (m1, m2)
            状态计数[状态] = 状态计数.get(状态, 0) + 1

        f.write(f"{'m1状态':<10} {'m2状态':<10} {'样本数':<10} {'百分比'}\n")
        for (m1, m2), count in sorted(状态计数.items()):
            percentage = count / len(y_m1_phase) * 100
            f.write(f"{阶段名称[m1]:<10} {阶段名称[m2]:<10} {count:<10} {percentage:5.1f}%\n")

        # 参数统计
        f.write("\n" + "-" * 70 + "\n")
        f.write("参数调整标签统计\n")
        f.write("-" * 70 + "\n")
        f.write(f"delta_torque:\n")
        f.write(f"  均值: {np.mean(y_params[:, 0]):7.3f}\n")
        f.write(f"  标准差: {np.std(y_params[:, 0]):7.3f}\n")
        f.write(f"  范围: [{np.min(y_params[:, 0]):7.3f}, {np.max(y_params[:, 0]):7.3f}]\n\n")

        f.write(f"delta_kd:\n")
        f.write(f"  均值: {np.mean(y_params[:, 1]):7.3f}\n")
        f.write(f"  标准差: {np.std(y_params[:, 1]):7.3f}\n")
        f.write(f"  范围: [{np.min(y_params[:, 1]):7.3f}, {np.max(y_params[:, 1]):7.3f}]\n\n")

        f.write(f"delta_scale:\n")
        f.write(f"  均值: {np.mean(y_params[:, 2]):7.3f}\n")
        f.write(f"  标准差: {np.std(y_params[:, 2]):7.3f}\n")
        f.write(f"  范围: [{np.min(y_params[:, 2]):7.3f}, {np.max(y_params[:, 2]):7.3f}]\n")

    print(f"\n✓ 统计报告已保存: {输出文件}")


def main():
    parser = argparse.ArgumentParser(description='双腿联合训练数据生成工具')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入已标注的CSV文件（包含m1和m2数据）')
    parser.add_argument('--scene', '-s', type=str, required=True,
                        choices=['平地', '爬楼'], help='场景名称')
    parser.add_argument('--window', '-w', type=int, default=50,
                        help='滑动窗口大小 (默认: 50)')
    parser.add_argument('--step', type=int, default=5,
                        help='滑动窗口步长 (默认: 5)')
    parser.add_argument('--ideal-curve', type=str, default=None,
                        help='理想曲线NPZ文件（可选）')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出NPZ文件名')

    args = parser.parse_args()

    print("=" * 70)
    print("双腿联合训练数据生成工具 - 方案A")
    print("=" * 70)

    # 加载数据
    print(f"\n加载数据: {args.input}")
    df = pd.read_csv(args.input)
    print(f"✓ 加载完成: {len(df)} 行")

    # 验证必要的列存在
    required_columns = ['m1_vel', 'm2_vel', 'm1_label', 'm2_label']
    missing_columns = [col for col in required_columns if col not in df.columns]
    if missing_columns:
        print(f"\n⚠ 错误: 缺少必要的列: {missing_columns}")
        print(f"  当前列: {list(df.columns)}")
        return

    # 创建生成器
    生成器 = 双腿联合训练数据生成器(
        df,
        窗口大小=args.window,
        步长=args.step
    )

    # 生成滑动窗口
    print("\n生成双腿联合滑动窗口数据...")
    X, y_m1_phase, y_m2_phase = 生成器.生成滑动窗口()

    # 生成参数标签
    print("\n生成参数调整标签...")
    y_params = 生成器.生成参数标签(理想曲线文件=args.ideal_curve)

    # 保存训练数据
    print("\n保存双腿联合训练数据...")
    生成器.保存训练数据(X, y_m1_phase, y_m2_phase, y_params, args.scene, 输出文件=args.output)

    print("\n" + "=" * 70)
    print("✓ 双腿联合训练数据生成完成！")
    print("=" * 70)
    print(f"\n后续步骤:")
    print(f"  1. 使用生成的训练数据训练双腿联合模型:")
    print(f"     python 模型训练/1_模型训练.py --input {args.output or f'训练数据_{args.scene}_双腿.npz'}")
    print(f"  2. 转换模型为TFLite格式:")
    print(f"     python 模型训练/2_模型转换.py --input motion_ai_model.h5 --data {args.output or f'训练数据_{args.scene}_双腿.npz'}")


if __name__ == '__main__':
    main()
