#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
训练数据生成工具

功能：
1. 加载已标注的数据
2. 加载理想曲线
3. 生成滑动窗口训练数据
4. 生成参数调整标签（对比实时vs理想曲线）
5. 保存为训练数据集（NPZ格式）

使用方法：
python 5_训练数据生成.py --input 平地数据_已修正.csv --scene 平地
(或使用已标注文件: python 5_训练数据生成.py --input 平地数据_已标注.csv --scene 平地)
"""

import pandas as pd
import numpy as np
import argparse
from pathlib import Path


class 训练数据生成器:
    """
    生成AI模型训练数据
    """

    IDLE = 0
    LIFTING = 1
    PRESSING = 2

    def __init__(self, df, 电机='m1', 窗口大小=50, 步长=5):
        """
        初始化生成器

        参数:
            df: 已标注的DataFrame
            电机: 'm1' 或 'm2'
            窗口大小: 滑动窗口大小（采样点数）
            步长: 滑动窗口步长
        """
        self.df = df
        self.电机 = 电机
        self.速度列 = f'{电机}_vel'
        self.标签列 = f'{电机}_label'
        self.窗口大小 = 窗口大小
        self.步长 = 步长

    def 生成滑动窗口(self):
        """
        生成滑动窗口训练数据

        返回:
            X: 特征数组 (N, 窗口大小)
            y_phase: 阶段标签 (N,)
        """
        速度 = self.df[self.速度列].values
        标签 = self.df[self.标签列].values

        X = []
        y_phase = []

        # 滑动窗口
        for i in range(self.窗口大小, len(速度), self.步长):
            窗口 = 速度[i - self.窗口大小:i]
            当前标签 = 标签[i - 1]  # 使用窗口最后一个点的标签

            X.append(窗口)
            y_phase.append(当前标签)

        X = np.array(X, dtype=np.float32)
        y_phase = np.array(y_phase, dtype=np.int32)

        print(f"生成滑动窗口数据:")
        print(f"  窗口大小: {self.窗口大小}")
        print(f"  步长: {self.步长}")
        print(f"  样本数: {len(X)}")
        print(f"  特征维度: {X.shape}")

        return X, y_phase

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
        速度 = self.df[self.速度列].values
        标签 = self.df[self.标签列].values

        # 如果有理想曲线文件，加载
        理想曲线 = None
        if 理想曲线文件 and Path(理想曲线文件).exists():
            理想曲线 = np.load(理想曲线文件, allow_pickle=True)
            print(f"加载理想曲线: {理想曲线文件}")

        y_params = []

        for i in range(self.窗口大小, len(速度), self.步长):
            当前速度 = 速度[i - 1]
            当前阶段 = 标签[i - 1]

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

    def 保存训练数据(self, X, y_phase, y_params, 场景名称, 输出文件=None):
        """
        保存训练数据为NPZ格式

        参数:
            X: 特征数组
            y_phase: 阶段标签
            y_params: 参数标签
            场景名称: 场景名称
            输出文件: 输出文件名
        """
        if 输出文件 is None:
            输出文件 = f'训练数据_{场景名称}_{self.电机}.npz'

        np.savez_compressed(
            输出文件,
            X=X,
            y_phase=y_phase,
            y_params=y_params,
            motor=self.电机,
            scene=场景名称,
            window_size=self.窗口大小,
            step=self.步长
        )

        print(f"\n✓ 训练数据已保存: {输出文件}")
        print(f"  文件大小: {Path(输出文件).stat().st_size / 1024:.1f} KB")


def 生成统计报告(X, y_phase, y_params, 输出文件='训练数据统计.txt'):
    """
    生成训练数据统计报告

    参数:
        X: 特征数组
        y_phase: 阶段标签
        y_params: 参数标签
        输出文件: 输出文本文件
    """
    with open(输出文件, 'w', encoding='utf-8') as f:
        f.write("=" * 60 + "\n")
        f.write("训练数据统计报告\n")
        f.write("=" * 60 + "\n\n")

        f.write(f"总样本数: {len(X)}\n")
        f.write(f"特征维度: {X.shape}\n\n")

        # 阶段分布
        f.write("-" * 60 + "\n")
        f.write("阶段标签分布\n")
        f.write("-" * 60 + "\n")
        阶段名称 = {0: '静止', 1: '抬腿', 2: '压腿'}
        for label in range(3):
            count = np.sum(y_phase == label)
            percentage = count / len(y_phase) * 100
            f.write(f"{阶段名称[label]:8s}: {count:6d} ({percentage:5.1f}%)\n")

        # 参数统计
        f.write("\n" + "-" * 60 + "\n")
        f.write("参数调整标签统计\n")
        f.write("-" * 60 + "\n")
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

    print(f"✓ 统计报告已保存: {输出文件}")


def main():
    parser = argparse.ArgumentParser(description='训练数据生成工具')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入已标注的CSV文件')
    parser.add_argument('--motor', '-m', type=str, default='m1',
                        choices=['m1', 'm2'], help='选择电机')
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

    # 加载数据
    print(f"加载数据: {args.input}")
    df = pd.read_csv(args.input)
    print(f"✓ 加载完成: {len(df)} 行")

    # 创建生成器
    生成器 = 训练数据生成器(
        df,
        电机=args.motor,
        窗口大小=args.window,
        步长=args.step
    )

    # 生成滑动窗口
    print("\n生成滑动窗口数据...")
    X, y_phase = 生成器.生成滑动窗口()

    # 生成参数标签
    print("\n生成参数调整标签...")
    y_params = 生成器.生成参数标签(理想曲线文件=args.ideal_curve)

    # 保存训练数据
    print("\n保存训练数据...")
    生成器.保存训练数据(X, y_phase, y_params, args.scene, 输出文件=args.output)

    # 生成统计报告
    报告文件 = f'训练数据统计_{args.scene}_{args.motor}.txt'
    生成统计报告(X, y_phase, y_params, 输出文件=报告文件)

    print("\n✓ 训练数据生成完成！")


if __name__ == '__main__':
    main()
