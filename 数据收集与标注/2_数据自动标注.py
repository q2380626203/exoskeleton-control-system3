#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据自动标注工具（增强版）

功能：
1. 加载CSV数据（包含位置和速度信息）
2. 自动标注运动阶段 (静止/抬腿/过渡/压腿)
3. 基于位置和速度双重判断
4. 区分左腿(m1)和右腿(m2)
5. 可视化标注结果（位置+速度）
6. 保存标注后的数据

使用方法：
python 2_数据自动标注.py --input 平地数据.csv --output 平地数据_已标注.csv
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

class 运动阶段标注器:
    """
    运动阶段自动标注器（基于位置和速度）

    电机定义：
    - m1: 左腿
    - m2: 右腿

    阶段定义：
    0 = IDLE (静止)：速度绝对值 < 阈值，位置变化小
    1 = LIFTING (抬腿)：速度正向增加 & 位置上升
    2 = TRANSITION (过渡)：速度从峰值下降到0附近 & 位置平稳
    3 = PRESSING (压腿)：速度负向增加 & 位置下降
    """

    # 阶段编码
    IDLE = 0
    LIFTING = 1
    TRANSITION = 2
    PRESSING = 3

    阶段名称 = {
        0: '静止',
        1: '抬腿',
        2: '过渡',
        3: '压腿'
    }

    阶段颜色 = {
        0: 'gray',
        1: 'green',
        2: 'orange',
        3: 'red'
    }

    电机名称 = {
        'm1': '左腿',
        'm2': '右腿'
    }

    def __init__(self, 静止速度阈值=2.0, 静止位置阈值=0.05, 平滑窗口=5):
        """
        初始化标注器

        参数:
            静止速度阈值: 速度绝对值小于此值视为静止 (rad/s)
            静止位置阈值: 位置变化小于此值视为静止 (rad)
            平滑窗口: 用于平滑速度曲线的窗口大小
        """
        self.静止速度阈值 = 静止速度阈值
        self.静止位置阈值 = 静止位置阈值
        self.平滑窗口 = 平滑窗口

    def 标注单个电机(self, df, 速度列='m1_vel', 位置列='m1_pos'):
        """
        自动标注单个电机的运动阶段（基于速度和位置）

        参数:
            df: DataFrame
            速度列: 速度数据的列名
            位置列: 位置数据的列名

        返回:
            标签数组 (numpy array)
        """
        vel = df[速度列].values
        pos = df[位置列].values
        标签 = np.zeros(len(vel), dtype=int)

        # 速度平滑处理
        速度平滑 = pd.Series(vel).rolling(
            window=self.平滑窗口,
            center=True,
            min_periods=1
        ).mean().values

        # 位置变化率（差分）
        位置变化 = np.gradient(pos)

        # 计算加速度（速度变化率）
        加速度 = np.gradient(速度平滑)

        # 逐点标注
        for i in range(len(vel)):
            v = 速度平滑[i]
            p_delta = 位置变化[i]
            a = 加速度[i]

            # 规则1: 静止判断（速度和位置变化都小）
            if abs(v) < self.静止速度阈值 and abs(p_delta) < self.静止位置阈值:
                标签[i] = self.IDLE

            # 规则2: 抬腿判断（速度正向 & 加速 & 位置上升）
            elif v > self.静止速度阈值 and a > 0.05 and p_delta > 0:
                标签[i] = self.LIFTING

            # 规则3: 过渡判断（速度正向但减速 & 位置变化小）
            elif v > self.静止速度阈值 and a <= 0.05 and abs(p_delta) < 0.01:
                标签[i] = self.TRANSITION

            # 规则4: 压腿判断（速度负向 & 位置下降）
            elif v < -self.静止速度阈值 and p_delta < 0:
                标签[i] = self.PRESSING

            # 规则5: 负速度但位置变化小（也算过渡）
            elif v < 0 and abs(p_delta) < 0.01:
                标签[i] = self.TRANSITION

            # 其他情况：保持上一状态或标为静止
            else:
                if i > 0:
                    标签[i] = 标签[i-1]
                else:
                    标签[i] = self.IDLE

        return 标签

    def 后处理优化(self, 标签, 最小持续=10):
        """
        后处理：去除过短的阶段片段

        参数:
            标签: 原始标签数组
            最小持续: 阶段最小持续点数

        返回:
            优化后的标签数组
        """
        优化标签 = 标签.copy()
        当前阶段 = 标签[0]
        阶段起始 = 0

        for i in range(1, len(标签)):
            if 标签[i] != 当前阶段:
                # 阶段变化
                阶段长度 = i - 阶段起始

                if 阶段长度 < 最小持续:
                    # 太短，合并到前一个阶段
                    if 阶段起始 > 0:
                        优化标签[阶段起始:i] = 优化标签[阶段起始-1]

                # 更新当前阶段
                当前阶段 = 标签[i]
                阶段起始 = i

        return 优化标签

    def 标注数据(self, df):
        """
        对整个数据集标注（左腿和右腿）

        参数:
            df: 原始DataFrame

        返回:
            添加了标签列的DataFrame
        """
        print("开始自动标注...")

        # 标注左腿(m1)
        print(f"  - 标注左腿(m1)...")
        标签1 = self.标注单个电机(df, 速度列='m1_vel', 位置列='m1_pos')
        标签1 = self.后处理优化(标签1)

        # 标注右腿(m2)
        print(f"  - 标注右腿(m2)...")
        标签2 = self.标注单个电机(df, 速度列='m2_vel', 位置列='m2_pos')
        标签2 = self.后处理优化(标签2)

        # 添加标签列
        df['m1_label'] = 标签1
        df['m2_label'] = 标签2

        # 统计
        print("\n标注统计 (左腿 m1):")
        for label, name in self.阶段名称.items():
            count = np.sum(标签1 == label)
            percentage = count / len(标签1) * 100
            print(f"  {name}: {count} ({percentage:.1f}%)")

        print("\n标注统计 (右腿 m2):")
        for label, name in self.阶段名称.items():
            count = np.sum(标签2 == label)
            percentage = count / len(标签2) * 100
            print(f"  {name}: {count} ({percentage:.1f}%)")

        return df


def 可视化标注结果(df, 输出文件='标注结果可视化.png'):
    """
    可视化标注结果（位置+速度双图）

    参数:
        df: 已标注的DataFrame
        输出文件: 输出图片文件名
    """
    print(f"\n生成可视化图表: {输出文件}")

    fig, axes = plt.subplots(4, 2, figsize=(20, 14), sharex=True)

    时间 = df['timestamp'] / 1000  # 转换为秒
    标注器 = 运动阶段标注器()

    # ========== 左列：左腿(m1) ==========
    # 图1: 左腿位置曲线
    ax_m1_pos = axes[0, 0]
    ax_m1_pos.plot(时间, df['m1_pos'], 'b-', linewidth=0.8, alpha=0.7)
    ax_m1_pos.set_ylabel('位置 (rad)', fontsize=12)
    ax_m1_pos.set_title('左腿(m1) 位置曲线', fontsize=14, fontweight='bold')
    ax_m1_pos.grid(True, alpha=0.3)

    # 图2: 左腿速度曲线
    ax_m1_vel = axes[1, 0]
    ax_m1_vel.plot(时间, df['m1_vel'], 'g-', linewidth=0.8, alpha=0.7)
    ax_m1_vel.set_ylabel('速度 (rad/s)', fontsize=12)
    ax_m1_vel.set_title('左腿(m1) 速度曲线', fontsize=14, fontweight='bold')
    ax_m1_vel.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax_m1_vel.grid(True, alpha=0.3)

    # 图3: 左腿标注结果（彩色散点）
    ax_m1_label = axes[2, 0]
    for label, name in 标注器.阶段名称.items():
        mask = df['m1_label'] == label
        ax_m1_label.scatter(时间[mask], df.loc[mask, 'm1_vel'],
                           c=标注器.阶段颜色[label], s=3, label=name, alpha=0.7)
    ax_m1_label.set_ylabel('速度 (rad/s)', fontsize=12)
    ax_m1_label.set_title('左腿(m1) 标注结果', fontsize=14, fontweight='bold')
    ax_m1_label.legend(loc='upper right', fontsize=10)
    ax_m1_label.grid(True, alpha=0.3)

    # 图4: 左腿阶段时序
    ax_m1_timeline = axes[3, 0]
    ax_m1_timeline.plot(时间, df['m1_label'], 'k-', linewidth=1.0)
    ax_m1_timeline.set_ylabel('阶段', fontsize=12)
    ax_m1_timeline.set_xlabel('时间 (秒)', fontsize=12)
    ax_m1_timeline.set_yticks([0, 1, 2, 3])
    ax_m1_timeline.set_yticklabels(['静止', '抬腿', '过渡', '压腿'])
    ax_m1_timeline.set_title('左腿(m1) 阶段时序', fontsize=12)
    ax_m1_timeline.grid(True, alpha=0.3)

    # ========== 右列：右腿(m2) ==========
    # 图5: 右腿位置曲线
    ax_m2_pos = axes[0, 1]
    ax_m2_pos.plot(时间, df['m2_pos'], 'b-', linewidth=0.8, alpha=0.7)
    ax_m2_pos.set_ylabel('位置 (rad)', fontsize=12)
    ax_m2_pos.set_title('右腿(m2) 位置曲线', fontsize=14, fontweight='bold')
    ax_m2_pos.grid(True, alpha=0.3)

    # 图6: 右腿速度曲线
    ax_m2_vel = axes[1, 1]
    ax_m2_vel.plot(时间, df['m2_vel'], 'r-', linewidth=0.8, alpha=0.7)
    ax_m2_vel.set_ylabel('速度 (rad/s)', fontsize=12)
    ax_m2_vel.set_title('右腿(m2) 速度曲线', fontsize=14, fontweight='bold')
    ax_m2_vel.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax_m2_vel.grid(True, alpha=0.3)

    # 图7: 右腿标注结果（彩色散点）
    ax_m2_label = axes[2, 1]
    for label, name in 标注器.阶段名称.items():
        mask = df['m2_label'] == label
        ax_m2_label.scatter(时间[mask], df.loc[mask, 'm2_vel'],
                           c=标注器.阶段颜色[label], s=3, label=name, alpha=0.7)
    ax_m2_label.set_ylabel('速度 (rad/s)', fontsize=12)
    ax_m2_label.set_title('右腿(m2) 标注结果', fontsize=14, fontweight='bold')
    ax_m2_label.legend(loc='upper right', fontsize=10)
    ax_m2_label.grid(True, alpha=0.3)

    # 图8: 右腿阶段时序
    ax_m2_timeline = axes[3, 1]
    ax_m2_timeline.plot(时间, df['m2_label'], 'k-', linewidth=1.0)
    ax_m2_timeline.set_ylabel('阶段', fontsize=12)
    ax_m2_timeline.set_xlabel('时间 (秒)', fontsize=12)
    ax_m2_timeline.set_yticks([0, 1, 2, 3])
    ax_m2_timeline.set_yticklabels(['静止', '抬腿', '过渡', '压腿'])
    ax_m2_timeline.set_title('右腿(m2) 阶段时序', fontsize=12)
    ax_m2_timeline.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(输出文件, dpi=150, bbox_inches='tight')
    print(f"✓ 可视化图表已保存: {输出文件}")

    # 显示图表（可选）
    # plt.show()


def 生成统计报告(df, 输出文件='标注统计报告.txt'):
    """
    生成标注统计报告

    参数:
        df: 已标注的DataFrame
        输出文件: 输出文本文件名
    """
    标注器 = 运动阶段标注器()

    with open(输出文件, 'w', encoding='utf-8') as f:
        f.write("=" * 60 + "\n")
        f.write("运动阶段标注统计报告\n")
        f.write("=" * 60 + "\n\n")

        f.write(f"数据文件: {输出文件}\n")
        f.write(f"总样本数: {len(df)}\n")
        f.write(f"总时长: {df['timestamp'].iloc[-1] / 1000:.1f} 秒\n")
        f.write(f"采样率: {len(df) / (df['timestamp'].iloc[-1] / 1000):.1f} Hz\n\n")

        # 左腿统计
        f.write("-" * 60 + "\n")
        f.write("左腿(m1) 阶段统计\n")
        f.write("-" * 60 + "\n")
        for label, name in 标注器.阶段名称.items():
            count = np.sum(df['m1_label'] == label)
            percentage = count / len(df) * 100
            duration = count / (len(df) / (df['timestamp'].iloc[-1] / 1000))
            f.write(f"{name:8s}: {count:6d} 样本 ({percentage:5.1f}%) | 时长: {duration:6.1f}秒\n")

        # 右腿统计
        f.write("\n" + "-" * 60 + "\n")
        f.write("右腿(m2) 阶段统计\n")
        f.write("-" * 60 + "\n")
        for label, name in 标注器.阶段名称.items():
            count = np.sum(df['m2_label'] == label)
            percentage = count / len(df) * 100
            duration = count / (len(df) / (df['timestamp'].iloc[-1] / 1000))
            f.write(f"{name:8s}: {count:6d} 样本 ({percentage:5.1f}%) | 时长: {duration:6.1f}秒\n")

        # 周期统计
        f.write("\n" + "-" * 60 + "\n")
        f.write("运动周期统计\n")
        f.write("-" * 60 + "\n")

        # 找左腿的周期数
        m1_labels = df['m1_label'].values
        周期数_m1 = 0
        上一个标签 = m1_labels[0]
        for i in range(1, len(m1_labels)):
            if 上一个标签 != 标注器.IDLE and m1_labels[i] == 标注器.IDLE:
                周期数_m1 += 1
            上一个标签 = m1_labels[i]

        # 找右腿的周期数
        m2_labels = df['m2_label'].values
        周期数_m2 = 0
        上一个标签 = m2_labels[0]
        for i in range(1, len(m2_labels)):
            if 上一个标签 != 标注器.IDLE and m2_labels[i] == 标注器.IDLE:
                周期数_m2 += 1
            上一个标签 = m2_labels[i]

        f.write(f"左腿运动周期数: {周期数_m1}\n")
        f.write(f"右腿运动周期数: {周期数_m2}\n")

        if 周期数_m1 > 0:
            平均周期_m1 = (df['timestamp'].iloc[-1] / 1000) / 周期数_m1
            f.write(f"左腿平均周期: {平均周期_m1:.2f} 秒\n")
        if 周期数_m2 > 0:
            平均周期_m2 = (df['timestamp'].iloc[-1] / 1000) / 周期数_m2
            f.write(f"右腿平均周期: {平均周期_m2:.2f} 秒\n")

    print(f"✓ 统计报告已保存: {输出文件}")


def main():
    parser = argparse.ArgumentParser(description='运动阶段自动标注工具（增强版）')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入CSV文件')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出CSV文件 (默认: 输入文件名_已标注.csv)')
    parser.add_argument('--vel-threshold', type=float, default=2.0,
                        help='静止速度阈值 (默认: 2.0 rad/s)')
    parser.add_argument('--pos-threshold', type=float, default=0.05,
                        help='静止位置阈值 (默认: 0.05 rad)')
    parser.add_argument('--smooth', '-s', type=int, default=5,
                        help='平滑窗口大小 (默认: 5)')
    parser.add_argument('--no-viz', action='store_true',
                        help='不生成可视化图表')

    args = parser.parse_args()

    # 生成输出文件名
    if args.output is None:
        input_path = Path(args.input)
        args.output = str(input_path.parent / f"{input_path.stem}_已标注.csv")

    # 加载数据
    print(f"加载数据: {args.input}")
    df = pd.read_csv(args.input)
    print(f"✓ 加载完成: {len(df)} 行数据")

    # 检查必需列
    必需列 = ['timestamp', 'm1_pos', 'm1_vel', 'm2_pos', 'm2_vel']
    缺失列 = [col for col in 必需列 if col not in df.columns]
    if 缺失列:
        print(f"✗ 错误：缺少必需列: {缺失列}")
        print(f"当前列: {list(df.columns)}")
        return

    # 创建标注器
    标注器 = 运动阶段标注器(
        静止速度阈值=args.vel_threshold,
        静止位置阈值=args.pos_threshold,
        平滑窗口=args.smooth
    )

    # 自动标注
    df = 标注器.标注数据(df)

    # 保存标注结果
    print(f"\n保存标注结果: {args.output}")
    df.to_csv(args.output, index=False, encoding='utf-8')
    print(f"✓ 已保存")

    # 可视化
    if not args.no_viz:
        viz_file = str(Path(args.output).parent / f"{Path(args.output).stem}_可视化.png")
        可视化标注结果(df, 输出文件=viz_file)

    # 统计报告
    report_file = str(Path(args.output).parent / f"{Path(args.output).stem}_报告.txt")
    生成统计报告(df, 输出文件=report_file)

    print("\n✓ 标注完成！")


if __name__ == '__main__':
    main()
