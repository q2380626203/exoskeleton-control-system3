#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式标注修正工具（增强版）

功能：
1. 加载已自动标注的数据
2. 可视化位置+速度曲线和标注结果
3. 区分左腿(m1)和右腿(m2)
4. 交互式修正错误标注
5. 保存修正后的数据

使用方法：
python 3_交互式标注修正.py --input 平地数据_已标注.csv --leg 左腿
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import SpanSelector, Button, RadioButtons
import argparse
from pathlib import Path

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


class 交互式标注修正器:
    """
    交互式标注修正工具（增强版）

    操作说明：
    1. 点击单选按钮选择要修正为的标签
    2. 在速度图表上拖动鼠标框选需要修正的区域
    3. 点击"保存"按钮保存修正结果
    4. 点击"撤销"按钮撤销上一次修正
    5. 关闭窗口结束
    """

    阶段名称 = {
        0: '静止',
        1: '抬腿',
        2: '过渡',
        3: '压腿'
    }

    阶段颜色 = {
        0: 'gray',
        1: 'green',
        2: '过渡',
        3: 'red'
    }

    腿部映射 = {
        '左腿': 'm1',
        '右腿': 'm2'
    }

    def __init__(self, df, 腿部='左腿'):
        """
        初始化修正器

        参数:
            df: 已标注的DataFrame
            腿部: '左腿' 或 '右腿'
        """
        self.df = df.copy()
        self.原始df = df.copy()  # 备份
        self.腿部 = 腿部
        self.电机 = self.腿部映射[腿部]
        self.位置列 = f'{self.电机}_pos'
        self.速度列 = f'{self.电机}_vel'
        self.标签列 = f'{self.电机}_label'
        self.当前标签 = 0
        self.历史记录 = []  # 用于撤销

        # 创建UI
        self.创建界面()

    def 创建界面(self):
        """创建交互界面"""
        self.fig = plt.figure(figsize=(18, 12))
        self.fig.suptitle(f'{self.腿部}({self.电机.upper()}) 运动阶段标注修正工具',
                         fontsize=16, fontweight='bold')

        gs = self.fig.add_gridspec(4, 4, hspace=0.35, wspace=0.3,
                                   left=0.15, right=0.98, top=0.95, bottom=0.05)

        # 图1: 位置曲线（顶部）
        self.ax_pos = self.fig.add_subplot(gs[0, :])
        self.ax_pos.set_title(f'{self.腿部} 位置曲线', fontsize=14, fontweight='bold')
        self.ax_pos.set_ylabel('位置 (rad)', fontsize=12)
        self.ax_pos.grid(True, alpha=0.3)

        # 图2: 速度曲线（第二行）
        self.ax_vel = self.fig.add_subplot(gs[1, :], sharex=self.ax_pos)
        self.ax_vel.set_title(f'{self.腿部} 速度曲线', fontsize=14, fontweight='bold')
        self.ax_vel.set_ylabel('速度 (rad/s)', fontsize=12)
        self.ax_vel.grid(True, alpha=0.3)

        # 图3: 标注结果（第三行，可拖动框选）
        self.ax_label = self.fig.add_subplot(gs[2, :], sharex=self.ax_pos)
        self.ax_label.set_title(f'{self.腿部} 标注结果（拖动框选修正）',
                               fontsize=14, fontweight='bold', color='blue')
        self.ax_label.set_ylabel('速度 (rad/s)', fontsize=12)
        self.ax_label.grid(True, alpha=0.3)

        # 图4: 阶段时序（底部）
        self.ax_timeline = self.fig.add_subplot(gs[3, :], sharex=self.ax_pos)
        self.ax_timeline.set_title('阶段时序', fontsize=12)
        self.ax_timeline.set_ylabel('阶段', fontsize=12)
        self.ax_timeline.set_xlabel('时间 (秒)', fontsize=12)
        self.ax_timeline.set_yticks([0, 1, 2, 3])
        self.ax_timeline.set_yticklabels(['静止', '抬腿', '过渡', '压腿'])
        self.ax_timeline.grid(True, alpha=0.3)

        # 控制面板（左侧）
        # 单选按钮：选择标签
        ax_radio = plt.axes([0.02, 0.6, 0.10, 0.25])
        ax_radio.set_title('选择标签', fontsize=10, fontweight='bold')
        self.radio = RadioButtons(
            ax_radio,
            ['0 静止', '1 抬腿', '2 过渡', '3 压腿'],
            active=0
        )
        self.radio.on_clicked(self.切换标签)

        # 保存按钮
        ax_save = plt.axes([0.02, 0.50, 0.10, 0.05])
        self.btn_save = Button(ax_save, '保存修正', color='lightgreen')
        self.btn_save.on_clicked(self.保存修正)

        # 撤销按钮
        ax_undo = plt.axes([0.02, 0.43, 0.10, 0.05])
        self.btn_undo = Button(ax_undo, '撤销', color='lightyellow')
        self.btn_undo.on_clicked(self.撤销)

        # 重置按钮
        ax_reset = plt.axes([0.02, 0.36, 0.10, 0.05])
        self.btn_reset = Button(ax_reset, '重置全部', color='lightcoral')
        self.btn_reset.on_clicked(self.重置)

        # 统计信息文本
        ax_info = plt.axes([0.02, 0.05, 0.10, 0.25])
        ax_info.axis('off')
        ax_info.set_title('统计信息', fontsize=10, fontweight='bold')
        self.info_text = ax_info.text(
            0, 0.95, '', transform=ax_info.transAxes,
            fontsize=9, verticalalignment='top', family='monospace'
        )

        # 框选工具（用于修正标注）
        self.span_selector = SpanSelector(
            self.ax_label,
            self.框选回调,
            'horizontal',
            useblit=True,
            props=dict(alpha=0.5, facecolor='blue'),
            interactive=True,
            drag_from_anywhere=True
        )

        # 绘制初始图表
        self.更新图表()
        self.更新统计信息()

    def 更新图表(self):
        """更新所有图表"""
        时间 = self.df['timestamp'].values / 1000

        # 清空图表
        self.ax_pos.clear()
        self.ax_vel.clear()
        self.ax_label.clear()
        self.ax_timeline.clear()

        # 图1: 位置曲线
        self.ax_pos.plot(时间, self.df[self.位置列], 'b-', linewidth=1.0, alpha=0.8)
        self.ax_pos.set_title(f'{self.腿部} 位置曲线', fontsize=14, fontweight='bold')
        self.ax_pos.set_ylabel('位置 (rad)', fontsize=12)
        self.ax_pos.grid(True, alpha=0.3)

        # 图2: 速度曲线
        self.ax_vel.plot(时间, self.df[self.速度列], 'g-', linewidth=1.0, alpha=0.8)
        self.ax_vel.set_title(f'{self.腿部} 速度曲线', fontsize=14, fontweight='bold')
        self.ax_vel.set_ylabel('速度 (rad/s)', fontsize=12)
        self.ax_vel.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.ax_vel.grid(True, alpha=0.3)

        # 图3: 标注结果（彩色散点）
        for label, name in self.阶段名称.items():
            mask = self.df[self.标签列] == label
            self.ax_label.scatter(
                时间[mask], self.df.loc[mask, self.速度列],
                c=self.阶段颜色[label], s=4, label=name, alpha=0.8
            )
        self.ax_label.set_title(f'{self.腿部} 标注结果（拖动框选修正）',
                               fontsize=14, fontweight='bold', color='blue')
        self.ax_label.set_ylabel('速度 (rad/s)', fontsize=12)
        self.ax_label.legend(loc='upper right', fontsize=10)
        self.ax_label.grid(True, alpha=0.3)

        # 图4: 时序图
        self.ax_timeline.plot(时间, self.df[self.标签列], 'k-', linewidth=1.2)
        # 用颜色填充不同阶段
        for label, color in self.阶段颜色.items():
            mask = self.df[self.标签列] == label
            self.ax_timeline.fill_between(
                时间, 0, 3,
                where=mask,
                alpha=0.2,
                color=color,
                step='mid'
            )
        self.ax_timeline.set_title('阶段时序', fontsize=12)
        self.ax_timeline.set_ylabel('阶段', fontsize=12)
        self.ax_timeline.set_xlabel('时间 (秒)', fontsize=12)
        self.ax_timeline.set_yticks([0, 1, 2, 3])
        self.ax_timeline.set_yticklabels(['静止', '抬腿', '过渡', '压腿'])
        self.ax_timeline.grid(True, alpha=0.3)

        self.fig.canvas.draw_idle()

    def 更新统计信息(self):
        """更新统计信息文本"""
        统计文本 = f"【{self.腿部}统计】\n\n"
        总样本 = len(self.df)

        for label, name in self.阶段名称.items():
            count = np.sum(self.df[self.标签列] == label)
            percentage = count / 总样本 * 100
            统计文本 += f"{name}:\n  {count}样本\n  {percentage:.1f}%\n\n"

        统计文本 += f"\n修正次数:\n  {len(self.历史记录)}"

        self.info_text.set_text(统计文本)
        self.fig.canvas.draw_idle()

    def 切换标签(self, label_text):
        """单选按钮回调：切换当前标签"""
        label_map = {
            '0 静止': 0,
            '1 抬腿': 1,
            '2 过渡': 2,
            '3 压腿': 3
        }
        self.当前标签 = label_map[label_text]
        print(f"当前选择: {self.阶段名称[self.当前标签]}")

    def 框选回调(self, xmin, xmax):
        """框选回调：修正选中区域的标签"""
        # 保存历史（用于撤销）
        self.历史记录.append(self.df[self.标签列].copy())

        # 找到时间范围内的索引
        时间 = self.df['timestamp'].values / 1000
        mask = (时间 >= xmin) & (时间 <= xmax)
        修正数量 = np.sum(mask)

        if 修正数量 > 0:
            # 修正标签
            self.df.loc[mask, self.标签列] = self.当前标签
            print(f"✓ 修正了 {修正数量} 个样本为 '{self.阶段名称[self.当前标签]}'")

            # 更新图表
            self.更新图表()
            self.更新统计信息()

    def 撤销(self, event):
        """撤销上一次修正"""
        if self.历史记录:
            self.df[self.标签列] = self.历史记录.pop()
            print("✓ 已撤销上一次修正")
            self.更新图表()
            self.更新统计信息()
        else:
            print("✗ 没有可撤销的操作")

    def 重置(self, event):
        """重置到原始标注"""
        self.df[self.标签列] = self.原始df[self.标签列].copy()
        self.历史记录.clear()
        print("✓ 已重置到原始标注")
        self.更新图表()
        self.更新统计信息()

    def 保存修正(self, event):
        """保存修正结果"""
        输出文件 = f'{self.腿部}_{self.电机}_修正后.csv'
        self.df.to_csv(输出文件, index=False, encoding='utf-8')
        print(f"✓ 修正结果已保存: {输出文件}")

    def 运行(self):
        """显示界面"""
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='交互式标注修正工具（增强版）')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入已标注的CSV文件')
    parser.add_argument('--leg', '-l', type=str, default='左腿',
                        choices=['左腿', '右腿'], help='选择腿部 (左腿/右腿)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出文件名（默认在交互界面中保存）')

    args = parser.parse_args()

    # 加载数据
    print(f"加载数据: {args.input}")
    df = pd.read_csv(args.input)
    print(f"✓ 加载完成: {len(df)} 行")

    # 检查必需列
    腿部映射 = {'左腿': 'm1', '右腿': 'm2'}
    电机 = 腿部映射[args.leg]
    必需列 = ['timestamp', f'{电机}_pos', f'{电机}_vel', f'{电机}_label']
    缺失列 = [col for col in 必需列 if col not in df.columns]
    if 缺失列:
        print(f"✗ 错误：缺少必需列: {缺失列}")
        print(f"当前列: {list(df.columns)}")
        return

    # 创建修正器
    修正器 = 交互式标注修正器(df, 腿部=args.leg)

    print(f"\n=== {args.leg}({电机.upper()}) 标注修正工具 ===")
    print("\n操作说明:")
    print("1. 点击左侧单选按钮选择要修正为的标签")
    print("2. 在第3个图表（标注结果）上拖动鼠标框选需要修正的区域")
    print("3. 点击'保存修正'按钮保存结果")
    print("4. 点击'撤销'按钮撤销上一次修正")
    print("5. 点击'重置全部'恢复到原始标注")
    print("6. 关闭窗口结束\n")

    # 运行
    修正器.运行()

    # 如果指定了输出文件，保存
    if args.output:
        修正器.df.to_csv(args.output, index=False, encoding='utf-8')
        print(f"✓ 最终结果已保存: {args.output}")


if __name__ == '__main__':
    main()
