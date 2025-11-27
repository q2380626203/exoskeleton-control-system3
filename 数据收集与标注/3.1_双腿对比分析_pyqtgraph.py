#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双腿对比分析工具（PyQtGraph高性能版）
Dual-Leg Comparison Analysis Tool (PyQtGraph Version - High Performance)

功能特性 (Features):
1. 加载自动标注数据 (Load auto-labeled data)
2. 在同一图表中对比左右腿数据 (Compare left and right leg data in same plots)
3. 按阶段颜色可视化位置+速度曲线 (Visualize position + velocity curves with phase colors)
4. 左右腿静止阶段用不同颜色区分 (Different colors for left/right leg idle phases)
5. 位置偏移调整功能（对齐两腿起始点） (Position offset adjustment to align legs)
6. GPU硬件加速渲染 (GPU-accelerated rendering)
7. 用于整体运动分析 (For overall motion analysis)

使用方法 (Usage):
python 3.1_双腿对比分析_pyqtgraph.py --input 平地数据_已标注.csv

依赖安装 (Dependencies):
pip install pyqtgraph PyQt5 pandas numpy

操作控制 (Controls):
- 左键拖动: 平移视图 (Left-click drag: Pan view)
- 右键拖动X轴: 缩放X轴 (Right-click drag X-axis: Zoom X-axis)
- 右键拖动Y轴: 缩放Y轴 (Right-click drag Y-axis: Zoom Y-axis)
- 鼠标滚轮: 同时缩放XY轴 (Mouse wheel: Zoom both axes)
- 调整偏移量: 对齐位置曲线 (Adjust offset: Align position curves)

颜色说明 (Color Legend):
- 静止: 灰色 | 抬腿: 绿色 | 压腿: 红色
- 左腿标记: 蓝色方块 | 右腿标记: 橙色三角
"""

import sys
import pandas as pd
import numpy as np
import argparse
from pathlib import Path
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# 启用抗锯齿
pg.setConfigOptions(antialias=True)
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


class DualLegAnalysisTool(QtWidgets.QWidget):
    """
    双腿对比分析工具（PyQtGraph版本）
    Dual-Leg Comparison Analysis Tool (PyQtGraph Version)

    性能优化 (Performance optimizations):
    - PyQtGraph的OpenGL硬件加速渲染
    - 快速图表更新，无卡顿
    - 流畅的缩放和平移
    """

    PHASE_NAMES = {
        0: '静止',
        1: '抬腿',
        2: '压腿'
    }

    # 统一配色方案（左右腿相同）
    PHASE_COLORS = {
        0: (128, 128, 128),  # 灰色 - 静止
        1: (0, 255, 0),      # 绿色 - 抬腿
        2: (255, 0, 0)       # 红色 - 压腿
    }

    # 左右腿标记颜色（用于曲线起始点标记）
    LEG_MARKER_COLORS = {
        'm1': (0, 0, 255),    # 蓝色 Blue - 左腿标记
        'm2': (255, 128, 0)   # 橙色 Orange - 右腿标记
    }

    def __init__(self, df):
        super().__init__()

        self.df = df.copy()

        # 预计算时间数组
        self.time_array = self.df['timestamp'].values / 1000

        # 位置偏移量（用于对齐两条曲线）
        self.m1_pos_offset = 0.0
        self.m2_pos_offset = 0.0

        self.init_ui()

    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle('双腿运动对比分析工具 - PyQtGraph')
        self.setGeometry(100, 100, 1800, 1000)

        # 主布局
        main_layout = QtWidgets.QVBoxLayout()

        # 标题
        title = QtWidgets.QLabel('双腿运动对比分析')
        title.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;")
        title.setAlignment(QtCore.Qt.AlignCenter)
        main_layout.addWidget(title)

        # 创建图表区域
        self.create_plots()
        main_layout.addWidget(self.plot_widget)

        # 底部统计面板
        stats_panel = self.create_stats_panel()
        main_layout.addWidget(stats_panel)

        self.setLayout(main_layout)

        # 绘制初始图表
        self.update_plots()
        self.update_stats()

    def create_plots(self):
        """创建图表区域"""
        # 创建图表布局控件
        self.plot_widget = pg.GraphicsLayoutWidget()

        # 图表1: 位置曲线对比
        self.plot_pos = self.plot_widget.addPlot(row=0, col=0, title='位置曲线对比 (方块=左腿, 三角=右腿)')
        self.plot_pos.setLabel('left', '位置 (rad)')
        self.plot_pos.setLabel('bottom', '时间 (秒)')
        self.plot_pos.showGrid(x=True, y=True, alpha=0.3)
        self.plot_pos.addLegend()

        # 图表2: 速度曲线对比
        self.plot_vel = self.plot_widget.addPlot(row=1, col=0, title='速度曲线对比 (方块=左腿, 三角=右腿)')
        self.plot_vel.setLabel('left', '速度 (rad/s)')
        self.plot_vel.setLabel('bottom', '时间 (秒)')
        self.plot_vel.showGrid(x=True, y=True, alpha=0.3)
        self.plot_vel.addLegend()

        # 图表3: 左腿阶段标注
        self.plot_m1_label = self.plot_widget.addPlot(row=2, col=0, title='左腿(m1) 阶段标注')
        self.plot_m1_label.setLabel('left', '速度 (rad/s)')
        self.plot_m1_label.setLabel('bottom', '时间 (秒)')
        self.plot_m1_label.showGrid(x=True, y=True, alpha=0.3)
        self.plot_m1_label.addLegend()

        # 图表4: 右腿阶段标注
        self.plot_m2_label = self.plot_widget.addPlot(row=3, col=0, title='右腿(m2) 阶段标注')
        self.plot_m2_label.setLabel('left', '速度 (rad/s)')
        self.plot_m2_label.setLabel('bottom', '时间 (秒)')
        self.plot_m2_label.showGrid(x=True, y=True, alpha=0.3)
        self.plot_m2_label.addLegend()

        # 链接X轴
        self.plot_vel.setXLink(self.plot_pos)
        self.plot_m1_label.setXLink(self.plot_pos)
        self.plot_m2_label.setXLink(self.plot_pos)

    def create_stats_panel(self):
        """创建底部统计面板"""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout()

        # 左腿统计
        m1_stats_group = QtWidgets.QGroupBox('左腿(m1)统计')
        m1_stats_layout = QtWidgets.QVBoxLayout()
        self.m1_stats_text = QtWidgets.QTextEdit()
        self.m1_stats_text.setReadOnly(True)
        self.m1_stats_text.setMaximumHeight(100)
        m1_stats_layout.addWidget(self.m1_stats_text)
        m1_stats_group.setLayout(m1_stats_layout)
        layout.addWidget(m1_stats_group)

        # 右腿统计
        m2_stats_group = QtWidgets.QGroupBox('右腿(m2)统计')
        m2_stats_layout = QtWidgets.QVBoxLayout()
        self.m2_stats_text = QtWidgets.QTextEdit()
        self.m2_stats_text.setReadOnly(True)
        self.m2_stats_text.setMaximumHeight(100)
        m2_stats_layout.addWidget(self.m2_stats_text)
        m2_stats_group.setLayout(m2_stats_layout)
        layout.addWidget(m2_stats_group)

        # 位置偏移控制
        offset_group = QtWidgets.QGroupBox('位置偏移调整')
        offset_layout = QtWidgets.QVBoxLayout()

        # 左腿偏移
        m1_offset_layout = QtWidgets.QHBoxLayout()
        m1_offset_layout.addWidget(QtWidgets.QLabel('左腿偏移:'))
        self.m1_offset_spinbox = QtWidgets.QDoubleSpinBox()
        self.m1_offset_spinbox.setRange(-20.0, 20.0)
        self.m1_offset_spinbox.setSingleStep(0.1)
        self.m1_offset_spinbox.setValue(0.0)
        self.m1_offset_spinbox.setDecimals(2)
        self.m1_offset_spinbox.valueChanged.connect(self.on_m1_offset_changed)
        m1_offset_layout.addWidget(self.m1_offset_spinbox)
        offset_layout.addLayout(m1_offset_layout)

        # 右腿偏移
        m2_offset_layout = QtWidgets.QHBoxLayout()
        m2_offset_layout.addWidget(QtWidgets.QLabel('右腿偏移:'))
        self.m2_offset_spinbox = QtWidgets.QDoubleSpinBox()
        self.m2_offset_spinbox.setRange(-20.0, 20.0)
        self.m2_offset_spinbox.setSingleStep(0.1)
        self.m2_offset_spinbox.setValue(0.0)
        self.m2_offset_spinbox.setDecimals(2)
        self.m2_offset_spinbox.valueChanged.connect(self.on_m2_offset_changed)
        m2_offset_layout.addWidget(self.m2_offset_spinbox)
        offset_layout.addLayout(m2_offset_layout)

        # 重置偏移按钮
        reset_offset_btn = QtWidgets.QPushButton('重置偏移')
        reset_offset_btn.clicked.connect(self.reset_offsets)
        reset_offset_btn.setStyleSheet("background-color: lightblue; padding: 5px;")
        offset_layout.addWidget(reset_offset_btn)

        offset_group.setLayout(offset_layout)
        layout.addWidget(offset_group)

        # 操作说明
        instructions_group = QtWidgets.QGroupBox('操作说明')
        instructions_layout = QtWidgets.QVBoxLayout()
        instructions = QtWidgets.QLabel(
            '1. 左键拖动: 平移视图\n'
            '2. 右键拖X轴: 缩放X轴\n'
            '3. 右键拖Y轴: 缩放Y轴\n'
            '4. 滚轮: 同时缩放XY轴\n'
            '5. 调整偏移: 对齐位置曲线'
        )
        instructions.setStyleSheet("padding: 5px;")
        instructions_layout.addWidget(instructions)
        instructions_group.setLayout(instructions_layout)
        layout.addWidget(instructions_group)

        panel.setLayout(layout)
        panel.setMaximumHeight(180)
        return panel

    def update_plots(self):
        """更新所有图表"""
        time = self.time_array

        # 清空图表
        self.plot_pos.clear()
        self.plot_vel.clear()
        self.plot_m1_label.clear()
        self.plot_m2_label.clear()

        # 获取标注数据
        m1_label_data = self.df['m1_label'].values
        m2_label_data = self.df['m2_label'].values

        # 获取位置和速度数据（应用偏移）
        m1_pos_data = self.df['m1_pos'].values + self.m1_pos_offset
        m2_pos_data = self.df['m2_pos'].values + self.m2_pos_offset
        m1_vel_data = self.df['m1_vel'].values
        m2_vel_data = self.df['m2_vel'].values

        # 图表1: 位置曲线对比（按阶段颜色的散点图）
        # 绘制左腿
        legend_added = {'m1': set(), 'm2': set()}  # 跟踪已添加的图例

        for label, name in self.PHASE_NAMES.items():
            mask = m1_label_data == label
            if np.any(mask):
                color = self.PHASE_COLORS[label]
                self.plot_pos.plot(
                    time[mask], m1_pos_data[mask],
                    pen=None,
                    symbol='o',
                    symbolSize=3,
                    symbolBrush=color,
                    symbolPen=None,
                    name=f'{name}' if label not in legend_added['m1'] else None
                )
                legend_added['m1'].add(label)

        # 绘制右腿
        for label, name in self.PHASE_NAMES.items():
            mask = m2_label_data == label
            if np.any(mask):
                color = self.PHASE_COLORS[label]
                self.plot_pos.plot(
                    time[mask], m2_pos_data[mask],
                    pen=None,
                    symbol='o',
                    symbolSize=3,
                    symbolBrush=color,
                    symbolPen=None,
                    name=None  # 右腿不重复添加图例
                )

        # 添加左右腿标记（在曲线起始点）
        # 左腿标记
        if len(m1_pos_data) > 0:
            self.plot_pos.plot(
                [time[0]], [m1_pos_data[0]],
                pen=None,
                symbol='s',
                symbolSize=10,
                symbolBrush=self.LEG_MARKER_COLORS['m1'],
                symbolPen=pg.mkPen('w', width=2),
                name='左腿(m1)'
            )

        # 右腿标记
        if len(m2_pos_data) > 0:
            self.plot_pos.plot(
                [time[0]], [m2_pos_data[0]],
                pen=None,
                symbol='t',
                symbolSize=10,
                symbolBrush=self.LEG_MARKER_COLORS['m2'],
                symbolPen=pg.mkPen('w', width=2),
                name='右腿(m2)'
            )

        # 图表2: 速度曲线对比（连续曲线，按阶段颜色分段）
        legend_added_vel = {'m1': set(), 'm2': set()}  # 跟踪速度图的图例

        # 绘制左腿
        for label, name in self.PHASE_NAMES.items():
            mask = m1_label_data == label
            if np.any(mask):
                # 找到连续区域
                regions = self.get_continuous_regions(mask)
                color = self.PHASE_COLORS[label]

                for start_idx, end_idx in regions:
                    self.plot_vel.plot(
                        time[start_idx:end_idx+1],
                        m1_vel_data[start_idx:end_idx+1],
                        pen=pg.mkPen(color, width=2),
                        name=f'{name}' if (label not in legend_added_vel['m1'] and regions.index((start_idx, end_idx)) == 0) else None
                    )
                legend_added_vel['m1'].add(label)

        # 绘制右腿
        for label, name in self.PHASE_NAMES.items():
            mask = m2_label_data == label
            if np.any(mask):
                # 找到连续区域
                regions = self.get_continuous_regions(mask)
                color = self.PHASE_COLORS[label]

                for start_idx, end_idx in regions:
                    self.plot_vel.plot(
                        time[start_idx:end_idx+1],
                        m2_vel_data[start_idx:end_idx+1],
                        pen=pg.mkPen(color, width=2),
                        name=None  # 右腿不重复添加图例
                    )

        self.plot_vel.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.DashLine))

        # 添加左右腿标记（在速度曲线起始点）
        # 左腿标记
        if len(m1_vel_data) > 0:
            self.plot_vel.plot(
                [time[0]], [m1_vel_data[0]],
                pen=None,
                symbol='s',
                symbolSize=10,
                symbolBrush=self.LEG_MARKER_COLORS['m1'],
                symbolPen=pg.mkPen('w', width=2),
                name='左腿(m1)'
            )

        # 右腿标记
        if len(m2_vel_data) > 0:
            self.plot_vel.plot(
                [time[0]], [m2_vel_data[0]],
                pen=None,
                symbol='t',
                symbolSize=10,
                symbolBrush=self.LEG_MARKER_COLORS['m2'],
                symbolPen=pg.mkPen('w', width=2),
                name='右腿(m2)'
            )

        # 图表3: 左腿阶段标注（按阶段颜色分段）
        for label, name in self.PHASE_NAMES.items():
            mask = m1_label_data == label
            if np.any(mask):
                color = self.PHASE_COLORS[label]
                self.plot_m1_label.plot(
                    time[mask], m1_vel_data[mask],
                    pen=None,
                    symbol='o',
                    symbolSize=3,
                    symbolBrush=color,
                    symbolPen=None,
                    name=name
                )
        self.plot_m1_label.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.DashLine))

        # 图表4: 右腿阶段标注（按阶段颜色分段）
        for label, name in self.PHASE_NAMES.items():
            mask = m2_label_data == label
            if np.any(mask):
                color = self.PHASE_COLORS[label]
                self.plot_m2_label.plot(
                    time[mask], m2_vel_data[mask],
                    pen=None,
                    symbol='o',
                    symbolSize=3,
                    symbolBrush=color,
                    symbolPen=None,
                    name=name
                )
        self.plot_m2_label.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.DashLine))

    def on_m1_offset_changed(self, value):
        """左腿偏移值改变"""
        self.m1_pos_offset = value
        self.update_plots()

    def on_m2_offset_changed(self, value):
        """右腿偏移值改变"""
        self.m2_pos_offset = value
        self.update_plots()

    def reset_offsets(self):
        """重置所有偏移"""
        self.m1_offset_spinbox.setValue(0.0)
        self.m2_offset_spinbox.setValue(0.0)
        self.m1_pos_offset = 0.0
        self.m2_pos_offset = 0.0
        self.update_plots()
        print("✓ 已重置所有位置偏移")

    def get_continuous_regions(self, mask):
        """获取掩码中连续的True区域"""
        regions = []
        in_region = False
        start_idx = 0

        for i, val in enumerate(mask):
            if val and not in_region:
                start_idx = i
                in_region = True
            elif not val and in_region:
                regions.append((start_idx, i - 1))
                in_region = False

        if in_region:
            regions.append((start_idx, len(mask) - 1))

        return regions

    def update_stats(self):
        """更新统计文本"""
        # 左腿统计
        m1_stats = "【左腿统计】\n\n"
        total_samples = len(self.df)
        m1_label_data = self.df['m1_label'].values

        for label, name in self.PHASE_NAMES.items():
            count = np.count_nonzero(m1_label_data == label)
            percentage = count / total_samples * 100
            m1_stats += f"{name}: {count} ({percentage:.1f}%)\n"

        self.m1_stats_text.setPlainText(m1_stats)

        # 右腿统计
        m2_stats = "【右腿统计】\n\n"
        m2_label_data = self.df['m2_label'].values

        for label, name in self.PHASE_NAMES.items():
            count = np.count_nonzero(m2_label_data == label)
            percentage = count / total_samples * 100
            m2_stats += f"{name}: {count} ({percentage:.1f}%)\n"

        self.m2_stats_text.setPlainText(m2_stats)


def main():
    parser = argparse.ArgumentParser(description='双腿对比分析工具（PyQtGraph高性能版）')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入已标注的CSV文件')

    args = parser.parse_args()

    # 加载数据
    print(f"加载数据: {args.input}")
    df = pd.read_csv(args.input)
    print(f"✓ 加载完成: {len(df)} 行")

    # 检查必需列
    required_cols = ['timestamp',
                     'm1_pos', 'm1_vel', 'm1_label',
                     'm2_pos', 'm2_vel', 'm2_label']
    missing_cols = [col for col in required_cols if col not in df.columns]
    if missing_cols:
        print(f"✗ 错误：缺少必需列: {missing_cols}")
        print(f"当前列: {list(df.columns)}")
        return

    # 创建应用程序
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')

    # 设置中文字体
    font = QtGui.QFont("Microsoft YaHei", 10)
    app.setFont(font)

    # 创建工具
    tool = DualLegAnalysisTool(df)

    print(f"\n=== 双腿运动对比分析工具 - PyQtGraph高性能版 ===")
    print(f"性能配置:")
    print(f"  - 后端: PyQtGraph (OpenGL硬件加速)")
    print(f"  - 数据点: {len(df)}")
    print(f"  - 抗锯齿: 已启用")
    print("\n图表说明:")
    print("1. 图1: 位置曲线对比 - 按阶段颜色显示（方块=左腿，三角=右腿）")
    print("2. 图2: 速度曲线对比 - 按阶段颜色显示（方块=左腿，三角=右腿）")
    print("3. 图3: 左腿阶段标注 - 按阶段颜色显示")
    print("4. 图4: 右腿阶段标注 - 按阶段颜色显示")
    print("\n颜色说明:")
    print("  - 静止=灰色，抬腿=绿色，压腿=红色")
    print("  - 左腿标记=蓝色方块，右腿标记=橙色三角")
    print("\n操作说明:")
    print("1. 左键拖动：平移视图")
    print("2. 右键拖动X轴刻度：缩放X轴")
    print("3. 右键拖动Y轴刻度：缩放Y轴")
    print("4. 鼠标滚轮：同时缩放XY轴")
    print("5. 调整底部偏移量：对齐两腿位置曲线")
    print("6. 关闭窗口结束\n")
    print("优势:")
    print("  - GPU硬件加速渲染")
    print("  - 流畅的缩放和平移（无卡顿）")
    print("  - 快速的图表更新")
    print("  - 支持大数据集可视化")
    print("  - 双腿运动模式对比分析")
    print("  - 位置偏移调整功能\n")

    tool.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
