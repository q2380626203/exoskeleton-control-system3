#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双腿对比交互式标注工具（PyQtGraph高性能版）
Dual-Leg Interactive Labeling Tool (PyQtGraph Version - High Performance)

功能特性 (Features):
1. 加载自动标注数据 (Load auto-labeled data)
2. 在同一图表中对比左右腿数据 (Compare left and right leg data in same plots)
3. 按阶段颜色可视化位置+速度曲线 (Visualize position + velocity curves with phase colors)
4. 交互式修正左右腿标注 (Interactive correction for both legs)
5. 位置偏移调整功能（对齐两腿起始点） (Position offset adjustment to align legs)
6. GPU硬件加速渲染 (GPU-accelerated rendering)
7. 支持在图1/图2上选择左腿或右腿进行标注 (Select left/right leg for labeling on plot 1/2)

使用方法 (Usage):
python 3.2_双腿对比标注工具_pyqtgraph.py --input 平地数据_已标注.csv

依赖安装 (Dependencies):
pip install pyqtgraph PyQt5 pandas numpy

操作控制 (Controls):
- 左键拖动: 平移视图 (Left-click drag: Pan view)
- 右键拖动X轴: 缩放X轴 (Right-click drag X-axis: Zoom X-axis)
- 右键拖动Y轴: 缩放Y轴 (Right-click drag Y-axis: Zoom Y-axis)
- 在图1/2上左键点击两次: 框选区域进行修正 (Left-click twice: Select region to correct)
- 鼠标滚轮: 同时缩放XY轴 (Mouse wheel: Zoom both axes)

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


class DualLegLabelingTool(QtWidgets.QWidget):
    """
    双腿对比交互式标注工具（PyQtGraph版本）
    Dual-Leg Interactive Labeling Tool (PyQtGraph Version)

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

    def __init__(self, df, input_file=''):
        super().__init__()

        self.df = df.copy()
        self.original_df = df.copy()
        self.history = []  # 历史记录（用于撤销）
        self.input_file = input_file  # 保存输入文件名

        # 预计算时间数组
        self.time_array = self.df['timestamp'].values / 1000

        # 位置偏移量（用于对齐两条曲线）
        self.m1_pos_offset = 0.0
        self.m2_pos_offset = 0.0

        # 标注控制
        self.current_label = 0
        self.current_leg = 'm1'  # 当前选择的腿部（m1或m2）

        # 选择状态
        self.select_start = None
        self.select_end = None
        self.select_region = None
        self.active_plot = None

        self.init_ui()

    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle('双腿对比交互式标注工具 - PyQtGraph')
        self.setGeometry(100, 100, 1800, 1000)

        # 主布局
        main_layout = QtWidgets.QHBoxLayout()

        # 左侧控制面板
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)

        # 右侧图表区域
        plot_layout = self.create_plot_area()
        main_layout.addLayout(plot_layout, stretch=6)

        self.setLayout(main_layout)

        # 绘制初始图表
        self.update_plots()
        self.update_stats()

    def create_control_panel(self):
        """创建左侧控制面板"""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()

        # 标题
        title = QtWidgets.QLabel('控制面板')
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        # 选择腿部
        layout.addWidget(QtWidgets.QLabel('选择腿部:'))
        self.leg_group = QtWidgets.QButtonGroup()
        m1_btn = QtWidgets.QRadioButton('左腿(m1)')
        m2_btn = QtWidgets.QRadioButton('右腿(m2)')
        self.leg_group.addButton(m1_btn, 1)
        self.leg_group.addButton(m2_btn, 2)
        m1_btn.setChecked(True)
        layout.addWidget(m1_btn)
        layout.addWidget(m2_btn)
        self.leg_group.buttonClicked.connect(self.on_leg_changed)

        layout.addSpacing(10)

        # 标签选择
        layout.addWidget(QtWidgets.QLabel('选择标签:'))
        self.label_group = QtWidgets.QButtonGroup()
        for label, name in self.PHASE_NAMES.items():
            btn = QtWidgets.QRadioButton(f'{label} {name}')
            self.label_group.addButton(btn, label)
            layout.addWidget(btn)
            if label == 0:
                btn.setChecked(True)
        self.label_group.buttonClicked.connect(self.on_label_changed)

        layout.addSpacing(20)

        # 按钮
        save_btn = QtWidgets.QPushButton('保存修正')
        save_btn.clicked.connect(self.save_correction)
        save_btn.setStyleSheet("background-color: lightgreen; font-weight: bold; padding: 10px;")
        layout.addWidget(save_btn)

        undo_btn = QtWidgets.QPushButton('撤销')
        undo_btn.clicked.connect(self.undo)
        undo_btn.setStyleSheet("background-color: lightyellow; padding: 8px;")
        layout.addWidget(undo_btn)

        reset_btn = QtWidgets.QPushButton('重置全部')
        reset_btn.clicked.connect(self.reset_all)
        reset_btn.setStyleSheet("background-color: lightcoral; padding: 8px;")
        layout.addWidget(reset_btn)

        reset_view_btn = QtWidgets.QPushButton('重置视图')
        reset_view_btn.clicked.connect(self.reset_view)
        reset_view_btn.setStyleSheet("background-color: lightblue; padding: 8px;")
        layout.addWidget(reset_view_btn)

        layout.addSpacing(20)

        # 位置偏移控制
        offset_group = QtWidgets.QGroupBox('位置偏移调整')
        offset_layout = QtWidgets.QVBoxLayout()

        # 左腿偏移
        m1_offset_layout = QtWidgets.QHBoxLayout()
        m1_offset_layout.addWidget(QtWidgets.QLabel('左腿:'))
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
        m2_offset_layout.addWidget(QtWidgets.QLabel('右腿:'))
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

        layout.addSpacing(10)

        # 统计信息
        stats_group = QtWidgets.QGroupBox('统计信息')
        stats_layout = QtWidgets.QVBoxLayout()
        self.stats_text = QtWidgets.QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setMaximumHeight(150)
        stats_layout.addWidget(self.stats_text)
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)

        layout.addStretch()

        # 操作说明
        instructions = QtWidgets.QLabel(
            '操作说明:\n'
            '1. 选择腿部(左/右)\n'
            '2. 选择标签\n'
            '3. 在图1/2上点击\n'
            '   两次框选区域\n'
            '4. 左键拖动: 平移\n'
            '5. 右键拖轴: 缩放\n'
            '6. 滚轮: 缩放XY'
        )
        instructions.setStyleSheet("background-color: #f0f0f0; padding: 10px; border-radius: 5px;")
        layout.addWidget(instructions)

        panel.setLayout(layout)
        panel.setMaximumWidth(280)
        return panel

    def create_plot_area(self):
        """创建右侧绘图区域"""
        layout = QtWidgets.QVBoxLayout()

        # 标题
        title = QtWidgets.QLabel('双腿运动对比与标注')
        title.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;")
        title.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(title)

        # 创建图表布局控件
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        # 图表1: 位置曲线对比（可交互）
        self.plot_pos = self.plot_widget.addPlot(row=0, col=0, title='位置曲线对比（点击两次框选）')
        self.plot_pos.setLabel('left', '位置 (rad)')
        self.plot_pos.setLabel('bottom', '时间 (秒)')
        self.plot_pos.showGrid(x=True, y=True, alpha=0.3)
        self.plot_pos.addLegend()

        # 图表2: 速度曲线对比（可交互）
        self.plot_vel = self.plot_widget.addPlot(row=1, col=0, title='速度曲线对比（点击两次框选）')
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

        # 添加鼠标事件用于选择（图1和图2支持交互标注）
        # 注意：由于所有图表共享同一个scene，只连接一次scene事件
        self.plot_widget.scene().sigMouseClicked.connect(self.on_mouse_clicked)
        self.plot_widget.scene().sigMouseMoved.connect(self.on_mouse_moved)

        return layout

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
        legend_added = {'m1': set(), 'm2': set()}

        # 绘制左腿
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
                    name=None
                )

        # 添加左右腿标记（在曲线起始点）
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

        # 图表2: 速度曲线对比（连续曲线）
        legend_added_vel = {'m1': set(), 'm2': set()}

        # 绘制左腿
        for label, name in self.PHASE_NAMES.items():
            mask = m1_label_data == label
            if np.any(mask):
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
                regions = self.get_continuous_regions(mask)
                color = self.PHASE_COLORS[label]

                for start_idx, end_idx in regions:
                    self.plot_vel.plot(
                        time[start_idx:end_idx+1],
                        m2_vel_data[start_idx:end_idx+1],
                        pen=pg.mkPen(color, width=2),
                        name=None
                    )

        self.plot_vel.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.DashLine))

        # 添加左右腿标记
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

        # 图表3: 左腿阶段标注
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

        # 图表4: 右腿阶段标注
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

    def on_leg_changed(self, button):
        """腿部选择改变"""
        leg_id = self.leg_group.id(button)
        self.current_leg = 'm1' if leg_id == 1 else 'm2'
        leg_name = '左腿(m1)' if leg_id == 1 else '右腿(m2)'
        print(f"当前选择: {leg_name}")

    def on_label_changed(self, button):
        """标签选择改变"""
        self.current_label = self.label_group.id(button)
        print(f"当前标签: {self.PHASE_NAMES[self.current_label]}")

    def on_mouse_clicked(self, event):
        """鼠标点击事件"""
        if event.button() == QtCore.Qt.LeftButton:
            pos = event.scenePos()

            # 检查点击在哪个图表上
            current_plot = None
            if self.plot_pos.sceneBoundingRect().contains(pos):
                current_plot = self.plot_pos
            elif self.plot_vel.sceneBoundingRect().contains(pos):
                current_plot = self.plot_vel

            if current_plot is not None:
                mousePoint = current_plot.vb.mapSceneToView(pos)

                if self.select_start is None:
                    # 开始选择
                    self.select_start = mousePoint.x()
                    self.active_plot = current_plot

                    # 添加选择区域指示器（先清除之前的）
                    if self.select_region is not None:
                        try:
                            self.active_plot.removeItem(self.select_region)
                        except:
                            pass

                    self.select_region = pg.LinearRegionItem(
                        values=[self.select_start, self.select_start],
                        brush=(0, 0, 255, 50),
                        movable=False
                    )
                    current_plot.addItem(self.select_region)
                else:
                    # 结束选择（必须在同一个图表上）
                    if current_plot == self.active_plot:
                        self.select_end = mousePoint.x()

                        # 执行修正
                        xmin = min(self.select_start, self.select_end)
                        xmax = max(self.select_start, self.select_end)

                        # 重置选择状态
                        self.select_start = None
                        self.select_end = None

                        # 执行修正（会自动清理选择区域）
                        self.execute_correction(xmin, xmax)
                    else:
                        # 点击了不同的图表，取消当前选择
                        print("⚠ 请在同一图表上完成框选")
                        if self.select_region is not None and self.active_plot is not None:
                            try:
                                self.active_plot.removeItem(self.select_region)
                            except:
                                pass
                        self.select_start = None
                        self.select_end = None
                        self.select_region = None
                        self.active_plot = None

    def on_mouse_moved(self, pos):
        """鼠标移动事件"""
        if self.select_start is not None and self.select_region is not None and self.active_plot is not None:
            # 更新选择区域
            if self.active_plot.sceneBoundingRect().contains(pos):
                mousePoint = self.active_plot.vb.mapSceneToView(pos)
                self.select_region.setRegion([self.select_start, mousePoint.x()])

    def execute_correction(self, xmin, xmax):
        """执行标签修正"""
        # 清除选择区域（在更新图表之前）
        if self.select_region is not None and self.active_plot is not None:
            try:
                self.active_plot.removeItem(self.select_region)
            except:
                pass
            self.select_region = None
        self.active_plot = None

        # 保存历史
        self.history.append({
            'm1': self.df['m1_label'].copy(),
            'm2': self.df['m2_label'].copy()
        })

        # 查找时间范围内的索引
        time = self.time_array
        mask = (time >= xmin) & (time <= xmax)
        correction_count = np.count_nonzero(mask)

        if correction_count > 0:
            # 修正当前选择的腿部标签
            label_col = f'{self.current_leg}_label'
            label_array = self.df[label_col].values
            label_array[mask] = self.current_label
            self.df[label_col] = label_array

            leg_name = '左腿(m1)' if self.current_leg == 'm1' else '右腿(m2)'
            print(f"✓ {leg_name} 修正了 {correction_count} 个样本为 '{self.PHASE_NAMES[self.current_label]}'")

            # 更新图表
            self.update_plots()
            self.update_stats()

    def undo(self):
        """撤销上一次修正"""
        if self.history:
            last_state = self.history.pop()
            self.df['m1_label'] = last_state['m1']
            self.df['m2_label'] = last_state['m2']
            print("✓ 已撤销上一次修正")
            self.update_plots()
            self.update_stats()
        else:
            print("✗ 没有可撤销的操作")

    def reset_all(self):
        """重置到原始标签"""
        self.df['m1_label'] = self.original_df['m1_label'].copy()
        self.df['m2_label'] = self.original_df['m2_label'].copy()
        self.history.clear()
        print("✓ 已重置到原始标注")
        self.update_plots()
        self.update_stats()

    def save_correction(self):
        """保存修正结果"""
        # 根据输入文件名生成输出文件名
        if self.input_file:
            # 将 "已标注.csv" 替换为 "已修正.csv"
            if '已标注' in self.input_file:
                output_file = self.input_file.replace('已标注', '已修正')
            else:
                # 如果没有"已标注"字样，在文件名后添加"_已修正"
                output_file = self.input_file.replace('.csv', '_已修正.csv')
        else:
            output_file = '双腿标注_修正后.csv'

        self.df.to_csv(output_file, index=False, encoding='utf-8')
        print(f"✓ 修正结果已保存: {output_file}")

        # 显示消息框
        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Information)
        msg.setText(f"修正结果已保存: {output_file}")
        msg.setWindowTitle("保存成功")
        msg.exec_()

    def reset_view(self):
        """重置视图到原始范围"""
        self.plot_pos.autoRange()
        self.plot_vel.autoRange()
        self.plot_m1_label.autoRange()
        self.plot_m2_label.autoRange()
        print("✓ 已重置视图")

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

    def update_stats(self):
        """更新统计文本"""
        stats_text = "【标注统计】\n\n"
        total_samples = len(self.df)

        # 左腿统计
        stats_text += "左腿(m1):\n"
        m1_label_data = self.df['m1_label'].values
        for label, name in self.PHASE_NAMES.items():
            count = np.count_nonzero(m1_label_data == label)
            percentage = count / total_samples * 100
            stats_text += f"  {name}: {count} ({percentage:.1f}%)\n"

        stats_text += "\n右腿(m2):\n"
        m2_label_data = self.df['m2_label'].values
        for label, name in self.PHASE_NAMES.items():
            count = np.count_nonzero(m2_label_data == label)
            percentage = count / total_samples * 100
            stats_text += f"  {name}: {count} ({percentage:.1f}%)\n"

        stats_text += f"\n修正次数: {len(self.history)}"

        self.stats_text.setPlainText(stats_text)


def main():
    parser = argparse.ArgumentParser(description='双腿对比交互式标注工具（PyQtGraph高性能版）')
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
    tool = DualLegLabelingTool(df, input_file=args.input)

    print(f"\n=== 双腿对比交互式标注工具 - PyQtGraph高性能版 ===")
    print(f"性能配置:")
    print(f"  - 后端: PyQtGraph (OpenGL硬件加速)")
    print(f"  - 数据点: {len(df)}")
    print(f"  - 抗锯齿: 已启用")
    print("\n图表说明:")
    print("1. 图1: 位置曲线对比 - 可交互标注（方块=左腿，三角=右腿）")
    print("2. 图2: 速度曲线对比 - 可交互标注（方块=左腿，三角=右腿）")
    print("3. 图3: 左腿阶段标注 - 查看左腿标注结果")
    print("4. 图4: 右腿阶段标注 - 查看右腿标注结果")
    print("\n操作说明:")
    print("1. 选择腿部（左腿/右腿）")
    print("2. 选择标签（静止/抬腿/压腿）")
    print("3. 在图1（位置）或图2（速度）上点击两次左键框选区域进行修正")
    print("   - 第1次点击：框选起点")
    print("   - 第2次点击：框选终点并执行修正")
    print("4. 左键拖动：平移视图")
    print("5. 右键拖动X轴刻度：缩放X轴")
    print("6. 右键拖动Y轴刻度：缩放Y轴")
    print("7. 鼠标滚轮：同时缩放XY轴")
    print("8. 调整位置偏移：对齐两腿位置曲线")
    print("9. 点击'保存修正'按钮保存结果")
    print("10. 点击'撤销'按钮撤销上一次修正")
    print("11. 点击'重置视图'按钮恢复默认视图")
    print("12. 点击'重置全部'恢复到原始标注")
    print("13. 关闭窗口结束\n")
    print("颜色说明:")
    print("  - 静止=灰色，抬腿=绿色，压腿=红色")
    print("  - 左腿标记=蓝色方块，右腿标记=橙色三角")
    print("\n优势:")
    print("  - GPU硬件加速渲染")
    print("  - 流畅的缩放和平移（无卡顿）")
    print("  - 快速的图表更新")
    print("  - 支持大数据集可视化")
    print("  - 双腿运动模式对比分析")
    print("  - 交互式标注修正")
    print("  - 位置偏移调整功能\n")

    tool.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
