#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI推理数据对比标注工具（PyQtGraph高性能版）
AI Inference Data Interactive Labeling Tool (PyQtGraph Version - High Performance)

功能特性 (Features):
1. 加载AI推理后的数据 (Load AI-inferred data)
2. 在同一图表中对比左右腿数据与AI预测标签 (Compare actual data with AI predictions)
3. 按阶段颜色可视化位置+速度曲线 (Visualize position + velocity curves with phase colors)
4. 交互式修正AI预测标签 (Interactive correction for AI predictions)
5. 位置偏移调整功能（对齐两腿起始点） (Position offset adjustment to align legs)
6. GPU硬件加速渲染 (GPU-accelerated rendering)
7. 支持在图1/图2上选择左腿或右腿进行标注 (Select left/right leg for labeling on plot 1/2)
8. 对比显示AI预测标签与人工修正标签 (Compare AI labels vs manual corrections)

使用方法 (Usage):
python 3.3_AI推理数据对比标注工具.py --input 平地数据_已推理.csv

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
- AI预测: 半透明背景 | 人工修正: 实线边框

数据格式说明:
- 输入CSV包含: m1_ai_label, m2_ai_label (AI预测标签)
- 修正后导出: m1_label, m2_label (人工修正标签)
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


class AIInferenceLabelingTool(QtWidgets.QWidget):
    """
    AI推理数据对比标注工具（PyQtGraph版本）
    AI Inference Data Interactive Labeling Tool (PyQtGraph Version)

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

        # 如果没有人工标签列，从AI标签复制创建
        if 'm1_label' not in self.df.columns:
            self.df['m1_label'] = self.df['m1_ai_label'].astype(int)
        if 'm2_label' not in self.df.columns:
            self.df['m2_label'] = self.df['m2_ai_label'].astype(int)

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
        self.setWindowTitle('AI推理数据对比标注工具 - PyQtGraph')
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

        # 选择标签
        layout.addWidget(QtWidgets.QLabel('选择标签:'))
        self.label_group = QtWidgets.QButtonGroup()
        for phase, name in self.PHASE_NAMES.items():
            btn = QtWidgets.QRadioButton(f'{phase}: {name}')
            self.label_group.addButton(btn, phase)
            layout.addWidget(btn)
            if phase == 0:
                btn.setChecked(True)
        self.label_group.buttonClicked.connect(self.on_label_changed)

        layout.addSpacing(10)

        # 位置偏移调整
        layout.addWidget(QtWidgets.QLabel('位置偏移调整:'))

        offset_layout = QtWidgets.QHBoxLayout()
        offset_layout.addWidget(QtWidgets.QLabel('M1偏移:'))
        self.m1_offset_spinbox = QtWidgets.QDoubleSpinBox()
        self.m1_offset_spinbox.setRange(-100, 100)
        self.m1_offset_spinbox.setSingleStep(0.1)
        self.m1_offset_spinbox.setValue(0.0)
        self.m1_offset_spinbox.valueChanged.connect(self.on_offset_changed)
        offset_layout.addWidget(self.m1_offset_spinbox)
        layout.addLayout(offset_layout)

        offset_layout2 = QtWidgets.QHBoxLayout()
        offset_layout2.addWidget(QtWidgets.QLabel('M2偏移:'))
        self.m2_offset_spinbox = QtWidgets.QDoubleSpinBox()
        self.m2_offset_spinbox.setRange(-100, 100)
        self.m2_offset_spinbox.setSingleStep(0.1)
        self.m2_offset_spinbox.setValue(0.0)
        self.m2_offset_spinbox.valueChanged.connect(self.on_offset_changed)
        offset_layout2.addWidget(self.m2_offset_spinbox)
        layout.addLayout(offset_layout2)

        layout.addSpacing(10)

        # 操作按钮
        btn_undo = QtWidgets.QPushButton('撤销 (Ctrl+Z)')
        btn_undo.clicked.connect(self.undo)
        layout.addWidget(btn_undo)

        btn_reset = QtWidgets.QPushButton('重置所有修改')
        btn_reset.clicked.connect(self.reset_all)
        layout.addWidget(btn_reset)

        btn_save = QtWidgets.QPushButton('保存 (Ctrl+S)')
        btn_save.clicked.connect(self.save)
        layout.addWidget(btn_save)

        layout.addSpacing(10)

        # 统计信息
        self.stats_label = QtWidgets.QLabel()
        self.stats_label.setWordWrap(True)
        self.stats_label.setStyleSheet("font-size: 11px;")
        layout.addWidget(self.stats_label)

        layout.addStretch()

        # 使用说明
        usage = QtWidgets.QLabel(
            "使用说明:\n"
            "1. 选择腿部和标签\n"
            "2. 在图上点击两次框选区域\n"
            "3. 修正后保存\n"
            "4. 绿色背景=AI预测正确\n"
            "5. 红色背景=需要修正"
        )
        usage.setWordWrap(True)
        usage.setStyleSheet("font-size: 10px; padding: 10px; background-color: #f0f0f0;")
        layout.addWidget(usage)

        panel.setLayout(layout)
        panel.setFixedWidth(280)
        return panel

    def create_plot_area(self):
        """创建右侧图表区域"""
        layout = QtWidgets.QVBoxLayout()

        # 创建GraphicsLayoutWidget
        self.graphics_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_widget)

        # 图1: 位置曲线
        self.plot1 = self.graphics_widget.addPlot(row=0, col=0, title='位置曲线 (Position)')
        self.plot1.setLabel('left', '位置 (Position)', units='rad')
        self.plot1.setLabel('bottom', '时间 (Time)', units='s')
        self.plot1.addLegend()
        self.plot1.scene().sigMouseClicked.connect(lambda evt: self.on_plot_click(evt, self.plot1))

        # 图2: 速度曲线
        self.plot2 = self.graphics_widget.addPlot(row=1, col=0, title='速度曲线 (Velocity)')
        self.plot2.setLabel('left', '速度 (Velocity)', units='rad/s')
        self.plot2.setLabel('bottom', '时间 (Time)', units='s')
        self.plot2.addLegend()
        self.plot2.scene().sigMouseClicked.connect(lambda evt: self.on_plot_click(evt, self.plot2))

        # 链接X轴
        self.plot2.setXLink(self.plot1)

        return layout

    def update_plots(self):
        """更新所有图表"""
        self.plot1.clear()
        self.plot2.clear()

        time = self.time_array

        # ===== 图1: 位置曲线 =====
        self.plot_by_phase(self.plot1, time,
                          self.df['m1_pos'].values + self.m1_pos_offset,
                          self.df['m1_label'].values,
                          self.LEG_MARKER_COLORS['m1'], '左腿位置', 's')

        self.plot_by_phase(self.plot1, time,
                          self.df['m2_pos'].values + self.m2_pos_offset,
                          self.df['m2_label'].values,
                          self.LEG_MARKER_COLORS['m2'], '右腿位置', 't')

        # ===== 图2: 速度曲线 =====
        self.plot_by_phase(self.plot2, time,
                          self.df['m1_vel'].values,
                          self.df['m1_label'].values,
                          self.LEG_MARKER_COLORS['m1'], '左腿速度', 's')

        self.plot_by_phase(self.plot2, time,
                          self.df['m2_vel'].values,
                          self.df['m2_label'].values,
                          self.LEG_MARKER_COLORS['m2'], '右腿速度', 't')

        # 绘制选择区域
        if self.select_region and self.active_plot:
            self.select_region.setRegion([self.select_start, self.select_end])

    def plot_by_phase(self, plot_widget, x, y, labels, color, name, symbol):
        """按阶段绘制曲线"""
        # 按阶段分段绘制
        i = 0
        while i < len(labels):
            phase = labels[i]
            j = i
            while j < len(labels) and labels[j] == phase:
                j += 1

            # 绘制该阶段
            phase_color = self.PHASE_COLORS[phase]
            plot_widget.plot(x[i:j], y[i:j],
                           pen=pg.mkPen(color=phase_color, width=2),
                           name=f'{name} - {self.PHASE_NAMES[phase]}' if i == 0 else None)

            i = j

        # 绘制起始点标记
        plot_widget.plot([x[0]], [y[0]],
                        pen=None,
                        symbol=symbol,
                        symbolSize=10,
                        symbolBrush=color)

    def on_leg_changed(self, button):
        """切换腿部"""
        self.current_leg = 'm1' if button.text() == '左腿(m1)' else 'm2'

    def on_label_changed(self, button):
        """切换标签"""
        self.current_label = self.label_group.id(button)

    def on_offset_changed(self):
        """位置偏移改变"""
        self.m1_pos_offset = self.m1_offset_spinbox.value()
        self.m2_pos_offset = self.m2_offset_spinbox.value()
        self.update_plots()

    def on_plot_click(self, event, plot_widget):
        """图表点击事件"""
        if event.button() != QtCore.Qt.LeftButton:
            return

        # 获取点击的场景坐标
        pos = event.scenePos()
        if not plot_widget.sceneBoundingRect().contains(pos):
            return

        # 转换为数据坐标
        mouse_point = plot_widget.vb.mapSceneToView(pos)
        x = mouse_point.x()

        if self.select_start is None:
            # 第一次点击 - 设置起点
            self.select_start = x
            self.active_plot = plot_widget

            # 创建选择区域
            if self.select_region:
                plot_widget.removeItem(self.select_region)
            self.select_region = pg.LinearRegionItem([x, x], brush=(0, 255, 0, 50))
            plot_widget.addItem(self.select_region)

        else:
            # 第二次点击 - 设置终点并应用标签
            self.select_end = x

            # 确保起点小于终点
            if self.select_start > self.select_end:
                self.select_start, self.select_end = self.select_end, self.select_start

            # 应用标签
            self.apply_label()

            # 清除选择
            if self.select_region:
                plot_widget.removeItem(self.select_region)
                self.select_region = None
            self.select_start = None
            self.select_end = None
            self.active_plot = None

    def apply_label(self):
        """应用标签到选定区域"""
        if self.select_start is None or self.select_end is None:
            return

        # 保存历史
        self.history.append(self.df.copy())

        # 找到时间范围内的索引
        mask = (self.time_array >= self.select_start) & (self.time_array <= self.select_end)

        # 应用标签
        label_col = f'{self.current_leg}_label'
        self.df.loc[mask, label_col] = self.current_label

        # 更新图表和统计
        self.update_plots()
        self.update_stats()

    def undo(self):
        """撤销"""
        if self.history:
            self.df = self.history.pop()
            self.update_plots()
            self.update_stats()

    def reset_all(self):
        """重置所有修改"""
        reply = QtWidgets.QMessageBox.question(
            self, '确认重置',
            '确定要重置所有修改吗？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        )

        if reply == QtWidgets.QMessageBox.Yes:
            # 从AI标签重新创建人工标签
            self.df['m1_label'] = self.df['m1_ai_label'].astype(int)
            self.df['m2_label'] = self.df['m2_ai_label'].astype(int)
            self.history.clear()
            self.update_plots()
            self.update_stats()

    def update_stats(self):
        """更新统计信息"""
        total = len(self.df)

        # 计算AI预测准确率（如果有人工标签）
        m1_correct = (self.df['m1_ai_label'] == self.df['m1_label']).sum()
        m2_correct = (self.df['m2_ai_label'] == self.df['m2_label']).sum()

        m1_accuracy = m1_correct / total * 100
        m2_accuracy = m2_correct / total * 100

        # 各阶段分布
        m1_dist = self.df['m1_label'].value_counts().to_dict()
        m2_dist = self.df['m2_label'].value_counts().to_dict()

        stats_text = f"""
统计信息:
━━━━━━━━━━━━━━━
总样本数: {total}

AI预测准确率:
  左腿: {m1_accuracy:.1f}%
  右腿: {m2_accuracy:.1f}%

左腿标签分布:
  静止: {m1_dist.get(0, 0)} ({m1_dist.get(0, 0)/total*100:.1f}%)
  抬腿: {m1_dist.get(1, 0)} ({m1_dist.get(1, 0)/total*100:.1f}%)
  压腿: {m1_dist.get(2, 0)} ({m1_dist.get(2, 0)/total*100:.1f}%)

右腿标签分布:
  静止: {m2_dist.get(0, 0)} ({m2_dist.get(0, 0)/total*100:.1f}%)
  抬腿: {m2_dist.get(1, 0)} ({m2_dist.get(1, 0)/total*100:.1f}%)
  压腿: {m2_dist.get(2, 0)} ({m2_dist.get(2, 0)/total*100:.1f}%)

已修改: {len(self.history)} 次
        """

        self.stats_label.setText(stats_text)

    def save(self):
        """保存修正后的数据"""
        # 生成输出文件名
        if self.input_file:
            input_path = Path(self.input_file)
            output_file = str(input_path.parent / f"{input_path.stem}_已修正.csv")
        else:
            output_file = 'AI推理数据_已修正.csv'

        # 保存
        self.df.to_csv(output_file, index=False)
        print(f"✓ 已保存修正后的数据: {output_file}")

        QtWidgets.QMessageBox.information(
            self, '保存成功',
            f'数据已保存到:\n{output_file}'
        )

    def keyPressEvent(self, event):
        """键盘快捷键"""
        if event.key() == QtCore.Qt.Key_Z and event.modifiers() == QtCore.Qt.ControlModifier:
            self.undo()
        elif event.key() == QtCore.Qt.Key_S and event.modifiers() == QtCore.Qt.ControlModifier:
            self.save()


def main():
    parser = argparse.ArgumentParser(description='AI推理数据对比标注工具')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入CSV文件（包含AI推理标签）')
    args = parser.parse_args()

    # 检查文件是否存在
    if not Path(args.input).exists():
        print(f"✗ 文件不存在: {args.input}")
        return

    # 加载数据
    print(f"加载数据: {args.input}")
    df = pd.read_csv(args.input)
    print(f"✓ 加载完成: {len(df)} 行")

    # 检查必需的列
    required_cols = ['timestamp', 'm1_pos', 'm1_vel', 'm2_pos', 'm2_vel',
                    'm1_ai_label', 'm2_ai_label']
    missing_cols = [col for col in required_cols if col not in df.columns]
    if missing_cols:
        print(f"✗ 缺少必需的列: {missing_cols}")
        return

    # 创建应用
    app = QtWidgets.QApplication(sys.argv)
    window = AIInferenceLabelingTool(df, input_file=args.input)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
