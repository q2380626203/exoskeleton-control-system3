#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式标注修正工具（PyQtGraph高性能版）
Interactive Labeling Tool (PyQtGraph Version - High Performance)

功能特性 (Features):
1. 加载自动标注数据 (Load auto-labeled data)
2. 可视化位置+速度曲线和标注结果 (Visualize position + velocity curves and labeling results)
3. 支持左腿(m1)和右腿(m2) (Support left leg (m1) and right leg (m2))
4. 交互式修正错误标注 (Interactive correction of incorrect labels)
5. GPU硬件加速渲染 (GPU-accelerated rendering)
6. 保存修正后数据 (Save corrected data)

使用方法 (Usage):
python 3_interactive_labeling_pyqtgraph.py --input 平地数据_已标注.csv --leg 左腿

依赖安装 (Dependencies):
pip install pyqtgraph PyQt5 pandas numpy

操作控制 (Controls):
- 左键拖动: 平移视图 (Left-click drag: Pan view)
- 右键拖动X轴: 缩放X轴 (Right-click drag X-axis: Zoom X-axis)
- 右键拖动Y轴: 缩放Y轴 (Right-click drag Y-axis: Zoom Y-axis)
- 在图1/2/3上左键点击两次: 框选区域进行修正 (Left-click twice on any plot: Select region to correct)
- 鼠标滚轮: 同时缩放XY轴 (Mouse wheel: Zoom both axes)
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


class InteractiveLabelingTool(QtWidgets.QWidget):
    """
    交互式标注修正工具（PyQtGraph版本）
    Interactive Labeling Tool (PyQtGraph Version)

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

    PHASE_COLORS = {
        0: (128, 128, 128),  # 灰色 Gray
        1: (0, 255, 0),      # 绿色 Green
        2: (255, 0, 0)       # 红色 Red
    }

    LEG_MAPPING = {
        '左腿': 'm1',
        '右腿': 'm2'
    }

    def __init__(self, df, leg='左腿'):
        super().__init__()

        self.df = df.copy()
        self.original_df = df.copy()
        self.leg = leg
        self.motor = self.LEG_MAPPING[leg]
        self.pos_col = f'{self.motor}_pos'
        self.vel_col = f'{self.motor}_vel'
        self.label_col = f'{self.motor}_label'
        self.current_label = 0
        self.history = []

        # 预计算时间数组
        self.time_array = self.df['timestamp'].values / 1000

        # 选择状态
        self.select_start = None
        self.select_end = None
        self.select_region = None
        self.active_plot = None  # 当前活动的图表

        self.init_ui()

    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle(f'{self.leg}({self.motor.upper()}) 运动阶段标注工具 - PyQtGraph')
        self.setGeometry(100, 100, 1600, 1000)

        # 主布局
        main_layout = QtWidgets.QHBoxLayout()

        # 左侧控制面板
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)

        # 右侧绘图区域
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

        # 统计信息
        layout.addWidget(QtWidgets.QLabel('统计信息:'))
        self.stats_text = QtWidgets.QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setMaximumHeight(200)
        layout.addWidget(self.stats_text)

        layout.addStretch()

        # 操作说明
        instructions = QtWidgets.QLabel(
            '操作说明:\n'
            '1. 选择标签\n'
            '2. 在图1/2/3上点击\n'
            '   两次框选区域\n'
            '3. 左键拖动: 平移\n'
            '4. 右键拖X轴: 缩放X\n'
            '5. 右键拖Y轴: 缩放Y\n'
            '6. 滚轮: 同时缩放XY'
        )
        instructions.setStyleSheet("background-color: #f0f0f0; padding: 10px; border-radius: 5px;")
        layout.addWidget(instructions)

        panel.setLayout(layout)
        panel.setMaximumWidth(250)
        return panel

    def create_plot_area(self):
        """创建右侧绘图区域"""
        layout = QtWidgets.QVBoxLayout()

        # 创建图表布局控件
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        # 图表1: 位置曲线（可交互）
        self.plot_pos = self.plot_widget.addPlot(row=0, col=0, title=f'{self.leg} 位置曲线（点击两次框选）')
        self.plot_pos.setLabel('left', '位置 (rad)')
        self.plot_pos.setLabel('bottom', '时间 (秒)')
        self.plot_pos.showGrid(x=True, y=True, alpha=0.3)

        # 图表2: 速度曲线（可交互）
        self.plot_vel = self.plot_widget.addPlot(row=1, col=0, title=f'{self.leg} 速度曲线（点击两次框选）')
        self.plot_vel.setLabel('left', '速度 (rad/s)')
        self.plot_vel.setLabel('bottom', '时间 (秒)')
        self.plot_vel.showGrid(x=True, y=True, alpha=0.3)

        # 图表3: 标注结果（可交互）
        self.plot_label = self.plot_widget.addPlot(row=2, col=0, title=f'{self.leg} 标注结果（点击两次框选）')
        self.plot_label.setLabel('left', '速度 (rad/s)')
        self.plot_label.setLabel('bottom', '时间 (秒)')
        self.plot_label.showGrid(x=True, y=True, alpha=0.3)

        # 图表4: 阶段时序
        self.plot_timeline = self.plot_widget.addPlot(row=3, col=0, title='阶段时序')
        self.plot_timeline.setLabel('left', '阶段')
        self.plot_timeline.setLabel('bottom', '时间 (秒)')
        self.plot_timeline.showGrid(x=True, y=True, alpha=0.3)
        self.plot_timeline.getAxis('left').setTicks([[(0, '静止'), (1, '抬腿'), (2, '压腿')]])

        # 链接X轴
        self.plot_vel.setXLink(self.plot_pos)
        self.plot_label.setXLink(self.plot_pos)
        self.plot_timeline.setXLink(self.plot_pos)

        # 添加鼠标事件用于选择（所有三个图表都支持）
        self.plot_pos.scene().sigMouseClicked.connect(self.on_mouse_clicked)
        self.plot_pos.scene().sigMouseMoved.connect(self.on_mouse_moved)
        self.plot_vel.scene().sigMouseClicked.connect(self.on_mouse_clicked)
        self.plot_vel.scene().sigMouseMoved.connect(self.on_mouse_moved)
        self.plot_label.scene().sigMouseClicked.connect(self.on_mouse_clicked)
        self.plot_label.scene().sigMouseMoved.connect(self.on_mouse_moved)

        return layout

    def update_plots(self):
        """更新所有图表"""
        time = self.time_array
        label_data = self.df[self.label_col].values
        pos_data = self.df[self.pos_col].values
        vel_data = self.df[self.vel_col].values

        # 清空图表
        self.plot_pos.clear()
        self.plot_vel.clear()
        self.plot_label.clear()
        self.plot_timeline.clear()

        # 图表1: 位置曲线（按标注颜色分段）
        for label, name in self.PHASE_NAMES.items():
            mask = label_data == label
            if np.any(mask):
                color = self.PHASE_COLORS[label]
                self.plot_pos.plot(
                    time[mask], pos_data[mask],
                    pen=None,
                    symbol='o',
                    symbolSize=3,
                    symbolBrush=color,
                    symbolPen=None,
                    name=name
                )

        # 图表2: 速度曲线（连续曲线，按标注颜色分段）
        for label, name in self.PHASE_NAMES.items():
            mask = label_data == label
            if np.any(mask):
                # 找到连续区域
                regions = self.get_continuous_regions(mask)
                color = self.PHASE_COLORS[label]

                for start_idx, end_idx in regions:
                    self.plot_vel.plot(
                        time[start_idx:end_idx+1],
                        vel_data[start_idx:end_idx+1],
                        pen=pg.mkPen(color, width=2),
                        name=name if regions.index((start_idx, end_idx)) == 0 else None
                    )
        self.plot_vel.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.DashLine))

        # 图表3: 标注结果（按标注颜色分段）
        for label, name in self.PHASE_NAMES.items():
            mask = label_data == label
            if np.any(mask):
                color = self.PHASE_COLORS[label]
                self.plot_label.plot(
                    time[mask], vel_data[mask],
                    pen=None,
                    symbol='o',
                    symbolSize=3,
                    symbolBrush=color,
                    symbolPen=None,
                    name=name
                )

        # 图表4: 时序图（移除stepMode以兼容）
        self.plot_timeline.plot(time, label_data, pen=pg.mkPen('k', width=2))

        # 添加阶段背景色
        for label, color in self.PHASE_COLORS.items():
            mask = label_data == label
            if np.any(mask):
                regions = self.get_continuous_regions(mask)
                for start_idx, end_idx in regions:
                    region = pg.LinearRegionItem(
                        values=[time[start_idx], time[end_idx]],
                        brush=(*color, 30),
                        movable=False
                    )
                    self.plot_timeline.addItem(region)

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
        stats_text = f"【{self.leg}统计】\n\n"
        total_samples = len(self.df)
        label_data = self.df[self.label_col].values

        for label, name in self.PHASE_NAMES.items():
            count = np.count_nonzero(label_data == label)
            percentage = count / total_samples * 100
            stats_text += f"{name}:\n  {count} 样本\n  {percentage:.1f}%\n\n"

        stats_text += f"\n修正次数: {len(self.history)}"

        self.stats_text.setPlainText(stats_text)

    def on_label_changed(self, button):
        """标签选择改变"""
        self.current_label = self.label_group.id(button)
        print(f"当前选择: {self.PHASE_NAMES[self.current_label]}")

    def on_mouse_clicked(self, event):
        """鼠标点击事件"""
        if event.button() == QtCore.Qt.LeftButton:
            # 获取点击位置
            pos = event.scenePos()

            # 检查点击在哪个图表上
            current_plot = None
            if self.plot_pos.sceneBoundingRect().contains(pos):
                current_plot = self.plot_pos
            elif self.plot_vel.sceneBoundingRect().contains(pos):
                current_plot = self.plot_vel
            elif self.plot_label.sceneBoundingRect().contains(pos):
                current_plot = self.plot_label

            if current_plot is not None:
                mousePoint = current_plot.vb.mapSceneToView(pos)

                if self.select_start is None:
                    # 开始选择
                    self.select_start = mousePoint.x()
                    self.active_plot = current_plot

                    # 添加选择区域指示器
                    if self.select_region is not None and self.active_plot is not None:
                        self.active_plot.removeItem(self.select_region)

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
                        self.execute_correction(xmin, xmax)

                        # 重置选择状态
                        self.select_start = None
                        self.select_end = None
                        if self.select_region is not None and self.active_plot is not None:
                            self.active_plot.removeItem(self.select_region)
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
        # 保存历史
        self.history.append(self.df[self.label_col].copy())

        # 查找时间范围内的索引
        time = self.time_array
        mask = (time >= xmin) & (time <= xmax)
        correction_count = np.count_nonzero(mask)

        if correction_count > 0:
            # 修正标签
            label_array = self.df[self.label_col].values
            label_array[mask] = self.current_label
            self.df[self.label_col] = label_array

            print(f"✓ 修正了 {correction_count} 个样本为 '{self.PHASE_NAMES[self.current_label]}'")

            # 更新图表
            self.update_plots()
            self.update_stats()

    def undo(self):
        """撤销上一次修正"""
        if self.history:
            self.df[self.label_col] = self.history.pop()
            print("✓ 已撤销上一次修正")
            self.update_plots()
            self.update_stats()
        else:
            print("✗ 没有可撤销的操作")

    def reset_all(self):
        """重置到原始标签"""
        self.df[self.label_col] = self.original_df[self.label_col].copy()
        self.history.clear()
        print("✓ 已重置到原始标注")
        self.update_plots()
        self.update_stats()

    def save_correction(self):
        """保存修正结果"""
        output_file = f'{self.leg}_{self.motor}_修正后.csv'
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
        self.plot_label.autoRange()
        self.plot_timeline.autoRange()
        print("✓ 已重置视图")


def main():
    parser = argparse.ArgumentParser(description='交互式标注修正工具（PyQtGraph高性能版）')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入已标注的CSV文件')
    parser.add_argument('--leg', '-l', type=str, default='左腿',
                        choices=['左腿', '右腿'], help='选择腿部 (左腿/右腿)')

    args = parser.parse_args()

    # 加载数据
    print(f"加载数据: {args.input}")
    df = pd.read_csv(args.input)
    print(f"✓ 加载完成: {len(df)} 行")

    # 检查必需列
    leg_mapping = {'左腿': 'm1', '右腿': 'm2'}
    motor = leg_mapping[args.leg]
    required_cols = ['timestamp', f'{motor}_pos', f'{motor}_vel', f'{motor}_label']
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
    tool = InteractiveLabelingTool(df, leg=args.leg)

    print(f"\n=== {args.leg}({motor.upper()}) 标注修正工具 - PyQtGraph高性能版 ===")
    print(f"性能配置:")
    print(f"  - 后端: PyQtGraph (OpenGL硬件加速)")
    print(f"  - 数据点: {len(df)}")
    print(f"  - 抗锯齿: 已启用")
    print("\n操作说明:")
    print("1. 点击左侧单选按钮选择要修正为的标签")
    print("2. 在图1（位置）/图2（速度）/图3（标注）上点击两次左键框选区域进行修正")
    print("   - 第1次点击：框选起点")
    print("   - 第2次点击：框选终点并执行修正")
    print("3. 左键拖动：平移视图")
    print("4. 右键拖动X轴刻度：缩放X轴")
    print("5. 右键拖动Y轴刻度：缩放Y轴")
    print("6. 鼠标滚轮：同时缩放XY轴")
    print("7. 点击'保存修正'按钮保存结果")
    print("8. 点击'撤销'按钮撤销上一次修正")
    print("9. 点击'重置视图'按钮恢复默认视图")
    print("10. 点击'重置全部'恢复到原始标注")
    print("11. 关闭窗口结束\n")
    print("优势:")
    print("  - GPU硬件加速渲染")
    print("  - 流畅的缩放和平移（无卡顿）")
    print("  - 快速的图表更新")
    print("  - 支持大数据集可视化\n")

    tool.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
