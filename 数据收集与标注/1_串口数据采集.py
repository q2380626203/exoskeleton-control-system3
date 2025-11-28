#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口数据采集工具

功能：
1. 连接ESP32串口
2. 实时接收 motors:... 格式的数据
3. 解析并保存为CSV文件
4. 可视化实时曲线（可选）

使用方法：
python 1_串口数据采集.py --port COM18 --output 平地数据.csv --duration 300
python 1_串口数据采集.py --port COM18 --output 平地数据_已推理.csv --duration 300
"""

import serial
import argparse
import csv
import time
import sys
from datetime import datetime
import threading
import queue

class 串口数据采集器:
    def __init__(self, port, baudrate=115200, output_file='data.csv'):
        """
        初始化串口采集器

        参数:
            port: 串口号 (如 'COM3' 或 '/dev/ttyUSB0')
            baudrate: 波特率 (默认115200)
            output_file: 输出CSV文件名
        """
        self.port = port
        self.baudrate = baudrate
        self.output_file = output_file
        self.ser = None
        self.is_running = False
        self.sample_count = 0
        self.invalid_count = 0  # 无效数据计数
        self.start_time = None

        # CSV字段
        self.fieldnames = [
            'timestamp',      # 相对时间戳 (ms)
            'm1_pos',         # 电机1位置 (rad)
            'm1_vel',         # 电机1速度 (rad/s)
            'm1_torque',      # 电机1力矩 (N·m)
            'm2_pos',         # 电机2位置 (rad)
            'm2_vel',         # 电机2速度 (rad/s)
            'm2_torque',      # 电机2力矩 (N·m)
            'ch6_max',        # ch6最大差值
            'ch7_max',        # ch7最大差值
            'm1_ai_label',    # m1 AI预测标签 (0:静止, 1:抬腿, 2:压腿)
            'm2_ai_label'     # m2 AI预测标签 (0:静止, 1:抬腿, 2:压腿)
        ]

    def 连接串口(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            print(f"✓ 已连接到串口: {self.port} @ {self.baudrate}")
            time.sleep(2)  # 等待串口稳定
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False

    def 验证数据合理性(self, data):
        """
        验证数据是否在合理范围内

        返回: True=有效, False=无效
        """
        # 位置范围: -50 到 50 rad (约 -2865° 到 2865°)
        if abs(data['m1_pos']) > 50 or abs(data['m2_pos']) > 50:
            return False

        # 速度范围: -100 到 100 rad/s (实际电机速度很少超过此值)
        if abs(data['m1_vel']) > 100 or abs(data['m2_vel']) > 100:
            return False

        # 力矩范围: -10 到 10 N·m
        if abs(data['m1_torque']) > 10 or abs(data['m2_torque']) > 10:
            return False

        # ch6/ch7范围: 0 到 10 (最大差值)
        if not (0 <= data['ch6_max'] <= 10 and 0 <= data['ch7_max'] <= 10):
            return False

        # AI标签范围: 0 到 2 (0:静止, 1:抬腿, 2:压腿)
        if not (0 <= data['m1_ai_label'] <= 2 and 0 <= data['m2_ai_label'] <= 2):
            return False

        return True

    def 解析数据行(self, line):
        """
        解析 motors:... 格式的数据行

        格式: motors:0.123456,1.234567,2.345678,...
        返回: dict 或 None
        """
        try:
            if not line.startswith('motors:'):
                return None

            # 移除 "motors:" 前缀
            data_str = line[7:].strip()

            # 分割数据
            values = [float(x) for x in data_str.split(',')]

            if len(values) != 10:
                return None

            # 构建数据字典
            data = {
                'timestamp': int((time.time() - self.start_time) * 1000) if self.start_time else 0,
                'm1_pos': values[0],
                'm1_vel': values[1],
                'm1_torque': values[2],
                'm2_pos': values[3],
                'm2_vel': values[4],
                'm2_torque': values[5],
                'ch6_max': values[6],
                'ch7_max': values[7],
                'm1_ai_label': values[8],  # AI预测的m1阶段标签
                'm2_ai_label': values[9]   # AI预测的m2阶段标签
            }

            # 验证数据合理性
            if not self.验证数据合理性(data):
                return None

            return data
        except Exception as e:
            return None

    def 开始采集(self, duration=None):
        """
        开始采集数据

        参数:
            duration: 采集时长(秒)，None表示无限采集直到Ctrl+C
        """
        if not self.ser or not self.ser.is_open:
            print("✗ 串口未连接")
            return

        self.is_running = True
        self.start_time = time.time()

        # 打开CSV文件
        with open(self.output_file, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
            writer.writeheader()

            print(f"\n开始采集数据到: {self.output_file}")
            if duration:
                print(f"采集时长: {duration}秒")
            print("按 Ctrl+C 停止采集\n")

            try:
                while self.is_running:
                    # 检查时长
                    if duration and (time.time() - self.start_time) > duration:
                        print("\n已达到设定时长，停止采集")
                        break

                    # 读取一行
                    if self.ser.in_waiting:
                        try:
                            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                            # 解析数据
                            data = self.解析数据行(line)
                            if data:
                                writer.writerow(data)
                                self.sample_count += 1

                                # 每100个样本打印进度
                                if self.sample_count % 100 == 0:
                                    elapsed = time.time() - self.start_time
                                    rate = self.sample_count / elapsed if elapsed > 0 else 0
                                    invalid_rate = self.invalid_count / (self.sample_count + self.invalid_count) * 100 if (self.sample_count + self.invalid_count) > 0 else 0
                                    print(f"\r已采集: {self.sample_count} 样本 | "
                                          f"速率: {rate:.1f} Hz | "
                                          f"时长: {elapsed:.1f}s | "
                                          f"无效数据: {self.invalid_count} ({invalid_rate:.1f}%)", end='')
                            elif line.startswith('motors:'):
                                # 数据格式正确但验证失败
                                self.invalid_count += 1
                        except UnicodeDecodeError:
                            pass  # 忽略解码错误

            except KeyboardInterrupt:
                print("\n\n用户中断采集")

        self.is_running = False
        elapsed = time.time() - self.start_time
        total_packets = self.sample_count + self.invalid_count
        invalid_rate = self.invalid_count / total_packets * 100 if total_packets > 0 else 0
        print(f"\n\n采集完成:")
        print(f"  有效样本数: {self.sample_count}")
        print(f"  无效样本数: {self.invalid_count} ({invalid_rate:.2f}%)")
        print(f"  总时长: {elapsed:.1f}秒")
        print(f"  平均采样率: {self.sample_count/elapsed:.1f} Hz")
        print(f"  保存到: {self.output_file}")

    def 关闭(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ 串口已关闭")


def main():
    parser = argparse.ArgumentParser(description='ESP32 串口数据采集工具')
    parser.add_argument('--port', '-p', type=str, required=True,
                        help='串口号 (如 COM3 或 /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出CSV文件名 (默认: 自动生成时间戳文件名)')
    parser.add_argument('--duration', '-d', type=int, default=None,
                        help='采集时长(秒) (默认: 无限制)')
    parser.add_argument('--scene', '-s', type=str, choices=['平地', '爬楼'],
                        default='平地', help='采集场景 (平地/爬楼)')

    args = parser.parse_args()

    # 生成输出文件名
    if args.output is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        args.output = f'{args.scene}_数据_{timestamp}.csv'

    # 创建采集器
    采集器 = 串口数据采集器(
        port=args.port,
        baudrate=args.baudrate,
        output_file=args.output
    )

    # 连接串口
    if not 采集器.连接串口():
        sys.exit(1)

    # 开始采集
    try:
        采集器.开始采集(duration=args.duration)
    finally:
        采集器.关闭()


if __name__ == '__main__':
    main()
