#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32电机数据TCP接收服务器 (GUI版本) - 协议v10

功能：
1. 图形界面控制服务器启动/停止
2. 可配置端口号
3. 手动创建新CSV文件
4. 每天自动创建新CSV文件
5. 实时显示TCP数据（可开启/关闭/清空）
6. 多设备支持

使用方法：
    python tcp_data_server_gui.py
"""

import socket
import struct
import os
import sys
import time
import csv
from datetime import datetime, date
from threading import Thread, Lock
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import queue

# 配置
DEFAULT_PORT = 16384
BUFFER_SIZE = 16384
CSV_OUTPUT_DIR = './data'

# 协议v10数据包格式
PACKET_MAGIC = 0xAA55
PACKET_HEADER_SIZE = 14
SAMPLE_SIZE_NO_ROLL = 15
SAMPLE_SIZE_WITH_ROLL = 19
CRC_SIZE = 1

SAMPLES_PER_PACKET = 100
PACKET_SIZE_NO_ROLL = PACKET_HEADER_SIZE + SAMPLES_PER_PACKET * SAMPLE_SIZE_NO_ROLL + CRC_SIZE
PACKET_SIZE_WITH_ROLL = PACKET_HEADER_SIZE + SAMPLES_PER_PACKET * SAMPLE_SIZE_WITH_ROLL + CRC_SIZE

# 标志位定义
FLAG_SYNC = 0x01
FLAG_HAS_ROLL = 0x02

# 蓝牙IMU未连接标记值
BT_IMU_NOT_CONNECTED = 0x7FFF

# 对时协议定义
TIME_SYNC_REQUEST = bytes([0xAA, 0xCC])
TIME_SYNC_CONFIRM = bytes([0xBB, 0xBB])
TIME_SYNC_RESPONSE_SIZE = 10

# 通道名称
CHANNEL_NAMES = [
    'motor1_pos', 'motor1_vel', 'motor1_torque',
    'motor2_pos', 'motor2_vel', 'motor2_torque',
    'roll_left', 'roll_right',
    'm1_state_label', 'm2_state_label'
]

# CRC8查表
CRC8_TABLE = [
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
]


def calc_crc8(data: bytes) -> int:
    """计算CRC8校验值"""
    crc = 0x00
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


class DeviceInfo:
    """设备信息类"""
    def __init__(self, device_id, csv_file):
        self.device_id = device_id
        self.csv_file = csv_file
        self.packets_received = 0
        self.samples_received = 0
        self.bytes_received = 0
        self.packets_lost = 0
        self.last_seq = -1
        self.first_seen_time = time.time()
        self.last_packet_time = time.time()
        self.current_addr = None
        self.last_interval_ms = 0
        self.current_date = date.today()  # 记录当前CSV文件的日期


def get_server_time_ms():
    """获取服务器当前时间戳（毫秒）"""
    return int(time.time() * 1000)


def build_time_sync_response():
    """构建对时响应包"""
    timestamp_ms = get_server_time_ms()
    return bytes([0xCC, 0xAA]) + struct.pack('<q', timestamp_ms)


def create_csv_file_for_device(device_id):
    """为指定设备创建新的CSV文件"""
    if not os.path.exists(CSV_OUTPUT_DIR):
        os.makedirs(CSV_OUTPUT_DIR)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = os.path.join(CSV_OUTPUT_DIR, f'device_{device_id}_{timestamp}.csv')

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ['timestamp', 'seq'] + CHANNEL_NAMES
        writer.writerow(header)

    return filename


def check_and_rotate_csv(device):
    """检查是否需要创建新的CSV文件（每天自动切换）"""
    today = date.today()
    if device.current_date != today:
        # 新的一天，创建新文件
        old_file = device.csv_file
        device.csv_file = create_csv_file_for_device(device.device_id)
        device.current_date = today
        return old_file, device.csv_file
    return None, None


def parse_packet_v10(data):
    """解析协议v10数据包"""
    if len(data) < PACKET_SIZE_NO_ROLL:
        return None

    magic, device_id, version, seq, flags, interval_ms, reserved = struct.unpack('<HBBBBBB', data[:8])

    if magic != PACKET_MAGIC or version != 10:
        return None

    has_roll = (flags & FLAG_HAS_ROLL) != 0
    sync_flag = (flags & FLAG_SYNC) != 0

    sample_size = SAMPLE_SIZE_WITH_ROLL if has_roll else SAMPLE_SIZE_NO_ROLL
    expected_size = PACKET_SIZE_WITH_ROLL if has_roll else PACKET_SIZE_NO_ROLL

    if len(data) < expected_size:
        return None

    # 解析基准时间戳
    base_time_bytes = data[8:14]
    base_time_ms = struct.unpack('<Q', base_time_bytes + b'\x00\x00')[0] & 0xFFFFFFFFFFFF

    # 验证CRC8
    crc_data = data[:expected_size - 1]
    crc_received = data[expected_size - 1]
    crc_calculated = calc_crc8(crc_data)

    if crc_received != crc_calculated:
        return {
            'crc_error': True,
            'device_id': device_id,
            'seq': seq,
            'crc_received': crc_received,
            'crc_calculated': crc_calculated
        }

    # 解析采样数据
    samples = []
    offset = PACKET_HEADER_SIZE
    for i in range(SAMPLES_PER_PACKET):
        if has_roll:
            time_offset, pos1, pos2, vel1, vel2, torque1, torque2, roll_left_raw, roll_right_raw, states = \
                struct.unpack('<HhhhhhhhhB', data[offset:offset + sample_size])
            roll_left = None if roll_left_raw == BT_IMU_NOT_CONNECTED else roll_left_raw / 100.0
            roll_right = None if roll_right_raw == BT_IMU_NOT_CONNECTED else roll_right_raw / 100.0
        else:
            time_offset, pos1, pos2, vel1, vel2, torque1, torque2, states = \
                struct.unpack('<HhhhhhhB', data[offset:offset + sample_size])
            roll_left = None
            roll_right = None

        motor1_pos = pos1 / 100.0
        motor2_pos = pos2 / 100.0
        motor1_vel = vel1 / 100.0
        motor2_vel = vel2 / 100.0
        motor1_torque = torque1 / 100.0
        motor2_torque = torque2 / 100.0

        m1_state = (states >> 4) & 0x0F
        m2_state = states & 0x0F

        timestamp_ms = base_time_ms + time_offset

        channels = [
            motor1_pos, motor1_vel, motor1_torque,
            motor2_pos, motor2_vel, motor2_torque,
            roll_left, roll_right,
            m1_state, m2_state
        ]

        samples.append({
            'timestamp': timestamp_ms,
            'seq': seq,
            'device_id': device_id,
            'values': channels
        })
        offset += sample_size

    return {
        'magic': magic,
        'device_id': device_id,
        'seq': seq,
        'version': version,
        'sync_flag': sync_flag,
        'has_roll': has_roll,
        'interval_ms': interval_ms,
        'base_time_ms': base_time_ms,
        'count': SAMPLES_PER_PACKET,
        'samples': samples,
        'packet_size': expected_size,
        'crc_ok': True
    }


def save_to_csv(csv_file, samples, base_time_ms):
    """保存数据到CSV文件"""
    if base_time_ms == 0:
        return

    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        for sample in samples:
            values = ['' if v is None else v for v in sample['values']]
            timestamp_sec = sample['timestamp'] / 1000.0
            row = [f"{timestamp_sec:.3f}", sample['seq']] + values
            writer.writerow(row)


class TCPServerGUI:
    """TCP服务器GUI主类"""
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 TCP数据服务器 - 协议v10")
        self.root.geometry("1000x700")

        # 服务器状态
        self.server_running = False
        self.server_socket = None
        self.server_thread = None
        self.port = DEFAULT_PORT

        # 设备管理
        self.devices = {}
        self.devices_lock = Lock()
        self.crc_errors = 0
        self.crc_errors_lock = Lock()

        # 数据显示控制
        self.show_data = tk.BooleanVar(value=False)
        self.log_queue = queue.Queue()

        # 创建UI
        self.create_widgets()

        # 启动日志更新
        self.update_log()

        # 启动统计更新
        self.update_stats()

    def create_widgets(self):
        """创建UI组件"""
        # 顶部控制面板
        control_frame = ttk.LabelFrame(self.root, text="服务器控制", padding=10)
        control_frame.pack(fill=tk.X, padx=10, pady=5)

        # 端口设置
        ttk.Label(control_frame, text="端口:").grid(row=0, column=0, padx=5)
        self.port_entry = ttk.Entry(control_frame, width=10)
        self.port_entry.insert(0, str(DEFAULT_PORT))
        self.port_entry.grid(row=0, column=1, padx=5)

        # 启动/停止按钮
        self.start_btn = ttk.Button(control_frame, text="启动服务器", command=self.start_server)
        self.start_btn.grid(row=0, column=2, padx=5)

        self.stop_btn = ttk.Button(control_frame, text="停止服务器", command=self.stop_server, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=3, padx=5)

        # 创建新CSV按钮
        self.new_csv_btn = ttk.Button(control_frame, text="创建新CSV文件", command=self.create_new_csv, state=tk.DISABLED)
        self.new_csv_btn.grid(row=0, column=4, padx=5)

        # 服务器状态标签
        self.status_label = ttk.Label(control_frame, text="状态: 未运行", foreground="red")
        self.status_label.grid(row=0, column=5, padx=20)

        # 统计信息面板
        stats_frame = ttk.LabelFrame(self.root, text="设备统计", padding=10)
        stats_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 创建表格
        columns = ("设备ID", "状态", "包数", "样本数", "丢包", "采样率", "速率", "CSV文件")
        self.stats_tree = ttk.Treeview(stats_frame, columns=columns, show='headings', height=8)

        for col in columns:
            self.stats_tree.heading(col, text=col)
            if col == "CSV文件":
                self.stats_tree.column(col, width=200)
            else:
                self.stats_tree.column(col, width=80)

        self.stats_tree.pack(fill=tk.BOTH, expand=True)

        # 数据显示面板
        data_frame = ttk.LabelFrame(self.root, text="TCP数据显示", padding=10)
        data_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 数据显示控制
        control_bar = ttk.Frame(data_frame)
        control_bar.pack(fill=tk.X, pady=(0, 5))

        ttk.Checkbutton(control_bar, text="显示数据", variable=self.show_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_bar, text="清空显示", command=self.clear_log).pack(side=tk.LEFT, padx=5)

        # 日志文本框
        self.log_text = scrolledtext.ScrolledText(data_frame, height=10, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def log(self, message):
        """添加日志消息到队列"""
        self.log_queue.put(message)

    def update_log(self):
        """更新日志显示"""
        try:
            while True:
                message = self.log_queue.get_nowait()
                # 总是处理消息，只是根据show_data决定是否显示
                if self.show_data.get():
                    self.log_text.config(state=tk.NORMAL)
                    self.log_text.insert(tk.END, message + '\n')
                    self.log_text.see(tk.END)
                    self.log_text.config(state=tk.DISABLED)
                # 如果不显示，消息会被丢弃（这是正常的）
        except queue.Empty:
            pass
        self.root.after(100, self.update_log)

    def clear_log(self):
        """清空日志显示"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)

    def start_server(self):
        """启动服务器"""
        try:
            self.port = int(self.port_entry.get())
            if self.port < 1024 or self.port > 65535:
                messagebox.showerror("错误", "端口号必须在1024-65535之间")
                return
        except ValueError:
            messagebox.showerror("错误", "无效的端口号")
            return

        self.server_running = True
        self.server_thread = Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.new_csv_btn.config(state=tk.NORMAL)
        self.port_entry.config(state=tk.DISABLED)
        self.status_label.config(text=f"状态: 运行中 (端口 {self.port})", foreground="green")
        self.log(f"服务器已启动，监听端口 {self.port}")

    def stop_server(self):
        """停止服务器"""
        self.server_running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.new_csv_btn.config(state=tk.DISABLED)
        self.port_entry.config(state=tk.NORMAL)
        self.status_label.config(text="状态: 未运行", foreground="red")
        self.log("服务器已停止")

    def create_new_csv(self):
        """手动为所有设备创建新的CSV文件"""
        count = 0

        # 先检查是否有设备（在锁外检查以避免卡死）
        with self.devices_lock:
            if not self.devices:
                device_count = 0
            else:
                device_count = len(self.devices)

        if device_count == 0:
            messagebox.showinfo("提示", "当前没有连接的设备")
            return

        # 创建新CSV文件并重置统计数据
        with self.devices_lock:
            for device in self.devices.values():
                device.csv_file = create_csv_file_for_device(device.device_id)

                # 重置统计数据
                device.packets_received = 0
                device.samples_received = 0
                device.bytes_received = 0
                device.packets_lost = 0
                device.first_seen_time = time.time()
                device.last_seq = -1

                self.log(f"设备{device.device_id}: 创建新CSV文件 {os.path.basename(device.csv_file)}，统计数据已清零")
                count += 1

        # 在锁外弹出消息框
        messagebox.showinfo("成功", f"已为 {count} 个设备创建新的CSV文件")

    def update_stats(self):
        """更新统计信息显示"""
        with self.devices_lock:
            # 清空表格
            for item in self.stats_tree.get_children():
                self.stats_tree.delete(item)

            current_time = time.time()
            for device in sorted(self.devices.values(), key=lambda d: d.device_id):
                is_active = (current_time - device.last_packet_time) < 5
                status = "🟢" if is_active else "🔴"

                duration = current_time - device.first_seen_time
                kb_per_sec = device.bytes_received / duration / 1024 if duration > 0 else 0

                if device.last_interval_ms > 0:
                    sample_rate_hz = 1000.0 / device.last_interval_ms
                    sample_rate_str = f"{sample_rate_hz:.0f}Hz"
                else:
                    sample_rate_str = "N/A"

                csv_name = os.path.basename(device.csv_file)

                self.stats_tree.insert('', tk.END, values=(
                    f"设备{device.device_id}",
                    status,
                    device.packets_received,
                    device.samples_received,
                    device.packets_lost,
                    sample_rate_str,
                    f"{kb_per_sec:.1f}KB/s",
                    csv_name
                ))

        self.root.after(1000, self.update_stats)

    def get_or_create_device(self, device_id):
        """获取或创建设备信息"""
        with self.devices_lock:
            if device_id not in self.devices:
                csv_file = create_csv_file_for_device(device_id)
                self.devices[device_id] = DeviceInfo(device_id, csv_file)
                self.log(f"设备{device_id}: 新设备已注册")
            return self.devices[device_id]

    def run_server(self):
        """运行服务器主循环"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.settimeout(1.0)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.listen(10)

            self.log(f"服务器监听: 0.0.0.0:{self.port}")
            self.log(f"数据目录: {os.path.abspath(CSV_OUTPUT_DIR)}")

            while self.server_running:
                try:
                    conn, addr = self.server_socket.accept()
                    conn.settimeout(5.0)
                    client_thread = Thread(target=self.handle_client, args=(conn, addr), daemon=True)
                    client_thread.start()
                except socket.timeout:
                    continue
                except OSError:
                    break

        except Exception as e:
            self.log(f"服务器错误: {e}")
            messagebox.showerror("服务器错误", str(e))
        finally:
            if self.server_socket:
                self.server_socket.close()

    def handle_client(self, conn, addr):
        """处理客户端连接"""
        client_id = f"{addr[0]}:{addr[1]}"
        self.log(f"[{client_id}] 客户端已连接")

        buffer = b''
        current_device = None
        time_sync_done = False
        handshake_step = 0
        sync_request_count = 0

        def is_valid_data_packet_v10(data, pos):
            """验证是否是有效的协议v10数据包"""
            if pos + PACKET_HEADER_SIZE > len(data):
                return False
            if data[pos] != 0x55 or data[pos + 1] != 0xAA:
                return False
            device_id = data[pos + 2]
            if device_id == 0 or device_id > 100:
                return False
            version = data[pos + 3]
            if version != 10:
                return False
            flags = data[pos + 5]
            if flags & 0xFC:
                return False
            return True

        def get_packet_size(data, pos):
            """根据flags判断包大小"""
            if pos + 6 > len(data):
                return PACKET_SIZE_NO_ROLL
            flags = data[pos + 5]
            has_roll = (flags & FLAG_HAS_ROLL) != 0
            return PACKET_SIZE_WITH_ROLL if has_roll else PACKET_SIZE_NO_ROLL

        try:
            while self.server_running:
                # 接收数据
                try:
                    data = conn.recv(BUFFER_SIZE)
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    self.log(f"[{client_id}] 连接被重置")
                    break

                if not data:
                    self.log(f"[{client_id}] 客户端断开连接")
                    break

                buffer += data

                # 处理缓冲区中的所有数据包
                while len(buffer) >= 2:
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                    # 检查对时请求 (AA CC)
                    if buffer[:2] == TIME_SYNC_REQUEST:
                        sync_request_count += 1
                        try:
                            response = build_time_sync_response()
                            conn.send(response)
                            handshake_step = 1
                            server_time = get_server_time_ms()
                            self.log(f"[{timestamp}] [{client_id}] 对时请求 #{sync_request_count}, 已回复时间戳({server_time})")
                        except Exception as e:
                            self.log(f"[{timestamp}] [{client_id}] 对时响应失败: {e}")
                        buffer = buffer[2:]
                        continue

                    # 检查对时确认 (BB BB)
                    if buffer[:2] == TIME_SYNC_CONFIRM:
                        if handshake_step == 1:
                            time_sync_done = True
                            handshake_step = 2
                            self.log(f"[{timestamp}] [{client_id}] 对时确认，三次握手完成!")
                        buffer = buffer[2:]
                        continue

                    # 检查协议v10数据包
                    if len(buffer) < PACKET_HEADER_SIZE:
                        break

                    # 查找有效的数据包头
                    magic_pos = -1
                    for i in range(len(buffer) - 1):
                        if buffer[i] == 0x55 and buffer[i + 1] == 0xAA:
                            if is_valid_data_packet_v10(buffer, i):
                                magic_pos = i
                                break

                    if magic_pos == -1:
                        if len(buffer) > 14:
                            buffer = buffer[-14:]
                        break

                    if magic_pos > 0:
                        buffer = buffer[magic_pos:]

                    packet_size = get_packet_size(buffer, 0)

                    if len(buffer) < packet_size:
                        break

                    packet_data = buffer[:packet_size]
                    buffer = buffer[packet_size:]

                    packet = parse_packet_v10(packet_data)
                    if packet is None:
                        continue

                    # 检查CRC错误
                    if packet.get('crc_error'):
                        with self.crc_errors_lock:
                            self.crc_errors += 1
                        self.log(f"[{client_id}] CRC错误: seq={packet['seq']}")
                        continue

                    # 获取或创建设备
                    device_id = packet['device_id']
                    sync_flag = packet['sync_flag']

                    if current_device is None or current_device.device_id != device_id:
                        current_device = self.get_or_create_device(device_id)
                        current_device.current_addr = addr
                        self.log(f"[{client_id}] 识别为设备{device_id} (协议v10, {packet['packet_size']}字节)")

                    if sync_flag and not time_sync_done:
                        time_sync_done = True
                        self.log(f"[{client_id}] 设备时间已同步")

                    # 检查是否需要切换到新的CSV文件（每天自动）
                    old_file, new_file = check_and_rotate_csv(current_device)
                    if old_file and new_file:
                        self.log(f"设备{device_id}: 新的一天，切换CSV文件")
                        self.log(f"  旧文件: {os.path.basename(old_file)}")
                        self.log(f"  新文件: {os.path.basename(new_file)}")

                    # 更新设备统计信息
                    current_device.packets_received += 1
                    current_device.samples_received += packet['count']
                    current_device.bytes_received += packet['packet_size']
                    current_device.last_packet_time = time.time()
                    current_device.current_addr = addr
                    current_device.last_interval_ms = packet['interval_ms']

                    # 检测丢包
                    if current_device.last_seq >= 0:
                        expected_seq = (current_device.last_seq + 1) & 0xFF
                        if packet['seq'] != expected_seq:
                            lost = (packet['seq'] - expected_seq) & 0xFF
                            if lost < 128:
                                current_device.packets_lost += lost
                    current_device.last_seq = packet['seq']

                    # 保存到CSV
                    save_to_csv(current_device.csv_file, packet['samples'], packet['base_time_ms'])

        except Exception as e:
            self.log(f"[{client_id}] 处理数据时出错: {e}")
        finally:
            conn.close()
            if current_device:
                self.log(f"[{client_id}] 连接已关闭，设备{current_device.device_id}")
            else:
                self.log(f"[{client_id}] 连接已关闭（未识别到设备）")


def main():
    """主程序入口"""
    root = tk.Tk()
    app = TCPServerGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()

