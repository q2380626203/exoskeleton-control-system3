#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CSV数据回放服务器 - 通过TCP发送历史数据给ESP32进行算法复现

功能：
1. GUI界面选择CSV文件
2. 通过TCP连接发送给ESP32
3. 支持开始/暂停/停止回放控制
4. ESP32使用数据作为虚拟电机反馈，运行算法控制真实电机

协议格式（复用v10协议，Magic=0xBB66）：
    包头 (14字节):
      - magic: 0xBB66 (区分于上传的0xAA55)
      - device_id: 1
      - version: 10
      - seq: 1 (0-255循环)
      - flags: 1
          bit0: 最后一包标志
          bit1: has_roll
          bit4-7: 命令类型 (0=数据, 1=开始, 2=停止, 3=暂停)
      - count: 1 (数据条数, 0-100)
      - reserved: 1
      - base_timestamp: 6 (48bit毫秒时间戳)

    每条数据 (15字节, 不含roll):
      - time_offset_ms: uint16
      - pos1, pos2: int16×2
      - vel1, vel2: int16×2
      - torque1, torque2: int16×2
      - states: 1

    CRC8: 1字节

使用方法：
    python csv_replay_server.py

"""

import socket
import struct
import csv
import time
import os
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from threading import Thread, Event
import json

# 配置
SERVER_HOST = '0.0.0.0'
DEFAULT_PORT = 16385
SAMPLES_PER_PACKET = 100
CONFIG_FILE = 'replay_config.json'

# 协议常量
REPLAY_MAGIC = 0xBB66
PROTOCOL_VERSION = 10

# 命令类型 (flags高4位)
CMD_DATA = 0x00      # 数据包
CMD_START = 0x10     # 开始回放
CMD_STOP = 0x20      # 停止回放
CMD_PAUSE = 0x30     # 暂停回放
CMD_REQUEST = 0x40   # 请求更多数据 (ESP32->Server)

# 标志位 (flags低4位)
FLAG_LAST_PACKET = 0x01  # 最后一包
FLAG_HAS_ROLL = 0x02     # 包含roll数据

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


def load_csv_data(csv_file):
    """
    加载CSV文件数据
    返回: [(timestamp_ms, motor1_pos, motor1_vel, motor1_torque,
            motor2_pos, motor2_vel, motor2_torque,
            roll_left, roll_right, m1_state, m2_state), ...]
    """
    data = []
    with open(csv_file, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                # 时间戳转换为毫秒整数
                timestamp = float(row['timestamp'])
                timestamp_ms = int(timestamp * 1000)

                # 电机数据
                motor1_pos = float(row['motor1_pos'])
                motor1_vel = float(row['motor1_vel'])
                motor1_torque = float(row['motor1_torque'])
                motor2_pos = float(row['motor2_pos'])
                motor2_vel = float(row['motor2_vel'])
                motor2_torque = float(row['motor2_torque'])

                # roll数据（可能为空）
                roll_left = float(row['roll_left']) if row.get('roll_left', '').strip() else None
                roll_right = float(row['roll_right']) if row.get('roll_right', '').strip() else None

                # 状态标签
                m1_state = int(row.get('m1_state_label', 0))
                m2_state = int(row.get('m2_state_label', 0))

                data.append((
                    timestamp_ms,
                    motor1_pos, motor1_vel, motor1_torque,
                    motor2_pos, motor2_vel, motor2_torque,
                    roll_left, roll_right,
                    m1_state, m2_state
                ))
            except (ValueError, KeyError) as e:
                continue

    return data


def build_command_packet(cmd_type, device_id=1, seq=0):
    """
    构建命令包（无数据内容）
    cmd_type: CMD_START, CMD_STOP, CMD_PAUSE
    """
    flags = cmd_type  # 命令类型在高4位
    count = 0  # 无数据

    # 包头 (14字节)
    header = struct.pack('<HBBBBBB',
        REPLAY_MAGIC,      # magic
        device_id,         # device_id
        PROTOCOL_VERSION,  # version
        seq & 0xFF,        # seq
        flags,             # flags (命令类型)
        count,             # count = 0
        0                  # reserved
    )
    # base_timestamp (6字节, 当前时间)
    base_time_ms = int(time.time() * 1000)
    header += struct.pack('<Q', base_time_ms)[:6]

    # CRC8
    crc = calc_crc8(header)
    packet = header + bytes([crc])

    return packet


def build_data_packet(samples, device_id=1, seq=0, is_last=False):
    """
    构建数据包
    samples: [(timestamp_ms, m1_pos, m1_vel, m1_torque, m2_pos, m2_vel, m2_torque,
               roll_left, roll_right, m1_state, m2_state), ...]
    """
    if not samples:
        return build_command_packet(CMD_DATA, device_id, seq)

    count = len(samples)

    # 检查是否有roll数据
    has_roll = any(s[7] is not None or s[8] is not None for s in samples)

    # 构建flags
    flags = CMD_DATA
    if is_last:
        flags |= FLAG_LAST_PACKET
    if has_roll:
        flags |= FLAG_HAS_ROLL

    # 基准时间戳（第一条数据的时间）
    base_time_ms = samples[0][0]

    # 包头 (14字节)
    header = struct.pack('<HBBBBBB',
        REPLAY_MAGIC,
        device_id,
        PROTOCOL_VERSION,
        seq & 0xFF,
        flags,
        count,
        0  # reserved
    )
    header += struct.pack('<Q', base_time_ms)[:6]

    # 数据部分
    data_bytes = b''
    for sample in samples:
        timestamp_ms, m1_pos, m1_vel, m1_torque, m2_pos, m2_vel, m2_torque, \
            roll_left, roll_right, m1_state, m2_state = sample

        # 计算时间偏移
        time_offset = timestamp_ms - base_time_ms
        if time_offset < 0:
            time_offset = 0
        if time_offset > 65535:
            time_offset = 65535

        # 状态合并
        states = ((m1_state & 0x0F) << 4) | (m2_state & 0x0F)

        if has_roll:
            # 19字节格式
            rl = int(roll_left * 100) if roll_left is not None else 0x7FFF
            rr = int(roll_right * 100) if roll_right is not None else 0x7FFF
            data_bytes += struct.pack('<HhhhhhhhhB',
                time_offset,
                int(m1_pos * 100),
                int(m2_pos * 100),
                int(m1_vel * 100),
                int(m2_vel * 100),
                int(m1_torque * 100),
                int(m2_torque * 100),
                rl,
                rr,
                states
            )
        else:
            # 15字节格式: time_offset(2) + pos1,pos2,vel1,vel2,torque1,torque2(12) + states(1)
            data_bytes += struct.pack('<HhhhhhhB',
                time_offset,
                int(m1_pos * 100),
                int(m2_pos * 100),
                int(m1_vel * 100),
                int(m2_vel * 100),
                int(m1_torque * 100),
                int(m2_torque * 100),
                states
            )

    # 组合并计算CRC
    packet_without_crc = header + data_bytes
    crc = calc_crc8(packet_without_crc)
    packet = packet_without_crc + bytes([crc])

    return packet


class ReplayServer:
    """CSV回放服务器（后端逻辑）"""

    def __init__(self, log_callback=None):
        self.port = DEFAULT_PORT
        self.data = []
        self.csv_file = None
        self.server_socket = None
        self.client_socket = None
        self.server_running = False
        self.replay_running = False
        self.stop_event = Event()
        self.pause_event = Event()
        self.request_event = Event()  # ESP32请求数据事件
        self.seq = 0
        self.log_callback = log_callback
        self.progress_callback = None
        self.status_callback = None

    def log(self, msg):
        if self.log_callback:
            self.log_callback(msg)
        else:
            print(msg)

    def set_progress_callback(self, callback):
        self.progress_callback = callback

    def set_status_callback(self, callback):
        self.status_callback = callback

    def update_status(self, status):
        if self.status_callback:
            self.status_callback(status)

    def load_csv(self, csv_file):
        """加载CSV数据"""
        self.csv_file = csv_file
        self.log(f"正在加载: {os.path.basename(csv_file)}")
        self.data = load_csv_data(csv_file)

        if not self.data:
            self.log("错误: 无法加载数据")
            return False

        # 计算时间跨度
        start_time = self.data[0][0]
        end_time = self.data[-1][0]
        duration_sec = (end_time - start_time) / 1000.0

        # 计算平均间隔
        avg_interval = 0
        if len(self.data) > 1:
            intervals = [self.data[i+1][0] - self.data[i][0] for i in range(len(self.data)-1)]
            avg_interval = sum(intervals) / len(intervals)

        self.log(f"已加载 {len(self.data)} 条数据")
        self.log(f"时间跨度: {duration_sec:.2f} 秒")
        self.log(f"平均间隔: {avg_interval:.2f} ms")

        return True

    def start_server(self):
        """启动TCP服务器"""
        if self.server_running:
            return True

        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((SERVER_HOST, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)
            self.server_running = True
            self.log(f"服务器已启动，监听端口: {self.port}")
            self.update_status("等待连接")

            # 启动接受连接的线程
            Thread(target=self._accept_loop, daemon=True).start()
            return True

        except Exception as e:
            self.log(f"服务器启动失败: {e}")
            return False

    def stop_server(self):
        """停止TCP服务器"""
        self.server_running = False
        self.stop_replay()

        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None

        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
            self.server_socket = None

        self.log("服务器已停止")
        self.update_status("已停止")

    def _accept_loop(self):
        """接受客户端连接的循环"""
        while self.server_running:
            try:
                conn, addr = self.server_socket.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(5.0)

                self.client_socket = conn
                self.log(f"客户端已连接: {addr[0]}:{addr[1]}")
                self.update_status("已连接")

                # 保持连接直到断开
                while self.server_running and self.client_socket:
                    try:
                        # 非阻塞检查连接状态和接收数据
                        conn.setblocking(False)
                        try:
                            data = conn.recv(1024)
                            if not data:
                                break
                            # 解析ESP32发来的请求包
                            self._parse_request(data)
                        except BlockingIOError:
                            pass
                        conn.setblocking(True)
                        time.sleep(0.05)
                    except:
                        break

                self.client_socket = None
                self.log("客户端已断开")
                self.update_status("等待连接")

            except socket.timeout:
                continue
            except Exception as e:
                if self.server_running:
                    self.log(f"连接错误: {e}")

    def _parse_request(self, data):
        """解析ESP32发来的请求包"""
        if len(data) < 15:
            return

        # 检查Magic
        magic = data[0] | (data[1] << 8)
        if magic != REPLAY_MAGIC:
            return

        # 检查命令类型
        flags = data[5]
        cmd = flags & 0xF0

        if cmd == CMD_REQUEST:
            # ESP32请求更多数据
            # buffer_count = data[8] | (data[9] << 8)  # ESP32当前缓冲区数据量
            self.request_event.set()  # 触发发送

    def send_packet(self, packet):
        """发送数据包"""
        if self.client_socket:
            try:
                self.client_socket.send(packet)
                return True
            except Exception as e:
                self.log(f"发送失败: {e}")
                return False
        return False

    def start_replay(self):
        """开始回放"""
        if not self.data:
            self.log("错误: 未加载数据")
            return False

        if not self.client_socket:
            self.log("错误: 无客户端连接")
            return False

        if self.replay_running:
            self.log("回放已在进行中")
            return False

        self.stop_event.clear()
        self.pause_event.clear()
        self.replay_running = True

        Thread(target=self._replay_thread, daemon=True).start()
        return True

    def pause_replay(self):
        """暂停/继续回放"""
        if not self.replay_running:
            return

        if self.pause_event.is_set():
            self.pause_event.clear()
            self.log("继续回放")
            self.update_status("回放中")

            # 发送继续命令
            packet = build_command_packet(CMD_PAUSE, seq=self.seq)
            self.seq += 1
            self.send_packet(packet)
        else:
            self.pause_event.set()
            self.log("暂停回放")
            self.update_status("已暂停")

            # 发送暂停命令
            packet = build_command_packet(CMD_PAUSE, seq=self.seq)
            self.seq += 1
            self.send_packet(packet)

    def stop_replay(self):
        """停止回放"""
        if not self.replay_running:
            return

        self.stop_event.set()
        self.pause_event.clear()

        # 发送停止命令
        if self.client_socket:
            packet = build_command_packet(CMD_STOP, seq=self.seq)
            self.seq += 1
            self.send_packet(packet)

        self.log("回放已停止")
        self.update_status("已连接")

    def _replay_thread(self):
        """回放数据线程"""
        self.log("开始回放...")
        self.update_status("回放中")

        # 发送开始命令
        start_packet = build_command_packet(CMD_START, seq=self.seq)
        self.seq += 1
        if not self.send_packet(start_packet):
            self.log("发送开始命令失败")
            self.replay_running = False
            return

        # 清除请求事件并等待ESP32准备好
        self.request_event.clear()
        time.sleep(0.2)

        # 分批发送数据
        total_samples = len(self.data)
        sent_samples = 0
        packet_count = 0

        # 首次发送3个包（300条数据）预填充ESP32缓冲区（缓冲区容量500条）
        initial_packets = 3
        for _ in range(initial_packets):
            if sent_samples >= total_samples or self.stop_event.is_set():
                break

            batch_end = min(sent_samples + SAMPLES_PER_PACKET, total_samples)
            batch = self.data[sent_samples:batch_end]
            is_last = (batch_end >= total_samples)

            packet = build_data_packet(batch, seq=self.seq, is_last=is_last)
            self.seq += 1

            if self.send_packet(packet):
                sent_samples = batch_end
                packet_count += 1
                progress = sent_samples * 100 // total_samples
                if self.progress_callback:
                    self.progress_callback(progress, sent_samples, total_samples)
            else:
                self.log("发送初始数据包失败")
                break

        self.log(f"已发送初始数据: {packet_count} 包, {sent_samples} 条")

        # 后续按请求发送
        while sent_samples < total_samples and not self.stop_event.is_set():
            # 检查暂停
            while self.pause_event.is_set() and not self.stop_event.is_set():
                time.sleep(0.1)

            if self.stop_event.is_set():
                break

            # 等待ESP32请求（带超时）
            request_received = self.request_event.wait(timeout=0.5)
            if not request_received:
                # 超时检查连接状态
                if not self.client_socket:
                    self.log("客户端断开连接")
                    break
                continue

            # 收到请求，清除事件并发送数据
            self.request_event.clear()

            # 取一批数据
            batch_end = min(sent_samples + SAMPLES_PER_PACKET, total_samples)
            batch = self.data[sent_samples:batch_end]
            is_last = (batch_end >= total_samples)

            # 构建并发送数据包
            packet = build_data_packet(batch, seq=self.seq, is_last=is_last)
            self.seq += 1

            if self.send_packet(packet):
                sent_samples = batch_end
                packet_count += 1

                # 更新进度
                progress = sent_samples * 100 // total_samples
                if self.progress_callback:
                    self.progress_callback(progress, sent_samples, total_samples)
            else:
                self.log("发送数据包失败，停止回放")
                break

        # 回放完成
        if not self.stop_event.is_set():
            self.log(f"回放完成: 共发送 {packet_count} 包, {sent_samples} 条数据")

            # 发送停止命令
            stop_packet = build_command_packet(CMD_STOP, seq=self.seq)
            self.seq += 1
            self.send_packet(stop_packet)

        self.replay_running = False
        if self.progress_callback:
            self.progress_callback(0, 0, total_samples)
        self.update_status("已连接" if self.client_socket else "等待连接")


class ReplayGUI:
    """回放服务器GUI界面"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("CSV回放服务器")
        self.root.geometry("600x500")
        self.root.resizable(True, True)

        self.server = ReplayServer(log_callback=self.log)
        self.server.set_progress_callback(self.update_progress)
        self.server.set_status_callback(self.update_status)

        self.setup_ui()
        self.load_config()

    def setup_ui(self):
        """设置UI界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ===== 文件选择区域 =====
        file_frame = ttk.LabelFrame(main_frame, text="CSV文件", padding="5")
        file_frame.pack(fill=tk.X, pady=(0, 10))

        self.file_path_var = tk.StringVar()
        file_entry = ttk.Entry(file_frame, textvariable=self.file_path_var, state='readonly')
        file_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))

        browse_btn = ttk.Button(file_frame, text="浏览...", command=self.browse_file)
        browse_btn.pack(side=tk.LEFT)

        # ===== 服务器设置区域 =====
        server_frame = ttk.LabelFrame(main_frame, text="服务器设置", padding="5")
        server_frame.pack(fill=tk.X, pady=(0, 10))

        port_frame = ttk.Frame(server_frame)
        port_frame.pack(fill=tk.X)

        ttk.Label(port_frame, text="监听端口:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))
        port_entry = ttk.Entry(port_frame, textvariable=self.port_var, width=10)
        port_entry.pack(side=tk.LEFT, padx=5)

        self.server_btn = ttk.Button(port_frame, text="启动服务器", command=self.toggle_server)
        self.server_btn.pack(side=tk.LEFT, padx=10)

        self.status_var = tk.StringVar(value="未启动")
        status_label = ttk.Label(port_frame, textvariable=self.status_var, foreground="gray")
        status_label.pack(side=tk.RIGHT)

        # ===== 回放控制区域 =====
        control_frame = ttk.LabelFrame(main_frame, text="回放控制", padding="5")
        control_frame.pack(fill=tk.X, pady=(0, 10))

        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(fill=tk.X)

        self.start_btn = ttk.Button(btn_frame, text="▶ 开始回放", command=self.start_replay, state=tk.DISABLED)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.pause_btn = ttk.Button(btn_frame, text="⏸ 暂停", command=self.pause_replay, state=tk.DISABLED)
        self.pause_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(btn_frame, text="⏹ 停止", command=self.stop_replay, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        # 进度条
        progress_frame = ttk.Frame(control_frame)
        progress_frame.pack(fill=tk.X, pady=(10, 0))

        self.progress_var = tk.IntVar(value=0)
        self.progress_bar = ttk.Progressbar(progress_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))

        self.progress_label = ttk.Label(progress_frame, text="0 / 0")
        self.progress_label.pack(side=tk.RIGHT)

        # ===== 日志区域 =====
        log_frame = ttk.LabelFrame(main_frame, text="日志", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True)

        # 日志文本框
        log_scroll = ttk.Scrollbar(log_frame)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.log_text = tk.Text(log_frame, height=10, state=tk.DISABLED, yscrollcommand=log_scroll.set)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        log_scroll.config(command=self.log_text.yview)

        # 清空日志按钮
        clear_btn = ttk.Button(log_frame, text="清空日志", command=self.clear_log)
        clear_btn.pack(anchor=tk.E, pady=(5, 0))

        # ===== 窗口关闭处理 =====
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def browse_file(self):
        """浏览选择CSV文件"""
        initial_dir = os.path.dirname(self.file_path_var.get()) if self.file_path_var.get() else os.getcwd()

        file_path = filedialog.askopenfilename(
            title="选择CSV文件",
            initialdir=initial_dir,
            filetypes=[("CSV文件", "*.csv"), ("所有文件", "*.*")]
        )

        if file_path:
            self.file_path_var.set(file_path)
            self.server.load_csv(file_path)
            self.update_button_states()
            self.save_config()

    def toggle_server(self):
        """启动/停止服务器"""
        if self.server.server_running:
            self.server.stop_server()
            self.server_btn.config(text="启动服务器")
        else:
            try:
                self.server.port = int(self.port_var.get())
            except ValueError:
                messagebox.showerror("错误", "端口号必须是数字")
                return

            if self.server.start_server():
                self.server_btn.config(text="停止服务器")
                self.save_config()

        self.update_button_states()

    def start_replay(self):
        """开始回放"""
        if self.server.start_replay():
            self.update_button_states()

    def pause_replay(self):
        """暂停/继续回放"""
        self.server.pause_replay()

        if self.server.pause_event.is_set():
            self.pause_btn.config(text="▶ 继续")
        else:
            self.pause_btn.config(text="⏸ 暂停")

    def stop_replay(self):
        """停止回放"""
        self.server.stop_replay()
        self.pause_btn.config(text="⏸ 暂停")
        self.update_button_states()

    def update_button_states(self):
        """更新按钮状态"""
        has_data = len(self.server.data) > 0
        server_running = self.server.server_running
        has_client = self.server.client_socket is not None
        replay_running = self.server.replay_running

        # 开始按钮：需要数据、服务器运行、客户端连接、未在回放
        self.start_btn.config(state=tk.NORMAL if (has_data and server_running and has_client and not replay_running) else tk.DISABLED)

        # 暂停按钮：回放中
        self.pause_btn.config(state=tk.NORMAL if replay_running else tk.DISABLED)

        # 停止按钮：回放中
        self.stop_btn.config(state=tk.NORMAL if replay_running else tk.DISABLED)

    def update_progress(self, progress, current, total):
        """更新进度条"""
        self.progress_var.set(progress)
        self.progress_label.config(text=f"{current} / {total}")
        self.update_button_states()

    def update_status(self, status):
        """更新状态显示"""
        self.status_var.set(status)

        # 根据状态更新颜色
        color_map = {
            "未启动": "gray",
            "等待连接": "orange",
            "已连接": "green",
            "回放中": "blue",
            "已暂停": "purple",
            "已停止": "gray"
        }
        # 由于ttk.Label不直接支持foreground更新，这里简化处理
        self.update_button_states()

    def log(self, msg):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def clear_log(self):
        """清空日志"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)

    def load_config(self):
        """加载配置"""
        try:
            config_path = os.path.join(os.path.dirname(__file__), CONFIG_FILE)
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    if 'csv_file' in config and os.path.exists(config['csv_file']):
                        self.file_path_var.set(config['csv_file'])
                        self.server.load_csv(config['csv_file'])
                    if 'port' in config:
                        self.port_var.set(str(config['port']))
        except Exception as e:
            pass

    def save_config(self):
        """保存配置"""
        try:
            config = {
                'csv_file': self.file_path_var.get(),
                'port': int(self.port_var.get())
            }
            config_path = os.path.join(os.path.dirname(__file__), CONFIG_FILE)
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=2)
        except Exception as e:
            pass

    def on_closing(self):
        """窗口关闭处理"""
        self.server.stop_server()
        self.save_config()
        self.root.destroy()

    def run(self):
        """运行GUI"""
        self.root.mainloop()


def main():
    app = ReplayGUI()
    app.run()


if __name__ == '__main__':
    main()
