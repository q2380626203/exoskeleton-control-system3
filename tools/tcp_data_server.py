#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32电机数据TCP接收服务器 (多设备版) - 协议v10

功能：
1. 监听本地TCP端口，接收多个ESP32发送的电机数据
2. 每个设备独立CSV文件（按IP区分）
3. 独立的统计信息和丢包检测
4. 实时显示所有设备状态

使用方法：
    python tcp_data_server.py

配置：
    - 本地监听端口: 25565 (frp映射到此端口)
    - CSV输出目录: ./data/

协议v10格式 (精确时间戳版):
    包头 (14字节):
      - magic: 2 (0xAA55)
      - device_id: 1
      - version: 1 (10)
      - seq: 1 (0-255循环)
      - flags: 1 (bit0=sync_flag, bit1=has_roll)
      - interval_ms: 1 (实际采样间隔，仅供参考)
      - reserved: 1
      - base_timestamp: 6 (48bit毫秒时间戳，第一条数据的真实时间)

    每条数据 - 不含roll (15字节):
      - time_offset_ms: uint16 相对于base_timestamp的毫秒偏移 (0~65535ms)
      - pos1, pos2: int16×2 位置×100
      - vel1, vel2: int16×2 速度×100
      - torque1, torque2: int16×2 转矩×100
      - states: 1 (高4位m1_state, 低4位m2_state)

    每条数据 - 含roll (19字节, has_roll=1时):
      - time_offset_ms: uint16 相对于base_timestamp的毫秒偏移 (0~65535ms)
      - pos1, pos2: int16×2 位置×100
      - vel1, vel2: int16×2 速度×100
      - torque1, torque2: int16×2 转矩×100
      - roll_left, roll_right: int16×2 陀螺仪roll角度×100
      - states: 1 (高4位m1_state, 低4位m2_state)

    CRC: 1字节 (CRC8)

    不带roll: 14 + 100×15 + 1 = 1515字节
    带roll:   14 + 100×19 + 1 = 1915字节
"""

import socket
import struct
import os
import sys
import time
import csv
from datetime import datetime
from threading import Thread, Lock
import signal

# 配置
SERVER_HOST = '0.0.0.0'  # 监听所有网卡
SERVER_PORT = 16384       # 本地端口（frp内网穿透映射到此端口）
BUFFER_SIZE = 16384       # 接收缓冲区大小
CSV_OUTPUT_DIR = './data'

# 协议v10数据包格式 (精确时间戳版)
PACKET_MAGIC = 0xAA55
PACKET_HEADER_SIZE = 14   # magic(2) + device_id(1) + version(1) + seq(1) + flags(1) + interval_ms(1) + reserved(1) + base_time(6)
SAMPLE_SIZE_NO_ROLL = 15  # time_offset(2) + pos1(2) + pos2(2) + vel1(2) + vel2(2) + torque1(2) + torque2(2) + states(1)
SAMPLE_SIZE_WITH_ROLL = 19  # time_offset(2) + pos1(2) + pos2(2) + vel1(2) + vel2(2) + torque1(2) + torque2(2) + roll_left(2) + roll_right(2) + states(1)
CRC_SIZE = 1              # CRC8

# 每包100条数据
SAMPLES_PER_PACKET = 100
PACKET_SIZE_NO_ROLL = PACKET_HEADER_SIZE + SAMPLES_PER_PACKET * SAMPLE_SIZE_NO_ROLL + CRC_SIZE      # 1515字节
PACKET_SIZE_WITH_ROLL = PACKET_HEADER_SIZE + SAMPLES_PER_PACKET * SAMPLE_SIZE_WITH_ROLL + CRC_SIZE  # 1915字节

# 标志位定义
FLAG_SYNC = 0x01      # bit0: 时间已同步
FLAG_HAS_ROLL = 0x02  # bit1: 包含roll数据

# 蓝牙IMU未连接标记值
BT_IMU_NOT_CONNECTED = 0x7FFF  # int16最大正值，表示未连接

# 对时协议定义（三次握手）
TIME_SYNC_REQUEST = bytes([0xAA, 0xCC])    # 设备发送的对时请求
TIME_SYNC_CONFIRM = bytes([0xBB, 0xBB])    # 设备发送的对时确认
TIME_SYNC_RESPONSE_SIZE = 10               # CC AA + 8字节时间戳

# 通道名称
CHANNEL_NAMES = [
    'motor1_pos', 'motor1_vel', 'motor1_torque',
    'motor2_pos', 'motor2_vel', 'motor2_torque',
    'roll_left', 'roll_right',  # 蓝牙IMU roll角度，None表示未连接
    'm1_state_label', 'm2_state_label'  # 状态标签: 0=空闲, 1=抬腿, 2=压腿, 3=检测速度触发
]

# 客户端管理 - 按设备ID分组
clients = {}  # {client_id: ClientInfo}
clients_lock = Lock()
devices = {}  # {device_id: DeviceInfo} - 按设备ID管理
devices_lock = Lock()

# 全局CRC错误统计
crc_errors = 0
crc_errors_lock = Lock()

# 运行标志
running = True


# CRC8查表法（多项式0x07，初始值0x00）
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
    """设备信息类 - 按设备ID管理"""
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
        self.current_addr = None  # 当前连接的地址
        self.last_interval_ms = 0  # 最新的采样间隔(ms)


class ClientInfo:
    """客户端信息类"""
    def __init__(self, addr, csv_file):
        self.addr = addr
        self.client_id = f"{addr[0]}:{addr[1]}"
        self.csv_file = csv_file
        self.packets_received = 0
        self.samples_received = 0
        self.bytes_received = 0
        self.packets_lost = 0
        self.last_seq = -1
        self.connect_time = time.time()
        self.last_packet_time = time.time()
        self.connected = True


def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global running
    print('\n正在关闭服务器...')
    running = False


def create_csv_file_for_device(device_id):
    """为指定设备创建新的CSV文件"""
    # 确保输出目录存在
    if not os.path.exists(CSV_OUTPUT_DIR):
        os.makedirs(CSV_OUTPUT_DIR)

    # 生成文件名（包含设备ID）
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = os.path.join(CSV_OUTPUT_DIR, f'device_{device_id}_{timestamp}.csv')

    # 创建CSV文件并写入表头
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ['timestamp', 'seq'] + CHANNEL_NAMES  # timestamp格式: 毫秒时间戳
        writer.writerow(header)

    print(f'[设备{device_id}] 创建CSV文件: {filename}')
    return filename


def get_or_create_device(device_id):
    """获取或创建设备信息"""
    with devices_lock:
        if device_id not in devices:
            csv_file = create_csv_file_for_device(device_id)
            devices[device_id] = DeviceInfo(device_id, csv_file)
            print(f'[设备{device_id}] 新设备已注册')
        return devices[device_id]


def parse_packet_v10(data):
    """
    解析协议v10数据包 (精确时间戳版)

    包头格式 (14字节):
    - magic: uint16 (0xAA55)
    - device_id: uint8
    - version: uint8 (10)
    - seq: uint8 (0-255循环)
    - flags: uint8 (bit0=sync, bit1=has_roll)
    - interval_ms: uint8 (实际采样间隔，仅供参考)
    - reserved: uint8
    - base_time: 6字节 (48bit毫秒时间戳，小端序，第一条数据的真实时间)

    每条数据 - 不含roll (15字节):
    - time_offset_ms: uint16 相对于base_timestamp的毫秒偏移
    - pos1, pos2: int16×2
    - vel1, vel2: int16×2
    - torque1, torque2: int16×2
    - states: uint8 (高4位m1_state, 低4位m2_state)

    每条数据 - 含roll (19字节):
    - time_offset_ms: uint16 相对于base_timestamp的毫秒偏移
    - pos1, pos2: int16×2
    - vel1, vel2: int16×2
    - torque1, torque2: int16×2
    - roll_left, roll_right: int16×2
    - states: uint8 (高4位m1_state, 低4位m2_state)

    CRC8: 1字节

    返回值:
    - None: 长度不足或magic错误
    - {'crc_error': True, ...}: CRC校验失败
    - {..., 'crc_ok': True}: 解析成功
    """
    # 先检查最小包大小
    if len(data) < PACKET_SIZE_NO_ROLL:
        return None

    # 解析包头前8字节
    magic, device_id, version, seq, flags, interval_ms, reserved = struct.unpack('<HBBBBBB', data[:8])

    if magic != PACKET_MAGIC:
        return None

    # 检查协议版本
    if version != 10:
        return None

    # 判断是否有roll数据
    has_roll = (flags & FLAG_HAS_ROLL) != 0
    sync_flag = (flags & FLAG_SYNC) != 0

    # 确定包大小和每条数据大小
    if has_roll:
        sample_size = SAMPLE_SIZE_WITH_ROLL
        expected_size = PACKET_SIZE_WITH_ROLL
    else:
        sample_size = SAMPLE_SIZE_NO_ROLL
        expected_size = PACKET_SIZE_NO_ROLL

    if len(data) < expected_size:
        return None

    # 解析基准时间戳 (48bit，小端序)
    base_time_bytes = data[8:14]
    base_time_ms = struct.unpack('<Q', base_time_bytes + b'\x00\x00')[0] & 0xFFFFFFFFFFFF

    # 验证CRC8校验
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
            # 解析每条数据 (19字节，含roll)
            time_offset, pos1, pos2, vel1, vel2, torque1, torque2, roll_left_raw, roll_right_raw, states = struct.unpack('<HhhhhhhhhB', data[offset:offset + sample_size])
            roll_left = None if roll_left_raw == BT_IMU_NOT_CONNECTED else roll_left_raw / 100.0
            roll_right = None if roll_right_raw == BT_IMU_NOT_CONNECTED else roll_right_raw / 100.0
        else:
            # 解析每条数据 (15字节，不含roll)
            time_offset, pos1, pos2, vel1, vel2, torque1, torque2, states = struct.unpack('<HhhhhhhB', data[offset:offset + sample_size])
            roll_left = None
            roll_right = None

        # 还原电机数据 (÷100)
        motor1_pos = pos1 / 100.0
        motor2_pos = pos2 / 100.0
        motor1_vel = vel1 / 100.0
        motor2_vel = vel2 / 100.0
        motor1_torque = torque1 / 100.0
        motor2_torque = torque2 / 100.0

        # 解析状态标签
        m1_state = (states >> 4) & 0x0F
        m2_state = states & 0x0F

        # 计算时间戳 (基准时间 + 每条数据的真实偏移)
        timestamp_ms = base_time_ms + time_offset

        # 构造通道数据
        channels = [
            motor1_pos, motor1_vel, motor1_torque,
            motor2_pos, motor2_vel, motor2_torque,
            roll_left, roll_right,
            m1_state, m2_state
        ]

        samples.append({
            'timestamp': timestamp_ms,  # 毫秒时间戳
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
    """保存数据到CSV文件（过滤掉时间戳为0的未同步数据）"""
    # 如果基准时间戳为0，说明设备时间未同步，跳过整个包
    if base_time_ms == 0:
        return

    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        for sample in samples:
            # 将None值转为空字符串，便于CSV中表示未连接状态
            values = ['' if v is None else v for v in sample['values']]
            # 时间戳转换为 秒.毫秒 格式（便于阅读和分析）
            timestamp_sec = sample['timestamp'] / 1000.0
            row = [f"{timestamp_sec:.3f}", sample['seq']] + values
            writer.writerow(row)


def get_server_time_ms():
    """获取服务器当前时间戳（毫秒）"""
    return int(time.time() * 1000)


def build_time_sync_response():
    """
    构建对时响应包: CC AA + 8字节时间戳(毫秒,小端序)

    三次握手对时协议:
    1. 设备发送 AA CC (对时请求)
    2. 服务端回复 CC AA + 8字节时间戳 (对时响应) <- 本函数
    3. 设备发送 BB BB (对时确认)
    """
    timestamp_ms = get_server_time_ms()
    # CC AA + 8字节时间戳(小端序)
    return bytes([0xCC, 0xAA]) + struct.pack('<q', timestamp_ms)


def handle_client(conn, addr):
    """处理客户端连接"""
    global clients, devices, running, crc_errors

    client_id = f"{addr[0]}:{addr[1]}"
    print(f'[{client_id}] 客户端已连接，等待识别设备ID...')

    buffer = b''
    current_device = None  # 当前连接对应的设备
    response_count = 0     # 响应次数统计
    time_sync_done = False # 时间同步是否完成（只响应一次）
    local_crc_errors = 0   # 本连接的CRC错误计数

    # 串口对时握手状态
    handshake_step = 0     # 0=等待AA CC, 1=已发送响应等待BB BB, 2=完成
    sync_request_count = 0 # 对时请求计数

    def is_valid_data_packet_v10(data, pos):
        """验证是否是有效的协议v10数据包"""
        if pos + PACKET_HEADER_SIZE > len(data):
            return False
        # 检查magic (0xAA55 little-endian = 0x55 0xAA)
        if data[pos] != 0x55 or data[pos + 1] != 0xAA:
            return False
        # 检查device_id范围 (1-100)
        device_id = data[pos + 2]
        if device_id == 0 or device_id > 100:
            return False
        # 检查version (协议v10)
        version = data[pos + 3]
        if version != 10:
            return False
        # 检查flags (只使用bit0和bit1)
        flags = data[pos + 5]
        if flags & 0xFC:  # 高6位应该为0
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
        while running:
            # 接收数据
            try:
                data = conn.recv(BUFFER_SIZE)
            except socket.timeout:
                continue
            except ConnectionResetError:
                print(f'[{client_id}] 连接被重置')
                break

            if not data:
                print(f'[{client_id}] 客户端断开连接')
                break

            buffer += data

            # 处理缓冲区中的所有数据包
            while len(buffer) >= 2:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                # ========== 检查串口对时协议 ==========
                # 检查是否是对时请求 (AA CC) - 第一步
                if buffer[:2] == TIME_SYNC_REQUEST:
                    sync_request_count += 1

                    # 发送对时响应 (CC AA + 时间戳) - 第二步
                    try:
                        response = build_time_sync_response()
                        conn.send(response)
                        handshake_step = 1
                        server_time = get_server_time_ms()
                        print(f'[{timestamp}] [{client_id}] 收到对时请求 AA CC #{sync_request_count}, '
                              f'已回复 CC AA + 时间戳({server_time})')
                        response_count += 1
                    except Exception as e:
                        print(f'[{timestamp}] [{client_id}] 对时响应发送失败: {e}')

                    buffer = buffer[2:]  # 移除已处理的请求包
                    continue

                # 检查是否是对时确认 (BB BB) - 第三步
                if buffer[:2] == TIME_SYNC_CONFIRM:
                    if handshake_step == 1:
                        time_sync_done = True
                        handshake_step = 2
                        print(f'[{timestamp}] [{client_id}] 收到对时确认 BB BB, 三次握手对时完成!')
                    else:
                        print(f'[{timestamp}] [{client_id}] 收到意外的 BB BB (握手步骤={handshake_step})')

                    buffer = buffer[2:]  # 移除已处理的确认包
                    continue

                # ========== 检查协议v10数据包 ==========
                if len(buffer) < PACKET_HEADER_SIZE:
                    break  # 数据不足，等待更多

                # 查找有效的数据包头
                magic_pos = -1

                for i in range(len(buffer) - 1):
                    if buffer[i] == 0x55 and buffer[i + 1] == 0xAA:
                        if is_valid_data_packet_v10(buffer, i):
                            magic_pos = i
                            break

                if magic_pos == -1:
                    # 没有找到有效的包头，保留最后几个字节
                    if len(buffer) > 14:
                        buffer = buffer[-14:]
                    break

                # 丢弃包头之前的数据
                if magic_pos > 0:
                    buffer = buffer[magic_pos:]

                # 获取包大小
                packet_size = get_packet_size(buffer, 0)

                # 检查数据是否完整
                if len(buffer) < packet_size:
                    break  # 数据不完整，等待更多数据

                packet_data = buffer[:packet_size]
                buffer = buffer[packet_size:]

                packet = parse_packet_v10(packet_data)
                if packet is None:
                    continue

                # 检查CRC错误
                if packet.get('crc_error'):
                    local_crc_errors += 1
                    with crc_errors_lock:
                        crc_errors += 1
                    print(f'[{client_id}] CRC错误 #{local_crc_errors}: seq={packet["seq"]}, '
                          f'收到=0x{packet["crc_received"]:02X}, 计算=0x{packet["crc_calculated"]:02X}')
                    continue

                # 获取或创建设备
                device_id = packet['device_id']
                sync_flag = packet['sync_flag']

                if current_device is None or current_device.device_id != device_id:
                    current_device = get_or_create_device(device_id)
                    current_device.current_addr = addr
                    print(f'[{client_id}] 识别为设备 {device_id} (协议v10, 包大小={packet["packet_size"]}字节)')

                # 检查设备对时状态
                if sync_flag and not time_sync_done:
                    time_sync_done = True
                    print(f'[{client_id}] 设备时间已同步 (sync_flag=1)')

                # 更新设备统计信息
                current_device.packets_received += 1
                current_device.samples_received += packet['count']
                current_device.bytes_received += packet['packet_size']
                current_device.last_packet_time = time.time()
                current_device.current_addr = addr
                current_device.last_interval_ms = packet['interval_ms']  # 更新采样间隔

                # 检测丢包（seq是0-255循环）
                if current_device.last_seq >= 0:
                    expected_seq = (current_device.last_seq + 1) & 0xFF
                    if packet['seq'] != expected_seq:
                        lost = (packet['seq'] - expected_seq) & 0xFF
                        if lost < 128:  # 合理的丢包数（避免seq回绕误判）
                            current_device.packets_lost += lost
                current_device.last_seq = packet['seq']

                # 保存到设备对应的CSV（传入base_time_ms用于过滤未同步数据）
                save_to_csv(current_device.csv_file, packet['samples'], packet['base_time_ms'])

    except Exception as e:
        print(f'[{client_id}] 处理数据时出错: {e}')

    finally:
        conn.close()
        if current_device:
            print(f'[{client_id}] 连接已关闭，设备{current_device.device_id} 数据保存到: {current_device.csv_file}')
            print(f'[{client_id}] 设备{current_device.device_id} 统计: 包={current_device.packets_received}, 样本={current_device.samples_received}, 丢包={current_device.packets_lost}, CRC错误={local_crc_errors}, 响应={response_count}次')
        else:
            print(f'[{client_id}] 连接已关闭（未识别到设备, CRC错误={local_crc_errors}, 响应={response_count}次）')


def print_stats():
    """定期打印所有设备统计信息"""
    global devices, running, crc_errors

    start_time = time.time()

    while running:
        time.sleep(5)  # 每5秒打印一次

        with devices_lock:
            if not devices:
                continue

            current_time = time.time()
            elapsed = current_time - start_time

            # 统计活跃设备（最近5秒有数据）
            active_devices = [d for d in devices.values()
                            if (current_time - d.last_packet_time) < 5]

            with crc_errors_lock:
                current_crc_errors = crc_errors

            print(f'\n{"="*80}')
            print(f' 服务器运行时间: {elapsed:.1f}秒 | 活跃设备: {len(active_devices)}/{len(devices)} | CRC错误: {current_crc_errors}')
            print(f'{"="*80}')
            print(f' {"设备ID":^8} | {"状态":^4} | {"包数":^10} | {"样本数":^12} | {"丢包":^6} | {"采样率":^10} | {"速率":^10}')
            print(f'{"-"*80}')

            for device in sorted(devices.values(), key=lambda d: d.device_id):
                # 判断是否活跃（最近5秒有数据）
                is_active = (current_time - device.last_packet_time) < 5
                status = "🟢" if is_active else "🔴"

                duration = current_time - device.first_seen_time
                if duration > 0:
                    packets_per_sec = device.packets_received / duration
                    kb_per_sec = device.bytes_received / duration / 1024
                else:
                    packets_per_sec = 0
                    kb_per_sec = 0

                # 计算采样率 (1000ms / interval_ms)
                if device.last_interval_ms > 0:
                    sample_rate_hz = 1000.0 / device.last_interval_ms
                    sample_rate_str = f"{sample_rate_hz:.0f}Hz ({device.last_interval_ms}ms)"
                else:
                    sample_rate_str = "N/A"

                addr_str = ""
                if device.current_addr:
                    addr_str = f" ({device.current_addr[0]})"

                print(f' 设备 {device.device_id:<3}{addr_str:<18} | {status}  | '
                      f'{device.packets_received:>8} | '
                      f'{device.samples_received:>10} | '
                      f'{device.packets_lost:>5} | '
                      f'{sample_rate_str:>14} | '
                      f'{kb_per_sec:>6.1f} KB/s')

            print(f'{"="*80}\n')


def cleanup_disconnected():
    """清理函数（设备模式下不需要清理，保留统计）"""
    global running

    while running:
        time.sleep(60)
        # 设备模式下保留所有设备记录用于统计


def main():
    global running

    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)

    # 创建服务器socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(1.0)  # 设置超时以便能响应Ctrl+C

    try:
        server.bind((SERVER_HOST, SERVER_PORT))
        server.listen(10)  # 最多10个等待连接
        print(f'{"="*70}')
        print(f' ESP32 电机数据TCP服务器 (协议v10)')
        print(f'{"="*70}')
        print(f' 监听地址: {SERVER_HOST}:{SERVER_PORT}')
        print(f' FRP外网:  frp-any.com:18214')
        print(f' 数据目录: {os.path.abspath(CSV_OUTPUT_DIR)}')
        print(f' 协议版本: v10 (精确时间戳版, 1515/1915字节/包)')
        print(f'{"="*70}')
        print(f' 等待ESP32连接...')
        print(f' 按Ctrl+C停止服务器')
        print(f'{"="*70}\n')

        # 启动统计打印线程
        stats_thread = Thread(target=print_stats, daemon=True)
        stats_thread.start()

        # 启动清理线程
        cleanup_thread = Thread(target=cleanup_disconnected, daemon=True)
        cleanup_thread.start()

        while running:
            try:
                conn, addr = server.accept()
                conn.settimeout(5.0)

                # 创建新线程处理客户端
                client_thread = Thread(target=handle_client, args=(conn, addr), daemon=True)
                client_thread.start()

            except socket.timeout:
                continue
            except OSError:
                break

    except Exception as e:
        print(f'服务器错误: {e}')

    finally:
        server.close()
        print(f'\n服务器已关闭')

        # 打印最终统计（按设备）
        with devices_lock:
            if devices:
                with crc_errors_lock:
                    total_crc_errors = crc_errors

                print(f'\n{"="*80}')
                print(f' 最终统计（按设备ID）')
                print(f'{"="*80}')

                total_packets = 0
                total_samples = 0
                total_bytes = 0
                total_lost = 0

                for device in sorted(devices.values(), key=lambda d: d.device_id):
                    # 计算采样率
                    if device.last_interval_ms > 0:
                        sample_rate_hz = 1000.0 / device.last_interval_ms
                        sample_rate_str = f"{sample_rate_hz:.0f}Hz ({device.last_interval_ms}ms)"
                    else:
                        sample_rate_str = "N/A"

                    print(f' 设备 {device.device_id}:')
                    print(f'   CSV文件: {device.csv_file}')
                    print(f'   接收包数: {device.packets_received}')
                    print(f'   接收样本: {device.samples_received}')
                    print(f'   丢包数: {device.packets_lost}')
                    print(f'   采样率: {sample_rate_str}')
                    print()

                    total_packets += device.packets_received
                    total_samples += device.samples_received
                    total_bytes += device.bytes_received
                    total_lost += device.packets_lost

                print(f' 汇总:')
                print(f'   总设备数: {len(devices)}')
                print(f'   总包数: {total_packets}')
                print(f'   总样本: {total_samples}')
                print(f'   总字节: {total_bytes / 1024 / 1024:.2f} MB')
                print(f'   总丢包: {total_lost}')
                print(f'   总CRC错误: {total_crc_errors}')
                print(f'{"="*80}')


if __name__ == '__main__':
    main()
