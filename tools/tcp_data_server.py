#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32电机数据TCP接收服务器 (多设备版)

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
SERVER_PORT = 25565       # 本地端口（frp内网穿透映射到此端口）
BUFFER_SIZE = 8192
CSV_OUTPUT_DIR = './data'

# 数据包格式 (v5: int64时间戳 + int16电机数据)
PACKET_MAGIC = 0xAA55
PACKET_HEADER_SIZE = 8    # magic(2) + device_id(1) + version(1) + seq(2) + count(2)
SAMPLES_PER_PACKET = 100
CHANNELS_PER_SAMPLE = 6   # 6个电机数据通道
SAMPLE_SIZE = 8 + CHANNELS_PER_SAMPLE * 2 + 2  # int64 + 6×int16 + 2×int8 = 8 + 12 + 2 = 22 bytes
PACKET_SIZE = PACKET_HEADER_SIZE + SAMPLES_PER_PACKET * SAMPLE_SIZE  # 8 + 2200 = 2208 bytes

# 通道名称
CHANNEL_NAMES = [
    'motor1_pos', 'motor1_vel', 'motor1_torque',
    'motor2_pos', 'motor2_vel', 'motor2_torque',
    'm1_state_label', 'm2_state_label'  # 状态标签: 0=空闲, 1=抬腿, 2=压腿, 3=检测速度触发
]

# 客户端管理 - 按设备ID分组
clients = {}  # {client_id: ClientInfo}
clients_lock = Lock()
devices = {}  # {device_id: DeviceInfo} - 按设备ID管理
devices_lock = Lock()

# 运行标志
running = True


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
        header = ['timestamp', 'seq', 'device_id'] + CHANNEL_NAMES
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


def create_csv_file(addr):
    """为指定客户端创建新的CSV文件"""
    # 确保输出目录存在
    if not os.path.exists(CSV_OUTPUT_DIR):
        os.makedirs(CSV_OUTPUT_DIR)

    # 生成文件名（包含IP地址）
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    ip_str = addr[0].replace('.', '_')
    filename = os.path.join(CSV_OUTPUT_DIR, f'motor_data_{ip_str}_{timestamp}.csv')

    # 创建CSV文件并写入表头
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ['timestamp_ms', 'seq'] + CHANNEL_NAMES
        writer.writerow(header)

    print(f'[{addr[0]}:{addr[1]}] 创建CSV文件: {filename}')
    return filename


def parse_packet(data):
    """
    解析数据包

    数据包格式 (v5, 8字节包头):
    - magic: uint16 (0xAA55)
    - device_id: uint8 (设备ID: 1, 2, 3...)
    - version: uint8 (协议版本=5)
    - seq: uint16 (序列号)
    - count: uint16 (数据条数)
    - samples: [tcp_sample_t] 每条22字节:
        - timestamp_ms: int64 (8字节, 毫秒时间戳如1734567890123)
        - channels: int16[6] (12字节, 6个电机数据通道, 原值×100)
        - m1_state: int8 (1字节)
        - m2_state: int8 (1字节)
    """
    if len(data) < PACKET_HEADER_SIZE:
        return None

    # 解析包头 (8字节): magic(2) + device_id(1) + version(1) + seq(2) + count(2)
    magic, device_id, version, seq, count = struct.unpack('<HBBHH', data[:8])

    if magic != PACKET_MAGIC:
        return None

    if count > SAMPLES_PER_PACKET:
        count = SAMPLES_PER_PACKET

    # 解析数据
    samples = []
    offset = PACKET_HEADER_SIZE
    for i in range(count):
        if offset + SAMPLE_SIZE > len(data):
            break

        # 解析int64时间戳 + 6个int16通道 + 2个int8状态
        timestamp_ms = struct.unpack('<q', data[offset:offset + 8])[0]
        channels_raw = struct.unpack('<6h', data[offset + 8:offset + 20])
        m1_state, m2_state = struct.unpack('<bb', data[offset + 20:offset + 22])

        # 还原电机数据 (÷100)
        channels = [ch / 100.0 for ch in channels_raw]
        # 添加状态标签
        channels.append(m1_state)
        channels.append(m2_state)

        # 时间戳转为秒.毫秒格式
        timestamp = timestamp_ms / 1000.0

        samples.append({
            'timestamp': round(timestamp, 3),  # 保留3位小数（毫秒精度）
            'seq': seq,
            'device_id': device_id,
            'values': channels
        })
        offset += SAMPLE_SIZE

    return {
        'magic': magic,
        'device_id': device_id,
        'seq': seq,
        'version': version,
        'count': count,
        'samples': samples
    }


def save_to_csv(csv_file, samples):
    """保存数据到CSV文件（过滤掉时间戳为0的未同步数据）"""
    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        for sample in samples:
            # 跳过NTP时间未同步的数据（timestamp为0）
            if sample['timestamp'] == 0.0:
                continue
            row = [sample['timestamp'], sample['seq'], sample['device_id']] + list(sample['values'])
            writer.writerow(row)


def handle_client(conn, addr):
    """处理客户端连接"""
    global clients, devices, running

    client_id = f"{addr[0]}:{addr[1]}"
    print(f'[{client_id}] 客户端已连接，等待识别设备ID...')

    buffer = b''
    current_device = None  # 当前连接对应的设备

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

            # 处理完整的数据包
            while len(buffer) >= PACKET_SIZE:
                # 查找包头 (0xAA55 in little-endian = 0x55 0xAA)
                magic_pos = -1
                for i in range(len(buffer) - 1):
                    if buffer[i] == 0x55 and buffer[i + 1] == 0xAA:
                        magic_pos = i
                        break

                if magic_pos == -1:
                    # 没有找到有效的包头，保留最后一个字节
                    buffer = buffer[-1:]
                    break

                # 丢弃包头之前的数据
                if magic_pos > 0:
                    buffer = buffer[magic_pos:]

                # 检查是否有完整的包
                if len(buffer) < PACKET_SIZE:
                    break

                # 解析数据包
                packet_data = buffer[:PACKET_SIZE]
                buffer = buffer[PACKET_SIZE:]

                packet = parse_packet(packet_data)
                if packet is None:
                    continue

                # 获取或创建设备
                device_id = packet['device_id']
                if current_device is None or current_device.device_id != device_id:
                    current_device = get_or_create_device(device_id)
                    current_device.current_addr = addr
                    print(f'[{client_id}] 识别为设备 {device_id}')

                # 更新设备统计信息
                current_device.packets_received += 1
                current_device.samples_received += packet['count']
                current_device.bytes_received += PACKET_SIZE
                current_device.last_packet_time = time.time()
                current_device.current_addr = addr

                # 检测丢包
                if current_device.last_seq >= 0:
                    expected_seq = (current_device.last_seq + 1) & 0xFFFF
                    if packet['seq'] != expected_seq:
                        lost = (packet['seq'] - expected_seq) & 0xFFFF
                        if lost < 1000:  # 合理的丢包数
                            current_device.packets_lost += lost
                current_device.last_seq = packet['seq']

                # 保存到设备对应的CSV
                save_to_csv(current_device.csv_file, packet['samples'])

    except Exception as e:
        print(f'[{client_id}] 处理数据时出错: {e}')

    finally:
        conn.close()
        if current_device:
            print(f'[{client_id}] 连接已关闭，设备{current_device.device_id} 数据保存到: {current_device.csv_file}')
            print(f'[{client_id}] 设备{current_device.device_id} 统计: 包={current_device.packets_received}, 样本={current_device.samples_received}, 丢包={current_device.packets_lost}')
        else:
            print(f'[{client_id}] 连接已关闭（未识别到设备）')


def print_stats():
    """定期打印所有设备统计信息"""
    global devices, running

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

            print(f'\n{"="*70}')
            print(f' 服务器运行时间: {elapsed:.1f}秒 | 活跃设备: {len(active_devices)}/{len(devices)}')
            print(f'{"="*70}')
            print(f' {"设备ID":^8} | {"状态":^4} | {"包数":^10} | {"样本数":^12} | {"丢包":^6} | {"速率":^10}')
            print(f'{"-"*70}')

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

                addr_str = ""
                if device.current_addr:
                    addr_str = f" ({device.current_addr[0]})"

                print(f' 设备 {device.device_id:<3}{addr_str:<18} | {status}  | '
                      f'{device.packets_received:>8} | '
                      f'{device.samples_received:>10} | '
                      f'{device.packets_lost:>5} | '
                      f'{kb_per_sec:>6.1f} KB/s')

            print(f'{"="*70}\n')


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
        print(f' ESP32 电机数据TCP服务器 (按设备ID管理)')
        print(f'{"="*70}')
        print(f' 监听地址: {SERVER_HOST}:{SERVER_PORT}')
        print(f' FRP外网:  frp-any.com:18214')
        print(f' 数据目录: {os.path.abspath(CSV_OUTPUT_DIR)}')
        print(f' 设备ID:   在ESP32的 tcp_data_upload.h 中配置 DEVICE_ID')
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
                print(f'\n{"="*70}')
                print(f' 最终统计（按设备ID）')
                print(f'{"="*70}')

                total_packets = 0
                total_samples = 0
                total_bytes = 0
                total_lost = 0

                for device in sorted(devices.values(), key=lambda d: d.device_id):
                    print(f' 设备 {device.device_id}:')
                    print(f'   CSV文件: {device.csv_file}')
                    print(f'   接收包数: {device.packets_received}')
                    print(f'   接收样本: {device.samples_received}')
                    print(f'   丢包数: {device.packets_lost}')
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
                print(f'{"="*70}')


if __name__ == '__main__':
    main()
