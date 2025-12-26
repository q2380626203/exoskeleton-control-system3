#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32串口透传数据验证工具

功能：
1. 接收串口921600波特率的二进制数据
2. 自动区分ESP日志文本和二进制数据包
3. 解析与TCP相同格式的数据包
4. 验证数据包完整性，统计丢包率
5. 显示实时数据和日志

使用方法：
    python serial_passthrough_test.py COM3
    python serial_passthrough_test.py /dev/ttyUSB0

数据包格式 (v6, 2608字节):
    - magic: uint16 (0xAA55)
    - device_id: uint8
    - version: uint8
    - seq: uint16
    - count: uint16
    - samples[100]: 每条26字节
"""

import serial
import struct
import sys
import time
import argparse
from datetime import datetime

# 数据包格式常量
PACKET_MAGIC = 0xAA55
PACKET_MAGIC_BYTES = b'\x55\xAA'  # little-endian
PACKET_HEADER_SIZE = 8    # magic(2) + device_id(1) + version(1) + seq(2) + count(2)
SAMPLES_PER_PACKET = 100
CHANNELS_PER_SAMPLE = 6
SAMPLE_SIZE = 26  # int64(8) + 6×int16(12) + 2×int16(4) + 2×int8(2)
PACKET_SIZE = PACKET_HEADER_SIZE + SAMPLES_PER_PACKET * SAMPLE_SIZE  # 2608 bytes

# 蓝牙IMU未连接标记值
BT_IMU_NOT_CONNECTED = 0x7FFF

# ESP日志前缀标识
LOG_PREFIXES = [b'I (', b'W (', b'E (', b'D (', b'V (']  # Info, Warning, Error, Debug, Verbose
TEXT_LINE_CHARS = set(b'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 \t\r\n:.,-_()[]{}!@#$%^&*+=/<>?\'"\\|`~')

# 统计信息
stats = {
    'packets_received': 0,
    'packets_valid': 0,
    'packets_invalid': 0,
    'bytes_received': 0,
    'bytes_discarded': 0,
    'packets_lost': 0,
    'log_lines': 0,
    'last_seq': -1,
    'start_time': None,
    'last_print_time': 0,
}


def parse_packet(data):
    """解析数据包"""
    if len(data) < PACKET_HEADER_SIZE:
        return None

    # 解析包头
    magic, device_id, version, seq, count = struct.unpack('<HBBHH', data[:8])

    if magic != PACKET_MAGIC:
        return None

    if count > SAMPLES_PER_PACKET:
        count = SAMPLES_PER_PACKET

    # 解析样本数据
    samples = []
    offset = PACKET_HEADER_SIZE
    for i in range(count):
        if offset + SAMPLE_SIZE > len(data):
            break

        # 解析: int64时间戳 + 6×int16通道 + 2×int16 roll + 2×int8状态
        timestamp_ms = struct.unpack('<q', data[offset:offset + 8])[0]
        channels_raw = struct.unpack('<6h', data[offset + 8:offset + 20])
        roll_left_raw, roll_right_raw = struct.unpack('<2h', data[offset + 20:offset + 24])
        m1_state, m2_state = struct.unpack('<bb', data[offset + 24:offset + 26])

        # 还原数据
        channels = [ch / 100.0 for ch in channels_raw]
        roll_left = None if roll_left_raw == BT_IMU_NOT_CONNECTED else roll_left_raw / 100.0
        roll_right = None if roll_right_raw == BT_IMU_NOT_CONNECTED else roll_right_raw / 100.0

        samples.append({
            'timestamp_ms': timestamp_ms,
            'channels': channels,
            'roll_left': roll_left,
            'roll_right': roll_right,
            'm1_state': m1_state,
            'm2_state': m2_state,
        })
        offset += SAMPLE_SIZE

    return {
        'magic': magic,
        'device_id': device_id,
        'version': version,
        'seq': seq,
        'count': count,
        'samples': samples,
    }


def print_stats():
    """打印统计信息"""
    global stats

    now = time.time()
    if stats['start_time'] is None:
        return

    elapsed = now - stats['start_time']
    if elapsed <= 0:
        return

    # 计算速率
    packets_per_sec = stats['packets_valid'] / elapsed
    bytes_per_sec = stats['bytes_received'] / elapsed
    loss_rate = 0
    if stats['packets_valid'] + stats['packets_lost'] > 0:
        loss_rate = stats['packets_lost'] / (stats['packets_valid'] + stats['packets_lost']) * 100

    print(f"\n{'='*70}")
    print(f" 串口透传数据验证统计 (运行 {elapsed:.1f} 秒)")
    print(f"{'='*70}")
    print(f" 有效数据包:   {stats['packets_valid']:>10}")
    print(f" 无效数据包:   {stats['packets_invalid']:>10}")
    print(f" 丢失数据包:   {stats['packets_lost']:>10}")
    print(f" 丢包率:       {loss_rate:>10.2f}%")
    print(f" ESP日志行数:  {stats['log_lines']:>10}")
    print(f" 接收字节:     {stats['bytes_received']:>10} ({stats['bytes_received']/1024:.1f} KB)")
    print(f" 丢弃字节:     {stats['bytes_discarded']:>10}")
    print(f" 数据包速率:   {packets_per_sec:>10.2f} 包/秒")
    print(f" 字节速率:     {bytes_per_sec/1024:>10.2f} KB/秒")
    print(f"{'='*70}\n")


def print_sample_data(packet):
    """打印采样数据（只打印第一条和最后一条）"""
    if not packet['samples']:
        return

    first = packet['samples'][0]
    last = packet['samples'][-1]

    # 时间戳格式化
    if first['timestamp_ms'] > 0:
        ts_str = datetime.fromtimestamp(first['timestamp_ms'] / 1000).strftime('%H:%M:%S.%f')[:-3]
    else:
        ts_str = "未同步"

    print(f"  seq={packet['seq']:5d} device={packet['device_id']} "
          f"time={ts_str} "
          f"m1_vel={first['channels'][1]:6.2f} m2_vel={first['channels'][4]:6.2f} "
          f"state=({first['m1_state']},{first['m2_state']})")


def is_text_data(data):
    """检查数据是否是文本（ESP日志等）"""
    if not data:
        return False
    # 检查是否以日志前缀开头
    for prefix in LOG_PREFIXES:
        if data.startswith(prefix):
            return True
    # 检查是否是可打印ASCII字符为主
    printable_count = sum(1 for b in data if 32 <= b <= 126 or b in (10, 13, 9))
    return printable_count > len(data) * 0.8


def extract_log_lines(buffer):
    """从缓冲区提取完整的日志行，返回(日志行列表, 剩余缓冲区)"""
    lines = []
    remaining = buffer

    while b'\n' in remaining:
        newline_pos = remaining.find(b'\n')
        line = remaining[:newline_pos]
        remaining = remaining[newline_pos + 1:]

        # 检查这一行是否是文本日志
        if line and is_text_data(line):
            try:
                decoded = line.decode('utf-8', errors='replace').strip()
                if decoded:
                    lines.append(decoded)
            except:
                pass

    return lines, remaining


def find_packet_start(buffer):
    """
    在缓冲区中查找数据包起始位置
    返回: (位置, 是否找到)
    如果找到日志文本，返回日志结束位置
    """
    # 查找magic字节 0x55 0xAA
    pos = buffer.find(PACKET_MAGIC_BYTES)
    return pos


def main():
    parser = argparse.ArgumentParser(description='ESP32串口透传数据验证工具')
    parser.add_argument('port', help='串口名称，如 COM3 或 /dev/ttyUSB0')
    parser.add_argument('-b', '--baud', type=int, default=921600, help='波特率 (默认: 921600)')
    parser.add_argument('-v', '--verbose', action='store_true', help='显示详细数据')
    parser.add_argument('-q', '--quiet', action='store_true', help='静默模式，只显示统计')
    parser.add_argument('-l', '--show-log', action='store_true', help='显示ESP日志')
    args = parser.parse_args()

    print(f"{'='*70}")
    print(f" ESP32 串口透传数据验证工具")
    print(f"{'='*70}")
    print(f" 串口:     {args.port}")
    print(f" 波特率:   {args.baud}")
    print(f" 数据包:   {PACKET_SIZE} 字节")
    print(f" 显示日志: {'是' if args.show_log else '否'}")
    print(f"{'='*70}")
    print(f" 等待数据... (Ctrl+C 退出)")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"[OK] 串口已打开: {args.port}")
    except Exception as e:
        print(f"[错误] 无法打开串口: {e}")
        sys.exit(1)

    buffer = b''
    stats['start_time'] = time.time()
    stats['last_print_time'] = time.time()

    try:
        while True:
            # 读取数据
            try:
                data = ser.read(4096)
                if data:
                    buffer += data
                    stats['bytes_received'] += len(data)
            except Exception as e:
                print(f"[错误] 读取串口失败: {e}")
                break

            # 先提取并处理日志行
            log_lines, buffer = extract_log_lines(buffer)
            for line in log_lines:
                stats['log_lines'] += 1
                if args.show_log:
                    print(f"[LOG] {line}")

            # 处理二进制数据包
            while len(buffer) >= PACKET_SIZE:
                # 查找包头 (0x55 0xAA in little-endian)
                magic_pos = find_packet_start(buffer)

                if magic_pos == -1:
                    # 没有找到有效的包头
                    # 检查是否有可能是日志文本
                    if is_text_data(buffer[:100]):
                        # 可能是不完整的日志，保留等待更多数据
                        break
                    else:
                        # 丢弃大部分数据，保留最后几个字节
                        discard_len = max(0, len(buffer) - 10)
                        stats['bytes_discarded'] += discard_len
                        buffer = buffer[-10:] if len(buffer) > 10 else buffer
                        break

                # 丢弃包头之前的数据（可能是日志或垃圾数据）
                if magic_pos > 0:
                    # 检查被丢弃的是否是日志
                    discarded = buffer[:magic_pos]
                    if is_text_data(discarded):
                        try:
                            log_text = discarded.decode('utf-8', errors='replace').strip()
                            if log_text and args.show_log:
                                for line in log_text.split('\n'):
                                    if line.strip():
                                        print(f"[LOG] {line.strip()}")
                                        stats['log_lines'] += 1
                        except:
                            pass
                    else:
                        stats['bytes_discarded'] += magic_pos
                    buffer = buffer[magic_pos:]

                # 检查是否有完整的包
                if len(buffer) < PACKET_SIZE:
                    break

                # 提取数据包
                packet_data = buffer[:PACKET_SIZE]
                buffer = buffer[PACKET_SIZE:]
                stats['packets_received'] += 1

                # 解析数据包
                packet = parse_packet(packet_data)
                if packet is None:
                    stats['packets_invalid'] += 1
                    if not args.quiet:
                        print(f"[警告] 无效数据包 (magic不匹配)")
                    continue

                stats['packets_valid'] += 1

                # 检测丢包
                if stats['last_seq'] >= 0:
                    expected_seq = (stats['last_seq'] + 1) & 0xFFFF
                    if packet['seq'] != expected_seq:
                        lost = (packet['seq'] - expected_seq) & 0xFFFF
                        if lost < 1000:  # 合理的丢包数
                            stats['packets_lost'] += lost
                            if not args.quiet:
                                print(f"[丢包] 丢失 {lost} 个包 (期望 {expected_seq}, 收到 {packet['seq']})")
                stats['last_seq'] = packet['seq']

                # 打印数据
                if args.verbose and not args.quiet:
                    print_sample_data(packet)
                elif not args.quiet and stats['packets_valid'] % 10 == 0:
                    # 每10个包打印一次简要信息
                    print(f"[收到] 包 #{stats['packets_valid']}, seq={packet['seq']}, "
                          f"设备={packet['device_id']}, 样本={packet['count']}")

            # 定期打印统计信息
            now = time.time()
            if now - stats['last_print_time'] >= 5:
                print_stats()
                stats['last_print_time'] = now

    except KeyboardInterrupt:
        print("\n\n[中断] 用户停止")

    finally:
        ser.close()
        print_stats()
        print("[完成] 串口已关闭")


if __name__ == '__main__':
    main()
