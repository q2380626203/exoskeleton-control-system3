#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCP接收测试服务器 - 带对时功能

功能：
1. 监听本地TCP端口，接收客户端数据
2. 三次握手对时协议：
   - 收到 AA CC -> 回复 CC AA + 8字节时间戳(毫秒)
   - 收到 BB BB -> 对时完成
3. 显示接收到的数据（hex和ASCII）

使用方法：
    python tcp_echo_server.py

配置：
    - 本地监听端口: 25565 (frp映射到此端口)
    - FRP外网地址: frp-any.com:18214

对时协议：
    1. 设备发送: 0xAA 0xCC (对时请求)
    2. 服务端回复: 0xCC 0xAA + 8字节时间戳(毫秒,小端序) (对时响应)
    3. 设备发送: 0xBB 0xBB (对时确认)
"""

import socket
import time
import struct
from datetime import datetime
from threading import Thread
import signal

# 配置
SERVER_HOST = '0.0.0.0'   # 监听所有网卡
SERVER_PORT = 16384       # 本地端口（frp内网穿透映射到此端口）
BUFFER_SIZE = 4096

# 对时协议定义
TIME_SYNC_REQUEST = bytes([0xAA, 0xCC])    # 设备发送的对时请求
TIME_SYNC_CONFIRM = bytes([0xBB, 0xBB])    # 设备发送的对时确认

# 运行标志
running = True


def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global running
    print('\n正在关闭服务器...')
    running = False


def format_hex(data, max_bytes=64):
    """格式化数据为hex字符串"""
    hex_str = data[:max_bytes].hex(' ')
    if len(data) > max_bytes:
        hex_str += f' ... (共{len(data)}字节)'
    return hex_str


def format_ascii(data, max_bytes=64):
    """格式化数据为可打印ASCII"""
    ascii_chars = []
    for b in data[:max_bytes]:
        if 32 <= b < 127:
            ascii_chars.append(chr(b))
        else:
            ascii_chars.append('.')
    result = ''.join(ascii_chars)
    if len(data) > max_bytes:
        result += f' ... (共{len(data)}字节)'
    return result


def get_server_time_ms():
    """获取服务器当前时间戳（毫秒）"""
    return int(time.time() * 1000)


def build_time_sync_response():
    """构建对时响应包: CC AA + 8字节时间戳(毫秒,小端序)"""
    timestamp_ms = get_server_time_ms()
    # CC AA + 8字节时间戳(小端序)
    return bytes([0xCC, 0xAA]) + struct.pack('<q', timestamp_ms)


def handle_client(conn, addr):
    """处理客户端连接"""
    global running

    client_id = f"{addr[0]}:{addr[1]}"
    print(f'\n[{datetime.now().strftime("%H:%M:%S")}] [{client_id}] 客户端已连接')

    total_bytes = 0
    total_packets = 0
    time_synced = False    # 是否已完成对时
    handshake_step = 0     # 握手步骤: 0=等待AA CC, 1=已发送响应等待BB BB, 2=完成
    sync_request_count = 0 # 对时请求计数

    buffer = b''  # 接收缓冲区

    try:
        while running:
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
            total_bytes += len(data)
            total_packets += 1

            # 处理缓冲区中的数据
            while len(buffer) >= 2:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

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
                    except Exception as e:
                        print(f'[{timestamp}] [{client_id}] 对时响应发送失败: {e}')

                    buffer = buffer[2:]  # 移除已处理的请求包
                    continue

                # 检查是否是对时确认 (BB BB) - 第三步
                if buffer[:2] == TIME_SYNC_CONFIRM:
                    if handshake_step == 1:
                        time_synced = True
                        handshake_step = 2
                        print(f'[{timestamp}] [{client_id}] 收到对时确认 BB BB, 三次握手对时完成!')
                    else:
                        print(f'[{timestamp}] [{client_id}] 收到意外的 BB BB (握手步骤={handshake_step})')

                    buffer = buffer[2:]  # 移除已处理的确认包
                    continue

                # 查找下一个协议包位置 (AA CC 或 BB BB)
                next_packet = -1
                for i in range(len(buffer) - 1):
                    if (buffer[i] == 0xAA and buffer[i+1] == 0xCC) or \
                       (buffer[i] == 0xBB and buffer[i+1] == 0xBB):
                        next_packet = i
                        break

                if next_packet > 0:
                    # 协议包前面有其他数据，显示它
                    other_data = buffer[:next_packet]
                    print(f'\n[{timestamp}] [{client_id}] 收到 {len(other_data)} 字节 ({"已对时" if time_synced else "未对时"}):')
                    print(f'  HEX: {format_hex(other_data)}')
                    buffer = buffer[next_packet:]
                elif next_packet == -1:
                    # 没有找到协议包，显示所有数据
                    if len(buffer) > 0:
                        print(f'\n[{timestamp}] [{client_id}] 收到 {len(buffer)} 字节 ({"已对时" if time_synced else "未对时"}):')
                        print(f'  HEX: {format_hex(buffer)}')
                        buffer = b''
                    break

    except Exception as e:
        print(f'[{client_id}] 处理数据时出错: {e}')

    finally:
        conn.close()
        print(f'[{client_id}] 连接已关闭 | 总计: {total_packets}包, {total_bytes}字节, 对时请求: {sync_request_count}次, 已对时: {time_synced}')


def main():
    global running

    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)

    # 创建服务器socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(1.0)

    try:
        server.bind((SERVER_HOST, SERVER_PORT))
        server.listen(5)

        print(f'{"="*60}')
        print(f' TCP 接收测试服务器 (带对时功能)')
        print(f'{"="*60}')
        print(f' 本地监听: {SERVER_HOST}:{SERVER_PORT}')
        print(f' FRP外网:  frp-any.com:18214')
        print(f'{"="*60}')
        print(f' 对时协议:')
        print(f'   设备发送: AA CC')
        print(f'   服务回复: CC AA + 8字节时间戳(毫秒)')
        print(f'   设备确认: BB BB')
        print(f'{"="*60}')
        print(f' 按Ctrl+C停止服务器')
        print(f'{"="*60}')
        print(f'\n等待客户端连接...')

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


if __name__ == '__main__':
    main()
