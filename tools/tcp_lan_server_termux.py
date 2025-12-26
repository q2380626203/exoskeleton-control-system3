#!/usr/bin/env python3
"""
ESP32 外骨骼数据接收服务器 - Termux 版本
支持多设备区分，数据按设备ID分别保存到不同CSV文件

Termux 安装步骤:
1. 从 F-Droid 安装 Termux: https://f-droid.org/packages/com.termux/
2. 打开 Termux，运行以下命令:
   pkg update && pkg upgrade -y
   pkg install python -y
   termux-setup-storage  # 授权访问存储

3. 将此脚本复制到手机，然后运行:
   python /sdcard/Download/tcp_lan_server_termux.py

4. 保持 Termux 在前台运行（或使用 tmux）
"""

import socket
import struct
import time
import os
from datetime import datetime
import threading
import signal

# ==================== 配置 ====================
HOST = '0.0.0.0'
PORT = 8888

# 数据保存路径
SAVE_DIRS = [
    os.path.expanduser("~/storage/downloads"),  # Termux 存储授权后的路径
    "/sdcard/Download",
    os.path.expanduser("~"),
    "."
]

SAVE_DIR = None
for d in SAVE_DIRS:
    try:
        if os.path.exists(d) and os.access(d, os.W_OK):
            SAVE_DIR = d
            break
    except:
        continue

if SAVE_DIR is None:
    SAVE_DIR = "."

FLUSH_INTERVAL = 100

# ==================== 协议常量 ====================
MAGIC = 0xAA55
PACKET_SIZE = 2608
SAMPLES_PER_PACKET = 100
BT_IMU_NOT_CONNECTED = 0x7FFF

# ==================== 全局状态 ====================
running = True

def signal_handler(sig, frame):
    global running
    print("\n[Ctrl+C] 正在停止服务器...")
    running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


class DeviceLogger:
    """单个设备的数据记录器"""

    def __init__(self, device_id, session_time):
        self.device_id = device_id
        self.file = None
        self.filename = None
        self.sample_count = 0
        self.packet_count = 0
        self.pending_lines = []
        self.last_seq = -1
        self.packets_lost = 0

        timestamp = session_time.strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(SAVE_DIR, f"device_{device_id}_{timestamp}.csv")
        self.file = open(self.filename, 'w', encoding='utf-8')

        # CSV表头与 tcp_data_server.py 一致
        self.file.write("timestamp,seq,device_id,"
                       "motor1_pos,motor1_vel,motor1_torque,"
                       "motor2_pos,motor2_vel,motor2_torque,"
                       "roll_left,roll_right,"
                       "m1_state_label,m2_state_label\n")
        self.file.flush()

        print(f"[设备{device_id}] 创建CSV文件: {self.filename}")

    def write_sample(self, sample, seq):
        """写入一条数据，格式与tcp_data_server.py一致"""
        ts_ms = sample['timestamp_ms']

        # 跳过NTP时间未同步的数据（timestamp为0）
        if ts_ms == 0:
            return

        # 时间戳转为秒.毫秒格式，保留3位小数
        timestamp = round(ts_ms / 1000.0, 3)

        # roll数据：None转为空字符串
        roll_l = "" if sample['roll_left'] is None else f"{sample['roll_left']:.2f}"
        roll_r = "" if sample['roll_right'] is None else f"{sample['roll_right']:.2f}"

        # CSV行格式与tcp_data_server.py一致
        line = (f"{timestamp},{seq},{self.device_id},"
                f"{sample['motor1']['pos']:.2f},{sample['motor1']['vel']:.2f},{sample['motor1']['torque']:.2f},"
                f"{sample['motor2']['pos']:.2f},{sample['motor2']['vel']:.2f},{sample['motor2']['torque']:.2f},"
                f"{roll_l},{roll_r},"
                f"{sample['m1_state']},{sample['m2_state']}\n")

        self.pending_lines.append(line)
        self.sample_count += 1

        if len(self.pending_lines) >= FLUSH_INTERVAL:
            self.flush()

    def add_packet(self, seq):
        """记录包数并检测丢包"""
        self.packet_count += 1

        # 检测丢包
        if self.last_seq >= 0:
            expected_seq = (self.last_seq + 1) & 0xFFFF
            if seq != expected_seq:
                lost = (seq - expected_seq) & 0xFFFF
                if lost < 1000:  # 合理的丢包数
                    self.packets_lost += lost
        self.last_seq = seq

    def flush(self):
        if self.file and self.pending_lines:
            self.file.writelines(self.pending_lines)
            self.file.flush()
            self.pending_lines = []

    def close(self):
        if self.file:
            self.flush()
            self.file.close()
            print(f"[设备{self.device_id}] 已保存 {self.sample_count} 条, 丢包 {self.packets_lost} -> {self.filename}")
            self.file = None


class MultiDeviceManager:
    """多设备管理器"""

    def __init__(self):
        self.devices = {}
        self.session_time = datetime.now()
        self.lock = threading.Lock()

    def get_logger(self, device_id):
        with self.lock:
            if device_id not in self.devices:
                self.devices[device_id] = DeviceLogger(device_id, self.session_time)
            return self.devices[device_id]

    def close_all(self):
        with self.lock:
            for logger in self.devices.values():
                logger.close()
            self.devices.clear()

    def get_status(self):
        status = []
        with self.lock:
            for dev_id, logger in sorted(self.devices.items()):
                status.append(
                    f"设备{dev_id}: 包={logger.packet_count}, 样本={logger.sample_count}, 丢包={logger.packets_lost}"
                )
        return status


def parse_packet(data):
    if len(data) < 8:
        return None
    magic, device_id, version, seq, count = struct.unpack('<HBBHH', data[:8])
    if magic != MAGIC:
        return None
    return {'device_id': device_id, 'version': version, 'seq': seq, 'count': count}


def parse_sample(data, index):
    offset = 8 + index * 26
    if len(data) < offset + 26:
        return None

    sample_data = data[offset:offset+26]
    timestamp_ms, = struct.unpack('<q', sample_data[0:8])
    channels = struct.unpack('<6h', sample_data[8:20])
    roll_left, roll_right = struct.unpack('<2h', sample_data[20:24])
    m1_state, m2_state = struct.unpack('<2b', sample_data[24:26])

    return {
        'timestamp_ms': timestamp_ms,
        'motor1': {'pos': channels[0]/100.0, 'vel': channels[1]/100.0, 'torque': channels[2]/100.0},
        'motor2': {'pos': channels[3]/100.0, 'vel': channels[4]/100.0, 'torque': channels[5]/100.0},
        'roll_left': None if roll_left == BT_IMU_NOT_CONNECTED else roll_left/100.0,
        'roll_right': None if roll_right == BT_IMU_NOT_CONNECTED else roll_right/100.0,
        'm1_state': m1_state,
        'm2_state': m2_state
    }


def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "未知"


def handle_client(conn, addr, manager):
    global running
    print(f"[+] 连接: {addr[0]}:{addr[1]}")

    buffer = b''
    device_ids_seen = set()

    try:
        conn.settimeout(1.0)
        while running:
            try:
                data = conn.recv(4096)
                if not data:
                    break

                buffer += data

                while len(buffer) >= PACKET_SIZE:
                    packet_data = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]

                    header = parse_packet(packet_data)
                    if header:
                        device_id = header['device_id']
                        seq = header['seq']
                        device_ids_seen.add(device_id)

                        logger = manager.get_logger(device_id)
                        logger.add_packet(seq)

                        for i in range(header['count']):
                            sample = parse_sample(packet_data, i)
                            if sample:
                                logger.write_sample(sample, seq)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[!] 接收错误: {e}")
                break

    except Exception as e:
        print(f"[!] 连接错误: {e}")
    finally:
        conn.close()
        print(f"[-] 断开: {addr[0]} (设备: {device_ids_seen})")


def main():
    global running

    print()
    print("=" * 60)
    print("  ESP32 外骨骼数据接收服务器 - Termux 版")
    print("=" * 60)
    print(f"  本机IP:   {get_local_ip()}")
    print(f"  端口:     {PORT}")
    print(f"  保存目录: {SAVE_DIR}")
    print("=" * 60)
    print("  按 Ctrl+C 停止服务器")
    print("=" * 60)
    print()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(1.0)

    try:
        server.bind((HOST, PORT))
        server.listen(5)
        print(f"[*] 服务器已启动，等待连接...")
        print()
    except Exception as e:
        print(f"[!] 启动失败: {e}")
        return

    manager = MultiDeviceManager()

    # 状态显示线程
    def status_printer():
        while running:
            time.sleep(2)
            status = manager.get_status()
            if status:
                print("-" * 60)
                for s in status:
                    print(f"  {s}")

    status_thread = threading.Thread(target=status_printer, daemon=True)
    status_thread.start()

    # 主循环
    while running:
        try:
            conn, addr = server.accept()
            client_thread = threading.Thread(
                target=handle_client,
                args=(conn, addr, manager),
                daemon=True
            )
            client_thread.start()
        except socket.timeout:
            continue
        except Exception as e:
            if running:
                print(f"[!] Accept错误: {e}")
            break

    # 清理
    print()
    print("[*] 正在保存数据...")
    manager.close_all()
    server.close()
    print("[*] 服务器已停止")


if __name__ == "__main__":
    main()
