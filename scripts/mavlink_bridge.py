"""
MAVLink UDP Bridge: Windows <-> WSL2 PX4
监听 Windows 端口，转发到 WSL2 PX4 并返回
"""
import socket
import time
import threading

WSL2_IP = "192.168.101.236"
PX4_PORT = 18570  # PX4 MAVLink 监听端口
QGC_PORT = 14550   # QGC 监听端口
BRIDGE_PORT = 14553 # 桥接本地端口

running = True
relay_count = 0

def px4_to_qgc():
    """读取 PX4 的 MAVLink 数据并转发给 QGC"""
    global relay_count
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2)

    # 发送一个 MAVLink 协议探测包来触发 PX4 响应
    # MAVLink v1 heartbeat: FE 09 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    probe = bytes.fromhex('FE0901000000000000000000000000000000')

    while running:
        try:
            # 发送探测包触发 PX4 发送心跳
            sock.sendto(probe, (WSL2_IP, PX4_PORT))
            time.sleep(0.1)

            data, addr = sock.recvfrom(65536)
            if data and len(data) > 4:
                relay_count += 1
                # 转发到 QGC (Windows)
                qs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                qs.sendto(data, ('127.0.0.1', QGC_PORT))
                qs.close()
                if relay_count % 10 == 1:
                    print(f"[桥接] 已转发 {relay_count} 包 MAVLink → QGC")
        except socket.timeout:
            pass
        except Exception as e:
            pass
        time.sleep(0.02)
    sock.close()

def qgc_to_px4():
    """读取 QGC 的指令并转发给 PX4"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1)
    sock.bind(('127.0.0.1', BRIDGE_PORT))

    while running:
        try:
            data, addr = sock.recvfrom(65536)
            if data and len(data) > 4:
                ps = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                ps.sendto(data, (WSL2_IP, PX4_PORT))
                ps.close()
        except:
            pass
    sock.close()

print("=" * 50)
print("MAVLink 桥接器 v2")
print("=" * 50)
print(f"  PX4:   {WSL2_IP}:{PX4_PORT}")
print(f"  QGC:   127.0.0.1:{QGC_PORT}")
print(f"  桥接:  127.0.0.1:{BRIDGE_PORT}")
print("=" * 50)
print("启动双向桥接...")

t1 = threading.Thread(target=px4_to_qgc, daemon=True)
t2 = threading.Thread(target=qgc_to_px4, daemon=True)
t1.start()
t2.start()

print("桥接已运行，等待数据...")

try:
    while True:
        time.sleep(10)
        if relay_count > 0:
            print(f"[桥接] 状态: 已转发 {relay_count} 个 MAVLink 包")
except KeyboardInterrupt:
    running = False
    print("\n桥接停止")
