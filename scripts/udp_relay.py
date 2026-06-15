"""
UDP Relay: WSL2 PX4 MAVLink -> Windows QGC
从 WSL2 的 PX4 读取 MAVLink 数据，转发给 Windows 上的 QGC
"""
import socket
import time

WSL2_IP = "192.168.101.236"
PX4_MAVLINK_PORT = 18570
QGC_PORT = 14550

def main():
    print(f"[转发] PX4(WSL2:{WSL2_IP}:{PX4_MAVLINK_PORT}) → QGC(127.0.0.1:{QGC_PORT})")

    # 只负责从 PX4 读取并转发给 QGC，不监听 QGC 的端口
    # 这样就不会和 QGC 冲突了
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2)

    qgc_addr = ('127.0.0.1', QGC_PORT)
    last_log = 0

    print("[转发] 启动成功，等待 PX4 数据...")

    while True:
        try:
            # 请求 PX4 发送数据 (先发个空包触发响应)
            sock.sendto(b'', (WSL2_IP, PX4_MAVLINK_PORT))
            data, addr = sock.recvfrom(65536)
            if data and len(data) > 4:
                # 转发给 QGC
                qgc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                qgc_sock.sendto(data, qgc_addr)
                qgc_sock.close()

                now = time.time()
                if now - last_log > 5:
                    print(f"[转发] → QGC: {len(data)} bytes MAVLink ✓")
                    last_log = now
        except socket.timeout:
            pass
        except Exception as e:
            now = time.time()
            if now - last_log > 15:
                print(f"[转发] 等待 PX4...")
                last_log = now
        time.sleep(0.05)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[转发] 停止")
