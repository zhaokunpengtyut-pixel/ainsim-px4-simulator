"""
AirSim 一键启动器
==================
启动顺序:
  1. 检查 UE4 (须先手动 Play)
  2. 在 WSL 中启动 PX4 SITL
  3. 启动 QGroundControl
  4. 启动向下摄像头实时画面 (带 YOLO 检测可选)

用法:
  python run_all.py              # 基础模式：启动 PX4 + QGC + 摄像头
  python run_all.py --yolo       # 带 YOLO 目标检测
  python run_all.py --no-qgc     # 不启动 QGC
  python run_all.py --no-cam     # 不启动摄像头
  python run_all.py --stop       # 停止所有组件
"""
import sys
import os
import subprocess
import time
import signal
import argparse
import atexit
import json
import socket
from pathlib import Path

# ============ 配置 ============
AIRSIM_RPC_PORT = 41451
PX4_TCP_PORT = 4560
WSL_DISTRO = "Ubuntu-22.04"
WSL_USER = "hw"
PX4_DIR = "/home/hw/PX4-Autopilot"
PX4_LOG = "/home/hw/px4_output.log"
QGC_PATH = r"C:\Program Files\QGroundControl\bin\QGroundControl.exe"
YOLO_SCRIPT = Path(__file__).parent / "yolo_detect.py"
CAM_SCRIPT = Path(__file__).parent / "camera_view.py"
# =============================

# 保存子进程句柄
processes = []


def log(msg):
    print(f"[一键启动] {msg}")


def check_airsim():
    """检查 AirSim (UE4) 是否在运行"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex(("127.0.0.1", AIRSIM_RPC_PORT))
        sock.close()
        return result == 0
    except:
        return False


def wait_for_airsim(timeout=60):
    """等待 AirSim 就绪"""
    log("等待 AirSim (UE4) 就绪...")
    start = time.time()
    while time.time() - start < timeout:
        if check_airsim():
            log(f"AirSim 已就绪! (端口 {AIRSIM_RPC_PORT})")
            return True
        time.sleep(2)
    log(f"等待超时 ({timeout}s)，请确认 UE4 已启动并点击了 Play")
    return False


def start_px4():
    """在 WSL 中启动 PX4 SITL"""
    log("正在启动 PX4 SITL (WSL)...")

    # 检查是否已在运行
    result = subprocess.run(
        ["wsl", "-d", WSL_DISTRO, "-u", WSL_USER,
         "--", "bash", "-c",
         f"ps aux | grep -v grep | grep -q 'bin/px4' && echo 'running' || echo 'stopped'"],
        capture_output=True, text=True, timeout=10
    )
    if "running" in result.stdout:
        log("PX4 已在运行，跳过启动")
        return True

    # 启动 PX4 (使用 setsid 确保完全后台运行)
    cmd = [
        "wsl", "-d", WSL_DISTRO, "-u", WSL_USER,
        "--", "bash", "-c",
        f"cd {PX4_DIR} && rm -f {PX4_LOG} && setsid sh -c 'PX4_SYS_AUTOSTART=10016 ./build/px4_sitl_default/bin/px4 > {PX4_LOG} 2>&1 &'"
    ]
    try:
        subprocess.run(cmd, timeout=10)
        log("PX4 启动命令已发送")

        # 等待连接
        for i in range(15):
            time.sleep(1)
            result = subprocess.run(
                ["wsl", "-d", WSL_DISTRO, "-u", WSL_USER,
                 "--", "bash", "-c",
                 f"grep -c 'Simulator connected' {PX4_LOG} 2>/dev/null || echo '0'"],
                capture_output=True, text=True, timeout=5
            )
            if "Simulator connected" in result.stdout or "1" in result.stdout.strip():
                log("PX4 已连接到 AirSim!")
                return True

        # 检查是否至少运行了
        result = subprocess.run(
            ["wsl", "-d", WSL_DISTRO, "-u", WSL_USER,
             "--", "bash", "-c",
             f"tail -5 {PX4_LOG}"],
            capture_output=True, text=True, timeout=5
        )
        log(f"PX4 日志 (最后5行):\n{result.stdout.strip()}")
        return True

    except Exception as e:
        log(f"PX4 启动失败: {e}")
        return False


def stop_px4():
    """停止 WSL 中的 PX4"""
    log("正在停止 PX4...")
    try:
        subprocess.run(
            ["wsl", "-d", WSL_DISTRO, "-u", WSL_USER,
             "--", "bash", "-c",
             "pkill -f 'bin/px4' 2>/dev/null; echo 'done'"],
            timeout=10
        )
        log("PX4 已停止")
    except Exception as e:
        log(f"停止 PX4 出错: {e}")


def start_qgc():
    """启动 QGroundControl"""
    log("正在启动 QGroundControl...")
    if not os.path.exists(QGC_PATH):
        log(f"找不到 QGC: {QGC_PATH}")
        return False

    try:
        proc = subprocess.Popen([QGC_PATH], shell=True)
        processes.append(proc)
        log("QGroundControl 已启动")
        return True
    except Exception as e:
        log(f"QGC 启动失败: {e}")
        return False


def start_camera():
    """启动向下摄像头画面"""
    log("正在启动向下摄像头画面...")
    if not CAM_SCRIPT.exists():
        log(f"找不到脚本: {CAM_SCRIPT}")
        return False

    try:
        proc = subprocess.Popen(
            [sys.executable, str(CAM_SCRIPT)],
            shell=True, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
        )
        processes.append(proc)
        log("摄像头画面已启动")
        return True
    except Exception as e:
        log(f"摄像头启动失败: {e}")
        return False


def start_yolo():
    """启动 YOLO 检测"""
    log("正在启动 YOLO 目标检测...")
    if not YOLO_SCRIPT.exists():
        log(f"找不到脚本: {YOLO_SCRIPT}")
        return False

    try:
        proc = subprocess.Popen(
            [sys.executable, str(YOLO_SCRIPT)],
            shell=True, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
        )
        processes.append(proc)
        log("YOLO 检测已启动")
        return True
    except Exception as e:
        log(f"YOLO 启动失败: {e}")
        return False


def cleanup():
    """清理所有子进程"""
    if processes:
        log("正在关闭启动器启动的子进程...")
        for proc in processes:
            try:
                proc.terminate()
            except:
                pass
        processes.clear()


def stop_all():
    """停止所有组件（除 UE4 外）"""
    print("=" * 50)
    print("  停止所有组件")
    print("=" * 50)
    stop_px4()

    log("请手动关闭 QGC 和摄像头窗口 (或继续使用)")
    print()
    log("完成。可以通过以下命令重新启动:")
    log("  python run_all.py")


def main():
    parser = argparse.ArgumentParser(description="AirSim 一键启动器")
    parser.add_argument("--yolo", action="store_true", help="同时启动 YOLO 目标检测")
    parser.add_argument("--no-qgc", action="store_true", help="不启动 QGroundControl")
    parser.add_argument("--no-cam", action="store_true", help="不启动摄像头画面")
    parser.add_argument("--stop", action="store_true", help="停止所有组件")

    args = parser.parse_args()

    # 注册清理
    atexit.register(cleanup)

    if args.stop:
        stop_all()
        return

    print("=" * 50)
    print("  AirSim 一键启动器")
    print("=" * 50)
    print()

    # 1. 检查 AirSim
    if not check_airsim():
        log("!  AirSim (UE4) 未就绪")
        log("  请确保: 1) UE4 已启动  2) 已点击 Play")
        log(f"  等待中 (最长 60 秒)...")
        if not wait_for_airsim():
            print()
            log("AirSim 未就绪，请手动启动 UE4 并点击 Play 后重试")
            sys.exit(1)
    else:
        log("AirSim (UE4) 已就绪")

    print()

    # 2. 启动 PX4
    start_px4()
    print()

    # 3. 启动 QGC
    if not args.no_qgc:
        start_qgc()
        print()

    # 4. 启动摄像头
    if args.yolo:
        time.sleep(1)
        start_yolo()
    elif not args.no_cam:
        time.sleep(1)
        start_camera()

    print()
    print("=" * 50)
    log("所有组件启动完成!")
    log("  UE4 + AirSim        -> 手动管理 (编辑器)")
    log("  PX4 SITL (WSL)      -> 已启动")
    if not args.no_qgc:
        log("  QGroundControl      -> 已启动")
    if args.yolo:
        log("  YOLO 目标检测       -> 已启动")
    elif not args.no_cam:
        log("  向下摄像头画面      -> 已启动")
    print()
    log("管理命令:")
    log("  查看 PX4 日志:  wsl -d Ubuntu-22.04 -u hw -- bash -c 'tail -f ~/px4_output.log'")
    log("  停止 PX4:       wsl -d Ubuntu-22.04 -u hw -- bash -c 'pkill -f bin/px4'")
    log("  停止所有:       python run_all.py --stop")
    print("=" * 50)

    # 保持运行，等待用户 Ctrl+C
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()
        log("收到退出信号")
        cleanup()
        log("已退出")


if __name__ == "__main__":
    main()
