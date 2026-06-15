# 环境搭建指南

## 目录
1. [准备工作](#准备工作)
2. [AirSim 安装配置](#airsim-安装配置)
3. [PX4 SITL 编译配置](#px4-sitl-编译配置)
4. [QGroundControl 安装](#qgroundcontrol-安装)
5. [MAVLink 配置](#mavlink-配置)
6. [网络与防火墙配置](#网络与防火墙配置)
7. [启动仿真](#启动仿真)

---

## 准备工作

### 系统要求
- **操作系统**: Windows 11 (64位)
- **WSL2**: Ubuntu-22.04
- **内存**: ≥ 16 GB
- **磁盘**: ≥ 100 GB 可用空间
- **GPU**: 支持 DX11/DX12 的独立显卡

### 安装 WSL2
```powershell
# 以管理员运行 PowerShell
wsl --install -d Ubuntu-22.04
wsl --set-version Ubuntu-22.04 2
```

### 创建 WSL 用户
```bash
# 在 WSL 内
sudo adduser hw
sudo usermod -aG sudo hw
```

---

## AirSim 安装配置

### 下载
从 [AirSim Releases](https://github.com/Microsoft/AirSim/releases) 下载 AirSim 1.8.1 Windows 版本。

### 设置
复制 `settings.json` 到 `%USERPROFILE%\Documents\AirSim\`：

关键配置参数:
```json
{
    "SimMode": "Multirotor",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "UseTcp": true,
            "TcpPort": 4560,
            "LocalHostIp": "<YOUR_WINDOWS_WSL_IP>",
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580
        }
    }
}
```

> **提示**: `LocalHostIp` 需设为 Windows 端 WSL vEthernet 网卡的 IP 地址。可通过 `ipconfig` 查看。

---

## PX4 SITL 编译配置

### 获取源码
下载 PX4 v1.15.2 源码到 WSL2:
```bash
wsl -d Ubuntu-22.04 -u hw
cd ~
# 解压或克隆
unzip px4v1.15.2.zip
ln -s ~/px4v1.15.2 ~/PX4-Autopilot
```

### 编译 (首次)
```bash
cd ~/PX4-Autopilot
make px4_sitl_default
```

### 配置环境变量
添加到 `~/.bashrc`:
```bash
export PX4_SIM_HOST_ADDR=<YOUR_WINDOWS_WSL_IP>
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=:0
```

### 修改 MAVLink 配置
将 `config/px4-rc.mavlink` 复制到:
```
~/PX4-Autopilot/build/px4_sitl_default/rootfs/0/etc/init.d-posix/px4-rc.mavlink
```

该配置增加了 QGC 直连链路，将 MAVLink 数据直接发送到 Windows 宿主机。

---

## QGroundControl 安装

从 [QGC Releases](https://github.com/mavlink/qgroundcontrol/releases) 下载并安装 QGroundControl 4.4.4。

默认监听 UDP 14550 端口。

---

## MAVLink 配置

### PX4 MAVLink 通道
修改后的 `px4-rc.mavlink` 增加了 QGC 专用链路:
```sh
# QGC link - send to Windows QGC
mavlink start -u 14550 -t 172.28.208.1 -o 14550 -r 50000
mavlink stream -r 50 -s HEARTBEAT -u 14550
mavlink stream -r 10 -s SYS_STATUS -u 14550
mavlink stream -r 10 -s GLOBAL_POSITION_INT -u 14550
mavlink stream -r 10 -s ATTITUDE -u 14550
```

说明:
- `-u 14550`: WSL 本地监听端口
- `-t 172.28.208.1`: 目标 IP (Windows 宿主机)
- `-o 14550`: 目标端口 (QGC 监听端口)
- `-r 50000`: 数据速率 50 KB/s

### 桥接方案
如需双向 MAVLink 桥接，使用 `scripts/mavlink_bridge.py`:
```bash
python mavlink_bridge.py
```

---

## 网络与防火墙配置

### 获取当前 IP
```powershell
# 查看 Windows WSL 网卡 IP
ipconfig | findstr "vEthernet"

# 查看 WSL2 IP
wsl -d Ubuntu-22.04 -u hw hostname -I
```

### 添加防火墙规则
```powershell
# 放行 MAVLink 流量
New-NetFirewallRule -DisplayName "MAVLink WSL2 Bridge" -Direction Inbound `
  -Protocol UDP -LocalPort 14550 -RemoteAddress 172.28.209.0/24 -Action Allow
New-NetFirewallRule -DisplayName "MAVLink WSL2 Bridge Out" -Direction Outbound `
  -Protocol UDP -LocalPort 14550 -RemoteAddress 172.28.209.0/24 -Action Allow
```

### WSL2 IP 变化时的更新
每次重启 WSL2 后 IP 可能变化，需要更新:
1. `settings.json` 中的 `LocalHostIp`
2. WSL 中的 `PX4_SIM_HOST_ADDR`
3. `px4-rc.mavlink` 中的 `-t` 参数

---

## 启动仿真

### 完整启动流程

1. **打开 UE4 编辑器**
   - 双击 `CityParkEnvironmentCollec.uproject`
   - 等待编辑器加载完成
   - 点击 **Play** 按钮

2. **启动 PX4 SITL**
   ```bash
   wsl -d Ubuntu-22.04 -u hw bash -c "cd ~/PX4-Autopilot && \
     PX4_SYS_AUTOSTART=10016 PX4_SIM_HOST_ADDR=172.28.208.1 \
     nohup ./build/px4_sitl_default/bin/px4 -i 0 > ~/px4_output.log 2>&1 &"
   ```

3. **启动 QGC**
   ```bash
   start "" "C:\Program Files\QGroundControl\bin\QGroundControl.exe"
   ```

4. **验证连接**
   - QGC 显示 "Ready"
   - AirSim 中无人机响应 PX4 控制

### 一键启动
```bash
cd %USERPROFILE%\Documents\AirSim
python run_all.py
```

### 停止仿真
```bash
# 停止 PX4
wsl -d Ubuntu-22.04 -u hw pkill -f "bin/px4"

# 关闭 QGC (手动或)
taskkill /IM QGroundControl.exe /F

# 停止 UE4 Play (在编辑器中点击 Stop)
```
