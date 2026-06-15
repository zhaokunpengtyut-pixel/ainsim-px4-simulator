# 故障排查手册

## 目录
- [PX4 启动问题](#px4-启动问题)
- [AirSim 连接问题](#airsim-连接问题)
- [QGC 通信问题](#qgc-通信问题)
- [MAVLink 数据流问题](#mavlink-数据流问题)
- [网络与防火墙](#网络与防火墙)

---

## PX4 启动问题

### PX4 无法在后台运行
**现象**: PX4 进程在 WSL 命令退出后被终止。

**原因**: WSL2 在没有打开的 shell 会话时会终止子进程。

**解决**:
```bash
# 使用 setsid + nohup 确保进程独立运行
wsl -d Ubuntu-22.04 -u hw bash -c "setsid sh -c 'cd ~/PX4-Autopilot && \
  PX4_SYS_AUTOSTART=10016 PX4_SIM_HOST_ADDR=172.28.208.1 \
  nohup ./build/px4_sitl_default/bin/px4 -i 0 > ~/px4_output.log 2>&1 &'"
```

### "Preflight Fail: ekf2 missing data"
**现象**: PX4 启动后显示 EKF2 缺少数据警告。

**原因**: PX4 尚未收到 AirSim 的传感器数据，属于正常启动过程。

**解决**: 确认 UE4 已点击 Play，等待 5-10 秒自动恢复。

---

## AirSim 连接问题

### TCP 4560 连接失败
**现象**: PX4 日志显示 `Waiting for simulator to accept connection on TCP port 4560`。

**检查步骤**:
1. UE4 编辑器是否已打开关卡
2. 是否已点击 Play
3. `settings.json` 中的 `LocalHostIp` 是否正确
4. Windows 防火墙是否放行 4560 端口

**查看当前 WSL IP 并更新设置**:
```powershell
# Windows 端 WSL vEthernet IP
ipconfig | findstr "vEthernet"

# WSL 端 IP
wsl -d Ubuntu-22.04 -u hw hostname -I
```

### AirSim RPC 端口 (41451) 已监听但 PX4 无法连接
**原因**: AirSim 已加载但未进入 Play 模式。

**解决**: 在 UE4 编辑器中点击 **Play** 按钮。

---

## QGC 通信问题

### QGC 显示 "comms lost"
**现象**: QGC 短暂显示无人机后显示通信丢失。

**可能原因**:
1. PX4 到 QGC 的连接是单向的（QGC 指令无法到达 PX4）
2. Windows 防火墙阻止返回流量
3. MAVLink 数据率过高

**排查**:
```bash
# 检查 PX4 14550 端口是否收到数据
wsl -d Ubuntu-22.04 -u hw bash -c "grep 38F4 /proc/net/udp"
# rx_queue > 0 表示有数据到达
```

**解决**:
1. 添加防火墙规则（见下文）
2. 在 QGC 中手动添加 UDP 链接:
   - Settings → Comm Links → Add → UDP
   - 端口 14550

### QGC 显示 "Preflight Fail: ekf2 missing data"
**现象**: QGC 状态栏显示预检失败。

**原因**: PX4 的 EKF2 滤波器在等待传感器数据初始化。

**解决**:
- 确认 AirSim 正在运行并发送数据
- 通常 5-10 秒内自动清除
- 如持续存在，检查 AirSim 是否点击了 Play

### QGC 无法读取参数
**现象**: 参数列表为空或读取超时。

**原因**: MAVLink 双向通信未建立。

**解决**:
1. 确认 WSL 到 Windows 的 UDP 通路正常
2. 添加防火墙规则放行 14550 端口

---

## MAVLink 数据流问题

### PX4 不发送 MAVLink 数据
**现象**: Windows 端收不到任何 MAVLink 数据包。

**原因**: PX4 的 MAVLink 实例没有配置目标 IP。
- 默认 `-x` 标志将数据限制在 localhost
- 无 `-t` 参数时无目标地址

**解决**: 在 `px4-rc.mavlink` 中添加带 `-t` 参数的 MAVLink 实例:
```sh
mavlink start -u 14550 -t <WINDOWS_HOST_IP> -o 14550 -r 50000
```

### PX4 18570 端口无响应
**现象**: 向 PX4:18570 发送 UDP 探测无回应。

**原因**: MAVLink UDP 通道仅响应有效的 MAVLink 消息，不响应空探测包。这是正常行为。

**解决**: 使用带 `-t` 参数的主动发送方式（14550 端口），而非被动监听式（18570 端口）。

### 重复的 MAVLink 数据流
**现象**: QGC 收到来自同一车辆的多条数据流。

**原因**: PX4 的 GCS 链路(18570)和 QGC 链路(14550)都在发送数据。

**解决**: 确保只有一个实例发送到 Windows:14550。

---

## 网络与防火墙

### WSL2 到 Windows 的 UDP 通信失败
**检查**:
```bash
# 测试 WSL -> Windows
wsl -d Ubuntu-22.04 -u hw bash -c "timeout 3 bash -c \
  'echo > /dev/tcp/172.28.208.1/4560' 2>&1"

# 查看 Windows 防火墙规则
powershell "Get-NetFirewallRule | Where-Object DisplayName -match 'WSL|MAVLink'"
```

**添加防火墙规则**:
```powershell
New-NetFirewallRule -DisplayName "MAVLink WSL2 Bridge" -Direction Inbound `
  -Protocol UDP -LocalPort 14550 -RemoteAddress 172.28.209.0/24 -Action Allow
New-NetFirewallRule -DisplayName "MAVLink WSL2 Bridge Out" -Direction Outbound `
  -Protocol UDP -LocalPort 14550 -RemoteAddress 172.28.209.0/24 -Action Allow
```

### WSL2 IP 变化
重启 WSL 后 IP 可能变化，需更新以下位置:
1. `C:\Users\<USER>\Documents\AirSim\settings.json` → `LocalHostIp`
2. WSL `~/.bashrc` → `PX4_SIM_HOST_ADDR`
3. `px4-rc.mavlink` → `-t` 参数

**快速查看新 IP**:
```powershell
# Windows 端
ipconfig
# 找到 "vEthernet (WSL)" 的 IPv4 地址

# WSL 端
wsl -d Ubuntu-22.04 -u hw hostname -I
```
