# 系统架构详解

## 整体架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Windows 11 宿主机                                 │
│                                                                     │
│  ┌─────────────────────────────────┐                                │
│  │      UE4 Editor (CityPark)      │                                │
│  │  ┌───────────────────────────┐  │                                │
│  │  │     AirSim Plugin         │  │                                │
│  │  │  ┌────────┐ ┌──────────┐ │  │                                │
│  │  │  │RPC API │ │PX4 TCP   │ │  │                                │
│  │  │  │:41451  │ │:4560     │ │  │                                │
│  │  │  └────────┘ └────┬─────┘ │  │                                │
│  │  └──────────────────┼────────┘  │                                │
│  └─────────────────────┼───────────┘                                │
│                        │                                            │
│  ┌─────────────────────┼───────────┐                                │
│  │     QGroundControl   │           │                                │
│  │   UDP 监听 :14550 ◄─┼───────────┼── PX4 MAVLink 数据流          │
│  │                     │           │                                │
│  └─────────────────────┼───────────┘                                │
│                        │                                            │
├────────────────────────┼────────────────────────────────────────────┤
│                  WSL2  │  vEthernet: 172.28.208.1                   │
│                        │                                            │
│  ┌─────────────────────┼─────────────────────────────────────────┐  │
│  │              WSL2 Ubuntu-22.04   IP: 172.28.209.214            │  │
│  │                                                                 │  │
│  │  ┌─────────────────────────────────────────────────────────┐    │  │
│  │  │                   PX4 SITL (v1.15.2)                     │    │  │
│  │  │                                                          │    │  │
│  │  │  TCP :4560 ◄──── AirSim 仿真数据 (位置、IMU、GPS)        │    │  │
│  │  │                                                          │    │  │
│  │  │  MAVLink 通道:                                            │    │  │
│  │  │  ┌────────────────────────────────────────────────────┐  │    │  │
│  │  │  │ Port 18570: GCS 链路 (本地监听)                     │  │    │  │
│  │  │  │ Port 14550: QGC 链路 -t 172.28.208.1 -o 14550      │  │    │  │
│  │  │  │ Port 14580: Offboard 链路 -o 14540                  │  │    │  │
│  │  │  │ Port 14280: Onboard Payload 链路 -o 14030           │  │    │  │
│  │  │  │ Port 13030: Gimbal 链路 -o 13280                    │  │    │  │
│  │  │  └────────────────────────────────────────────────────┘  │    │  │
│  │  └─────────────────────────────────────────────────────────┘    │  │
│  └─────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## 数据流详解

### 1. 仿真数据流 (AirSim → PX4)

```
UE4/AirSim (Windows)                    PX4 SITL (WSL2)
     │                                       │
     │  TCP Connect :4560                    │
     │──────────────────────────────────────►│
     │                                       │
     │  Simulator data (IMU, GPS, etc.)      │
     │◄──────────────────────────────────────│  PX4 lockstep
     │                                       │
     │  PX4_SIM_HOST_ADDR=172.28.208.1       │
     │  (Windows 宿主机 IP, 通过 WSL2 vEthernet)  │
     │                                       │
```

AirSim 作为 TCP 服务器监听 `0.0.0.0:4560`。  
PX4 作为客户端连接到该端口。  
连接建立后双方通过 lockstep 机制同步仿真时钟。

### 2. MAVLink 控制流 (PX4 → QGC)

```
PX4 SITL (WSL2)                         QGC (Windows)
     │                                       │
     │  UDP sendto(172.28.208.1:14550)       │
     │──────────────────────────────────────►│  listen 0.0.0.0:14550
     │                                       │
     │  MAVLink v2 Heartbeat @ 1Hz           │
     │  MAVLink Parameter, Position, Attitude │
     │                                       │
     │  QGC 回复发送到 WSL:14550              │
     │◄──────────────────────────────────────│
     │                                       │
```

### 3. MAVLink 控制流 (PX4 → AirSim)

```
PX4 SITL (WSL2)                         AirSim (Windows)
     │                                       │
     │  UDP sendto(172.28.208.1:14540)       │
     │──────────────────────────────────────►│  ControlPortLocal:14540
     │                                       │
     │  MAVLink control commands              │
     │  (Arm/Disarm, Mode change, etc.)       │
     │                                       │
     │  AirSim 回复到 PX4:14580              │
     │◄──────────────────────────────────────│  ControlPortRemote:14580
     │                                       │
```

## PX4 MAVLink 通道配置

配置见 `config/px4-rc.mavlink`，基于 PX4 标准 `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` 修改：

| 通道 | 本地端口 | 目标 | 速率 | 用途 |
|------|---------|------|------|------|
| GCS | 18570 | 本地监听 | 4 MB/s | 地面站 (备用) |
| QGC | 14550 | → 172.28.208.1:14550 | 50 KB/s | QGC 主链路 |
| Offboard | 14580 | → 172.28.208.1:14540 | 4 MB/s | AirSim 控制 |
| Onboard | 14280 | → 172.28.208.1:14030 | 4 KB/s | 机载 payload |
| Gimbal | 13030 | → 172.28.208.1:13280 | 400 KB/s | 云台控制 |

## AirSim 配置

见 `config/settings.json`。关键参数：

- `SimMode`: Multirotor
- `UseTcp`: true — PX4 通过 TCP 连接 AirSim
- `LocalHostIp`: 172.28.208.1 — Windows 宿主机 WSL 网卡 IP
- `ControlPortLocal`: 14540 — AirSim 监听 MAVLink 端口
- `ControlPortRemote`: 14580 — PX4 MAVLink 端口

## 防火墙配置

需添加 Windows Defender 防火墙规则放行 WSL 与宿主机间的 UDP 14550 通信：

```powershell
New-NetFirewallRule -DisplayName "MAVLink WSL2" -Direction Inbound ^
  -Protocol UDP -LocalPort 14550 -RemoteAddress 172.28.209.0/24 -Action Allow
New-NetFirewallRule -DisplayName "MAVLink WSL2 Out" -Direction Outbound ^
  -Protocol UDP -LocalPort 14550 -RemoteAddress 172.28.209.0/24 -Action Allow
```
