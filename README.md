# AirSim + PX4 SITL + QGC 无人机仿真环境

基于 **AirSim 1.8.1** + **PX4 v1.15.2 SITL** + **QGroundControl 4.4.4** 的无人机仿真系统。  
运行在 **Windows 11 + WSL2 (Ubuntu-22.04)** 环境下。

---

## 系统架构

```
┌─────────────────────────────────────────────────────┐
│                   Windows 11                         │
│                                                      │
│  ┌──────────────┐    ┌──────────────────────────┐    │
│  │   UE4 Editor  │    │  QGroundControl (QGC)     │    │
│  │  CityParkEnv  │    │  监听 UDP :14550          │    │
│  │  + AirSim     │    │                           │    │
│  │  TCP:4560 ◄───┼────┼─── PX4 MAVLink ─────────►│    │
│  └──────┬───────┘    └──────────────────────────┘    │
│         │                                              │
│         │ WSL2 vEthernet (172.28.208.1)                │
├─────────┼──────────────────────────────────────────────┤
│         │         WSL2 (Ubuntu-22.04)                  │
│  ┌──────┴──────────────────────────┐                   │
│  │         PX4 SITL                 │                  │
│  │  ┌─────────────────────────┐    │                  │
│  │  │  MAVLink 18570 (GCS)    │    │                  │
│  │  │  MAVLink 14550 (QGC)    │    │                  │
│  │  │  MAVLink 14580 (Offboard)│   │                  │
│  │  └─────────────────────────┘    │                  │
│  │  UDP:18570 → Windows:14550      │                  │
│  │  UDP:14550 → Windows:14550      │                  │
│  └──────────────────────────────────┘                  │
└─────────────────────────────────────────────────────┘
```

## 文件结构

```
├── config/
│   ├── settings.json          AirSim PX4Multirotor 配置
│   └── px4-rc.mavlink         PX4 MAVLink 数据流配置
├── scripts/
│   ├── run_all.py             一键启动器
│   ├── start_px4.sh           PX4 启动脚本 (WSL)
│   ├── mavlink_bridge.py      MAVLink 双向桥接
│   └── udp_relay.py           UDP 中继转发
├── docs/
│   ├── architecture.md        系统架构详解
│   ├── setup-guide.md         完整搭建指南
│   └── troubleshooting.md     故障排查手册
└── README.md                  本文件
```

## 快速启动

### 前提条件
- Windows 11 + WSL2 (Ubuntu-22.04)
- PX4 v1.15.2 已编译
- UE4 + AirSim 插件 + CityParkEnvironment
- QGroundControl 4.4.4

### 启动步骤

**1. 启动 UE4 编辑器**
打开 `CityParkEnvironmentCollec.uproject`，点击 **Play**

**2. 启动 PX4 SITL（WSL）**
```bash
wsl -d Ubuntu-22.04 -u hw bash -c "cd ~/PX4-Autopilot && \
  PX4_SYS_AUTOSTART=10016 PX4_SIM_HOST_ADDR=172.28.208.1 \
  ./build/px4_sitl_default/bin/px4 -i 0"
```

**3. 启动 QGroundControl**
```bash
start "" "C:\Program Files\QGroundControl\bin\QGroundControl.exe"
```

**或用一键启动脚本：**
```bash
cd C:\Users\59636\Documents\AirSim
python run_all.py
```

## 网络配置

| 组件 | IP 地址 | 端口 |
|------|---------|------|
| Windows 宿主机 | 172.28.208.1 | - |
| WSL2 (Ubuntu) | 172.28.209.214 | - |
| AirSim (PX4 TCP) | 172.28.208.1 | 4560 |
| PX4 MAVLink (QGC) | → Windows | 14550 |
| PX4 MAVLink (GCS) | 0.0.0.0 | 18570 |
| PX4 MAVLink (Offboard) | → Windows:14540 | 14580 |
| QGC 监听 | 0.0.0.0 | 14550 |

> **注意**: WSL2 IP 由虚拟交换机动态分配，重启后可能变化。  
> 需更新 `settings.json` 中的 `LocalHostIp` 和 WSL 中的 `PX4_SIM_HOST_ADDR`。

## 注意事项

1. **防火墙**: 需放行 UDP 14550 端口的 WSL 流量
2. **WSL2 IP**: 每次重启 WSL 可能变化，需检查并更新
3. **AirSim Play**: 必须在 UE4 中点击 Play 后 PX4 才能连接
4. **MAVLink**: PX4 通过 `-t` 参数直接发送到 Windows 宿主机

## 相关资源

- [AirSim 文档](https://microsoft.github.io/AirSim/)
- [PX4 SITL 指南](https://docs.px4.io/main/en/simulation/)
- [QGroundControl 用户手册](https://docs.qgroundcontrol.com/master/en/)

## 坐标系模块 v2.0

`src/coordinate_system.py` — UAV 坐标系构建与空间分析

**核心功能：**
- **Homography 映射** — 用区域4个角点直接建立像素→地面的映射（无需相机内参）
- **复合权重中心** — `置信度 × 边界框面积` 加权的目标群中心
- **HDBSCAN 聚类** — 自动发现空间聚类分布
- **Alpha Shape 边界** — 目标群边界 + 离散度 + 凸包面积

**两种坐标转换方案：**
| 方案 | 方法 | 适用场景 |
|------|------|----------|
| A | 针孔模型 + 地面投影 | AirSim 仿真（需 FOV/高度） |
| B ⭐ | Homography 矩阵 | 仿真+现实（用区域角点标定） |

**使用：**
```bash
cd src
python run_coordinate_system.py --test-viewer  # 独立3D演示模式
python run_coordinate_system.py --homography   # Homography 模式(推荐)
```
