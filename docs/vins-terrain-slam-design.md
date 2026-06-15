# Visual-Inertial SLAM + Terrain Perception — Technical Design

> UAV pose estimation and terrain detection using monocular vision + IMU + GPS
> in AirSim + PX4 SITL simulation environment
> Document version: v1.0 · 2026-06-14

---

## 1. Background

In UAV terrain perception systems, the drone needs to perceive terrain variations (undulations, obstacles) beneath it for safe flight path planning. The current system provides:

- ✅ AirSim + PX4 SITL simulation environment (Windows + WSL2)
- ✅ YOLO object detection (bottom_center camera)
- ✅ Pixel → GPS coordinate conversion (coordinate_system.py)
- ✅ Geofence definition

**Current objective:** Integrate visual-inertial SLAM into the simulation environment, using monocular images + IMU + GPS to generate dense point clouds for terrain undulation detection and obstacle detection.

---

## 2. System Architecture

```
Windows 11 (UE4 + AirSim)                  WSL2 Ubuntu-22.04 (ROS2 Humble)
┌─────────────────────────────┐           ┌──────────────────────────────────────┐
│  AirSim SDK                 │           │  AirSim ROS Bridge                    │
│  ┌───────────────────┐     │──TCP:4560─►│  ┌──────────────────────────────┐   │
│  │ bottom_center RGB  │     │           │  │ /airsim/camera/bottom_center  │   │
│  │ IMU + GPS          │     │           │  │ /airsim/imu                   │   │
│  └───────────────────┘     │           │  └──────────┬───────────────────┘   │
│                            │           │             │                       │
│  ┌───────────────────┐     │           │             ▼                       │
│  │ Depth Anything v2  │     │           │  ┌──────────────────────────────┐   │
│  │ Monocular Depth    │     │           │  │  VINS-Fusion-ROS2            │   │
│  │ (PyTorch + GPU)    │     │           │  │  Mono + IMU + GPS Fusion     │   │
│  └─────────┬─────────┘     │           │  │  → /vins_odom (pose)          │   │
│            │               │           │  │  → /keyframe_point (sparse)   │   │
│         Depth Map          │           │  └────────────┬─────────────────┘   │
│       (UDP forward)        │           │               │                     │
│            │               │           │               ▼                     │
│            └───────────────┼───────────┼──►  ┌────────────────────────────┐ │
│                            │           │    │  Terrain Analysis Node      │ │
│                            │           │    │  ┌──────────────────────┐  │ │
│                            │           │    │  │ ① RANSAC Ground Fit  │  │ │
│                            │           │    │  │   → Elevation Map     │  │ │
│                            │           │    │  │ ② DBSCAN Clustering  │  │ │
│                            │           │    │  │   → Obstacle List     │  │ │
│                            │           │    │  └──────────────────────┘  │ │
│                            │           │    └────────────────────────────┘ │
│                            │           └──────────────────────────────────────┘
```

### 2.1 Key Constraints

- **Hardware-realistic:** Monocular RGB only (bottom_center camera), no depth camera or LiDAR
- **Backward compatible:** Existing coordinate conversion, geofence remain unchanged
- **Real-time:** End-to-end latency < 50ms

---

## 3. Component Details

### 3.1 VINS-Fusion-ROS2

**Repository:** https://github.com/zinuok/VINS-Fusion-ROS2

VINS-Fusion is a visual-inertial navigation state estimator supporting monocular + IMU + GPS fusion.

#### Input/Output

| Direction | Data | Topic | Rate | Description |
|-----------|------|-------|------|-------------|
| Input | RGB Image | `/airsim/camera/bottom_center` | 30fps | Published by AirSim ROS Bridge |
| Input | IMU Data | `/airsim/imu` | 200Hz | Accelerometer + Gyroscope |
| Input | GPS Data | `/airsim/gps` | 50Hz | Global pose fusion |
| Output | 6-DOF Pose | `/vins_odom` | 30fps | UAV position + attitude |
| Output | Sparse features | `/keyframe_point` | 10fps | Visual feature 3D positions |
| Output | Trajectory | `/vins_path` | 30fps | Historical path |

#### Camera Calibration Parameters

Based on current AirSim configuration (bottom_center, 640×640, FOV=120°):

```yaml
# config/airsim_bottom_camera.yaml
model_type: PINHOLE
camera_name: bottom_center
image_width: 640
image_height: 640
distortion_parameters:
   k1: 0.0     # AirSim rendering has no lens distortion
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: 369.0   # = (640/2) / tan(120°/2)
   fy: 369.0   # square pixels, fx == fy
   cx: 320.0
   cy: 320.0
```

IMU to camera extrinsics (body_T_cam0): Assuming camera is mounted vertically downward at UAV center bottom:
```
body_T_cam0 = Rotate -90° around X-axis (NED body frame → camera frame)
```

#### VINS Configuration Notes

```yaml
# Monocular + IMU + GPS mode
imu: 1
num_of_cam: 1
estimate_extrinsic: 0       # Extrinsics known, no optimization
max_cnt: 200                # Max feature points (terrain scenes have fewer textures)
min_dist: 25                # Min feature point spacing
freq: 10                    # Pose publish frequency
keyframe_parallax: 15.0     # Keyframe selection threshold

# IMU parameters (AirSim simulation values can be relaxed)
acc_n: 0.1
gyr_n: 0.01
acc_w: 0.001
gyr_w: 0.0001
g_norm: 9.81

# GPS fusion: Enable in global_fusion node
```

### 3.2 Depth Anything v2 (Monocular Depth Estimation)

**Model:** Depth Anything v2 ([GitHub](https://github.com/DepthAnything/Depth-Anything-V2))

Runs on the **Windows side** (using existing NVIDIA GPU) via Python + PyTorch.

#### Deployment

| Item | Value |
|------|-------|
| Runtime | Windows 11 (Python + CUDA) |
| Model | Depth-Anything-V2-Small (24MB) |
| Input | 640×640 RGB (from AirSim SDK) |
| Output | 640×640 depth map (float32, relative depth) |
| Framerate | >60fps (RTX 3060 class GPU) |
| Data channel | UDP to WSL2 ROS2 node |

#### Relative Depth → Metric Depth

Depth Anything v2 outputs relative depth (0~1 range). Convert to metric depth using VINS-Fusion pose information:

```
Method 1: Known UAV altitude (barometer/GPS)
  1. Center region downward direction = UAV height (known)
  2. scale = drone_altitude / median(depth_center)
  3. depth_metric = depth_relative × scale

Method 2: Adjacent frame VO constraints
  1. Consecutive depth frames + known pose transform
  2. Minimize photometric reprojection error for scale
```

Method 1 is recommended — simplest and fully utilizes known GPS/barometer data.

### 3.3 Terrain Analysis Node

ROS2 node (C++ or Python), subscribing to depth map and pose, publishing terrain information.

#### ① Point Cloud Generation

```
Per frame:
  depth_map (640×640)
    → Camera intrinsics unprojection → 3D point cloud
    → Camera frame → World frame (using VINS pose)
    → Accumulate into global map

Key parameters:
  - Sampling: every 4th pixel (640/4=160 → ~25600 points/frame)
  - Accumulation window: last 50 frames (~1.7s)
  - Voxel filter: 0.2m resolution downsampling
```

#### ② Terrain Undulation Detection

```
Input: 3D point cloud (world coordinates)
Pipeline:
  1. RANSAC plane fitting
     - Model: ax + by + cz + d = 0
     - Iterations: 1000
     - Threshold: 0.15m (inlier to ground)
     - Max slope: 30°
  
  2. Elevation Map generation
     - Grid resolution: 0.5m
     - Per-cell: mean height, min height, height variance
     - Range: 50×50m centered on UAV

  3. Terrain Classification
     | Category  | Criteria         | Meaning                     |
     |-----------|------------------|-----------------------------|
     | Flat      | σ_height < 0.3m  | Safe flight                 |
     | Gentle    | 0.3m ≤ σ < 1.0m | Passable, adjust attitude   |
     | Rough     | σ_height ≥ 1.0m  | Gain altitude               |

Output Topics:
  - /terrain/elevation_map: nav_msgs/OccupancyGrid
  - /terrain/slope_map: nav_msgs/OccupancyGrid
```

#### ③ Obstacle Detection

```
Input: Non-ground point cloud (RANSAC outliers)
Pipeline:
  1. Project to 2D (top-down view)
  2. DBSCAN clustering
     - eps: 1.0m (max intra-cluster distance)
     - min_samples: 5 (min points for obstacle)
  
  3. Obstacle feature extraction
     - Position (cluster center)
     - Size (bounding box)
     - Height (max - min elevation)
     - Classification:
       | Height   | Category  | Threat |
       |----------|-----------|--------|
       | < 0.5m   | Small rock| Low    |
       | 0.5~2m   | Bush      | Medium |
       | > 2m     | Tree      | High   |

Output Topics:
  - /terrain/obstacles: custom obstacle list message
  - /terrain/obstacles_viz: visualization_msgs/MarkerArray
```

---

## 4. Data Flow & Real-time Performance

### 4.1 Per-frame Processing Timeline

```
Frame N (t=0ms):
  AirSim SDK acquire RGB
    ├── (Windows) Depth Anything → Depth Map ──────────────────┐
    │                        ↓ GPU ~8ms                        │
    └── (WSL2) AirSim Bridge → VINS-Fusion ──┐                 │
                                   ↓ ~5ms   │                 │
                              Pose odometry  │                 │
                                             ▼                 ▼
                                    ┌──────────────────────────┐
                                    │ Terrain Analysis Node    │
                                    │ Depth unproject → PCD    │
                                    │ RANSAC fit ~5ms          │
                                    │ DBSCAN clustering ~5ms   │
                                    │ Publish results ~1ms     │
                                    └──────────────────────────┘
t=~25ms: Terrain/obstacle info output
```

### 4.2 Latency Budget

| Stage | Time | Notes |
|-------|------|-------|
| AirSim image + transfer | ~5ms | TCP uncompressed |
| Depth Anything v2 inference | ~8ms | Small model, RTX3060 |
| UDP forward (Windows→WSL2) | ~1ms | Local loopback |
| VINS-Fusion pose estimation | ~5ms | Feature tracking + optimization |
| Point cloud + terrain analysis | ~14ms | Multi-threaded |
| **End-to-end latency** | **~33ms** | >30fps real-time |

---

## 5. Integration with Existing System

### 5.1 Module Relationships

```
┌──────────────────────────────────────────────────────────────┐
│                  UAV Simulation System                        │
├──────────────────────────────────────────────────────────────┤
│  YOLO Detection ──→ Coordinate Conversion ──→ GPS Positions  │
│  (unchanged)       (unchanged)            (unchanged)        │
│                                                              │
│  VINS-Fusion ──→ Pose ──→ Terrain Analysis ──→ Obstacle Map │
│  (new)                   (new)              (new)            │
│                                                              │
│  Terrain Info ──→ Path Planning / Decision (future)          │
└──────────────────────────────────────────────────────────────┘
```

### 5.2 Unchanged Components

- `coordinate_system.py` — unchanged
- `run_coordinate_system.py` — unchanged
- `run_all.py` — unchanged
- AirSim configuration files — unchanged

### 5.3 AirSim Configuration

Existing `settings.json` requires no modifications. AirSim ROS Bridge reads the same configuration via RPC.

---

## 6. ROS2 Package Structure

```
airsim-px4-simulator/
├── config/
│   ├── settings.json                    # AirSim config (existing)
│   ├── vins_config.yaml                 # VINS-Fusion config (new)
│   └── airsim_bottom_camera.yaml        # Camera calibration (new)
├── ros_ws/                              # ROS2 workspace (new)
│   └── src/
│       ├── airsim_ros_pkgs/             # AirSim ROS Bridge (submodule)
│       ├── VINS-Fusion-ROS2/            # VINS-Fusion (submodule)
│       └── terrain_analysis/            # Terrain analysis (custom)
│           ├── src/
│           │   ├── terrain_analysis_node.cpp
│           │   ├── ground_fitter.cpp
│           │   └── obstacle_detector.cpp
│           ├── include/terrain_analysis/
│           ├── msg/
│           │   └── Obstacle.msg
│           ├── launch/
│           │   └── terrain_analysis.launch.py
│           ├── package.xml
│           └── CMakeLists.txt
├── windows/                              # Windows-side scripts
│   ├── depth_anything_server.py
│   └── requirements_depth.txt
└── docs/
    ├── architecture.md
    └── vins-terrain-slam-design.md
```

---

## 7. Environment Dependencies

### 7.1 WSL2 (Ubuntu-22.04)

| Dependency | Version | Description |
|-----------|---------|-------------|
| ROS2 Humble | Latest | Robot OS |
| AirSim ROS Bridge | 1.8.1 | AirSim ROS2 interface |
| VINS-Fusion-ROS2 | main | Visual-inertial SLAM |
| OpenCV | 4.x | Image processing |
| Eigen3 | 3.x | Linear algebra |
| Ceres Solver | 2.x | Nonlinear optimization (VINS dep) |

### 7.2 Windows

| Dependency | Version | Description |
|-----------|---------|-------------|
| Python | 3.10+ | Run Depth Anything |
| PyTorch | 2.x + CUDA | Depth estimation inference |
| Depth Anything v2 | Small model | Monocular depth estimation |
| ONNX Runtime | latest (optional) | Accelerated inference |

---

## 8. Implementation Steps

### Phase 1: WSL2 Environment Setup (1 day)

1. Install ROS2 Humble (desktop)
2. Create ROS2 workspace `ros_ws`
3. Build AirSim ROS Bridge
4. Build VINS-Fusion-ROS2
5. Verify: AirSim Play → ROS2 receives `/airsim/camera/bottom_center` and `/airsim/imu`

### Phase 2: VINS-Fusion Configuration & Calibration (0.5 day)

1. Write camera intrinsics calibration file
2. Write VINS configuration file
3. Configure IMU extrinsics
4. Configure GPS global fusion
5. Verify: VINS outputs `/vins_odom`, compare with AirSim ground truth

### Phase 3: Depth Anything v2 Integration (1 day)

1. Install PyTorch + Depth Anything v2 (Windows)
2. Write `depth_anything_server.py`
3. Write WSL2 UDP receiver node, publish as ROS2 topic

### Phase 4: Terrain Analysis Node (1.5 days)

1. Point cloud generation module
2. RANSAC ground fitting module
3. Elevation map generation module
4. DBSCAN obstacle clustering module
5. ROS2 node + launch file
6. RViz visualization configuration

### Phase 5: Integration & Testing (1 day)

1. Full end-to-end verification
2. Parameter tuning (RANSAC threshold, DBSCAN eps, etc.)
3. Performance optimization (latency within 50ms)
4. Parallel testing with existing detection/coordinate system

---

## 9. Evaluation Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| VINS pose error | < 5% drift (100m flight) | Compare with AirSim ground truth |
| Depth estimation accuracy | RMSE < 0.3m @ 30m | Compare with AirSim depth camera |
| Terrain detection | Accuracy > 85% | Manual validation set |
| Obstacle detection | Recall > 80% | Manual validation set |
| End-to-end latency | < 50ms | Timestamp tracing |
| System framerate | > 20fps | Continuous run statistics |

---

## 10. References

- [VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2) — SLAM framework
- [Depth Anything v2](https://github.com/DepthAnything/Depth-Anything-V2) — Monocular depth estimation
- [AirSim ROS2 Bridge](https://microsoft.github.io/AirSim/docs/ros2/) — AirSim ROS2 interface
- [PX4 SITL](https://docs.px4.io/main/en/simulation/) — PX4 simulation
- [airsim_ros_pkgs](https://github.com/Microsoft/AirSim/tree/main/ros/src/airsim_ros_pkgs) — AirSim ROS bridge packages
