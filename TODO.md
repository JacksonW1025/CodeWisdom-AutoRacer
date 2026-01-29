# TODO.md - AutoRacer vs Wheeltec 功能对比与待办清单

> 更新日期: 2026-01-27
> 目标: 将 AutoRacer 功能补全至 Wheeltec S200 同等水平

---

## 一、功能对比表

### 1.1 已完成功能

| 功能模块 | AutoRacer 包 | Wheeltec 参考包 | 状态 |
|---------|-------------|----------------|------|
| 底盘驱动 | `turn_on_autoracer_robot` | `turn_on_wheeltec_robot` | ✅ 完成 |
| 自定义消息 | `autoracer_interfaces` | `wheeltec_robot_msg` + `robot_interfaces` | ✅ 完成 |
| 键盘遥控 | `autoracer_keyboard` | `wheeltec_robot_keyboard` | ✅ 完成 |
| 镭神 LiDAR | `autoracer_lidar_ros2/lslidar_ros` | `wheeltec_lidar_ros2/lslidar_ros` | ✅ 完成 |
| 串口通信库 | `depend/serial_ros2` | `depend/serial_ros2` | ✅ 完成 |
| **ZED X 深度相机** | `zed-ros2-wrapper` | `zed-ros2-wrapper` (官方 v5.1.0) | ✅ 完成 (2026-01-26) |
| **ZED X RViz2 可视化** | `zed_display_rviz2` | `zed-ros2-examples/zed_display_rviz2` | ✅ 完成 (2026-01-27) |

**已完成: 7/35+ 模块**

### 1.2 缺失功能统计

| 类别 | Wheeltec 包数量 | AutoRacer 缺失 |
|-----|----------------|---------------|
| 核心基础 | 4 | 4 |
| 导航功能 | 4 | 4 |
| SLAM | 7 | 7 |
| 传感器驱动 | 8 | 7 |
| 视觉/AI | 6 | 6 |
| 工具/其他 | 6 | 6 |
| **总计** | **35+** | **34** |

---

## 二、传感器准备检查清单（用户需完成）

> 在 Agent 开始传感器集成之前，用户需要完成以下准备工作。

### 2.1 物理测量（相对后轴中点 base_footprint，单位：米）

| 传感器 | 需要测量 | Wheeltec 参考值 | 备注 |
|--------|---------|----------------|------|
| **镭神 LiDAR C32** | X, Y, Z, Yaw | X=0.24, Y≈0, Z=0.15, Yaw=π(朝后) | 安装位置可变，用户决定 |
| **ZED X 深度相机** | X, Y, Z, Pitch | X=0.31, Y≈0, Z=0.13, Pitch=0 | 安装位置可变，用户决定 |
| **N300 Pro IMU** | X, Y, Z（如不在中心） | X=0, Y=0, Z=0（与 base_footprint 重合） | 通常固定在底盘中心 |
| **G90 GNSS 主天线** | X, Y, Z | 无参考（按实际安装） | 需开阔天空视野 |
| **底盘外形** | 长, 宽, 高 | 无参考 | 用于后续 URDF 碰撞模型 |

### 2.2 硬件连接与设备确认

| 传感器 | 需要确认 | 确认方法 |
|--------|---------|---------|
| **N300 Pro IMU** | USB 设备路径、USB ID、serial number | 插入 USB 后: `ls /dev/ttyUSB* /dev/ttyACM*` + `lsusb` + `udevadm info -a /dev/ttyXXX \| grep serial` |
| **G90 GNSS** | USB 设备路径、USB ID、serial number | 同上 |
| **G90 双天线** | 两天线间距(baseline)、连线方向与车前进方向角度 | 用卷尺测量 |
| **G90 4G DTU** | SIM 卡已安装、NTRIP 服务商/用户名/密码/挂载点 | 在 G90 Web 管理界面配置 |
| **超声波模块** | HY2.0 线缆是否已接到 STM32、通道映射（哪个通道对应哪个方向） | 检查 STM32 接口和线缆 |

### 2.3 用户决策项

| 决策 | 选项 | 建议 |
|------|-----|------|
| 是否用 N300 Pro 替代 STM32 板载 MPU6050？ | 是/否 | **推荐：是**（N300 Pro 精度远高于 MPU6050，wheeltec 也这样做） |
| IMU X 轴方向朝前 | 对齐车辆前进方向 |
| G90 使用 Unicore 还是 NMEA 协议？ | Unicore/NMEA | **推荐：Unicore**（包含双天线 heading/pitch/roll） |
| 是否启用超声波避障？ | 是/否 | 可后续启用 |
| 传感器安装位置是否已固定？ | 是/否 | 固定后再测量 |

---

## 三、待办清单（按优先级）

### P0 - 核心基础（URDF/TF/传感器融合）

> **实施策略**: Phase A: 静态 TF → Phase B: IMU/EKF 融合 → Phase C: URDF 模型

#### Phase A: 静态 TF 配置（快速验证）

| # | 任务 | 前置条件 | Agent/用户 | 参考 | 复杂度 |
|---|------|---------|-----------|------|-------|
| 1a | **测量传感器安装位置** | 传感器已物理安装 | 用户 | ✅ 完成 (2026-01-29): LiDAR/ZED X 位置已测量 | ⭐ |
| | - LiDAR: X, Y, Z (m), Yaw (rad) | | | 参考: xyz="0.24, 0, 0.15" rpy="-1.57, 0, 0" | |
| | - ZED X: X, Y, Z (m), Pitch (rad) | | | 参考: xyz="0.31, 0, 0.13" rpy="0, 0, 0" | |
| | - 底盘: 长, 宽, 高 (m) | | | | |
| 1b | **确认 N300 Pro USB 设备信息** ✅ | IMU 已连接 | 用户 | ✅ 已完成 (2026-01-28): idVendor=10c4, idProduct=ea60, serial=0003, /dev/ttyUSB0 | ⭐ |
| 1c | **确认 G90 USB 设备信息** | GNSS 已连接 | 用户 | 命令: `lsusb` + `udevadm info` | ⭐ |
| 2a | **添加 LiDAR 静态 TF** | 1a 完成 | Agent | ✅ 完成 (2026-01-29): `base_link → laser` (X=+0.24m, Z=+0.39m, yaw=+90°) | ⭐ |
| 2b | **添加 ZED X 静态 TF** | 1a 完成 | Agent | ✅ 完成 (2026-01-29): `base_link → zed_camera_link` (X=+0.34m, Z=+0.29m) | ⭐ |
| 2c | **RViz TF 验证** | 2a+2b 完成 | Agent+用户 | 待验证: RViz 中检查 TF 树、点云/深度数据对齐 | ⭐ |

#### Phase B: IMU + EKF 融合 ✅ (2026-01-28)

| # | 任务 | 前置条件 | Agent/用户 | 详细步骤 | 状态 |
|---|------|---------|-----------|---------|------|
| 3a | **创建 N300 Pro udev 规则** | 1b 完成 | Agent | `/etc/udev/rules.d/autoracer_imu.rules` → `/dev/autoracer_imu` | ✅ 完成 |
| 3b | **移植 hipnuc_imu 驱动** | 3a 完成 | Agent | 从 `reference/wheeltec_ros2/src/wheeltec_imu/hipnuc_imu/` 复制到 `src/` | ✅ 完成 |
| 3c | **移植 imu_tf_broadcaster** | 3b 完成 | Agent | 暂不需要（使用静态 TF） | ⏸️ 跳过 |
| 3d | **配置 IMU Madgwick 滤波器** | 3b 完成 | Agent | `config/imu.yaml` + `ros-humble-imu-filter-madgwick` | ✅ 完成 |
| 3e | **配置 EKF 融合** | 3d 完成 | Agent | `config/ekf.yaml` + `launch/autoracer_ekf.launch.py` | ✅ 完成 |
| 3f | **更新主 bringup launch** | 3b~3e 完成 | Agent | `turn_on_autoracer_robot.launch.py` 集成 hipnuc_imu + Madgwick + topic remapping | ✅ 完成 |

#### Phase C: GNSS 集成

| # | 任务 | 前置条件 | Agent/用户 | 详细步骤 | 复杂度 |
|---|------|---------|-----------|---------|-------|
| 4a | **创建 G90 udev 规则** | 1c 完成 | Agent | 创建 `/etc/udev/rules.d/autoracer_gnss.rules`，symlink `/dev/autoracer_gnss` | ⭐ |
| 4b | **移植 GNSS 驱动包** | 4a 完成 | Agent | 复制 `reference/WHEELTEC_G90/wheeltec-gps-ros2-20250929/src/` 相关包到 `src/` | ⭐⭐ |
| | | | | 包含: `wheeltec_dual_rtk_driver`, `wheeltec_gps_driver`, `nmea_navsat_driver`, `nmea_msgs` | |
| | | | | 修改配置: port → `/dev/autoracer_gnss` | |
| | | | | 安装依赖: `pyproj`, `transforms3d`, `pyserial`, `ros-humble-tf-transformations` | |
| 4c | **测试 GNSS 数据** | 4b + 4G/NTRIP 配置完成 | Agent+用户 | `ros2 launch wheeltec_gps_driver wheeltec_dual_rtk_driver_unicore.launch.py` | ⭐⭐ |
| | | | | 检查话题: `/gps/fix`, `/gps/utm_pose`, `/gps/euler` | |
| 4d | **添加 GNSS navsat_link TF** | 测量完成 | Agent | `base_link → navsat_link` 静态 TF | ⭐ |

#### Phase D: URDF 模型（完善可视化）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 5 | **创建 AutoRacer URDF** ✅ | `wheeltec_robot_urdf/urdf/top_akm_bs_robot.urdf` | ✅ 完成 (2026-01-29): `autoracer_robot_urdf` 包, Xacro URDF, robot_state_publisher 集成 | ⭐⭐⭐ |
| 6 | Ackermann 消息定义 | `depend/ackermann_msgs-ros2/` | Ackermann 转向标准消息类型 | ⭐ |
| 7 | RViz 配置包 | `wheeltec_rviz2/` | 预配置 RViz 显示，便于调试和可视化 | ⭐⭐ |

#### 测量参考（base_footprint = 后轴中点地面投影）

**LiDAR (镭神 C32)**:
- X: 前后距离 (前为正)
- Y: 左右偏移 (左为正)
- Z: 地面到 LiDAR 光学中心高度
- Yaw: 朝向 (正向=0, 反向=π)
- Wheeltec top_akm_bs: xyz="0.240, 0, 0.153" rpy="-1.57, 0, 0"

**ZED X 深度相机**:
- X: 前后距离
- Y: 左右偏移
- Z: 地面到安装螺丝孔高度
- Pitch: 向下倾斜角度 (向下为负, 弧度)
- Wheeltec top_akm_bs: xyz="0.312, 0, 0.135" rpy="0, 0, 0"

**N300 Pro IMU**:
- 通常与 base_footprint 重合 (0, 0, 0)
- 确认 X 轴朝前

**G90 GNSS 主天线**:
- X, Y, Z: 主天线相对后轴中点位置
- 双天线 baseline 长度（影响 heading 精度，建议 > 0.5m）

### P1 - 导航基础（实现自主导航）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 8 | pointcloud_to_laserscan | `wheeltec_lidar_ros2/pointcloud_to_laserscan-humble/` | 将 3D 点云转换为 2D LaserScan，Nav2 需要 | ⭐⭐ |
| 9 | Nav2 配置包 | `wheeltec_robot_nav2/` | Nav2 参数、launch 文件、地图配置 | ⭐⭐⭐ |
| 10 | 航点导航 | `nav2_waypoint_cycle/` | 多航点循环导航功能 | ⭐⭐ |
| 11 | 路径跟随 | `wheeltec_path_follow/` | 预定义路径跟踪控制 | ⭐⭐ |

### P2 - SLAM 建图（环境感知）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 12 | Cartographer | `wheeltec_robot_slam/wheeltec_cartographer/` | Google 激光 SLAM，推荐首选 | ⭐⭐⭐ |
| 13 | SLAM Toolbox | `wheeltec_robot_slam/wheeltec_slam_toolbox/` | 另一个流行的 2D SLAM | ⭐⭐⭐ |
| 14 | GMapping | `wheeltec_robot_slam/slam_gmapping/` | 经典 2D SLAM（备选） | ⭐⭐ |
| 15 | RTAB-Map | `wheeltec_robot_rtab/` | 3D SLAM，支持视觉+激光 | ⭐⭐⭐⭐ |
| 16 | ORB-SLAM2 | `wheeltec_robot_slam/orb_slam_2_ros-ros2/` | 视觉 SLAM | ⭐⭐⭐⭐ |
| 17 | LeGO-LOAM | `wheeltec_robot_slam/LeGO-LOAM-SR-master/` | 3D LiDAR SLAM | ⭐⭐⭐⭐ |
| 18 | LIO-SAM | `wheeltec_robot_slam/LIO-SAM-ROS2/` | LiDAR-IMU 紧耦合 SLAM | ⭐⭐⭐⭐ |

### P3 - 传感器扩展（增强感知）

| # | 任务 | 参考包路径 | 详细步骤 | 复杂度 |
|---|------|-----------|---------|-------|
| 19 | **超声波避障** | `wheeltec_ultrasonic_avoid/` | 1. 确认 STM32 接线和通道映射 | ⭐⭐ |
| | | | 2. 验证 STM32 上报超声波数据（`/Distance` 话题） | |
| | | | 3. 移植 `ultrasonic_avoid` 包（Python, 订阅 `/Distance` + `/cmd_vel`，发布 `/cmd_vel_avoid`） | |
| | | | 4. 调整避障阈值（前 0.5m, 侧 0.3m） | |
| 20 | USB 摄像头 | `usb_cam-ros2/` | 通用 USB 摄像头驱动 | ⭐⭐ |
| 21 | 手柄控制 | `wheeltec_joy/` | PS4/Xbox 手柄遥控 | ⭐⭐ |
| 22 | 麦克风 | `wheeltec_mic/` | 语音输入 | ⭐⭐ |

> 注: N300 Pro IMU 和 G90 GNSS 已提升到 P0（核心基础），因为它们是导航和 SLAM 的前置依赖。

### P4 - 视觉/AI（智能功能）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 23 | 目标跟随 | `simple_follower_ros2/` | 跟随移动目标 | ⭐⭐⭐ |
| 24 | KCF 目标跟踪 | `wheeltec_robot_kcf/` | KCF 算法视觉跟踪 | ⭐⭐⭐ |
| 25 | YOLO 目标检测 | `ultralytics_ros2/` | YOLOv8 物体检测 | ⭐⭐⭐ |
| 26 | ArUco 标记检测 | `aruco_ros-humble-devel/` | ArUco 二维码定位 | ⭐⭐ |
| 27 | 人体姿态识别 | `wheeltec_bodyreader/` | 骨骼检测 | ⭐⭐⭐⭐ |
| 28 | LLM 集成 | `wheeltec_ollama/` | Ollama 大语言模型接口 | ⭐⭐⭐ |

### P5 - 工具/其他（可选功能）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 29 | Web 视频流 | `web_video_server-ros2/` | 通过 HTTP 查看摄像头画面 | ⭐⭐ |
| 30 | 语音合成 TTS | `tts_make_ros2/` | 文字转语音 | ⭐⭐ |
| 31 | RRT 路径规划 | `wheeltec_robot_rrt2/` + `wheeltec_rrt_msg/` | RRT 全局规划器 | ⭐⭐⭐ |
| 32 | Qt GUI | `qt_ros_test/` | 图形化控制界面 | ⭐⭐⭐ |
| 33 | 多机器人 | `wheeltec_multi/` | 多机协同 | ⭐⭐⭐⭐ |
| 34 | 自动充电 | `auto_recharge_ros2/` | 自动回充功能 | ⭐⭐⭐⭐ |

---

## 四、推荐实施路线

### Phase 1: 传感器准备与 TF 配置
```
用户: 测量传感器位置 + 确认 USB 设备 + 确认 G90 配置
  ↓
Agent: udev 规则 → 静态 TF → RViz 验证
```
**目标**: 所有传感器硬件就绪，TF 树正确

### Phase 2: IMU + EKF 融合
```
Agent: hipnuc_imu 驱动 → Madgwick 滤波 → EKF 融合 (odom + IMU)
  ↓
用户+Agent: 实车调试 EKF 参数
```
**目标**: 稳定的融合里程计 `/odom_combined`

### Phase 3: GNSS 集成
```
Agent: GNSS 驱动移植 → 用户: 4G/NTRIP 配置 → Agent+用户: RTK 验证
```
**目标**: 户外 GPS 定位 + RTK 厘米级精度

### Phase 4: URDF + SLAM + 导航
```
Agent: URDF 模型 → pointcloud_to_laserscan → Cartographer/SLAM → Nav2
```
**目标**: 自主建图与导航

### Phase 5: 高级功能（按需选择）
```
P4/P5 中根据实际需求选择
```

---

## 五、Wheeltec 传感器集成架构参考

```
turn_on_wheeltec_robot.launch.py（主入口）
├── base_serial.launch.py
│   ├── wheeltec_robot_node（底盘驱动，imu/data_raw → imu/data_board 重映射）
│   ├── hipnuc_imu（外部 IMU 驱动，发布 /imu/data_raw @100Hz）
│   └── ultrasonic_avoid（超声波避障，可选）
├── robot_state_publisher（URDF → TF）
├── joint_state_publisher（关节状态）
├── imu_filter_madgwick（IMU 方向滤波，use_mag=false）
├── ekf_node（EKF 融合: odom + IMU → odom_combined，30Hz，two_d_mode）
└── static_transform_publisher × N（base_footprint→base_link, →gyro_link, →laser, →camera_link）
```

**TF 树目标**:
```
map (SLAM 提供)
└── odom_combined (EKF 输出)
    └── base_footprint
        ├── base_link (URDF 根)
        │   └── [wheel frames, steering frames]
        ├── gyro_link (IMU frame, 与 base_footprint 重合)
        ├── laser (镭神 C32, 需测量位置)
        ├── zedx_camera_link (ZED X, 需测量位置)
        │   └── [ZED 内部 TF 链: left_camera_frame → left_camera_optical_frame ...]
        └── navsat_link (G90 GNSS 主天线, 需测量位置)
```

**EKF 融合策略**:
- odom0 (轮式里程计): 使用 vx, vy, vyaw（差分模式）
- imu0 (N300 Pro): 使用 yaw, vyaw（绝对模式，去重力）
- 输出: `odom_combined` frame

**udev 设备命名**:
| 设备 | Symlink | Serial # |
|------|---------|----------|
| STM32 控制器 | `/dev/autoracer_controller` | (已配置) |
| N300 Pro IMU | `/dev/autoracer_imu` | (待确认) |
| G90 GNSS | `/dev/autoracer_gnss` | (待确认) |

---

## 六、复杂度说明

| 符号 | 估计工作量 | 说明 |
|-----|-----------|------|
| ⭐ | 简单 | 直接复制/配置 |
| ⭐⭐ | 中等 | 需要适配修改 |
| ⭐⭐⭐ | 较复杂 | 需要理解代码并调试 |
| ⭐⭐⭐⭐ | 复杂 | 需要深入研究，可能有依赖问题 |

---

## 七、备注

1. **命名规则**: 迁移时将 `wheeltec` 改为 `autoracer`，其余保持一致
2. **依赖管理**: 优先使用 `rosdep` 安装系统依赖
3. **参考顺序**: 先读 reference 代码，理解后再迁移
4. **测试验证**: 每完成一个模块都要测试验证后再继续
5. **IMU 优先级提升**: N300 Pro IMU 从 P3 提升到 P0，因为 EKF 融合是导航的前置依赖
6. **GNSS 优先级提升**: G90 GNSS 从 P3 提升到 P0，因为户外定位是导航的基础

---

*此文档由 Claude Code 生成并维护，基于 reference/ 深入分析*
