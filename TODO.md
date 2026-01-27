# TODO.md - AutoRacer vs Wheeltec 功能对比与待办清单

> 生成日期: 2026-01-25
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

## 二、待办清单（按优先级）

### P0 - 核心基础（URDF/TF 配置）

> **实施策略**: 分步实施 - Phase A: 静态 TF (快速验证) → Phase B: URDF 模型 (完善可视化)

#### Phase A: 静态 TF 配置（快速验证）

| # | 任务 | 参考/说明 | 复杂度 |
|---|------|----------|-------|
| 1a | **测量传感器安装位置** | 测量 LiDAR (X,Y,Z,Yaw) 和 ZED X (X,Y,Z,Pitch) 相对于**后轴中点**的偏移 | ⭐ |
| 1b | **测量底盘外形尺寸** | 测量底盘长/宽/高，用于后续 URDF 碰撞模型 | ⭐ |
| 2a | **添加 LiDAR 静态 TF** | 在 `turn_on_autoracer_robot.launch.py` 添加 `base_link → laser` 静态变换 | ⭐ |
| 2b | **添加 ZED X 静态 TF** | 在 launch 中添加 `base_link → zedx_camera_link` 静态变换 | ⭐ |
| 2c | **RViz TF 验证** | 在 RViz 中检查 TF 树、LiDAR 点云和 ZED 数据是否正确对齐 | ⭐ |

#### Phase B: URDF 模型创建（完善）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 3 | **创建 AutoRacer URDF** | `wheeltec_robot_urdf/urdf/top_akm_bs_robot.urdf` | 底盘 box + 4 轮子 cylinder + 传感器 link | ⭐⭐⭐ |
| 4 | Ackermann 消息定义 | `depend/ackermann_msgs-ros2/` | Ackermann 转向标准消息类型 | ⭐ |
| 5 | RViz 配置包 | `wheeltec_rviz2/` | 预配置 RViz 显示，便于调试和可视化 | ⭐⭐ |

#### 测量参考（base_footprint = 后轴中点地面投影）

**LiDAR (镭神 C32)**:
- X: 前后距离 (前为正)
- Y: 左右偏移 (左为正)
- Z: 地面到 LiDAR 中心高度
- Yaw: 朝向 (正向=0, 反向=π)

**ZED X 深度相机**:
- X: 前后距离
- Y: 左右偏移
- Z: 地面到安装螺丝孔高度
- Pitch: 向下倾斜角度 (向下为负, 弧度)

### P1 - 导航基础（实现自主导航）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 5 | pointcloud_to_laserscan | `wheeltec_lidar_ros2/pointcloud_to_laserscan-humble/` | 将 3D 点云转换为 2D LaserScan，Nav2 需要 | ⭐⭐ |
| 6 | Nav2 配置包 | `wheeltec_robot_nav2/` | Nav2 参数、launch 文件、地图配置 | ⭐⭐⭐ |
| 7 | 航点导航 | `nav2_waypoint_cycle/` | 多航点循环导航功能 | ⭐⭐ |
| 8 | 路径跟随 | `wheeltec_path_follow/` | 预定义路径跟踪控制 | ⭐⭐ |

### P2 - SLAM 建图（环境感知）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 9 | Cartographer | `wheeltec_robot_slam/wheeltec_cartographer/` | Google 激光 SLAM，推荐首选 | ⭐⭐⭐ |
| 10 | SLAM Toolbox | `wheeltec_robot_slam/wheeltec_slam_toolbox/` | 另一个流行的 2D SLAM | ⭐⭐⭐ |
| 11 | GMapping | `wheeltec_robot_slam/slam_gmapping/` | 经典 2D SLAM（备选） | ⭐⭐ |
| 12 | RTAB-Map | `wheeltec_robot_rtab/` | 3D SLAM，支持视觉+激光 | ⭐⭐⭐⭐ |
| 13 | ORB-SLAM2 | `wheeltec_robot_slam/orb_slam_2_ros-ros2/` | 视觉 SLAM | ⭐⭐⭐⭐ |
| 14 | LeGO-LOAM | `wheeltec_robot_slam/LeGO-LOAM-SR-master/` | 3D LiDAR SLAM | ⭐⭐⭐⭐ |
| 15 | LIO-SAM | `wheeltec_robot_slam/LIO-SAM-ROS2/` | LiDAR-IMU 紧耦合 SLAM | ⭐⭐⭐⭐ |

### P3 - 传感器扩展（增强感知）

| # | 任务 | 参考包路径 | 说明 | 复杂度 |
|---|------|-----------|------|-------|
| 16 | USB 摄像头 | `usb_cam-ros2/` | 通用 USB 摄像头驱动 | ⭐⭐ |
| 17 | 超声波避障 | `wheeltec_ultrasonic_avoid/` | 8 通道超声波传感器避障（需实现 Supersonic 发布） | ⭐⭐ |
| 18 | 手柄控制 | `wheeltec_joy/` | PS4/Xbox 手柄遥控 | ⭐⭐ |
| 19 | IMU 驱动增强 | `wheeltec_imu/` | 独立 IMU 驱动包（hipnuc 等） | ⭐⭐ |
| 20 | GPS 驱动 | `wheeltec_gps/` | GNSS/GPS 定位 | ⭐⭐⭐ |
| 21 | Astra 深度相机 | `ros2_astra_camera-master/` | Orbbec Astra 深度相机驱动 | ⭐⭐⭐ |
| 22 | 麦克风 | `wheeltec_mic/` | 语音输入 | ⭐⭐ |

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

## 三、推荐实施路线

### Phase 1: 基础完善（建议优先完成）
```
P0-1a 测量传感器位置 → P0-2a LiDAR TF → P0-2b ZED X TF → P0-2c RViz 验证
        ↓ (验证通过后)
P0-3 创建 URDF 模型 → P0-5 RViz 配置包
```
**目标**: 可在 RViz 中完整可视化机器人状态、传感器数据正确对齐

### Phase 2: 导航能力
```
P1-5 pointcloud_to_laserscan → P2-9 Cartographer/SLAM Toolbox → P1-6 Nav2 配置
```
**目标**: 实现建图与自主导航

### Phase 3: 感知增强
```
P3-16 USB 摄像头 → P4-25 YOLO 检测 → P4-23 目标跟随
```
**目标**: 实现视觉感知与目标跟踪

### Phase 4: 高级功能（按需选择）
```
P4/P5 中根据实际需求选择
```

---

## 四、复杂度说明

| 符号 | 估计工作量 | 说明 |
|-----|-----------|------|
| ⭐ | 简单 | 直接复制/配置，< 1 小时 |
| ⭐⭐ | 中等 | 需要适配修改，1-4 小时 |
| ⭐⭐⭐ | 较复杂 | 需要理解代码并调试，4-8 小时 |
| ⭐⭐⭐⭐ | 复杂 | 需要深入研究，可能有依赖问题，> 1 天 |

---

## 五、备注

1. **命名规则**: 迁移时将 `wheeltec` 改为 `autoracer`，其余保持一致
2. **依赖管理**: 优先使用 `rosdep` 安装系统依赖
3. **参考顺序**: 先读 reference 代码，理解后再迁移
4. **测试验证**: 每完成一个模块都要测试验证后再继续

---

*此文档由 Claude Code 自动生成，基于 reference/wheeltec_ros2 分析*
