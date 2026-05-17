# slam_gmapping — GMapping 2D SLAM

基于粒子滤波（Rao-Blackwellized Particle Filter）的 2D 栅格地图 SLAM 算法，从 Wheeltec 参考实现迁移。

当前阶段 3 验收入口和固定 case 以仓库根 `docs/启动与运行规范.md` 为准。正式验收必须使用 phase-1 canonical `/odom`，并关闭本 launch 内置的 legacy bringup include。

## 依赖

- `openslam_gmapping`：gmapping 核心算法库（同目录下）
- `pointcloud_to_laserscan`：系统包（`sudo apt install ros-humble-pointcloud-to-laserscan`）

## 启动

### 阶段 3 验收启动

```bash
# 先在其它终端按 docs/启动与运行规范.md 启动：
# 1. phase-1 Ackermann 底盘链路：/wheel_odom + /imu/data -> /odom
# 2. LiDAR 驱动：/point_cloud_raw

# 启动 GMapping 建图（含 RViz2）
ros2 launch slam_gmapping slam_gmapping.launch.py include_bringup:=false

# 不启动 RViz2
ros2 launch slam_gmapping slam_gmapping.launch.py include_bringup:=false use_rviz:=false
```

`include_bringup:=true` 会拉起 legacy/default bringup，不能作为阶段 3 验收入口。

### 本地 smoke test（非阶段验收）

底盘未连接时可以用静态 TF 做 launch/topic smoke test。该方式没有真实 odom 和车辆运动，不能记录为阶段 3 通过。

```bash
# 终端 1: 启动 LiDAR
ros2 launch lslidar_driver lslidar_cx_launch.py

# 终端 2: 发布假的 odom TF（车辆静止在原点）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint

# 终端 3: 启动 GMapping
ros2 launch slam_gmapping slam_gmapping.launch.py include_bringup:=false
```

> **注意**：静态 TF 仅用于验证建图流程，车辆不会移动，地图只能看到当前位置周围的扫描。

### 验证数据流

```bash
ros2 topic hz /scan     # 确认 2D scan 数据（应 ~20Hz）
ros2 topic hz /map      # 确认地图在更新
```

阶段 3 记录还必须回填根目录验收追踪表，包括 `lidar_topic`、`tf_chain`、`slam_map_live`、`map_save_reload`、`mapping_bag_replay`。

## 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/scan` | LaserScan | 订阅 | 2D 激光扫描（由 pointcloud_to_laserscan 从 `/point_cloud_raw` 转换） |
| `/map` | OccupancyGrid | 发布 | 栅格地图 |
| `/map_metadata` | MapMetaData | 发布 | 地图元数据 |
| `/entropy` | Float64 | 发布 | 位姿熵（不确定性指标） |

## TF

- 广播：`map` → `odom`（20Hz）
- 需要：`odom` → `base_footprint`（来自底盘驱动）
- 完整 TF 链：`map → odom → base_footprint → base_link → laser`

## 关键参数（硬编码于 slam_gmapping.cpp）

| 参数 | 值 | 说明 |
|------|-----|------|
| `base_frame_` | `base_footprint` | 机器人基础坐标系 |
| `odom_frame_` | `odom` | 里程计坐标系 |
| `map_frame_` | `map` | 地图坐标系 |
| `particles_` | 30 | 粒子数量 |
| `delta_` | 0.05 | 地图分辨率 (m/cell) |
| `maxUrange_` | 80.0 | 最大有效激光距离 (m) |
