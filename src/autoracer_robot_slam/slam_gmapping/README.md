# slam_gmapping — GMapping 2D SLAM

基于粒子滤波（Rao-Blackwellized Particle Filter）的 2D 栅格地图 SLAM 算法，从 Wheeltec 参考实现迁移。

## 依赖

- `openslam_gmapping`：gmapping 核心算法库（同目录下）
- `pointcloud_to_laserscan`：系统包（`sudo apt install ros-humble-pointcloud-to-laserscan`）

## 启动

### 一键启动（含底盘 + LiDAR）

```bash
# 启动 GMapping 建图（含 RViz2）
ros2 launch slam_gmapping slam_gmapping.launch.py

# 不启动 RViz2
ros2 launch slam_gmapping slam_gmapping.launch.py use_rviz:=false

# 仅 SLAM（需自行在其它终端启动底盘 + LiDAR）
ros2 launch slam_gmapping slam_gmapping.launch.py include_bringup:=false
```

### 无底盘临时测试（仅 LiDAR）

底盘未连接时，`autoracer_robot` 节点会因串口打不开而崩溃，导致缺少 `odom → base_footprint` TF，GMapping 无法建图。
可用静态 TF 临时替代：

```bash
# 终端 1: 启动 GMapping（底盘节点会报错退出，不影响其他节点）
ros2 launch slam_gmapping slam_gmapping.launch.py

# 终端 2: 发布假的 odom TF（车辆静止在原点）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint
```

> **注意**：静态 TF 仅用于验证建图流程，车辆不会移动，地图只能看到当前位置周围的扫描。
> 正式使用请连接底盘驱动提供真实里程计。

### 验证数据流

```bash
ros2 topic hz /scan     # 确认 2D scan 数据（应 ~20Hz）
ros2 topic hz /map      # 确认地图在更新
```

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
