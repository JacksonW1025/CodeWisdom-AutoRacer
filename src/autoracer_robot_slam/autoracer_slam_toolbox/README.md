# autoracer_slam_toolbox — SLAM Toolbox 2D 建图

SLAM Toolbox 2D 建图配置包，适配 AutoRacer (Ackermann 转向 RC 小车) 平台。

## 功能

- 使用 SLAM Toolbox 进行 2D 激光建图
- 集成 pointcloud_to_laserscan 将 C32 3D 点云转换为 2D 扫描
- 支持 loop closure (回环检测)

## 依赖

- `slam_toolbox`：系统包（`sudo apt install ros-humble-slam-toolbox`）
- `pointcloud_to_laserscan`：系统包（`sudo apt install ros-humble-pointcloud-to-laserscan`）

## 启动

### 一键启动（含底盘 + LiDAR）

```bash
# 启动 SLAM Toolbox 建图（含 RViz2）
ros2 launch autoracer_slam_toolbox slam.launch.py

# 不启动 RViz2
ros2 launch autoracer_slam_toolbox slam.launch.py use_rviz:=false

# 使用 EKF 融合后的里程计
# EKF（扩展卡尔曼滤波）将轮式里程计和 IMU 数据融合，输出更平滑准确的 odom_combined
ros2 launch autoracer_slam_toolbox slam.launch.py odom_frame:=odom_combined

# 仅 SLAM（需自行在其它终端启动底盘 + LiDAR）
ros2 launch autoracer_slam_toolbox slam.launch.py include_bringup:=false
```

### 无底盘临时测试（仅 LiDAR）

底盘未连接时，`autoracer_robot` 节点会因串口打不开而崩溃，导致缺少 `odom → base_footprint` TF，SLAM Toolbox 无法建图。
可用静态 TF 临时替代：

```bash
# 终端 1: 启动 SLAM Toolbox（底盘节点会报错退出，不影响其他节点）
ros2 launch autoracer_slam_toolbox slam.launch.py

# 终端 2: 发布假的 odom TF（车辆静止在原点）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint

# 若使用 odom_frame:=odom_combined，则静态 TF 也需对应修改：
# 终端 1: ros2 launch autoracer_slam_toolbox slam.launch.py odom_frame:=odom_combined
# 终端 2: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_combined base_footprint
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

## TF

- 广播：`map` → `odom`
- 需要：`odom` → `base_footprint`（来自底盘驱动）
- 完整 TF 链：`map → odom → base_footprint → base_link → laser`

## 参考

- `reference/wheeltec_ros2/src/wheeltec_robot_slam/wheeltec_slam_toolbox/`
