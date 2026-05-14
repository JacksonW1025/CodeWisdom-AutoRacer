# autoracer_robot_nav2

Nav2 导航配置包，适配 AutoRacer Ackermann 转向平台。

当前运行入口、阶段验收要求和 legacy 禁止项以仓库根 `docs/启动与运行规范.md` 为准。本文只保留本包用途和典型命令提示，不作为阶段验收标准。

## 功能

- Nav2 自主导航（AMCL 定位 + Hybrid A* 规划 + MPPI Ackermann 控制）
- 地图加载与保存
- pointcloud_to_laserscan 集成

## 依赖

- `navigation2` (系统包 `ros-humble-navigation2`)
- `nav2_bringup` (系统包 `ros-humble-nav2-bringup`)
- `pointcloud_to_laserscan` (系统包 `ros-humble-pointcloud-to-laserscan`)

## 使用

### 建图流程

```bash
# 1. 按 docs/启动与运行规范.md 启动 phase-1 Ackermann 底盘链路和 LiDAR
# 底盘验收路径必须使用 ackermann_chassis.launch.py counts_per_meter:=<实测值> use_ekf:=true
ros2 launch lslidar_driver lslidar_cx_launch.py

# 2. 启动 SLAM Toolbox 建图；阶段 3 验收必须关闭 legacy bringup include
ros2 launch autoracer_slam_toolbox slam.launch.py include_bringup:=false

# 3. 低速移动或按阶段 3 验收流程扫图
ros2 run autoracer_keyboard keyboard_control

# 4. 保存地图
ros2 launch autoracer_robot_nav2 save_map.launch.py
```

### 导航流程

```bash
# 1. 按 docs/启动与运行规范.md 启动 phase-1 Ackermann 底盘链路和 LiDAR
ros2 launch lslidar_driver lslidar_cx_launch.py

# 2. 启动 Nav2 导航（加载地图）
ros2 launch autoracer_robot_nav2 navigation.launch.py map:=/path/to/autoracer_map.yaml

# 3. 在 RViz2 中设置初始位姿和目标点
```

旧 `turn_on_autoracer_robot.launch.py` 默认路径属于 legacy/default bringup，不能作为阶段验收入口。

## AutoRacer 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| min_turning_radius | 1.45m | 最小转弯半径 |
| motion_model | Ackermann | MPPI 运动模型 |
| planner | SmacPlannerHybrid | Reeds-Shepp 曲线 |
| footprint | 0.85m × 0.50m | 车身轮廓 |

## 参考

- `reference/wheeltec_ros2/src/wheeltec_robot_nav2/`
- `reference/wheeltec_ros2/src/wheeltec_robot_nav2/param/wheeltec_params/param_top_akm_bs.yaml`
