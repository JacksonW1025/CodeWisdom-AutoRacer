# autoracer_robot_nav2

Nav2 导航配置包，适配 AutoRacer Ackermann 转向平台。

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
# 1. 启动底盘 + LiDAR
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py
ros2 launch lslidar_driver lslidar_cx_launch.py

# 2. 启动 SLAM Toolbox 建图
ros2 launch autoracer_slam_toolbox slam.launch.py

# 3. 遥控小车建图
ros2 run autoracer_keyboard keyboard_control

# 4. 保存地图
ros2 launch autoracer_robot_nav2 save_map.launch.py
```

### 导航流程

```bash
# 1. 启动底盘 + LiDAR
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py
ros2 launch lslidar_driver lslidar_cx_launch.py

# 2. 启动 Nav2 导航（加载地图）
ros2 launch autoracer_robot_nav2 navigation.launch.py map:=/path/to/autoracer_map.yaml

# 3. 在 RViz2 中设置初始位姿和目标点
```

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
