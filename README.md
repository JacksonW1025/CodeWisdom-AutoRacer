# CodeWisdom-AutoRacer

基于 ROS 2 Humble 的 Ackermann RC 小车自主驾驶工作区，聚焦底盘 bringup、传感器接入、SLAM 建图、导航与调试可视化。

## Current Status

- 默认底盘入口：`ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py`
  - 启动 `autoracer_robot`
  - 可选接入 N300 Pro IMU
  - 默认启动 Madgwick 与 `robot_localization` EKF，形成 `/imu/data_raw -> /imu/data -> /odom_combined`
  - 启动 URDF / TF
- 已接入模块：
  - LiDAR C32: `lslidar_driver`
  - IMU: `hipnuc_imu`
  - URDF / RViz: `autoracer_robot_urdf`, `autoracer_imu_tf_broadcaster`
  - 2D SLAM: `autoracer_slam_toolbox`, `slam_gmapping`
  - 3D SLAM: `lio_sam`，当前唯一支持入口为 `ros2 launch lio_sam autoracer_run.launch.py`
  - Navigation: `autoracer_robot_nav2`

## Quick Start

```bash
source /home/car/CodeWisdom-AutoRacer/source_all.sh
colcon build --symlink-install

# 默认 bringup（含 EKF）
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py

# LiDAR
ros2 launch lslidar_driver lslidar_cx_launch.py

# 2D SLAM
ros2 launch autoracer_slam_toolbox slam.launch.py
# or
ros2 launch slam_gmapping slam_gmapping.launch.py

# 3D SLAM
ros2 launch lio_sam autoracer_run.launch.py
```

## Notes

- 当前仓库方向已基本对齐 Wheeltec 参考实现，但默认使用体验仍在持续收敛中，不应表述为“已完全对齐”。
- 本轮 Stage Review 修复只做了静态检查、构建和无串口依赖的 launch 校验。
- 当前 STM32 串口硬件未接入，因此没有进行底盘控制、串口收发、`/odom` 或 `/odom_combined` 的联机实测。
- `PJINFO.md` 记录了更完整的包清单、运行链路和工作状态。
