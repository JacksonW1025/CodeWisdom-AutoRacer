# autoracer_imu_tf_broadcaster

AutoRacer 的可视化专用 IMU TF 广播包。

这个包只用于 RViz 姿态联调，不参与主 TF 链、SLAM、Nav2 或其他算法输入。

## 功能

- 订阅 `/imu/data_raw`
- 发布动态 TF `viz_world -> viz_base_link`
- 平移固定为车体安装高度，旋转直接使用 IMU 原始四元数

## 典型用法

```bash
source /home/car/CodeWisdom-AutoRacer/source_all.sh
ros2 launch autoracer_robot_urdf imu_attitude_viz.launch.py
```

前提是 `hipnuc_imu` 已经在发布 `/imu/data_raw`。
