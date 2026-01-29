# autoracer_robot_urdf

AutoRacer Ackermann 转向 RC 小车的 URDF 机器人模型。

## 结构

```
autoracer_robot_urdf/
├── urdf/
│   └── autoracer.urdf.xacro   # Xacro 格式 URDF 模型
├── launch/
│   └── robot_description.launch.py  # 加载 URDF + robot_state_publisher
└── rviz/
    └── autoracer.rviz          # RViz2 预配置显示
```

## URDF 模型内容

- **底盘** (base_link): 长 0.85m × 宽 0.50m × 高 0.20m 方块
- **后轮** × 2: 半径 0.11m 圆柱，continuous joint
- **前轮** × 2: 带转向节 (revolute ±22.5°) + 车轮 (continuous)
- **LiDAR C32** (laser): 固定在 base_link 前方 X=+0.24m, Z=+0.39m
- **ZED X** (zed_camera_link): 固定在 base_link 前方 X=+0.34m, Z=+0.29m

## 运行

```bash
# 单独启动 URDF 可视化
ros2 launch autoracer_robot_urdf robot_description.launch.py

# 带关节 GUI 调试
ros2 launch autoracer_robot_urdf robot_description.launch.py use_joint_state_publisher_gui:=true

# RViz2 可视化
rviz2 -d $(ros2 pkg prefix autoracer_robot_urdf)/share/autoracer_robot_urdf/rviz/autoracer.rviz
```

## 参考

- Wheeltec: `reference/wheeltec_ros2/src/wheeltec_robot_urdf/`
- URDF 模型: `urdf/top_akm_bs_robot.urdf` (Ackermann 底盘)
