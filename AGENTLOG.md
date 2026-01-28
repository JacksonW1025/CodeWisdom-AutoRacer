# AGENTLOG.md

## Entry 1: 初始化包选择与骨架创建

- **日期**: 2026-01-20
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### 摘要
- 创建了 `.gitignore`，忽略 `reference/`、agent 文件和 ROS2 构建产物
- 分析了 `reference/wheeltec_ros2`，识别出与底盘/STM32 相关的功能包
- 确定了 Ackermann RC 小车底盘启动的最小迁移集合
- 创建了 `autoracer_interfaces` 包（自定义消息/服务）
- 创建了 `turn_on_autoracer_robot` 包，包含占位节点
- 占位节点可以编译运行，用于验证包结构

### 选择的功能包（来自 reference）

| 功能包 | 参考路径 | 选择原因 |
|--------|----------|----------|
| `turn_on_wheeltec_robot` | `src/turn_on_wheeltec_robot/` | 核心底盘驱动 - 处理 STM32/C63A 串口通信，发布里程计/IMU/电压，订阅 cmd_vel。底盘控制的必要组件。 |
| `robot_interfaces` | `src/wheeltec_ultrasonic_avoid/robot_interfaces/` | 自定义 ROS2 消息定义（Supersonic.msg, SetRgb.srv），被底盘驱动使用。 |

**未迁移（超出最小底盘启动范围）：**
- `serial` (serial_ros2) - 后续可作为依赖添加，或使用系统包
- 导航包 (wheeltec_nav2 等)
- SLAM 包 (wheeltec_cartographer, wheeltec_slam_toolbox 等)
- 传感器驱动 (wheeltec_lidar_ros2, wheeltec_imu 等)
- 手柄控制 (wheeltec_joy)

### 创建的功能包（在 src 下）

| 功能包 | 路径 | 内容 |
|--------|------|------|
| `autoracer_interfaces` | `src/autoracer_interfaces/` | 自定义消息: `Supersonic.msg`，服务: `SetRgb.srv` |
| `turn_on_autoracer_robot` | `src/turn_on_autoracer_robot/` | 主启动包，包含占位节点、launch 文件、配置文件 |

### 运行方式

```bash
# 编译工作空间
cd /home/car/CodeWisdom-AutoRacer
colcon build

# source 工作空间
source install/setup.bash

# 直接运行占位节点
ros2 run turn_on_autoracer_robot autoracer_robot_node

# 或使用 launch 文件
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py
```

### 备注 / 下一步

1. **添加串口库依赖** - 需要添加 `serial` 包（来自 `reference/wheeltec_ros2/src/depend/serial_ros2/` 或 ROS2 生态）用于实际 STM32 通信
2. **实现 STM32 协议** - 参考 `wheeltec_robot.cpp` 中的帧格式：
   - 发送帧: 帧头 `0x7B`，11 字节，帧尾 `0x7D`
   - 接收帧: 帧头 `0x7B`，24 字节，帧尾 `0x7D`
   - BCC 校验
3. **配置串口设备** - 设置 udev 规则以获得固定设备名（参考: `wheeltec_udev.sh`）
4. **填写 PROMPT.md 中的 TODO 参数**：
   - RC 小车底盘参数（轴距、最大转角、轮径）
   - STM32 通信细节（串口设备名、波特率、协议）
   - ROS 接口偏好（话题名、frame ID）
5. **实现 Ackermann 运动学** - wheeltec 代码支持多种底盘类型，需要适配 Ackermann 转向几何

---

## Entry 2: Serial Communication Implementation

- **Date**: 2026-01-20
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 复制 `serial_ros2` 库到 `src/depend/serial_ros2/`
- 创建 `autoracer_robot.hpp` 头文件，包含帧定义、数据结构、类声明
- 创建 `Quaternion_Solution.hpp` 和 `Quaternion_Solution.cpp`，用于 IMU 四元数解算
- 实现 `autoracer_robot.cpp` 串口通信节点，完整实现：
  - 串口初始化与读写
  - BCC 校验
  - 速度命令订阅 (`/cmd_vel`)
  - 里程计发布 (`/odom`)
  - IMU 数据发布 (`/imu/data_raw`)
  - 电压发布 (`/PowerVoltage`)
- 更新 `CMakeLists.txt` 和 `package.xml` 添加 serial 依赖
- 创建 `autoracer_serial.launch.py` 仅启动串口节点

### Modified/Created files
- `src/depend/serial_ros2/`: 从 reference 复制的串口库
- `src/turn_on_autoracer_robot/include/turn_on_autoracer_robot/autoracer_robot.hpp`: 主头文件
- `src/turn_on_autoracer_robot/include/turn_on_autoracer_robot/Quaternion_Solution.hpp`: 四元数头文件
- `src/turn_on_autoracer_robot/src/autoracer_robot.cpp`: 串口节点实现
- `src/turn_on_autoracer_robot/src/Quaternion_Solution.cpp`: 四元数解算实现
- `src/turn_on_autoracer_robot/CMakeLists.txt`: 添加 serial 依赖和新源文件
- `src/turn_on_autoracer_robot/package.xml`: 添加 serial 依赖
- `src/turn_on_autoracer_robot/launch/autoracer_serial.launch.py`: 新增仅串口 launch 文件
- `src/turn_on_autoracer_robot/launch/turn_on_autoracer_robot.launch.py`: 更新可执行文件名

### How to run

```bash
# 编译工作空间
cd /home/car/CodeWisdom-AutoRacer
colcon build --symlink-install

# source 工作空间
source install/setup.bash

# 仅启动串口节点
ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py

# 自定义串口参数
ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py usart_port_name:=/dev/ttyUSB0

# 完整启动（包含 TF）
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py
```

### Serial Protocol (与 wheeltec 保持一致)

| 参数 | 值 | 说明 |
|------|-----|------|
| FRAME_HEADER | 0x7B | 帧头 |
| FRAME_TAIL | 0x7D | 帧尾 |
| SEND_DATA_SIZE | 11 | ROS→STM32 发送长度 |
| RECEIVE_DATA_SIZE | 24 | STM32→ROS 接收长度 |
| 默认串口 | /dev/ttyACM0 | 可配置 |
| 波特率 | 115200 | 可配置 |

### ROS Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | geometry_msgs/Twist | Subscribe |
| `/odom` | nav_msgs/Odometry | Publish |
| `/imu/data_raw` | sensor_msgs/Imu | Publish |
| `/PowerVoltage` | std_msgs/Float32 | Publish |

### Notes / Next
- 需要连接实际 STM32 硬件进行测试
- 可能需要配置 udev 规则获得固定设备名
- 如需 Ackermann 运动学特定处理，需要后续扩展

---

## Entry 3: STM32 设备检查与 udev 配置

- **Date**: 2026-01-20
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 检测到 STM32 串口设备 `/dev/ttyACM0`
- 识别设备为 QinHeng Electronics (WCH) USB Serial
  - idVendor: 1a86
  - idProduct: 55d4
  - manufacturer: WCH.CN
- 将用户 `car` 添加到 `dialout` 组
- 配置设备权限为 `0666`
- 创建 udev 规则 `/etc/udev/rules.d/autoracer.rules`
- 编译工作空间成功（3 个包）
- 测试串口节点启动正常
  - 串口打开成功
  - 话题 `/odom`, `/imu/data_raw`, `/PowerVoltage`, `/cmd_vel` 已注册
  - 发送 cmd_vel 测试命令成功

### Device Info

| 属性 | 值 |
|------|-----|
| 设备路径 | /dev/ttyACM0 |
| idVendor | 1a86 |
| idProduct | 55d4 |
| manufacturer | WCH.CN |
| product | USB Single Serial |
| 波特率 | 115200 |

### Modified/Created files
- `/etc/udev/rules.d/autoracer.rules`: udev 规则文件，重插设备后生成 `/dev/autoracer_controller` 符号链接

### How to run

```bash
# 编译
cd /home/car/CodeWisdom-AutoRacer
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 启动串口节点
source install/setup.bash
ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py

# 检查话题
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /imu/data_raw
ros2 topic echo /PowerVoltage

# 发送速度命令测试
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Notes / Next
- 当前只连接了 AGX Orin 和 STM32，未连接传感器，因此话题无数据是正常的
- 重新插拔设备后 `/dev/autoracer_controller` 符号链接才会生效
- 下一步：连接传感器后测试数据接收
- 注意：用户需要重新登录才能使 dialout 组权限生效（或使用 `newgrp dialout`）

---

## Entry 4: 键盘遥控功能包

- **Date**: 2026-01-21
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 参考 `wheeltec_robot_keyboard` 创建 `autoracer_keyboard` 包
- 使用 ament_python 构建类型
- 实现键盘遥控节点，发布 `/cmd_vel` 话题
- 支持 i/j/k/l 方向控制、q/z/w/x/e/c 速度调节
- 实现平滑加减速（避免速度突变）
- 移除 OmniMode（AutoRacer 是 Ackermann 转向）
- 更新 CLAUDE.md 文档

### Modified/Created files
- `src/autoracer_keyboard/package.xml`: ament_python 包配置
- `src/autoracer_keyboard/setup.py`: Python 入口点配置
- `src/autoracer_keyboard/setup.cfg`: 安装配置
- `src/autoracer_keyboard/resource/autoracer_keyboard`: ament 资源标记
- `src/autoracer_keyboard/autoracer_keyboard/__init__.py`: 包初始化
- `src/autoracer_keyboard/autoracer_keyboard/keyboard_control.py`: 主控制节点 (~150行)
- `CLAUDE.md`: 添加 keyboard 包文档和按键说明

### Keyboard Mapping

| 按键 | 功能 |
|------|------|
| i | 前进 |
| , | 后退 |
| j | 左转 |
| l | 右转 |
| u/o | 前进+转向 |
| m/. | 后退+转向 |
| k/空格 | 急停 |
| q/z | 同时增/减速度 (10%) |
| w/x | 仅增/减线速度 (10%) |
| e/c | 仅增/减角速度 (10%) |

### Default Parameters
- 线速度: 0.2 m/s
- 角速度: 1.0 rad/s
- 平滑步长: 线速度 ±0.1 m/s，角速度 ±0.5 rad/s

### How to run

```bash
# 编译
colcon build --packages-select autoracer_keyboard --symlink-install

# 运行键盘控制（需要交互式终端）
source install/setup.bash
ros2 run autoracer_keyboard keyboard_control

# 配合串口节点使用（两个终端）
# 终端1: ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py
# 终端2: ros2 run autoracer_keyboard keyboard_control
```

### Notes / Next
- 键盘控制需要真实 TTY 终端（termios），无法在非交互式环境运行
- 可扩展添加 launch 文件同时启动串口和键盘节点
- 可考虑添加速度上下限保护

---

## Entry 5: 硬件串口通信验证

- **Date**: 2026-01-21
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 验证 AGX Orin 与 STM32 串口通信正常工作
- 确认串口设备 `/dev/ttyACM0` 存在且权限正确
- 启动 `autoracer_serial.launch.py`，串口打开成功
- 发送 `cmd_vel` 测试命令 (linear.x=0.2, angular.z=0.5)
- 用户确认 STM32 显示屏显示数值变化：a=10, b=10, c=390, d=390
- 硬件通信链路验证通过

### Modified/Created files
- `PJINFO.md`: 更新已完成列表，移除待办中的硬件测试项
- `AGENTLOG.md`: 添加本次日志

### How to run

```bash
# 启动串口节点
source source_all.sh
ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py

# 另开终端发送测试命令
source source_all.sh
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}" --rate 10
```

### Notes / Next
- 串口通信正常，可进行后续功能开发
- 下一步建议：Ackermann 运动学参数配置、IMU 校准
- STM32 显示的 a/b/c/d 值含义待确认（可能是电机 PWM 或速度值）

---

## Entry 6: Ackermann 运动学参数配置

- **Date**: 2026-01-23
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 根据用户实测数据配置 Ackermann 运动学参数
- 更新 `autoracer_params.yaml` 填入实测参数值
- 参数包括：wheelbase(轴距)、track_width(轮距)、wheel_radius(轮径)、max_steering_angle(最大转向角)
- 计算得出最小转弯半径约 1.45m（用于后续 Nav2 路径规划）
- 更新项目文档 CLAUDE.md 和 PJINFO.md

### Measured Parameters

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| wheelbase | 0.60 | m | 前后轴距 |
| track_width | 0.48 | m | 左右轮距 |
| wheel_radius | 0.11 | m | 车轮半径 |
| max_steering_angle | 0.393 | rad | 最大转向角(≈22.5°) |
| min_turning_radius | ~1.45 | m | 计算值 |

### Modified/Created files
- `src/turn_on_autoracer_robot/config/autoracer_params.yaml`: 填入 Ackermann 参数
- `CLAUDE.md`: 更新 Key Parameters 和 Development Status
- `PJINFO.md`: 更新已完成列表，移除已知问题中的 Ackermann 项
- `AGENTLOG.md`: 添加本次日志

### How to run

```bash
# 参数已配置，无需额外操作
# 查看参数文件
cat src/turn_on_autoracer_robot/config/autoracer_params.yaml

# 启动时参数会自动加载
ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py
```

### Notes / Next
- 当前参数仅存储在 yaml 文件中，ROS2 节点尚未读取使用（STM32 固件内部处理运动学）
- 后续集成 Nav2 时需要使用 min_turning_radius 参数
- 下一步建议：IMU 校准、LiDAR 驱动集成

---

## Entry 7: 镭神 LiDAR C32 连接验证

- **Date**: 2026-01-23
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 验证镭神 LiDAR C32 网络连接正常
- ping 192.168.1.200 成功，延迟约 0.62ms，0% 丢包
- 确认 Jetson AGX Orin IP 为 192.168.1.102 (eno1 接口)
- 识别 LiDAR 通信端口：数据端口 2368/UDP (msop)，设备端口 2369/UDP (difop)
- 找到参考驱动包 `lslidar_ros` 在 `reference/wheeltec_ros2/src/wheeltec_lidar_ros2/`
- 确认驱动支持 C32 型号，适用于 ROS2 Humble
- 更新项目文档记录 LiDAR 硬件信息

### LiDAR Info

| 属性 | 值 |
|------|-----|
| 型号 | 镭神 Leishen C32 |
| IP | 192.168.1.200 |
| 数据端口 | 2368/UDP |
| 设备端口 | 2369/UDP |
| 协议 | LSLIDAR_CX |
| 参考驱动 | lslidar_ros (LSLIDAR_CX_V4.2.4) |

### Modified/Created files
- `PJINFO.md`: 添加 LiDAR 硬件信息、更新待办优先级、添加连接测试命令
- `CLAUDE.md`: 添加 LiDAR 章节、更新开发状态
- `AGENTLOG.md`: 添加本次日志

### How to run

```bash
# 测试 LiDAR 网络连接
ping 192.168.1.200

# 查看 Jetson IP 配置
ip addr show eno1
```

### Notes / Next
- LiDAR 网络连接正常，可进行驱动集成
- 下一步：复制 `lslidar_ros` 到 `src/` 并适配 autoracer 工作空间
- 驱动依赖：ros-humble-pcl-ros, ros-humble-pluginlib, ros-humble-pcl-conversions, libpcap-dev, libboost-dev
- 默认配置文件位于 `lslidar_driver/params/lslidar_cx.yaml`，IP 已设置为 192.168.1.200

---

## Entry 8: 镭神 LiDAR C32 驱动集成

- **Date**: 2026-01-25
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 阅读镭神 C32 用户手册，了解 32 线 LiDAR 规格（905nm, 360° 水平, -16°~15° 垂直, 200m 探测）
- 分析 reference 中的 lslidar_ros 驱动结构（v4.2.4），确认支持 ROS2 Humble
- 复制 `lslidar_ros` 到 `src/autoracer_lidar_ros2/` 目录
- 修复 package.xml 中的错误依赖（移除 rslidar_input、message_runtime）
- 初始化 rosdep 并安装依赖（libpcap-dev 需降级安装解决版本冲突）
- 成功编译 lslidar_msgs 和 lslidar_driver 包
- 测试驱动：检测到 C32 LiDAR (version 3.0)，点云以 ~20Hz 发布

### LiDAR 驱动配置

| 参数 | 值 |
|------|-----|
| 驱动版本 | lslidar_ros v4.2.4 |
| LiDAR 型号 | C32, version 3.0 |
| IP 地址 | 192.168.1.200 |
| 数据端口 | 2368/UDP |
| 设备端口 | 2369/UDP |
| frame_id | laser |
| 点云话题 | /point_cloud_raw |
| LaserScan 话题 | /scan_raw |
| 发布频率 | ~20 Hz |

### Modified/Created files
- `src/autoracer_lidar_ros2/lslidar_ros/`: 从 reference 复制的 LiDAR 驱动包
- `src/autoracer_lidar_ros2/lslidar_ros/lslidar_driver/package.xml`: 移除无效依赖 rslidar_input、message_runtime
- `PJINFO.md`: 更新项目结构、当前状态、重要命令
- `CLAUDE.md`: 添加 LiDAR 运行命令

### How to run

```bash
# Source 工作空间
source source_all.sh

# 编译 LiDAR 驱动
colcon build --packages-select lslidar_msgs lslidar_driver --symlink-install

# 启动 LiDAR 驱动
ros2 launch lslidar_driver lslidar_cx_launch.py

# 启动 LiDAR + RViz 可视化
ros2 launch lslidar_driver lslidar_cx_rviz_launch.py

# 检查点云话题频率
ros2 topic hz /point_cloud_raw

# 查看话题信息
ros2 topic info /point_cloud_raw -v
```

### Notes / Next
- LiDAR 驱动工作正常，点云数据持续发布
- 当前角度过滤配置会屏蔽 110°-250° 范围，如需完整 360° 扫描需修改 `lslidar_cx.yaml`
- 下一步建议：
  - 添加 LiDAR 到 base_link 的静态 TF 变换（需测量安装位置）
  - 集成 pointcloud_to_laserscan 用于 Nav2 导航
  - 创建完整启动文件同时启动底盘驱动和 LiDAR

---

## Entry 9: ZED X 深度相机驱动集成

- **Date**: 2026-01-26
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 将 zed-ros2-wrapper v5.1.0 从 `reference/` 迁移到 `src/zed-ros2-wrapper/`
- 复制 3 个 ROS2 包：zed_ros2 (Meta), zed_components (C++组件), zed_wrapper (Launch/Config)
- 排除 docker/, images/, CONTRIBUTING.md, CHANGELOG.rst
- 确认前置依赖：ZED SDK v5.1+ 已安装 (`/usr/local/zed/`)，zed_msgs 已安装
- 成功编译 3 个包（zed_components 2分钟，zed_wrapper 3秒，zed_ros2 1秒）
- 测试启动 ZED X 相机节点，检测到相机 S/N: 42256159
- 更新项目文档 PJINFO.md, CLAUDE.md

### ZED X 相机信息

| 属性 | 值 |
|------|-----|
| 型号 | StereoLabs ZED X |
| 序列号 | 42256159 |
| 接口 | GMSL2 |
| 设备路径 | /dev/i2c-9 |
| 分辨率 | HD1200 (1920x1200) |
| 帧率 | 30 fps |
| 驱动版本 | zed-ros2-wrapper v5.1.0 |

### Modified/Created files
- `src/zed-ros2-wrapper/zed_ros2/`: Meta 包
- `src/zed-ros2-wrapper/zed_components/`: C++ ROS2 组件（双目相机、深度、IMU、Visual Odometry）
- `src/zed-ros2-wrapper/zed_wrapper/`: Launch 文件和配置文件
  - `launch/zed_camera.launch.py`: 主 Launch 文件
  - `config/zedx.yaml`: ZED X 相机配置
  - `config/common_stereo.yaml`: 立体相机通用配置
  - `urdf/`: 相机 URDF 模型
- `src/zed-ros2-wrapper/README.md`: 官方文档
- `src/zed-ros2-wrapper/CLAUDE.md`: 项目说明
- `PJINFO.md`: 更新项目结构、当前状态、重要命令
- `CLAUDE.md`: 添加 ZED X 章节、更新开发状态

### How to run

```bash
# Source 工作空间
source source_all.sh

# 编译 ZED 驱动（如需重新编译）
colcon build --packages-select zed_components zed_wrapper zed_ros2 --symlink-install --parallel-workers 2

# 启动 ZED X 深度相机
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx

# 查看相机列表
/usr/local/zed/tools/ZED_Explorer -a

# 检查话题
ros2 topic list | grep zed
```

### ZED X Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/zed/zed_node/left/image_rect_color` | Image | 左目 RGB 图像 |
| `/zed/zed_node/depth/depth_registered` | Image | 深度图 |
| `/zed/zed_node/point_cloud/cloud_registered` | PointCloud2 | 深度点云 |
| `/zed/zed_node/imu/data` | Imu | IMU 数据 |
| `/zed/zed_node/odom` | Odometry | Visual Odometry |

### Notes / Next
- ZED X 驱动工作正常，相机已检测并可启动
- ZED SDK 和 zed_msgs 已预装在系统中，无需额外安装
- 下一步建议：
  - 添加 ZED 相机到 base_link 的 TF 变换（需测量安装位置）
  - 将 ZED Visual Odometry 与底盘里程计融合
  - 创建完整启动文件同时启动底盘、LiDAR 和 ZED X
  - 集成 ZED 深度点云用于避障或 SLAM

---

## Entry 10: URDF/TF 配置规划与 TODO 更新

- **Date**: 2026-01-26
- **Agent**: Claude Code (claude-opus-4-5-20251101)

### Summary
- 分析 Wheeltec 参考实现的 URDF/TF 配置方案
- 分析 ZED X ROS2 wrapper 的 TF 帧结构
- 分析 LiDAR 驱动的 frame_id 配置
- 创建详细的 URDF/TF 配置指南 (`~/.claude/plans/validated-juggling-tulip.md`)
- 更新 TODO.md，细化 P0 核心基础任务为分步实施方案
- 确定测量参考系：base_footprint = 后轴中点地面投影

### 配置方案

**Phase A: 静态 TF 配置（快速验证）**
1. 测量传感器安装位置（LiDAR X,Y,Z,Yaw + ZED X X,Y,Z,Pitch）
2. 添加 LiDAR 静态 TF (`base_link → laser`)
3. 添加 ZED X 静态 TF (`base_link → zedx_camera_link`)
4. RViz TF 验证

**Phase B: URDF 模型创建（完善）**
1. 测量底盘外形尺寸
2. 参考 Wheeltec `top_akm_bs_robot.urdf` 创建 AutoRacer URDF
3. 添加 robot_state_publisher

### TF 树目标结构

```
map
└── odom
    └── base_footprint
        ├── base_link
        │   └── [wheel frames]
        ├── gyro_link (STM32 IMU)
        ├── laser (镭神 C32)
        └── zedx_camera_link (ZED X)
            └── [ZED internal frames]
```

### Modified/Created files
- `TODO.md`: 细化 P0 任务为 Phase A (静态 TF) + Phase B (URDF)，新增已完成模块 ZED X
- `~/.claude/plans/validated-juggling-tulip.md`: URDF/TF 配置完整指南

### Key References
- Wheeltec URDF: `reference/wheeltec_ros2/src/wheeltec_robot_urdf/wheeltec_robot_urdf/urdf/top_akm_bs_robot.urdf`
- Wheeltec TF Launch: `reference/wheeltec_ros2/src/turn_on_wheeltec_robot/launch/robot_mode_description.launch.py`
- ZED X URDF 宏: `src/zed-ros2-wrapper/zed_wrapper/urdf/zed_macro.urdf.xacro`
- LiDAR 配置: `src/autoracer_lidar_ros2/lslidar_ros/lslidar_driver/params/lslidar_cx.yaml` (frame_id: laser)

### Notes / Next
- 用户需要先测量传感器安装位置（LiDAR 和 ZED X 相对于后轴中点的偏移）
- 测量完成后，可以快速添加静态 TF 进行验证
- 验证通过后再创建完整 URDF 模型
- 详细测量指南见 `~/.claude/plans/validated-juggling-tulip.md`

---

## Entry 11: ZED X RViz2 可视化验证

- Date: 2026-01-27
- Agent: Claude Code (claude-opus-4-5-20251101)
- Summary:
  - 从 `reference/zed-ros2-examples/zed_display_rviz2/` 复制到 `src/zed_display_rviz2/`
  - 检查依赖：`grid_map_rviz_plugin` 未安装（运行时可选，不影响编译）
  - 成功编译 `zed_display_rviz2` 包（2.17s）
  - 使用 `display_zed_cam.launch.py camera_model:=zedx` 启动 ZED X + RViz2
  - ZED 相机成功启动（S/N 42256159, HD1200@30, NEURAL LIGHT 深度模式）
  - RViz2 成功启动（OpenGL 4.6），加载 `zed_stereo.rviz` 配置
  - RViz2 插件缺失警告（非致命）：`rviz_plugin_zed_od/ZedOdDisplay`（物体检测可视化）、`nav2_rviz_plugins`（Nav2 面板）
  - ZED SDK 报 CORRUPTED FRAME 警告（GMSL2 连接偶发问题），但用户确认 RViz2 中已成功显示 RGB 图像和深度图
  - 更新 PJINFO.md（项目结构、ROS2 packages、启动方式、已完成状态、重要命令）
  - 更新 CLAUDE.md（Run Commands、Development Status）
  - 更新 TODO.md（已完成列表新增 zed_display_rviz2）
- Modified/Created files:
  - `src/zed_display_rviz2/`: 从 reference 复制的 ZED RViz2 可视化包
  - `PJINFO.md`: 更新项目结构、包列表、启动方式、已完成状态、重要命令
  - `CLAUDE.md`: 添加 RViz2 启动命令、更新开发状态
  - `TODO.md`: 新增已完成模块 zed_display_rviz2
- How to run:
  - `ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx`
  - `ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx start_zed_node:=False`（仅 RViz2）
- Notes / Next:
  - CORRUPTED FRAME 问题需排查 GMSL2 线缆连接
  - 缺失的 RViz 插件（`rviz_plugin_zed_od`、`nav2_rviz_plugins`）可后续按需安装
  - `grid_map_rviz_plugin` 可通过 `sudo apt install ros-humble-grid-map-rviz-plugin` 安装
  - 下一步建议：添加 LiDAR/ZED 静态 TF 变换，完成 P0 核心基础任务

---

## Entry 12: 传感器配置准备指南与 TODO 细化

- Date: 2026-01-27
- Agent: Claude Code (claude-opus-4-5-20251101)
- Summary:
  - 深入研究 reference/ 中全部传感器驱动与配置：N300 Pro IMU（hipnuc_imu）、G90 GNSS+RTK（wheeltec_dual_rtk_driver）、wheeltec 主 bringup（launch/TF/URDF/EKF）、超声波避障
  - 提取了 5 个传感器的完整配置清单：硬件连接、串口配置、udev 规则、ROS2 话题/frame_id、yaml 参数、launch 参数
  - 分析了 wheeltec 的传感器集成架构：base_serial → hipnuc_imu → imu_filter_madgwick → ekf_node，理解了 IMU 重映射策略和 EKF 融合配置
  - 生成了结构化传感器配置准备指南，区分：Agent 可直接完成 / 用户需手动测量 / 用户需确认硬件 / 用户需做出决定
  - 重构 TODO.md：新增"传感器准备检查清单"章节（物理测量、硬件确认、用户决策），将 P0 拆分为 Phase A~D（静态 TF → IMU+EKF → GNSS → URDF），将 IMU/GNSS 从 P3 提升到 P0，细化每个任务的具体子步骤和前置条件
  - 新增 wheeltec 集成架构参考（launch 结构、TF 树目标、EKF 融合策略、udev 设备命名）
  - 更新 PJINFO.md 待办列表
- Modified/Created files:
  - `TODO.md`: 重构，新增传感器准备检查清单（第二章）、P0 Phase A~D 细化、wheeltec 架构参考（第五章）
  - `PJINFO.md`: 更新待办列表，反映 IMU/GNSS 优先级提升和准备检查清单
  - `AGENTLOG.md`: 追加本次日志
- How to run:
  - 无新代码，本次为规划和文档工作
- Notes / Next:
  - 用户需完成传感器准备检查清单（TODO.md 第二章）后，Agent 才能开始传感器集成
  - 优先级：(1) 测量 LiDAR/ZED X 安装位置 → (2) 连接 N300 Pro 确认 USB 设备 → (3) 连接 G90 确认 USB 设备
  - Agent 下一步可执行：添加静态 TF（需用户提供测量数据）→ 移植 hipnuc_imu 驱动（需用户提供 USB ID）

---

## Entry 13: N300 Pro IMU 设备检测

- Date: 2026-01-28
- Agent: Claude Code (claude-opus-4-5-20251101)
- Summary:
  - 使用 `lsusb` 检测到 Silicon Labs CP210x UART Bridge (10c4:ea60)
  - 确认设备路径为 `/dev/ttyUSB0`，内核驱动为 cp210x
  - 使用 `udevadm info` 获取详细信息：idVendor=10c4, idProduct=ea60, serial=0003
  - 参考 `reference/WHEELTEC_N300Pro/` 和 wheeltec udev 脚本，确认 serial=0003 对应 N300 Pro IMU
  - 测试串口数据流正常（二进制 IMU 数据）
  - 驱动配置：115200bps, frame_id=gyro_link, topic=/imu/data_raw
- Modified/Created files:
  - `PJINFO.md`: 更新当前状态（N300 Pro 检测通过）、重要命令（IMU 检测命令）
  - `AGENTLOG.md`: 追加本次日志
  - `CLAUDE.md`: 添加 N300 Pro 设备信息
  - `TODO.md`: 更新任务 1b 状态为已完成
- How to run:
  - `lsusb | grep "10c4:ea60"` - 检测 CP210x 设备
  - `ls -la /dev/ttyUSB*` - 查看串口设备
  - `udevadm info -a /dev/ttyUSB0 | grep -E '(idVendor|idProduct|serial)'` - 获取详细信息
- Notes / Next:
  - N300 Pro IMU 设备确认，可进行下一步 udev 规则创建和驱动移植
  - 建议符号链接：`/dev/autoracer_imu`
  - 下一步：创建 udev 规则 (Task 3a) → 移植 hipnuc_imu 驱动 (Task 3b)

### N300 Pro IMU 设备信息

| 属性 | 值 |
|------|-----|
| 设备名称 | CP2102N USB to UART Bridge Controller |
| 制造商 | Silicon Labs |
| idVendor | 10c4 |
| idProduct | ea60 |
| Serial | 0003 |
| 设备路径 | /dev/ttyUSB0 |
| 内核驱动 | cp210x |
| 波特率 | 115200 |
| ROS2 话题 | /imu/data_raw |
| frame_id | gyro_link |
| 符号链接建议 | /dev/autoracer_imu |

---

## Entry 14: N300 Pro IMU 驱动集成

- Date: 2026-01-28
- Agent: Claude Code (claude-opus-4-5-20251101)
- Summary:
  - 从 `reference/wheeltec_ros2/src/wheeltec_imu/hipnuc_imu/` 复制 hipnuc_imu 驱动到 `src/`
  - 创建 udev 规则 `/etc/udev/rules.d/autoracer_imu.rules`，建立 `/dev/autoracer_imu` 符号链接
  - 修改 `hipnuc_config.yaml`：serial_port → `/dev/autoracer_imu`
  - 更新 `turn_on_autoracer_robot.launch.py`：添加 `use_n300pro_imu` 参数，实现 STM32 IMU topic remapping (`/imu/data_raw` → `/imu/data_board`)
  - 创建 `config/imu.yaml`：Madgwick 滤波器配置（use_mag=false, world_frame=enu）
  - 创建 `config/ekf.yaml`：EKF 融合配置（odom + IMU → odom_combined）
  - 创建 `launch/autoracer_ekf.launch.py`：EKF 单独启动文件
  - 安装 `ros-humble-imu-filter-madgwick` 依赖
  - 编译测试：hipnuc_imu 驱动正常工作，IMU 数据成功发布到 `/imu/data_raw`
- Modified/Created files:
  - `src/hipnuc_imu/`: 从 reference 复制的 N300 Pro IMU 驱动包
  - `src/hipnuc_imu/config/hipnuc_config.yaml`: 修改串口为 `/dev/autoracer_imu`
  - `src/hipnuc_imu/README.md`: 新增说明文档
  - `/etc/udev/rules.d/autoracer_imu.rules`: N300 Pro udev 规则
  - `src/turn_on_autoracer_robot/launch/turn_on_autoracer_robot.launch.py`: 集成 N300 Pro IMU
  - `src/turn_on_autoracer_robot/launch/autoracer_ekf.launch.py`: EKF 启动文件
  - `src/turn_on_autoracer_robot/config/imu.yaml`: Madgwick 滤波器配置
  - `src/turn_on_autoracer_robot/config/ekf.yaml`: EKF 融合配置
  - `src/turn_on_autoracer_robot/package.xml`: 添加 hipnuc_imu, imu_filter_madgwick, robot_localization 依赖
  - `PJINFO.md`, `CLAUDE.md`, `TODO.md`: 更新文档
- How to run:
  - `ros2 launch hipnuc_imu imu_spec_msg.launch.py` - 单独启动 N300 Pro IMU
  - `ros2 topic echo /imu/data_raw` - 查看 IMU 数据
  - `ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py` - 完整启动（含 N300 Pro）
  - `ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py use_n300pro_imu:=false` - 使用 STM32 板载 MPU6050
  - `ros2 launch turn_on_autoracer_robot autoracer_ekf.launch.py` - 单独启动 EKF 融合
- Notes / Next:
  - N300 Pro IMU 驱动已集成，数据发布正常（四元数、角速度、线性加速度）
  - EKF 融合配置已就绪，需与底盘驱动同时运行才能测试融合效果
  - 下一步建议：
    - 测量 LiDAR 和 ZED X 安装位置，添加静态 TF（Phase A）
    - 实车测试 EKF 融合，调整噪声协方差参数
    - 集成 G90 GNSS 驱动（Phase C）

### IMU 集成架构

```
STM32 (MPU6050)                 N300 Pro (HI13)
     │                               │
     └── /imu/data_board ←──────┐    └── /imu/data_raw
         (remapped, unused)     │         (active, ~100Hz)
                                │              │
                                │              ▼
                                │    imu_filter_madgwick
                                │         (滤波)
                                │              │
                                │              ▼
┌─────────────────┐             │         /imu/data
│ 轮式里程计      │             │              │
│ /odom (STM32)   │─────────────┴──────────────┤
└─────────────────┘                            │
                                               ▼
                                          ekf_node
                                     (robot_localization)
                                               │
                                               ▼
                                        /odom_combined
                                      (融合后里程计)
```
