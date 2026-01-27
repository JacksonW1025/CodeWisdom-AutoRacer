# PJINFO.md (Project Information)

> 每次运行 Agent 必读；每次 work 完成后必须更新【自动】部分。  
> 规则：只更新标注为【自动】的段落；【手动】不改；【只增不删】只能追加不删除。

## 项目背景（【手动】）
- 目标/范围：基于ROS2 Humble的Ackermann转向RC小车自主驾驶栈，实现底层硬件通信、传感器数据处理、运动控制、SLAM建图与导航功能
- 关键约束（可选）：嵌入式边缘计算平台（Jetson AGX Orin）资源受限；实时性要求高；Ackermann转向运动学模型
- 术语（可选）：
  - Ackermann转向：汽车式前轮转向模型，内外轮转角不同以避免轮胎滑移
  - BCC校验：Block Check Character，异或(XOR)校验算法
  - TF：Transform，ROS2中的坐标变换框架
  - IMU：惯性测量单元，包含加速度计和陀螺仪
  - Odometry：里程计，通过积分计算机器人位姿

## 硬件环境（【手动】）
- autoracer 平台/设备：NVIDIA Jetson Orin NX Super 定制版（当前设备）
- 硬件架构：主控传感器（激光雷达、深度相机、IMU、GNSS）→ 上位机 Orin NX → C63A (STM32)（板上连接蓝牙、遥控等传感器）→ 底层电机
- 传感器/外设：
  - 主控传感器：
    - **镭神 LiDAR C32**（已连接 2026-01-23）
      - IP: 192.168.1.200
      - 数据端口: 2368/UDP (msop)
      - 设备端口: 2369/UDP (difop)
      - 协议: 镭神私有协议 (LSLIDAR_CX)
      - 驱动参考: `reference/wheeltec_ros2/src/wheeltec_lidar_ros2/lslidar_ros/`
    - **StereoLabs ZED X**（深度相机，GMSL2 接口）
      - 驱动参考: `reference/zed-ros2-wrapper/`
    - **Wheeltec N300 Pro**（9轴 IMU）
    - **Wheeltec G90**（GNSS + RTK 定位模块，双飞碟形状天线）
      - 4G 通信模块：华允物联 EP-D200
      - 驱动参考: `reference/WHEELTEC_G90/`
    - **防水超声波模块**（接口 HY2.0 4PIN，已物理安装，未连接）
  - STM32上的传感器（待最终确定，仅作参考）：IMU(MPU6050, ±500°/s陀螺仪, ±2g加速度计)、8通道超声波传感器、RGB LED指示灯、电源电压检测
- STM32控制器：C63A (STM32)，WCH USB串口芯片（idVendor:0x1a86, idProduct:0x55d4），符号链接建议：/dev/autoracer_controller
- 通信接口：USB串口 115200bps，帧格式（0x7B头/0x7D尾/BCC校验）

## 软件环境（【手动】）
- OS / ROS2 / Python：L4T 36.4.7 (Ubuntu 22.04) / ROS2 Humble / Python 3.10.12
- Jetpack：6.2 (L4T 36.x系列)
- 构建工具：colcon + ament_cmake/ament_python
- 编译器标志：-Wall -Wextra -Wpedantic

## 项目结构（【自动】常更新）
- Repo 关键路径：
  - `src/`：ROS2源包目录，包含所有自有功能包
  - `src/depend/`：外部依赖包（serial_ros2串口通信库）
  - `reference/`：极其重要的参考代码目录（已设置COLCON_IGNORE防止编译）
    - `wheeltec_ros2/`：Wheeltec S200机器人完整ROS2栈（约100个包，含导航/SLAM/传感器驱动等完整实现）
    - `WHEELTEC_C63A/`：**C63A控制器STM32F407固件源码**（差速/滑移转向底盘）
      - 运动模型：S100/S200/S260/S300（差速、滑移转向）
      - RTOS：FreeRTOS，Keil MDK-ARM工程
      - 电机控制：CAN总线伺服驱动（500kbps）
      - IMU：ICM20948（I2C）
      - ROS通信：UART4，115200bps
    - `WHEELTEC_C50X_2025.12.26/`：**C50X控制器STM32F407VE固件源码**（多种底盘类型支持）
      - 运动模型：**Ackermann（阿克曼转向）**、Differential（差速）、Mecanum（麦克纳姆轮）、4WD、Omni
      - RTOS：FreeRTOS，Keil MDK-ARM工程
      - 电机控制：TIM8 PWM驱动
      - IMU：ICM20948（SPI）或MPU6050（I2C）
      - ROS通信：USART3，115200bps
      - **Ackermann参数**：轮距0.164-0.175m，轴距0.26-0.28m，轮径0.125-0.135m
    - `docs/`：**硬件文档资料**（PDF/XLS）
      - `1.镭神智能_C32_用户手册_V2.7.8_20241015.pdf`：镭神 C32 LiDAR 用户手册
      - `C63A用户手册_2025.11.20(1).pdf`：C63A 控制器用户手册
      - `S系列小车底层通信协议内容一览表.xls`：S系列底层通信协议表格
      - `S系列底盘使用与开发手册_2025.10.22.pdf`：S系列底盘使用与开发手册
    - `zed-ros2-wrapper/`：**StereoLabs ZED X 深度相机 ROS2 驱动**（官方 v5.1.0）
      - 包: `zed_ros2`(Meta), `zed_wrapper`(Launch/Config), `zed_components`(C++ 组件)
      - 支持: ZED X (`zedx`), ZED X Mini (`zedxm`) 等全系列相机
      - 配置: `zed_wrapper/config/zedx.yaml`, `common_stereo.yaml`
      - Launch: `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx`
      - 功能: 深度图、点云、Visual Odometry、物体检测、人体追踪、GNSS 融合
      - 依赖: ZED SDK v5.1+, CUDA
      - 接口: 图像 `~/left/color/rect/image`, 深度 `~/depth/depth_map`, 点云 `~/depth/point_cloud`, IMU `~/imu/data`
    - `zed-ros2-examples/`：**StereoLabs ZED ROS2 示例与 RViz2 可视化**（官方）
      - `zed_display_rviz2/`：RViz2 预配置 Display 包（Launch + .rviz 配置文件）
      - `rviz-plugin-zed-od/`：ZED 物体检测/人体追踪 RViz 插件
      - `tutorials/`：ZED ROS2 教程
      - Launch: `ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx`
    - `WHEELTEC_G90/`：**Wheeltec G90 GNSS+RTK 定位模块文档与 ROS2 驱动**
      - 文档: `WHEELTEC_G90用户手册20251231.pdf`, `4G-DTU-D200说明书.pdf`, `NAME协议数据包解析_.pdf`
      - ROS2 驱动: `wheeltec-gps-ros2-20250929/src/`
        - `wheeltec_dual_rtk_driver`(Python): 双天线 RTK 驱动，支持 UM982 GNSS 模块
        - `wheeltec_gps_driver`(C++): GPS 路径可视化，launch 配置
        - `nmea_navsat_driver`: NMEA 协议解析
        - `nmea_msgs`: NMEA 消息定义
      - 协议: NMEA, Unicore (UM982)
      - Launch (G90): `wheeltec_dual_rtk_driver_nmea.launch.py` (NMEA), `wheeltec_dual_rtk_driver_unicore.launch.py` (Unicore)
      - 接口: `/gps/fix`(NavSatFix), `/gps/utm_pose`(Odometry), `/gps/euler`(Vector3Stamped heading/pitch/roll)
      - 串口: `/dev/wheeltec_gnss`, 115200bps
      - 4G 通信: 华允物联 EP-D200 DTU
    - `WHEELTEC_N300Pro/`：**Wheeltec N300 Pro 惯导 IMU 文档与 ROS2 驱动**
      - 文档: `1.N300Pro惯导用户手册.pdf`, `2.HI13_芯片产品手册.pdf`
      - 芯片: HI13（超核电子 9轴 IMU 芯片）
      - ROS2 驱动: `N300Pro_ros2_sdk/`
        - `hipnuc_imu`(C++): 超核电子 IMU 驱动，发布 sensor_msgs/Imu
        - `imu_tf_broadcaster`(Python): IMU TF 广播
      - Launch: `ros2 launch hipnuc_imu imu_spec_msg.launch.py`
      - 配置: `hipnuc_imu/config/hipnuc_config.yaml`
      - 接口: `/imu/data_raw`(sensor_msgs/Imu)，~100Hz
      - 串口: `/dev/wheeltec_IMU`, 115200bps, frame_id: `gyro_link`
  - `build/`：CMake编译工件（gitignored）
  - `install/`：ROS2安装空间（gitignored）
  - `log/`：ROS2日志目录（gitignored）
  - `source_all.sh`：ROS2环境自动检测与source脚本
  - `CLAUDE.md`：项目技术说明文档
  - `PJINFO.md`：非常完整的项目信息介绍，供agent使用
  - `AGENTLOG.md`：Agent工作的历史日志，供agent查阅
  - `PROMPT.md`：用于和Agent交互的文件，内含具体细致的工作规划
  - `TODO.md`：功能对比与待办清单（AutoRacer vs Wheeltec，按 P0-P5 优先级分类，共 34 项待办）
- ROS2 packages（如适用）：
  - `turn_on_autoracer_robot`：主底盘驱动包(C++) | nodes: `autoracer_robot` | launch: `autoracer_serial.launch.py`, `turn_on_autoracer_robot.launch.py` | 接口: sub `/cmd_vel`(Twist), pub `/odom`(Odometry), `/imu/data_raw`(Imu), `/PowerVoltage`(Float32)
  - `autoracer_interfaces`：自定义消息服务包 | nodes: 无 | launch: 无 | 接口: msg `Supersonic`(8通道超声波), srv `SetRgb`(RGB LED控制)
  - `autoracer_keyboard`：键盘遥控包(Python) | nodes: `keyboard_control` | launch: 无 | 接口: pub `/cmd_vel`(Twist)
  - `serial`(depend/serial_ros2)：跨平台串口通信库 | nodes: 无（库） | launch: 无 | 接口: C++ API
  - `lslidar_driver`(autoracer_lidar_ros2/lslidar_ros)：**镭神 C32 LiDAR 驱动** | nodes: `lslidar_driver_node` | launch: `lslidar_cx_launch.py`, `lslidar_cx_rviz_launch.py` | 接口: pub `/point_cloud_raw`(PointCloud2), `/scan_raw`(LaserScan)
  - `lslidar_msgs`(autoracer_lidar_ros2/lslidar_ros)：镭神 LiDAR 消息定义 | nodes: 无 | launch: 无 | 接口: 自定义消息类型
  - `zed_components`(zed-ros2-wrapper)：**ZED X 深度相机 C++ 组件** | nodes: `zed_node` (composable) | launch: 无（由 zed_wrapper 提供） | 接口: 双目相机、深度图、点云、IMU、Visual Odometry
  - `zed_wrapper`(zed-ros2-wrapper)：**ZED X 深度相机 Launch/Config 包** | nodes: 无 | launch: `zed_camera.launch.py` | 配置: `zedx.yaml`, `common_stereo.yaml`
  - `zed_ros2`(zed-ros2-wrapper)：ZED ROS2 Meta 包（依赖聚合） | nodes: 无 | launch: 无 | 接口: 无
  - `zed_display_rviz2`：**ZED X RViz2 可视化包** | nodes: 无（启动 rviz2） | launch: `display_zed_cam.launch.py` | 配置: `rviz2/zed_stereo.rviz`, `rviz2/zed_mono.rviz`
- 现聚焦的关键入口与运行链路：
  - 启动方式：
    - 完整启动（含TF）：`ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py`
    - 仅串口节点：`ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py`
    - 键盘控制（另开终端）：`ros2 run autoracer_keyboard keyboard_control`
    - **LiDAR 驱动**：`ros2 launch lslidar_driver lslidar_cx_launch.py`
    - **LiDAR + RViz**：`ros2 launch lslidar_driver lslidar_cx_rviz_launch.py`
    - **ZED X 深度相机**：`ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx`
    - **ZED X + RViz2 可视化**：`ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx`
    - **仅 RViz2（不启动相机）**：`ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx start_zed_node:=False`
  - 配置入口：
    - `src/turn_on_autoracer_robot/config/autoracer_params.yaml`（串口、坐标系、里程计校准参数）
    - `src/autoracer_lidar_ros2/lslidar_ros/lslidar_driver/params/lslidar_cx.yaml`（LiDAR IP、端口、点云参数）
    - `src/zed-ros2-wrapper/zed_wrapper/config/zedx.yaml`（ZED X 分辨率、帧率、曝光参数）
    - `src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml`（深度、位置追踪、物体检测参数）

## 重要代码参考（【手动】）
- `reference/wheeltec_ros2/`：Wheeltec S200机器人完整ROS2栈（约100个包），包含：
  - 导航与规划：`wheeltec_nav2/`(Nav2配置)、`nav2_bringup/`、`wheeltec_robot_rrt/`(RRT路径规划)
  - 传感器驱动：`rplidar_ros/`(RPLiDAR)、`ldlidar_stl_ros2/`(LD激光雷达)、**`lslidar_ros/`(镭神LiDAR C16/C32)**、`usb_cam/`(摄像头)、`hipnuc_imu/`(IMU)
  - SLAM与视觉：`orb_slam2_ros/`(ORB-SLAM2)、`wheeltec_robot_kcf/`(KCF跟踪)
  - 机器人模型：`wheeltec_robot_urdf/`(URDF模型)
  - 其他功能：`web_video_server/`(Web视频流)、`pointcloud_to_laserscan/`(点云转激光)
  - 这是项目的重要代码参考，在智驾和机器人算法方面的设计与实现应尽可能地参考里面的pkgs

- `reference/WHEELTEC_C63A/`：C63A控制器STM32固件（**差速/滑移转向底盘**），关键文件：
  - `WHEELTEC_APP/SerialControl_task.c`：ROS串口命令解析（0x7B/0x7D帧格式）
  - `WHEELTEC_APP/RobotControl_task.c`：电机控制、运动学计算、避障
  - `WHEELTEC_APP/data_task.c`：传感器数据上报ROS（20Hz）
  - `WHEELTEC_APP/imu_task.c`：IMU采样与校准
  - `WHEELTEC_APP/robot_select_init.c`：多机器人类型配置
  - `WHEELTEC_BSP/bsp_ServoDrive.c`：CAN伺服电机驱动
  - 串口协议：RX 11字节、TX 24字节，与autoracer现有实现兼容

- `reference/WHEELTEC_C50X_2025.12.26/`：C50X控制器STM32固件（**Ackermann阿克曼转向底盘**，与autoracer底盘最相关），关键文件：
  - `BALANCE/uartx_callback.c`：**ROS RX处理** - 解析cmd_vel
  - `BALANCE/data_task.c`：**ROS TX** - 发送里程计/IMU/电压
  - `BALANCE/balance_task.c`：电机控制循环、运动学
  - `BALANCE/control.c`：各车型正/逆运动学
  - `CarType/akm_robot_init.c`：**Ackermann参数**（轮距、轴距、转向角）
  - `HARDWARE/motor.c`：PWM电机驱动（TIM8）
  - `HARDWARE/encoder.c`：编码器中断
  - **重要**：Ackermann运动学参数在此，后续标定需参考

- `reference/zed-ros2-wrapper/`：**StereoLabs ZED X 深度相机 ROS2 驱动**（官方 v5.1.0），关键文件：
  - `zed_wrapper/launch/zed_camera.launch.py`：主 Launch 文件
  - `zed_wrapper/config/zedx.yaml`：ZED X 相机配置（分辨率、帧率、曝光）
  - `zed_wrapper/config/common_stereo.yaml`：立体相机通用配置（深度、位置追踪、物体检测）
  - `zed_components/src/zed_camera/`：ZED 相机 C++ 组件（~14,600 行）
  - 功能：RGB/深度图像、点云、Visual Odometry、物体检测、人体追踪、GNSS 融合
  - 依赖：ZED SDK v5.1+、CUDA、nvidia-jetpack（Jetson）
  - **重要**：深度相机与 SLAM/导航集成的核心参考

- `reference/zed-ros2-examples/`：**StereoLabs ZED ROS2 示例与 RViz2 可视化**（官方），关键文件：
  - `zed_display_rviz2/launch/display_zed_cam.launch.py`：ZED + RViz2 一体化 Launch
  - `zed_display_rviz2/rviz2/`：各型号相机的 `.rviz` 预配置文件
  - `rviz-plugin-zed-od/`：物体检测/人体追踪 RViz 可视化插件
  - **重要**：RViz2 可视化与调试的核心参考，`zed_display_rviz2` 可直接移植到 `src/`

- `reference/WHEELTEC_G90/`：**Wheeltec G90 GNSS+RTK 定位模块文档与 ROS2 驱动**，关键文件：
  - `wheeltec-gps-ros2-20250929/src/wheeltec_dual_rtk_driver/`：双天线 RTK Python 驱动
    - `wheeltec_dual_rtk_driver.py`：主驱动节点（UM982 GNSS 解析、NavSatFix/Odometry 发布）
    - `um982_serial.py`：UM982 串口通信与协议解析
  - `wheeltec-gps-ros2-20250929/src/wheeltec_gps_driver/`：GPS 路径可视化与 launch 配置
  - `wheeltec-gps-ros2-20250929/src/nmea_navsat_driver/`：标准 NMEA 协议解析
  - `WHEELTEC_G90用户手册20251231.pdf`：G90 模块硬件手册
  - `4G-DTU-D200说明书.pdf`：华允物联 4G DTU 说明
  - 协议：NMEA、Unicore (UM982)
  - **重要**：户外定位与 RTK 高精度导航的核心参考

- `reference/WHEELTEC_N300Pro/`：**Wheeltec N300 Pro 惯导 IMU 文档与 ROS2 驱动**，关键文件：
  - `N300Pro_ros2_sdk/hipnuc_imu/`：超核电子 IMU C++ 驱动
    - `src/hipnuc.h`：IMU 数据解析协议
    - `src/serial_port.cpp`：串口通信
    - `config/hipnuc_config.yaml`：IMU 配置（串口、波特率、话题）
    - `launch/imu_spec_msg.launch.py`：IMU Launch 文件
  - `N300Pro_ros2_sdk/imu_tf_broadcaster/`：IMU TF 广播 Python 包
  - `1.N300Pro惯导用户手册.pdf`：N300 Pro 用户手册
  - `2.HI13_芯片产品手册.pdf`：HI13 IMU 芯片手册
  - 接口：`/imu/data_raw`(sensor_msgs/Imu)，~100Hz
  - **重要**：高精度 IMU 数据融合与姿态估计的核心参考

## 上一次操作（【手动】参考 AGENTLOG.md）
- 日志索引：参考 AGENTLOG.md 获取最新操作记录

## 当前状态（【自动】每次 work 后更新）
- 已完成：
  - 工作区框架和包结构（4个自有包 + 1个依赖包）
  - STM32串口通信驱动（0x7B头/0x7D尾/BCC校验，115200bps）
  - IMU数据发布（`/imu/data_raw`，含四元数解算Madgwick算法）
  - 里程计数据计算和发布（`/odom`，含TF变换）
  - 电池电压监测（`/PowerVoltage`）
  - 速度控制订阅（`/cmd_vel`）
  - 自定义消息定义（Supersonic.msg 8通道超声波）
  - 自定义服务定义（SetRgb.srv RGB LED控制）
  - TF坐标树（odom -> base_footprint -> base_link/gyro_link）
  - 键盘遥控功能（8方向控制、平滑加减速、实时速度调节）
  - Launch文件（autoracer_serial.launch.py、turn_on_autoracer_robot.launch.py）
  - 验证方式：`ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py` + `ros2 run autoracer_keyboard keyboard_control`
  - **硬件通信验证通过**：AGX Orin ↔ STM32 串口通信正常，STM32 显示屏显示 cmd_vel 数值变化（a/b/c/d 值）
  - **Ackermann运动学参数已配置**（2026-01-23）：wheelbase=0.60m, track_width=0.48m, wheel_radius=0.11m, max_steering_angle=0.393rad(22.5°), min_turning_radius≈1.45m
  - **镭神 LiDAR C32 网络连接验证通过**（2026-01-23）：IP 192.168.1.200，ping 延迟 ~0.62ms，参考驱动 lslidar_ros 已确认
  - **镭神 LiDAR C32 驱动集成完成**（2026-01-25）：lslidar_ros v4.2.4 移植到 `src/autoracer_lidar_ros2/`，点云发布 `/point_cloud_raw` (~20Hz)，激光扫描 `/scan_raw`，验证方式：`ros2 launch lslidar_driver lslidar_cx_launch.py`
  - **ZED X 深度相机驱动集成完成**（2026-01-26）：zed-ros2-wrapper v5.1.0 移植到 `src/zed-ros2-wrapper/`，检测到相机 S/N:42256159，发布话题 `/zed/left/color/rect/image`(RGB)、`/zed/depth/depth_map`(深度)、`/zed/depth/point_cloud`(点云)、`/zed/imu/data`(IMU)、`/zed/odom`(Visual Odometry)，验证方式：`ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx`
  - **ZED X RViz2 可视化验证通过**（2026-01-27）：从 `reference/zed-ros2-examples/` 移植 `zed_display_rviz2` 到 `src/`，RViz2 中成功显示 RGB 图像和深度图，验证方式：`ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx`

- 待办（仅作记录，不代表现在要实现，详见 `TODO.md`）：
  - 【P0 核心基础】ZEDX pkgs, URDF 机器人模型、LiDAR TF 变换、Ackermann 消息、RViz 配置
  - 【P1 导航基础】pointcloud_to_laserscan、Nav2 配置、航点导航、路径跟随
  - 【P2 SLAM】Cartographer、SLAM Toolbox、GMapping、RTAB-Map、ORB-SLAM2、LeGO-LOAM、LIO-SAM
  - 【P3 传感器】USB 摄像头、超声波避障、手柄控制、IMU 增强、GPS、深度相机
  - 【P4 视觉/AI】目标跟随、KCF 跟踪、YOLO 检测、ArUco 标记、人体姿态、LLM 集成
  - 【P5 工具】Web 视频流、TTS 语音、RRT 规划、Qt GUI、多机器人、自动充电
  - 总计：已完成 5/35+ 模块，待完成 34 项（详见 TODO.md）
- 已知问题（仅作记录，不代表现在要解决）：
  - IMU校准参数未设置（需实际标定）

## 重要命令（【只增不删】）
- source source_all.sh用于source ros2和autoracer workspace
- LiDAR 连接测试：`ping 192.168.1.200`（镭神 C32，数据端口 2368/UDP）
- build：
  - 完整构建：`colcon build --symlink-install`
  - 单包构建：`colcon build --packages-select turn_on_autoracer_robot --symlink-install`
  - 资源受限构建（Jetson）：`colcon build --parallel-workers 2 --symlink-install`
- run：
  - 完整启动（含TF）：`ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py`
  - 仅串口节点：`ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py`
  - 自定义串口：`ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py usart_port_name:=/dev/ttyUSB0`
  - 键盘控制：`ros2 run autoracer_keyboard keyboard_control`
  - 直接运行节点：`ros2 run turn_on_autoracer_robot autoracer_robot`
  - **LiDAR 驱动**：`ros2 launch lslidar_driver lslidar_cx_launch.py`
  - **LiDAR + RViz**：`ros2 launch lslidar_driver lslidar_cx_rviz_launch.py`
  - 点云话题测试：`ros2 topic hz /point_cloud_raw`
  - **ZED X 深度相机**：`ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx`
  - ZED 相机列表：`/usr/local/zed/tools/ZED_Explorer -a`
  - ZED 话题测试：`ros2 topic list | grep zed`
  - **ZED X + RViz2 可视化**：`ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx`
  - **仅 RViz2（相机已运行时）**：`ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx start_zed_node:=False`

## 主要命令（【参考】CLAUDE.md）
- 关键入口：
  - 主启动入口：`ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py`（启动底盘驱动+TF变换）
  - 遥控入口：`ros2 run autoracer_keyboard keyboard_control`（键盘遥控，另开终端）
  - 调试命令：
    - 查看话题：`ros2 topic list`
    - 监听里程计：`ros2 topic echo /odom`
    - 监听IMU：`ros2 topic echo /imu/data_raw`
    - 监听电压：`ros2 topic echo /PowerVoltage`
    - 发送速度命令：`ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"`
    - 查看TF树：`ros2 run tf2_tools view_frames`
    - 查看参数：`ros2 param list /autoracer_robot`

## 设计原则（【手动】）
### 命名强制原则
如果你的代码参考了reference下的功能包或node（而且你很可能常常需要这么做），确保你的功能包和node命名风格和你参考的那个wheeltech功能包是一致的。
其中将wheeltec或wheeltec字样改为autoracer，其它部分需要保持一致。
例如，你参考了reference/web_video_server-ros2，你就需要原封不动地将web_video_server-ros2及其节点名沿用到autoracer工作空间；
例如，你参考了reference/wheeltec_lidar_ros2，你就需要将autoracer工作空间中的包命名为autoracer_lidar_ros2，节点也同理
### 核心信念
-   **增量进步优于一步到位** - 优先选择能够编译并顺利通过测试的小步迭代。
-   **从现有代码中学习** - 在动手实现新功能前，充分研究和规划。
-   **务实优于教条** - 灵活适应项目的实际情况，而非僵守原则。
-   **清晰意图优于巧妙代码** - 追求代码的直白与易懂，避免炫技。
### 简单意味着
-   每个函数或类只承担单一职责。
-   避免进行不成熟的抽象设计。
-   拒绝使用花哨的技巧，选择最稳妥、最直接的解决方案。
-   如果一段代码需要额外的解释才能被理解，那么它本身就过于复杂了。

## 其它注意事项（【手动】）
- READ reference/ first!
- 我给予你sudo权限，密码是car，但是请你不要滥用sudo的操作，并向我确认。
- 和我交互时使用简体中文，专业名称和表达使用英文。
- 如果你遇到困难，最多尝试 3 次，则向我寻求建议。
- 我会谨慎地检查所有你编写的代码，因此你需要在后续新增的pkg内增加简单的README.md来解释，并且编写一些简洁的中文注释。
