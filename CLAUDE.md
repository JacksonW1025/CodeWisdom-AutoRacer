# CodeWisdom-AutoRacer

ROS2 Humble workspace for an Ackermann-steering RC car autonomous driving stack on NVIDIA Jetson AGX Orin.

## Project Structure

```
CodeWisdom-AutoRacer/
├── src/                              # ROS2 source packages
│   ├── turn_on_autoracer_robot/      # Main bringup package (chassis driver)
│   │   ├── src/
│   │   │   ├── autoracer_robot.cpp        # Serial communication node
│   │   │   └── Quaternion_Solution.cpp    # IMU quaternion solver
│   │   ├── include/turn_on_autoracer_robot/
│   │   │   ├── autoracer_robot.hpp        # Main header with frame definitions
│   │   │   └── Quaternion_Solution.hpp    # Quaternion solver header
│   │   ├── launch/
│   │   │   ├── turn_on_autoracer_robot.launch.py  # Full launch with TF
│   │   │   └── autoracer_serial.launch.py         # Serial node only
│   │   └── config/autoracer_params.yaml
│   ├── autoracer_interfaces/         # Custom messages and services
│   │   ├── msg/Supersonic.msg        # 8-channel ultrasonic sensor data
│   │   └── srv/SetRgb.srv            # RGB LED control service
│   ├── autoracer_keyboard/           # Keyboard teleoperation (Python)
│   │   └── autoracer_keyboard/keyboard_control.py
│   ├── autoracer_lidar_ros2/         # LiDAR driver packages
│   │   └── lslidar_ros/              # Leishen C32 LiDAR driver (v4.2.4)
│   │       ├── lslidar_driver/       # Main driver package
│   │       └── lslidar_msgs/         # Custom messages
│   ├── zed-ros2-wrapper/             # ZED X depth camera driver (v5.1.0)
│   │   ├── zed_components/           # C++ ROS2 components
│   │   ├── zed_wrapper/              # Launch files and configs
│   │   └── zed_ros2/                 # Meta package
│   ├── hipnuc_imu/                   # N300 Pro IMU driver (HI13 chip)
│   │   ├── src/                      # Serial port + protocol decoder
│   │   ├── config/hipnuc_config.yaml # IMU configuration
│   │   └── launch/                   # Launch files
│   ├── autoracer_robot_urdf/         # URDF robot model (Ackermann)
│   │   ├── urdf/autoracer.urdf.xacro # Xacro URDF model
│   │   ├── launch/robot_description.launch.py  # robot_state_publisher
│   │   └── rviz/autoracer.rviz       # RViz2 config
│   └── depend/                       # Dependencies
│       └── serial_ros2/              # Serial communication library
├── build/                            # CMake build artifacts (gitignored)
├── install/                          # ROS2 install space (gitignored)
├── log/                              # ROS2 logs (gitignored)
├── TODO.md                           # Feature roadmap (AutoRacer vs Wheeltec, 34 tasks)
└── reference/                        # Reference code (gitignored)
    └── wheeltec_ros2/                # Wheeltec S200 robot reference (~100 packages)
```

## Build System

- **Build tool**: colcon with ament_cmake
- **CMake minimum**: 3.8
- **Compiler flags**: `-Wall -Wextra -Wpedantic`

### Build Commands

```bash
# Full workspace build
colcon build --symlink-install

# Single package build
colcon build --packages-select turn_on_autoracer_robot --symlink-install

# Resource-constrained build (Jetson)
colcon build --parallel-workers 2 --symlink-install

# Source workspace
source install/setup.bash
```

## Package Dependencies

**turn_on_autoracer_robot**:
- rclcpp, std_msgs, geometry_msgs, sensor_msgs, nav_msgs
- tf2, tf2_ros, tf2_geometry_msgs
- autoracer_interfaces
- serial (serial_ros2)

**autoracer_interfaces**:
- rosidl_default_generators, std_msgs

**autoracer_keyboard**:
- rclpy, geometry_msgs

**serial_ros2**:
- ament_cmake

**lslidar_driver**:
- rclcpp, sensor_msgs, pcl_conversions, libpcap, lslidar_msgs

**lslidar_msgs**:
- rosidl_default_generators, std_msgs, sensor_msgs

**zed_components**:
- rclcpp, rclcpp_components, image_transport
- std_msgs, geometry_msgs, nav_msgs, sensor_msgs, stereo_msgs
- tf2, tf2_ros, tf2_geometry_msgs
- zed_msgs, nmea_msgs, geographic_msgs
- ZED SDK v5.1+, CUDA

**zed_wrapper**:
- zed_components, robot_state_publisher

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity command (sub) |
| `/odom` | nav_msgs/Odometry | Odometry (pub) |
| `/imu/data_raw` | sensor_msgs/Imu | IMU data (pub) |
| `/PowerVoltage` | std_msgs/Float32 | Battery voltage (pub) |
| `/point_cloud_raw` | sensor_msgs/PointCloud2 | LiDAR point cloud (pub, ~20Hz) |
| `/scan_raw` | sensor_msgs/LaserScan | LiDAR laser scan (pub) |
| `/zed/zed_node/left/image_rect_color` | sensor_msgs/Image | ZED X left RGB image (pub) |
| `/zed/zed_node/depth/depth_registered` | sensor_msgs/Image | ZED X depth map (pub) |
| `/zed/zed_node/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | ZED X depth point cloud (pub) |
| `/zed/zed_node/imu/data` | sensor_msgs/Imu | ZED X IMU data (pub) |
| `/zed/zed_node/odom` | nav_msgs/Odometry | ZED X Visual Odometry (pub) |

## Key Parameters

```yaml
usart_port_name: "/dev/ttyACM0"    # STM32 serial port
serial_baud_rate: 115200
robot_frame_id: "base_footprint"
odom_frame_id: "odom"
gyro_frame_id: "gyro_link"

# Ackermann kinematics (measured 2026-01-23)
wheelbase: 0.54           # Front-to-rear axle distance (m)
track_width: 0.48         # Left-to-right wheel distance (m)
wheel_radius: 0.11        # Wheel radius (m)
max_steering_angle: 0.393 # Max steering angle (rad, ≈22.5°)
# min_turning_radius: ~1.45m (calculated)

# Vehicle dimensions (measured 2026-01-29)
vehicle_length: 0.85      # Total length (m)
vehicle_width: 0.50       # Total width (m)
vehicle_height: 0.40      # Total height (m)
front_axle_from_front: 0.15  # Front axle to front (m)
rear_axle_from_rear: 0.16    # Rear axle to rear (m)

# Sensor positions (relative to base_link, measured 2026-01-29)
# base_link: rear axle center, at axle height (0.11m above ground)
# LiDAR C32: X=+0.24m (forward), Y=0 (centerline), Z=+0.39m, yaw=-90°
# LiDAR coordinate (coordinate_opt=false): +Y=front(0°), +X=right(90°), +Z=up
# Cable exit at rear, LiDAR config cable position=180°, so 0°=front
# ZED X:     X=+0.34m (forward), Y=0 (centerline), Z=+0.29m, yaw=0°
```

## Run Commands

```bash
# Run serial node directly
ros2 run turn_on_autoracer_robot autoracer_robot

# Launch serial node only
ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py

# Launch with TF transforms + URDF model
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py

# Launch without URDF (fallback to static TF only)
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py use_urdf:=false

# Custom serial port
ros2 launch turn_on_autoracer_robot autoracer_serial.launch.py \
  usart_port_name:=/dev/ttyUSB0

# Keyboard teleoperation (in separate terminal)
ros2 run autoracer_keyboard keyboard_control

# LiDAR driver
ros2 launch lslidar_driver lslidar_cx_launch.py

# LiDAR with RViz visualization
ros2 launch lslidar_driver lslidar_cx_rviz_launch.py

# Check point cloud rate
ros2 topic hz /point_cloud_raw

# ZED X depth camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx

# ZED X with custom serial number
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx serial_number:=42256159

# List connected ZED cameras
/usr/local/zed/tools/ZED_Explorer -a

# Check ZED topics
ros2 topic list | grep zed

# ZED X + RViz2 visualization (starts both camera and RViz2)
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx

# RViz2 only (if ZED camera node already running)
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx start_zed_node:=False

# URDF model only (robot_state_publisher + joint_state_publisher)
ros2 launch autoracer_robot_urdf robot_description.launch.py

# URDF with joint GUI (debugging)
ros2 launch autoracer_robot_urdf robot_description.launch.py use_joint_state_publisher_gui:=true

# RViz2 with AutoRacer model
rviz2 -d $(ros2 pkg prefix autoracer_robot_urdf)/share/autoracer_robot_urdf/rviz/autoracer.rviz
```

## Keyboard Control

| Key | Function |
|-----|----------|
| i | Forward |
| , | Backward |
| j | Turn left |
| l | Turn right |
| u/o | Forward + turn |
| m/. | Backward + turn |
| k/Space | Emergency stop |
| q/z | Increase/decrease all speeds (10%) |
| w/x | Increase/decrease linear speed (10%) |
| e/c | Increase/decrease angular speed (10%) |

## Serial Protocol

Communication with STM32 controller uses the following frame format:

| Parameter | Value | Description |
|-----------|-------|-------------|
| FRAME_HEADER | 0x7B | Frame header |
| FRAME_TAIL | 0x7D | Frame footer |
| SEND_DATA_SIZE | 11 | ROS→STM32 frame length |
| RECEIVE_DATA_SIZE | 24 | STM32→ROS frame length |
| Default Port | /dev/ttyACM0 | Configurable |
| Baud Rate | 115200 | Configurable |
| Checksum | BCC | XOR checksum |

## Hardware Info

| Property | Value |
|----------|-------|
| Platform | NVIDIA Jetson AGX Orin |
| OS | Ubuntu 22.04 (aarch64) / L4T 36.4.7 |
| Controller | STM32 (WCH USB Serial) |
| idVendor | 1a86 |
| idProduct | 55d4 |
| Symlink | /dev/autoracer_controller |

## LiDAR (Leishen C32)

| Property | Value |
|----------|-------|
| Model | Leishen C32 (镭神 C32) |
| IP Address | 192.168.1.200 |
| Host IP | 192.168.1.102 (eno1) |
| Data Port | 2368/UDP (msop) |
| Device Port | 2369/UDP (difop) |
| Protocol | LSLIDAR_CX |
| Reference Driver | `reference/wheeltec_ros2/src/wheeltec_lidar_ros2/lslidar_ros/` |

### LiDAR Driver Dependencies
```bash
# Install via rosdep (recommended)
rosdep install --from-paths src --ignore-src -r -y

# Or manually
sudo apt-get install ros-humble-pcl-ros ros-humble-pluginlib ros-humble-pcl-conversions
sudo apt-get install --allow-downgrades libpcap0.8=1.10.1-4build1 libpcap0.8-dev=1.10.1-4build1
```

## ZED X Depth Camera

| Property | Value |
|----------|-------|
| Model | StereoLabs ZED X |
| Serial Number | 42256159 |
| Interface | GMSL2 |
| Device Path | /dev/i2c-9 |
| Resolution | HD1200 (1920x1200) |
| Frame Rate | 30 fps |
| Driver Version | zed-ros2-wrapper v5.1.0 |
| ZED SDK | v5.1+ |

### ZED X Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/zed/zed_node/left/image_rect_color` | Image | Left RGB image |
| `/zed/zed_node/depth/depth_registered` | Image | Depth map |
| `/zed/zed_node/point_cloud/cloud_registered` | PointCloud2 | Depth point cloud |
| `/zed/zed_node/imu/data` | Imu | IMU data |
| `/zed/zed_node/odom` | Odometry | Visual odometry |
| `/zed/zed_node/pose` | PoseStamped | Camera pose |

## N300 Pro IMU

| Property | Value |
|----------|-------|
| Model | Wheeltec N300 Pro (HI13 chip) |
| USB Chip | Silicon Labs CP2102N |
| idVendor | 10c4 |
| idProduct | ea60 |
| Serial | 0003 |
| Device Path | /dev/ttyUSB0 |
| Kernel Driver | cp210x |
| Baud Rate | 115200 |
| Symlink | /dev/autoracer_imu |
| Driver | `src/hipnuc_imu/` (integrated 2026-01-28) |

### N300 Pro Commands
```bash
# Launch N300 Pro IMU driver
ros2 launch hipnuc_imu imu_spec_msg.launch.py

# Check IMU data
ros2 topic echo /imu/data_raw

# Check IMU data rate
ros2 topic hz /imu/data_raw

# Launch full robot with N300 Pro (default)
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py

# Launch without N300 Pro (use STM32 onboard MPU6050)
ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py use_n300pro_imu:=false

# Launch EKF fusion separately
ros2 launch turn_on_autoracer_robot autoracer_ekf.launch.py
```

## Development Status

**Completed**:
- Workspace skeleton and package structure
- Custom message/service definitions (Supersonic.msg, SetRgb.srv)
- STM32 serial communication (0x7B header, 0x7D footer, BCC checksum)
- Odometry publishing (/odom)
- IMU data publishing with quaternion solving (/imu/data_raw)
- Battery voltage publishing (/PowerVoltage)
- Velocity command subscription (/cmd_vel)
- Launch files with static TF publishers
- Keyboard teleoperation package (autoracer_keyboard)
- udev rules configuration
- **Hardware verification**: AGX Orin ↔ STM32 serial communication tested OK (2026-01-21)
- **Ackermann parameters configured** (2026-01-23): wheelbase=0.60m, track_width=0.48m, wheel_radius=0.11m, max_steering_angle=22.5°
- **Leishen C32 LiDAR connection verified** (2026-01-23): IP 192.168.1.200, ping OK (~0.62ms)
- **Leishen C32 LiDAR driver integrated** (2026-01-25): lslidar_ros v4.2.4, C32 version 3.0 detected, point cloud ~20Hz
- **ZED X depth camera driver integrated** (2026-01-26): zed-ros2-wrapper v5.1.0, S/N 42256159 detected, RGB/depth/point cloud/IMU/Visual Odometry
- **ZED X RViz2 visualization verified** (2026-01-27): zed_display_rviz2 from zed-ros2-examples, RGB image and depth map displayed in RViz2
- **N300 Pro IMU device detected** (2026-01-28): CP2102N (10c4:ea60, serial=0003) at /dev/ttyUSB0, data stream verified
- **N300 Pro IMU driver integrated** (2026-01-28): hipnuc_imu driver, udev rule (/dev/autoracer_imu), Madgwick filter, EKF config, IMU data publishing verified (/imu/data_raw)
- **LiDAR C32 & ZED X static TF configured** (2026-01-29): base_link→laser (X=+0.24m, Z=+0.39m, yaw=-90°), base_link→zed_camera_link (X=+0.34m, Z=+0.29m)
- **URDF model created** (2026-01-29): autoracer_robot_urdf package, Xacro URDF with Ackermann steering (4 wheels + 2 sensors), robot_state_publisher + joint_state_publisher integrated into main bringup launch
- **RViz config created** (2026-01-29): autoracer.rviz with Grid + RobotModel + TF + LaserScan(/scan_raw) + Odometry(/odom), Fixed Frame=base_link, Orbit view

**TODO** (see TODO.md for full details):
- **P0 Phase A**: ~~Static TF for LiDAR/ZED X~~ ✅ Completed (2026-01-29)
- **P0 Phase B**: ~~N300 Pro IMU integration~~ ✅ Completed (hipnuc_imu + Madgwick filter + EKF fusion)
- **P0 Phase C**: G90 GNSS+RTK integration (Unicore/NMEA driver)
- **P0 Phase D**: ~~URDF model~~ ✅ Completed (2026-01-29), Ackermann messages (pending), ~~RViz config~~ ✅ Completed (2026-01-29)
- **P1**: pointcloud_to_laserscan, Nav2, waypoint navigation
- **P2**: SLAM (Cartographer, SLAM Toolbox, etc.)
- **Sensor preparation checklist**: See TODO.md Section 2

## Sensor Integration Reference (from Wheeltec)

**udev device naming**:
| Device | Symlink | Status |
|--------|---------|--------|
| STM32 | `/dev/autoracer_controller` | Configured |
| N300 Pro IMU | `/dev/autoracer_imu` | Detected (10c4:ea60, serial=0003) |
| G90 GNSS | `/dev/autoracer_gnss` | Pending (user USB ID needed) |

**EKF fusion strategy** (robot_localization):
- odom0: wheel odometry → vx, vy, vyaw (differential mode)
- imu0: N300 Pro → yaw, vyaw (absolute mode, remove gravity)
- Output: `odom_combined` frame, 30Hz, two_d_mode=true

**IMU integration pattern** (from Wheeltec):
- STM32 IMU remapped: `/imu/data_raw` → `/imu/data_board`
- External IMU (hipnuc_imu) publishes: `/imu/data_raw` @100Hz
- Madgwick filter: use_mag=false, world_frame=enu
- EKF fuses odom + filtered IMU

## Frame Hierarchy

**Current** (2026-01-29, with URDF):
```
odom
└── base_footprint (Z=0, ground level)
    ├── base_link (Z=+0.11m, axle height, URDF root)
    │   ├── rear_left_wheel_link (Y=+0.24m, continuous)
    │   ├── rear_right_wheel_link (Y=-0.24m, continuous)
    │   ├── front_left_steering_link (X=+0.54m, Y=+0.24m, revolute ±22.5°)
    │   │   └── front_left_wheel_link (continuous)
    │   ├── front_right_steering_link (X=+0.54m, Y=-0.24m, revolute ±22.5°)
    │   │   └── front_right_wheel_link (continuous)
    │   ├── laser (X=+0.24m, Z=+0.39m, yaw=-90°, fixed)
    │   └── zed_camera_link (X=+0.34m, Z=+0.29m, fixed)
    │       └── [ZED internal TF: camera_center, left/right_camera_frame, etc.]
    └── gyro_link (coincident with base_footprint)
```

**Target** (after GNSS integration):
```
map (from SLAM)
└── odom_combined (from EKF)
    └── base_footprint
        ├── base_link (URDF root, wheel/steering/sensor frames)
        ├── gyro_link (IMU)
        └── navsat_link (G90 GNSS antenna, pending)
```
