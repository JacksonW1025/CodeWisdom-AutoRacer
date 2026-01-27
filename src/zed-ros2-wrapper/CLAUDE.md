# ZED ROS2 Wrapper

Stereolabs ZED camera ROS2 wrapper (v5.1.0) providing depth sensing, visual odometry, object detection, and body tracking for ROS2 applications.

## Project Structure

```
zed-ros2-wrapper/
├── zed_ros2/                     # Meta package
├── zed_wrapper/                  # Launch files and configuration
│   ├── launch/
│   │   └── zed_camera.launch.py  # Main launch file
│   ├── config/
│   │   ├── common_stereo.yaml    # Stereo cameras common parameters
│   │   ├── common_mono.yaml      # Mono cameras common parameters
│   │   ├── zed.yaml, zedm.yaml, zed2.yaml, zed2i.yaml  # Camera-specific configs
│   │   ├── zedx.yaml, zedxm.yaml, zedxhdr.yaml         # ZED X series
│   │   ├── zedxonegs.yaml, zedxone4k.yaml              # ZED X One series
│   │   └── object_detection.yaml # Object detection parameters
│   └── urdf/                     # Robot description files
├── zed_components/               # ROS2 component plugins
│   └── src/
│       ├── zed_camera/           # Stereo camera component (~14,600 lines C++)
│       └── zed_camera_one/       # ZED X One component
├── docker/                       # Docker configurations
└── images/                       # Documentation images
```

## Build System

- **Build tool**: colcon with ament_cmake
- **CMake minimum**: 3.8
- **Prerequisites**: ZED SDK v5.1+, CUDA

### Build Commands

```bash
# Full workspace build
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

# Single package build
colcon build --packages-select zed_wrapper --symlink-install

# Resource-constrained build (Jetson)
colcon build --parallel-workers 2 --symlink-install

# Source workspace
source install/setup.bash
```

## Package Dependencies

**Core Dependencies**:
- rclcpp, rclcpp_components, image_transport
- ZED SDK (v5.1+), CUDA
- sensor_msgs, geometry_msgs, nav_msgs, tf2, tf2_ros
- zed_msgs (from zed-ros2-interfaces)
- diagnostic_updater, visualization_msgs

**Optional Dependencies**:
- point_cloud_transport (point cloud compression)
- isaac_ros_nitros (NVIDIA Isaac ROS acceleration)
- robot_localization, nmea_msgs, geographic_msgs (GNSS fusion)

## Supported Camera Models

| Type | Models |
|------|--------|
| Stereo | `zed`, `zedm`, `zed2`, `zed2i`, `zedx`, `zedxm`, `zedxhdr`, `zedxhdrmini`, `zedxhdrmax`, `virtual` |
| Mono | `zedxonegs`, `zedxone4k`, `zedxonehdr` |

## ROS2 Topics

### Video/Depth Topics
| Topic | Type | Description |
|-------|------|-------------|
| `~/left/color/rect/image` | sensor_msgs/Image | Left rectified RGB |
| `~/right/color/rect/image` | sensor_msgs/Image | Right rectified RGB |
| `~/depth/depth_map` | sensor_msgs/Image | Depth map (float32 meters) |
| `~/depth/confidence_map` | sensor_msgs/Image | Depth confidence |
| `~/depth/point_cloud` | sensor_msgs/PointCloud2 | 3D point cloud |

### Odometry/Pose Topics
| Topic | Type | Description |
|-------|------|-------------|
| `~/odom` | nav_msgs/Odometry | Visual odometry |
| `~/pose` | geometry_msgs/PoseStamped | Camera pose |
| `~/pose/status` | zed_msgs/PosTrackStatus | Tracking status |

### Sensor Topics
| Topic | Type | Description |
|-------|------|-------------|
| `~/imu/data` | sensor_msgs/Imu | Fused IMU data |
| `~/imu/data_raw` | sensor_msgs/Imu | Raw IMU data |
| `~/mag` | sensor_msgs/MagneticField | Magnetometer |
| `~/baro` | sensor_msgs/FluidPressure | Barometer |

### AI Topics
| Topic | Type | Description |
|-------|------|-------------|
| `~/od/objects` | zed_msgs/ObjectsStamped | Detected objects |
| `~/bt/skeleton_tracker` | zed_msgs/SkeletonArray | Body skeletons |

## ROS2 Services

| Service | Type | Description |
|---------|------|-------------|
| `reset_odometry` | std_srvs/Trigger | Reset odometry |
| `reset_pos_tracking` | std_srvs/Trigger | Reset positional tracking |
| `set_pose` | zed_msgs/SetPose | Set camera pose |
| `start_svo_recording` | zed_msgs/StartSvoRec | Start recording SVO |
| `stop_svo_recording` | std_srvs/Trigger | Stop recording |
| `enable_obj_det` | std_srvs/SetBool | Enable/disable object detection |
| `enable_body_tracking` | std_srvs/SetBool | Enable/disable body tracking |
| `enable_mapping` | std_srvs/SetBool | Enable/disable spatial mapping |

## Key Parameters

```yaml
# General
camera_timeout: 5.0           # Connection timeout (seconds)
serial_number: 0              # Camera serial (0 = first available)
pub_resolution: "MEDIUM"      # NATIVE/MEDIUM/LOW
pub_frame_rate: 15.0          # Publishing rate (Hz)

# Depth
depth_mode: "NEURAL_LIGHT"    # NEURAL_LIGHT/NEURAL/QUALITY/ULTRA/PERFORMANCE
depth_confidence: 50          # Confidence threshold (1-100)
point_cloud_freq: 10.0        # Point cloud rate (Hz)

# Positional Tracking
pos_tracking_enabled: true
pos_tracking_mode: "GEN_2"    # GEN_1/GEN_2/GEN_3
imu_fusion: true
publish_tf: true
map_frame: "map"
odom_frame: "odom"

# Sensors
publish_imu: true
publish_imu_raw: true

# Object Detection
od_enabled: false
detection_model: "MULTI_CLASS_BOX_MEDIUM"
max_range: 20.0               # Max detection range (m)

# Body Tracking
bt_enabled: false
model: "BODY_38"              # BODY_18/BODY_34/BODY_38
```

## Run Commands

```bash
# Launch ZED 2i camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

# Launch with custom name
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=front_camera

# Launch from SVO file
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i svo_path:=/path/to/file.svo

# Launch with simulation time
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i use_sim_time:=true

# Launch for Isaac Sim
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx sim_mode:=true

# Launch with GNSS fusion
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i enable_gnss:=true

# Check topics
ros2 topic list | grep zed

# Check point cloud rate
ros2 topic hz /zed/depth/point_cloud

# Reset odometry
ros2 service call /zed/reset_odometry std_srvs/srv/Trigger
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_model` | (required) | Camera model name |
| `camera_name` | `"zed"` | Camera namespace |
| `svo_path` | `"live"` | Path to SVO file or "live" |
| `use_sim_time` | `false` | Use simulation clock |
| `sim_mode` | `false` | Isaac Sim mode |
| `publish_urdf` | `true` | Publish URDF |
| `publish_tf` | `true` | Publish TF transforms |
| `enable_gnss` | `false` | Enable GNSS fusion |
| `enable_ipc` | `true` | Enable intra-process communication |

## Frame Hierarchy

```
map
└── odom
    └── camera_link
        ├── camera_left_link
        ├── camera_right_link
        └── imu_link
```

## Dynamic Parameters

These can be changed at runtime via `ros2 param set`:
- saturation, sharpness, gamma, exposure, gain, whitebalance
- depth_confidence, depth_texture_conf
- remove_saturated_areas, point_cloud_freq
- confidence_threshold (body tracking)

## Hardware Requirements

| Platform | Requirements |
|----------|--------------|
| x86_64 | Ubuntu 22.04+, CUDA-capable GPU, ZED SDK v5.1+ |
| Jetson | JetPack 6+, nvidia-jetpack packages |

## Related Repositories

- `zed-ros2-interfaces`: Custom message/service definitions (zed_msgs)
- `zed-ros2-examples`: Tutorials and example applications
- Official docs: https://www.stereolabs.com/docs/ros2/
