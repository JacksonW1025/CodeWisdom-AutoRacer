# hipnuc_imu

N300 Pro IMU ROS2 驱动包（超核电子 HI13 芯片）。

## 来源

从 `reference/wheeltec_ros2/src/wheeltec_imu/hipnuc_imu/` 复制。

## 功能

- 通过串口读取 N300 Pro IMU 数据（HI91 协议）
- 发布 `sensor_msgs/Imu` 消息到 `/imu/data_raw`
- 支持四元数、角速度、线性加速度

## 配置

```yaml
# config/hipnuc_config.yaml
IMU_publisher:
    ros__parameters:
        serial_port: "/dev/autoracer_imu"  # udev 符号链接
        baud_rate: 115200
        frame_id: "gyro_link"
        imu_topic: "/imu/data_raw"
```

## udev 规则

```bash
# /etc/udev/rules.d/autoracer_imu.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="autoracer_imu"
```

## 运行

```bash
# 单独运行 IMU 驱动
ros2 launch hipnuc_imu imu_spec_msg.launch.py

# 查看 IMU 数据
ros2 topic echo /imu/data_raw
```

## 硬件信息

| 属性 | 值 |
|------|-----|
| 设备 | Wheeltec N300 Pro |
| 芯片 | HI13 (超核电子) |
| USB 芯片 | CP2102N (Silicon Labs) |
| idVendor | 10c4 |
| idProduct | ea60 |
| serial | 0003 |
| 波特率 | 115200 |
