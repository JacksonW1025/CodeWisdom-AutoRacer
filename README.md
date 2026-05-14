# CodeWisdom-AutoRacer

基于 ROS 2 Humble 的 Ackermann RC 小车自主驾驶工作区，聚焦底盘 bringup、传感器接入、SLAM 建图、导航与调试可视化。

## Current Status

- 默认底盘入口保留 legacy/default 语义
  - 启动 `autoracer_robot`
  - 当前仍是 legacy/default 语义：订阅 `/cmd_vel`，发布 raw `/odom`
  - 可选接入 N300 Pro IMU
  - 默认启动 Madgwick 与 `robot_localization` EKF，形成 `/imu/data_raw -> /imu/data -> /odom_combined`
  - 启动 URDF / TF
- phase-1 Ackermann 验收入口以 `docs/启动与运行规范.md` 为准
  - 验收时必须满足 `counts_per_meter>0`
  - canonical odom 链路为 `/wheel_odom + /imu/data -> robot_localization -> /odom`
  - `/cmd_vel` 兼容入口只用于迁移、手动调试或 legacy 兼容，不作为正式 STM32 下行协议或阶段验收入口
- 已接入模块：
  - LiDAR C32: `lslidar_driver`
  - IMU: `hipnuc_imu`
  - URDF / RViz: `autoracer_robot_urdf`, `autoracer_imu_tf_broadcaster`
  - 2D SLAM: `autoracer_slam_toolbox`, `slam_gmapping`
  - 3D SLAM: `lio_sam`
  - Navigation: `autoracer_robot_nav2`

## Quick Start

启动入口、launch 参数和运行检查只维护在 [启动与运行规范](docs/启动与运行规范.md)。

## Documentation

- [docs/README.md](docs/README.md)：上位机文档索引。
- [docs/启动与运行规范.md](docs/启动与运行规范.md)：当前 ROS2 启动入口、launch 参数、topic 检查和阶段验收入口。
- [docs/开发流程与验证规范.md](docs/开发流程与验证规范.md)：上位机提交、构建验证、根文档同步和阶段验收回填规则。
- [tools/README.md](tools/README.md)：上位机辅助工具目录说明。
- [tools/acceptance/README.md](tools/acceptance/README.md)：阶段 1 最小验收检查和阶段 2 path fixture 检查。
- `docs/archive/`：历史 agent 记录和阶段 review，只作为回溯资料，不作为当前状态权威。

## Notes

- 当前仓库方向已基本对齐 Wheeltec 参考实现，但默认使用体验仍在持续收敛中，不应表述为“已完全对齐”。
- 历史 Stage Review 和旧 agent 文档已归档到 `docs/archive/`，只用于回溯，不作为当前运行或验收依据。
- 本仓库提交时机、提交粒度、构建验证和文档同步见 `docs/开发流程与验证规范.md`。
