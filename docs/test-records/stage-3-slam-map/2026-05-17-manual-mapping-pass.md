# 阶段 3 二楼环形通道最终扫图验收记录

日期: 2026-05-17
测试人员: 现场驾驶员 + Codex
阶段: 阶段 3
Case: `lidar_topic` / `tf_chain` / `slam_map_live` / `map_save_reload` / `mapping_bag_replay`
run id: `stage3-final-floor2-loop-20260517-101119`

## 结论

阶段 3 PASS。最终地图已生成、可加载，并作为阶段 4 固定地图定位测试的唯一阶段 3 地图输入。

人工判定:

- 手动 RC 扫图完成一圈闭环，并在起点附近短距离重叠后停车保存。
- 预览图未见明显错层、整体进墙或第二圈带偏问题。
- 旧的不完整阶段 3 地图和阶段 3 临时 rosbag 已清理。
- 本次最终 rosbag 仅保留本地用于阶段 4 4A 前的问题复盘，不提交 git。

保留要求:

- `docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.*` 是阶段 4 默认固定地图输入，不是临时测试产物。
- 没有新的阶段 3 PASS 记录、地图预览、map server 加载检查和阶段 4 地图引用更新时，不要删除、改名或替换这组地图文件。
- 如需重扫并替换地图，必须保留新旧 run id 的交接记录，说明替换原因和阶段 4 使用哪一份。

阶段 4 尚未完成。本记录只证明固定地图输入可用；阶段 4 仍需单独验证 map load、AMCL/localization、Nav2 plan、Collision Monitor 和 `/cmd_vel` 隔离。

## 启动与记录命令

```bash
cd CodeWisdom-AutoRacer
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch autoracer_bringup stage3_mapping.launch.py \
  counts_per_meter:=13.545 \
  use_rviz:=true \
  slam_params_file:=$(ros2 pkg prefix autoracer_slam_toolbox)/share/autoracer_slam_toolbox/config/mapper_params_floor2_dense.yaml

ros2 bag record -o docs/test-records/rosbags/stage3-final-floor2-loop-20260517-101119 \
  /chassis_state /wheel_odom /odom /imu/data /scan /map /map_metadata /tf /tf_static
```

正式 2D 建图未记录 `/point_cloud_raw`，只记录 `/scan` 等 2D SLAM 复盘必需 topic。3D SLAM、NDT/LIO-SAM 或点云外参问题需要另存 `-raw-cloud` bag。

地图保存命令:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'docs/test-records/maps/stage3-final-floor2-loop-20260517-101119'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: 'docs/test-records/maps/stage3-final-floor2-loop-20260517-101119'}"
```

地图加载检查命令:

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.yaml
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 service call /map_server/map nav_msgs/srv/GetMap {}
```

## 证据路径

- 地图 YAML: `docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.yaml`
- 地图图像: `docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.pgm`
- posegraph: `docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.posegraph`
- serialized data: `docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.data`
- 地图预览 PNG: `docs/test-records/maps/stage3-final-floor2-loop-20260517-101119-preview.png`
- 本地 rosbag: `docs/test-records/rosbags/stage3-final-floor2-loop-20260517-101119`

## 自动统计

| 字段 | 结果 |
| --- | --- |
| rosbag 大小 | `176.8 MiB` |
| rosbag 时长 | `325.216947225 s` |
| `/scan` 消息数 | `6311` |
| `/imu/data` 消息数 | `32523` |
| `/wheel_odom` 消息数 | `6505` |
| `/odom` 消息数 | `9754` |
| `/map` 消息数 | `163` |
| `/map_metadata` 消息数 | `163` |
| `/chassis_state` 消息数 | `6505` |
| `/tf` 消息数 | `25979` |
| `/tf_static` 消息数 | `3` |
| 最终地图尺寸 | `860 x 1201` cells |
| 地图分辨率 | `0.05 m/cell` |
| 地图物理范围 | `43.00 m x 60.05 m` |
| 最终地图 free / occupied / unknown | `284450` / `14785` / `733625` cells |
| map server 加载检查 | `configure` PASS, `activate` PASS, `GetMap` 返回 `860 x 1201 @ 0.05 m/cell` |

文件 sha256:

```text
17e519f33203bb781f884e8d3cbd4c1dae40d8e11b057e01d9ee2aec2c4e177b  stage3-final-floor2-loop-20260517-101119.yaml
84f669d5f4084d53a24d26d500a5cdc20d25816a03eef36ebf37c5e423663520  stage3-final-floor2-loop-20260517-101119.pgm
d1c84b73ef08289b71027b6df576ef71ef1d1db6d8e2335b94febdd8ecbd6bac  stage3-final-floor2-loop-20260517-101119.posegraph
66a501861a6855a79a89ea78ac4199e4640665a879a391fe12efd6430834fbc2  stage3-final-floor2-loop-20260517-101119.data
0c3bd047b7ebc8715f296f65606d7d07cad9b4ce860154991e381872abe4e05b  stage3-final-floor2-loop-20260517-101119-preview.png
3b245b024de9988c3d1aae6940b2f6fad78a21dbc96de6ded6a9b559a00cc4b4  rosbag metadata.yaml
```

## 追踪表

| Case | 自动检查 | 人工检查 | 证据文件 | 状态 |
| --- | --- | --- | --- | --- |
| `lidar_topic` | `/scan` 稳定记录到 `6311` 帧 | LiDAR 已接线并完成现场扫图 | rosbag metadata、启动日志 | PASS |
| `tf_chain` | SLAM 能发布 `/map` 并保存地图；`map_server` 可重载地图 | 未发现 TF 断链导致的建图中断 | rosbag、保存日志、加载检查 | PASS |
| `slam_map_live` | `/map` 与 `/map_metadata` 均有 `163` 帧；里程计和底盘反馈记录完整 | 人工 RC/manual 低速一圈闭环扫图完成 | rosbag、地图文件、预览图 | PASS |
| `map_save_reload` | `.yaml/.pgm/.posegraph/.data` 均生成；map server 重载成功 | 地图可作为阶段 4 固定地图定位输入 | 地图文件、加载检查 | PASS |
| `mapping_bag_replay` | rosbag 保留本地，可用于必要时离线复盘 `/scan + /odom + /tf` | 当前不提交 raw bag，避免仓库膨胀 | 本地 rosbag metadata | PASS |

## 未覆盖风险

- 阶段 4 4A 仍需加载固定地图做现场定位复测，确认 `map -> odom`、`/amcl_pose` 和 RViz 位姿贴合真实车位。
- 本次正式 2D bag 不含 `/point_cloud_raw`；若后续要分析 3D 点云、NDT/LIO-SAM 或外参，需要另录 raw-cloud bag。
