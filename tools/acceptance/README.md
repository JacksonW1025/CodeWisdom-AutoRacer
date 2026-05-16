# Acceptance Tools

本目录存放阶段验收工具和测试输入。默认规则：不打开串口、不发布非零运动命令、不替代人工安全确认。

从 `CodeWisdom-AutoRacer/` 仓库根目录执行。

## 文件说明

| 路径 | 作用 | 是否会让车运动 |
| --- | --- | --- |
| `stage1_acceptance_check.py` | 阶段 1 最小验收检查。offline 模式检查协议帧、telemetry 解析、`counts_per_meter` 门禁和 launch 契约；live 模式只检查已运行 topic 和类型。 | 不会 |
| `stage2_path_fixture_check.py` | 阶段 2 path fixture 静态检查。确认 `straight_2m`、左右圆弧、S 弯和到点停车 fixture 格式正确。 | 不会 |
| `stage2_tracker_fake_odom_check.py` | 阶段 2 tracker 离线检查。用 fixture 和 fake odom 验证 Pure Pursuit 输出方向、限幅、S 弯符号切换和到点停车。 | 不会 |
| `stage3_mapping_contract_check.py` | 阶段 3 mapping 固定入口、必需输入门禁、`pointcloud_to_laserscan` 和 `slam_toolbox` 参数契约检查。 | 不会 |
| `stage4_navigation_contract_check.py` | 阶段 4 fixed launch、必需输入门禁、Nav2 RPP/Smac/Collision Monitor 参数和 `twist_to_ackermann` adapter 离线契约检查。 | 不会 |
| `fixtures/stage2_paths/*.json` | 阶段 2 tracker 的标准输入，语义等价于 `nav_msgs/Path`。 | 本地数据文件 |

未实现工具的状态和启用时机以根仓库 `../docs/阶段路线图.md` 为准，不在本目录重复维护路线图。

## 目录层级

当前只有少量脚本，所以保持扁平结构：

```text
tools/
  acceptance/
    README.md
    stage1_acceptance_check.py
    stage2_path_fixture_check.py
    stage2_tracker_fake_odom_check.py
    stage3_mapping_contract_check.py
    stage4_navigation_contract_check.py
    fixtures/
      stage2_paths/
        straight_2m.json
        left_arc_r2m.json
        right_arc_r2m.json
        s_curve.json
        stop_at_end.json
```

单个阶段的脚本数量超过 3 个时，再拆成 `acceptance/stage1/`、`acceptance/stage2/`、`acceptance/stage3/`。当前保持扁平结构，减少空目录维护成本。

## Phase 1 Offline Checks

```bash
python3 tools/acceptance/stage1_acceptance_check.py --mode offline
```

Covered cases:

- `protocol_downlink`: Ackermann v1 11-byte command frame, flags, BCC, units, clamps.
- `telemetry_uplink`: 24-byte telemetry parsing, status flags/bits, BCC rejection.
- `wheel_odom_gate`: `counts_per_meter` and steering-valid gates before `/wheel_odom`.
- `launch_contract`: `ackermann_chassis.launch.py` and EKF `/odom` contract.

## Phase 1 Live Topic Check

Start the relevant ROS nodes first. This command only inspects topics and types.

```bash
python3 tools/acceptance/stage1_acceptance_check.py --mode live
python3 tools/acceptance/stage1_acceptance_check.py --mode live --require-odom
```

`--require-odom` 用于 `/imu/data`、`/wheel_odom` 和 EKF `/odom` 均已启动的检查场景。

## Phase 2 Fixture Check

```bash
python3 tools/acceptance/stage2_path_fixture_check.py
python3 tools/acceptance/stage2_tracker_fake_odom_check.py
```

fixture 检查只读取本地 JSON 文件并校验路径几何。fake-odom 检查调用 `autoracer_path_tracking`
里的 Pure Pursuit 控制逻辑，覆盖直线、左右圆弧、S 弯和到点停车的软件契约。

阶段 2 tracker 实现契约：

- test path publisher 发布 `/path_tracking/path`，类型 `nav_msgs/msg/Path`。
- Pure Pursuit tracker 订阅 `/path_tracking/path` 和 `/odom`，发布 `/ackermann_cmd`。
- diagnostics publisher 发布 `/path_tracking/diagnostics`，类型 `autoracer_interfaces/msg/PathTrackingDiagnostics`。
- `stage2_tracker_fake_odom_check.py` 使用本目录 fixture 检查转角方向、限幅、S 弯符号切换和到点停车。

## Phase 3/4 Offline Contract Checks

```bash
python3 tools/acceptance/stage3_mapping_contract_check.py
python3 tools/acceptance/stage4_navigation_contract_check.py
```

这两个脚本只读取 launch、参数、adapter 源码和本地纯函数，不启动 ROS graph、不打开串口、不发布运动命令。
脚本会检查固定入口是否具备必需输入门禁；真正的地图质量、日志落盘和 rosbag 内容仍以阶段验收记录为准。

阶段 3 检查覆盖：

- `stage3_mapping.launch.py` include IMU、URDF/TF、阶段 1 底盘入口、LiDAR 和 `slam_toolbox`。
- `stage3_mapping.launch.py` 在启动底盘链路时拒绝空值或 `<=0` 的 `counts_per_meter`。
- `slam.launch.py` 固定 `include_bringup=false`，并保持 `/point_cloud_raw -> /scan` 参数基线。
- `mapper_params_online_sync.yaml` 固定 `odom/map/base/scan` frame、地图分辨率、更新周期和回环参数。

阶段 4 检查覆盖：

- `stage4_navigation.launch.py` include IMU、URDF/TF、阶段 1、LiDAR、Nav2 localization/navigation、Collision Monitor 和 adapter。
- `stage4_navigation.launch.py` 在启动底盘链路时拒绝空值或 `<=0` 的 `counts_per_meter`；启动 Nav2 时拒绝空地图或不存在的地图路径。
- Nav2 controller 输出 remap 到 `/nav2_cmd_vel`，Collision Monitor 输出 `/safe_nav2_cmd_vel`，adapter 输出 `/ackermann_cmd`。
- `stage4_nav2_params.yaml` 使用 Smac Hybrid-A* `DUBIN`、RPP、Collision Monitor 默认安全区，不使用 MPPI。
- `twist_to_ackermann` 离线检查速度/转角限幅、不可行原地旋转、NaN、倒车禁用和倒车限幅。
