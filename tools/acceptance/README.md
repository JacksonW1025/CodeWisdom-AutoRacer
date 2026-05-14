# Acceptance Tools

本目录存放阶段验收工具和测试输入。默认规则：不打开串口、不发布非零运动命令、不替代人工安全确认。

从 `CodeWisdom-AutoRacer/` 仓库根目录执行。

## 文件说明

| 路径 | 作用 | 是否会让车运动 |
| --- | --- | --- |
| `stage1_acceptance_check.py` | 阶段 1 最小验收检查。offline 模式检查协议帧、telemetry 解析、`counts_per_meter` 门禁和 launch 契约；live 模式只检查已运行 topic 和类型。 | 不会 |
| `stage2_path_fixture_check.py` | 阶段 2 path fixture 静态检查。确认 `straight_2m`、左右圆弧、S 弯和到点停车 fixture 格式正确。 | 不会 |
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
    stage2_tracker_fake_odom_check.py   # 待实现
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
```

这些 fixture 是后续 tracker 的固定输入样例；fixture 检查只读取本地 JSON 文件并校验路径几何。

阶段 2 tracker 实现契约：

- test path publisher 发布 `/path_tracking/path`，类型 `nav_msgs/msg/Path`。
- Pure Pursuit tracker 订阅 `/path_tracking/path` 和 `/odom`，发布 `/ackermann_cmd`。
- diagnostics publisher 发布 `/path_tracking/diagnostics`，类型 `autoracer_interfaces/msg/PathTrackingDiagnostics`。
- `stage2_tracker_fake_odom_check.py` 实现后，必须使用本目录 fixture 检查转角方向、限幅、S 弯符号切换和到点停车。
