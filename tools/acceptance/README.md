# Acceptance Tools

本目录存放阶段验收工具和测试输入。默认规则：不打开串口、不发布非零运动命令、不替代人工安全确认。

从 `CodeWisdom-AutoRacer/` 仓库根目录执行。

## 文件说明

| 路径 | 作用 | 是否会让车运动 |
| --- | --- | --- |
| `stage1_acceptance_check.py` | 阶段 1 最小验收检查。offline 模式检查协议帧、telemetry 解析、`counts_per_meter` 门禁和 launch 契约；live 模式只检查已运行 topic 和类型。 | 不会 |
| `stage2_path_fixture_check.py` | 阶段 2 path fixture 静态检查。确认 `straight_2m`、左右圆弧、S 弯和到点停车 fixture 格式正确。 | 不会 |
| `fixtures/stage2_paths/*.json` | 阶段 2 tracker 的标准输入，语义等价于 `nav_msgs/Path`。 | 不会 |

未实现工具的状态和启用时机以根仓库 `../docs/阶段路线图.md` 为准，不在本目录重复维护路线图。

## 目录层级

当前只有少量脚本，所以保持扁平结构：

```text
tools/
  acceptance/
    README.md
    stage1_acceptance_check.py
    stage2_path_fixture_check.py
    fixtures/
      stage2_paths/
        straight_2m.json
        left_arc_r2m.json
        right_arc_r2m.json
        s_curve.json
        stop_at_end.json
```

后续如果每个阶段的脚本超过 3 个，再拆成 `acceptance/stage1/`、`acceptance/stage2/`、`acceptance/stage3/`。当前不提前拆目录，避免为了空结构增加维护成本。

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

Use `--require-odom` only after `/imu/data`, `/wheel_odom`, and EKF `/odom` are expected to be running.

## Phase 2 Fixture Check

```bash
python3 tools/acceptance/stage2_path_fixture_check.py
```

这些 fixture 是后续 tracker 的输入，不是 tracker 实现，也不会发布 `/ackermann_cmd`。
