# Phase 1 Ackermann Chassis Acceptance Record

日期:
测试人员:
阶段: 阶段 1
Case:
上位机仓库/分支/提交:
下位机仓库/分支/提交:
根仓库提交:
工作区是否有未提交改动:

测试类型: offline-protocol / launch-check / live-topic / bench-uart / jack-test / ground-low-speed
测试命令:
测试输入:
counts_per_meter 标定记录:
日志路径:
rosbag 路径:
串口日志路径:

## 追踪表

| Case | 自动检查 | 人工检查 | 证据文件 | 状态 |
| --- | --- | --- | --- | --- |
| `protocol_downlink` |  | 无 |  | 未实现/FAIL/PASS/BLOCKED |
| `telemetry_uplink` |  | 无 |  | 未实现/FAIL/PASS/BLOCKED |
| `wheel_odom_gate` |  | counts_per_meter 实测来源确认 |  | 未实现/FAIL/PASS/BLOCKED |
| `fused_odom` |  | IMU/底盘/TF 现场状态确认 |  | 未实现/FAIL/PASS/BLOCKED |
| `motion_basic` |  | 架空/落地安全、运动方向、停车效果 |  | 未实现/FAIL/PASS/BLOCKED |
| `safety_basic` |  | RC 接管、急停、刹车实际动作 |  | 未实现/FAIL/PASS/BLOCKED |

## 标准命令记录

| 命令 | speed_mps | steering_angle_rad | enable | brake | emergency_stop | 持续时间 | 证据 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `straight_low_speed` | `0.10` | `0.0` | `true` | `false` | `false` | `2 s` |  |
| `left_low_speed` | `0.10` | `+0.15` | `true` | `false` | `false` | `2 s` |  |
| `right_low_speed` | `0.10` | `-0.15` | `true` | `false` | `false` | `2 s` |  |
| `stop_command` | `0.0` | `0.0` | `true` | `true` | `false` | once |  |
| `timeout_stop` | 停止发布命令 |  |  |  |  |  |  |
| `estop_command` | `0.0` | `0.0` | `false` | `true` | `true` | once |  |

## 状态位记录

| 状态位 | 证据 | 状态 |
| --- | --- | --- |
| `AUTO_ENABLED` |  | 未实现/FAIL/PASS/BLOCKED |
| `RC_OVERRIDE_ACTIVE` |  | 未实现/FAIL/PASS/BLOCKED |
| `ESTOP_ACTIVE` |  | 未实现/FAIL/PASS/BLOCKED |
| `COMMAND_TIMEOUT` |  | 未实现/FAIL/PASS/BLOCKED |
| `BRAKE_ACTIVE` |  | 未实现/FAIL/PASS/BLOCKED |
| `SPEED_SATURATED` / `STEERING_SATURATED` |  | 未实现/FAIL/PASS/BLOCKED |
| `ACCEL_LIMITED` / `STEERING_RATE_LIMITED` |  | 未实现/FAIL/PASS/BLOCKED |

## 运动安全记录

车辆状态:
是否架空:
RC 是否可接管:
急停/断电路径:
电池电压:
最大速度:
最大转角:

## 结果

结论: 未实现 / PASS / FAIL / BLOCKED
失败现象:
后续处理:
未覆盖风险:
