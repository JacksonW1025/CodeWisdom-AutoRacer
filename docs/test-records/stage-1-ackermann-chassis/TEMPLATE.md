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
