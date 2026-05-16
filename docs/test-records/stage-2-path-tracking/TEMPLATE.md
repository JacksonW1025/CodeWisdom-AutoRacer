# Phase 2 Path Tracking Acceptance Record

日期:
测试人员:
阶段: 阶段 2
Case:
上位机仓库/分支/提交:
下位机仓库/分支/提交:
根仓库提交:
工作区是否有未提交改动:

测试类型: fixture-check / tracker-offline / fake-odom / jack-test / ground-low-speed
测试命令:
测试输入 fixture:
path topic: /path_tracking/path
diagnostics topic: /path_tracking/diagnostics
日志路径:
rosbag 路径:

## 追踪表

| Case | 自动检查 | 人工检查 | 证据文件 | 状态 |
| --- | --- | --- | --- | --- |
| `straight_2m` | path、`/odom`、`/ackermann_cmd`、误差和停车 | 实车是否偏航 |  | 未实现/FAIL/PASS/BLOCKED |
| `left_arc_r2m` | 转角方向、限幅、误差记录 | 实车左转方向和半径 |  | 未实现/FAIL/PASS/BLOCKED |
| `right_arc_r2m` | 转角方向、限幅、误差记录 | 实车右转方向和半径 |  | 未实现/FAIL/PASS/BLOCKED |
| `s_curve` | 转角切换、限幅、误差记录 | 是否平滑、是否抖动 |  | 未实现/FAIL/PASS/BLOCKED |
| `stop_at_end` | 终点停车命令和停车距离 | 是否实际停住 |  | 未实现/FAIL/PASS/BLOCKED |

## Tracker 参数

| 参数 | 值 |
| --- | --- |
| tracker | Pure Pursuit |
| `lookahead_m` | 0.60 |
| `goal_tolerance_m` | 0.05 |
| 实车验收窗口 | 0.10 |
| `control_rate_hz` | 20 |
| `allow_reverse` | false |
| `wheelbase_m` | 0.60 |
| `max_steering_angle_rad` | 0.262 |
| `target_speed_mps` |  |
| `max_target_speed_mps` | 默认 0.25；现场批准后最高 1.00 |
| `prearm_zero_before_motion` | true |
| 预使能通过条件 | `/chassis_state`: `auto_enabled=true`、`command_timeout=false`、`rc_override_active=false`、`estop_active=false` |
| STM32 自动命令限幅 | 前进 1.00 m/s；倒车 0.60 m/s |
| 霍尔超速保护 | 5.0 m/s 触发；4.5 m/s 释放；不作为测试目标速度 |

## Diagnostics 字段记录

| 字段 | 证据 |
| --- | --- |
| `case_name` |  |
| `target_index` |  |
| `lateral_error_m` |  |
| `heading_error_rad` |  |
| `target_speed_mps` / `actual_speed_mps` |  |
| `steering_command_rad` |  |
| `steering_limited` / `speed_limited` |  |
| `stop_commanded` / `remaining_distance_m` |  |

## 结果

结论: 未实现 / PASS / FAIL / BLOCKED
失败现象:
后续处理:
未覆盖风险:
