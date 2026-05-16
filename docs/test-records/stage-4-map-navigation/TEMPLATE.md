# 阶段 4 验收记录模板

日期:
测试人员:
阶段: 阶段 4
Case:
上位机仓库/分支/提交:
下位机仓库/分支/提交:
根仓库提交:
工作区是否有未提交改动:
run id:
artifact 目录:

测试类型: map-load / amcl-localization / nav2-plan / rpp-control / collision-monitor / cmd-vel-isolation / adapter / ground-low-speed
测试命令:
测试输入地图:
目标点:
Nav2 参数文件:
adapter 参数:
topic graph / node info 证据:
日志路径:
rosbag 路径:
checks JSON:

注意：键盘、manual 或 legacy `/cmd_vel` 只能作为迁移/排障记录，不能作为阶段 4 PASS 证据。
阶段 4A 是默认 PASS 范围；`reverse_plan_controlled` 是阶段 4B 扩展项，4A 通过时可记为 `未实现/4B 后续`，不得伪记为 PASS。

| Case | 自动检查 | 人工检查 | 证据文件 | 状态 |
| --- | --- | --- | --- | --- |
| `map_load` | map server active，`/map` 可用 | 地图与现场一致 |  | 未实现/FAIL/PASS/BLOCKED |
| `amcl_localization` | `/amcl_pose`、`map -> odom`、定位稳定 | 初始位姿方向正确 |  | 未实现/FAIL/PASS/BLOCKED |
| `global_plan_forward` | Smac Hybrid-A* path、最小转弯半径、footprint | 路径不穿墙、不明显不可达 |  | 未实现/FAIL/PASS/BLOCKED |
| `rpp_cmd_vel` | `/nav2_cmd_vel`、禁用原地旋转、速度限制 | 车辆行为平顺 |  | 未实现/FAIL/PASS/BLOCKED |
| `collision_monitor_stop` | `/safe_nav2_cmd_vel`、stop/slowdown/limit 触发 | 车未继续撞向障碍 |  | 未实现/FAIL/PASS/BLOCKED |
| `cmd_vel_isolation` | `/cmd_vel` 不直连串口桥；`/nav2_cmd_vel -> /safe_nav2_cmd_vel -> adapter` | 无旁路 |  | 未实现/FAIL/PASS/BLOCKED |
| `adapter_ackermann_output` | `/safe_nav2_cmd_vel -> /ackermann_cmd`、限幅、不可行请求停车、diagnostics | 转向/速度方向正确 |  | 未实现/FAIL/PASS/BLOCKED |
| `goal_reached_stop` | Nav2 goal 完成，`/ackermann_cmd` 停车 | 实车停在可接受范围 |  | 未实现/FAIL/PASS/BLOCKED |
| `blocked_no_path` | 停车、`BLOCKED`/失败原因 | 不硬撞、不原地乱转 |  | 未实现/FAIL/PASS/BLOCKED |
| `reverse_plan_controlled` | 倒车段、倒车限速、安全区证据 | 后方/侧方安全确认 |  | 未实现/FAIL/PASS/BLOCKED |

Nav2 / adapter 参数:

| 参数 | 值 |
| --- | --- |
| planner | Smac Hybrid-A* |
| `motion_model_for_search` | DUBIN / REEDS_SHEPP |
| controller | Regulated Pure Pursuit |
| `minimum_turning_radius` | 2.24 |
| `allow_reversing` | false / true |
| `use_rotate_to_heading` | false |
| collision monitor zones | 4A: `front_stop`、`front_slowdown`、`front_limit`; 4B extra: `rear_stop`、`rear_slowdown`、`left_side_stop`、`right_side_stop` |
| Nav2 internal cmd topic | `/nav2_cmd_vel` |
| safe cmd topic | `/safe_nav2_cmd_vel` |
| adapter diagnostics topic | `/twist_to_ackermann/diagnostics` |
| adapter diagnostics type | `diagnostic_msgs/msg/DiagnosticArray` |
| final chassis command | `/ackermann_cmd` |
| `min_turn_speed_mps` | 0.05 |
| `angular_deadband_radps` | 0.02 |
| `input_timeout_sec` | 0.50 |
| `brake_on_stop` | true |

复用边界:

| 项 | 值 |
| --- | --- |
| 是否使用 Nav2 官方 RPP | true / false |
| 是否使用 Collision Monitor | true / false |
| 是否使用自研 tracker 替代 RPP | false |
| `/cmd_vel` 是否直连底盘 | false |

adapter diagnostics 字段:

| 字段 | 值 |
| --- | --- |
| `input_vx_mps` |  |
| `input_wz_radps` |  |
| `output_speed_mps` |  |
| `output_steering_angle_rad` |  |
| `speed_limited` | true / false |
| `steering_limited` | true / false |
| `reverse` | true / false |
| `infeasible_spin` | true / false |
| `stop_reason` | none / zero_command / invalid_input / input_timeout / infeasible_spin / reverse_disabled |

安全状态:
是否架空/低速地面:
RC 是否可接管:
急停/断电路径:
最大速度:
最大转角:
是否触发安全状态:
倒车安全区证据:

结论: 未实现 / PASS / FAIL / BLOCKED
失败现象:
后续处理:
未覆盖风险:
