# 阶段 3 验收记录模板

日期:
测试人员:
阶段: 阶段 3
Case:
上位机仓库/分支/提交:
下位机仓库/分支/提交:
根仓库提交:
工作区是否有未提交改动:

测试类型: lidar-topic / tf-check / slam-live / map-save-reload / bag-replay
测试命令:
测试输入:
地图输出路径:
地图保存命令:
地图重载命令:
pose-graph / serialized map 路径:
日志路径:
rosbag 路径:
扫图运动方式:

## 追踪表

| Case | 自动检查 | 人工检查 | 证据文件 | 状态 |
| --- | --- | --- | --- | --- |
| `lidar_topic` | `/scan` topic、frame、频率、时间戳 | 安装方向和遮挡 |  | 未实现/FAIL/PASS/BLOCKED |
| `tf_chain` | `map -> odom -> base_footprint/base_link -> laser` 连通 | 外参方向 |  | 未实现/FAIL/PASS/BLOCKED |
| `slam_map_live` | `/map`、topic、日志 | 地图重影/形变 |  | 未实现/FAIL/PASS/BLOCKED |
| `map_save_reload` | `map.yaml`、地图图像、重载命令 | 地图可用性 |  | 未实现/FAIL/PASS/BLOCKED |
| `mapping_bag_replay` | `/scan + /odom + /tf` replay 结果 | bag 覆盖范围 |  | 未实现/FAIL/PASS/BLOCKED |

## 建图记录

地图质量:
LiDAR frame:
odom 来源:
TF 检查结果:
map save/reload 结果:
bag replay 结果:
是否沿用阶段 1 安全链路:
是否仅使用 RC/遥控扫图:

## 结果

结论: 未实现 / PASS / FAIL / BLOCKED
失败现象:
后续处理:
未覆盖风险:
