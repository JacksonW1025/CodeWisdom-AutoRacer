# autoracer_robot_nav2

Nav2 导航配置包，适配 AutoRacer Ackermann 转向平台。

当前运行入口、阶段验收要求和 legacy 禁止项以仓库根 `docs/启动与运行规范.md` 为准。本文只保留本包用途和典型命令提示，不作为阶段验收标准。

## 功能

- Nav2 地图导航历史/过渡配置参考
- Stage 4 稳定版 Nav2 参数 `param/stage4_nav2_params.yaml`
- `twist_to_ackermann` adapter
- 地图加载与保存
- pointcloud_to_laserscan 集成

注意：`param/autoracer_nav2_params.yaml` 仍保留 MPPI 历史/过渡参数，不作为阶段 4 稳定验收主线。
阶段 4 稳定版使用 `param/stage4_nav2_params.yaml`，并以
`CodeWisdom-AutoRacer/docs/启动与运行规范.md` 中的 AMCL + Smac Hybrid-A* + Regulated Pure Pursuit
+ Collision Monitor + `twist_to_ackermann` 链路为准。

## 依赖

- `navigation2` (系统包 `ros-humble-navigation2`)
- `nav2_bringup` (系统包 `ros-humble-nav2-bringup`)
- `pointcloud_to_laserscan` (系统包 `ros-humble-pointcloud-to-laserscan`)

## 使用

### 建图流程

```bash
# 1. 按 docs/启动与运行规范.md 启动 phase-1 Ackermann 底盘链路和 LiDAR
# 底盘验收路径必须使用 ackermann_chassis.launch.py counts_per_meter:=<实测值> use_ekf:=true
ros2 launch lslidar_driver lslidar_cx_launch.py

# 2. 启动 SLAM Toolbox 建图；阶段 3 验收必须关闭 legacy bringup include
ros2 launch autoracer_slam_toolbox slam.launch.py include_bringup:=false

# 3. 按阶段 3 验收流程低速扫图；运动证据必须通过 /ackermann_cmd 和 STM32 安全层
# 本包不定义阶段 3 扫图控制入口，不得把 legacy/manual /cmd_vel 控制当作 PASS 证据

# 4. 保存地图
ros2 launch autoracer_robot_nav2 save_map.launch.py \
  map_name:=<map_name> \
  output_dir:=artifacts/<run-id>/maps
```

`output_dir` 应指向测试记录的正式 artifacts 目录；不要把阶段 3 地图成果依赖在
`install/share` 或包内默认 map 目录中。

### 导航流程

```bash
# 1. 按 docs/启动与运行规范.md 启动 phase-1 Ackermann 底盘链路和 LiDAR
ros2 launch lslidar_driver lslidar_cx_launch.py

# 2. 启动当前过渡 Nav2 入口（加载地图）；不能作为阶段 4 稳定版 PASS 证据
ros2 launch autoracer_robot_nav2 navigation.launch.py map:=/path/to/autoracer_map.yaml

# 3. 在 RViz2 中设置初始位姿和目标点
```

阶段 4 稳定版入口：

```bash
ros2 launch autoracer_bringup stage4_navigation.launch.py map:=/path/to/map.yaml counts_per_meter:=<实测值>
```

离线契约检查：

```bash
python3 tools/acceptance/stage4_navigation_contract_check.py
```

旧 `turn_on_autoracer_robot.launch.py` 默认路径属于 legacy/default bringup，不能作为阶段验收入口。

## AutoRacer 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| min_turning_radius | 2.24m | 最小转弯半径 |
| motion_model | Ackermann | 阶段 4A 通过 Smac Dubin + RPP + adapter 实现 forward-only |
| planner | SmacPlannerHybrid | 阶段 4A 使用 Dubin/forward-only |
| controller | Regulated Pure Pursuit | Nav2 内部输出 `/nav2_cmd_vel` |
| adapter | `twist_to_ackermann` | `/safe_nav2_cmd_vel -> /ackermann_cmd` |
| footprint | 0.8775m × 0.57m | 车身碰撞轮廓 |

## 参考

- `reference/wheeltec_ros2/src/wheeltec_robot_nav2/`
- `reference/wheeltec_ros2/src/wheeltec_robot_nav2/param/wheeltec_params/param_top_akm_bs.yaml`
