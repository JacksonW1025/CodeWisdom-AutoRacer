# PROMPT.md

> 使用方式：  
> - `do 1 @PROMPT.md` = WORK：按“本次任务/步骤”执行  
> - `xxx（task）do 2 @PROMPT.md` = UPDATE：只更新“本次任务/步骤”（其余不动）,并非真正开始coding或者测试

## 固定流程（不更新）
1) 完整阅读 `PJINFO.md`  
2) 完整阅读 `AGENTLOG.md`  
3) 判断模式：do 1(work) / do 2(update)  
4) 严格遵循 PJINFO 的设计原则与注意事项

## 关键参考（一般不更新）
- 状态与结构：`PJINFO.md`
- 历史记录：`AGENTLOG.md`
- 常用命令/关键点：`CLAUDE.md`
- 参考资料：`reference/`
- 工作区/仓库根路径

## 本次任务（我在对话框输入或简单写在此处，UPDATE 会具体细化）
现在，参考wheeltec的gmapping功能，将它相同地迁移到autoracer，需要保证启动方式这些都完全一致。你需要完成：查看wheeltec的实现方式，选择迁移目录（和其它slam一致，应该为现存的autoracer_robot_slam)，注意所有已经在PJINFO文件中限制的开发规范，完成功能迁移并编译。

### 前置条件
- C32 LiDAR 驱动已集成（`/point_cloud_raw` PointCloud2, `/scan_raw` LaserScan）, but not running
- N300 Pro IMU 驱动已集成（`/imu/data_raw`）, but not running 
- 底盘驱动C63A尚未安装，不考虑运动，只考虑建图功能
- URDF/TF 已配置


## 任务步骤 （UPDATE 会具体细化，需要理解我的口头表述，将一个复杂的任务拆解）

### Step 1: 研究 wheeltec gmapping 实现
- 参考路径：`reference/wheeltec_ros2/src/wheeltec_robot_slam/`
- 涉及两个包：
  - `openslam_gmapping/` — 纯 C++ gmapping 核心算法库（无 ROS 依赖，粒子滤波 RBPF + 扫描匹配 + 栅格地图）
  - `slam_gmapping/` — ROS2 wrapper（`SlamGmapping` 节点，订阅 `/scan`，发布 `/map`，广播 `map→odom` TF）
- 确认 launch 文件结构：`slam_gmapping.launch.py` 同时启动 bringup + LiDAR + gmapping 节点
- 确认依赖：`std_msgs`, `nav_msgs`, `tf2`, `tf2_ros`, `message_filters`, `rclcpp`, `sensor_msgs`, `openslam_gmapping`

### Step 2: 复制 openslam_gmapping 核心库
- 复制 `reference/wheeltec_ros2/src/wheeltec_robot_slam/openslam_gmapping/` → `src/autoracer_robot_slam/openslam_gmapping/`
- 此包名为 `openslam_gmapping`，不含 "wheeltec" 字样，**保持原名不改**
- 检查 `package.xml` 和 `CMakeLists.txt`，移除无效依赖（参考 lslidar 迁移经验）

### Step 3: 复制 slam_gmapping ROS2 wrapper
- 复制 `reference/wheeltec_ros2/src/wheeltec_robot_slam/slam_gmapping/` → `src/autoracer_robot_slam/slam_gmapping/`
- 此包名为 `slam_gmapping`，不含 "wheeltec" 字样，**保持原名不改**
- 检查 `package.xml` 和 `CMakeLists.txt`，确保依赖正确

### Step 4: 适配 launch 文件
- **重写** `slam_gmapping/launch/slam_gmapping.launch.py`，与 autoracer 现有模式保持一致：
  - 参考 `autoracer_slam_toolbox/launch/slam.launch.py` 的模式
  - **不包含** bringup 和传感器驱动启动（与 LIO-SAM、SLAM Toolbox 一致，用户自行在其它终端启动）
  - 包含 `pointcloud_to_laserscan` 节点（C32 `/point_cloud_raw` PointCloud2 → `/scan` LaserScan，高度 0.1-1.5m）
  - 包含 `slam_gmapping` 节点
  - 包含可选的 RViz2（`use_rviz` 参数）
- 关键参数适配：
  - `base_frame_` = `base_footprint`（与 autoracer TF 树一致）
  - `odom_frame_` = `odom`（autoracer 当前使用 `odom`，EKF 启用后可切换为 `odom_combined`）
  - `map_frame_` = `map`
  - 输入话题：`/scan`（来自 pointcloud_to_laserscan 转换）
- 添加 `odom_frame` launch 参数（默认 `odom`，可切换为 `odom_combined`）

### Step 5: 编译验证
- `colcon build --packages-select openslam_gmapping slam_gmapping --symlink-install --parallel-workers 2`
- 修复编译错误（如有）
- 验证节点可执行：`ros2 run slam_gmapping slam_gmapping --ros-args --help`

### Step 6: 创建 README.md
- 在 `src/autoracer_robot_slam/slam_gmapping/` 中创建简洁的中文 README
- 说明包功能、启动方式、参数配置

### Step 7: 更新项目文档
- `PJINFO.md`：更新【自动】段落（项目结构、ROS2 packages、启动方式、已完成状态、重要命令）
- `AGENTLOG.md`：追加 Entry 20
- `CLAUDE.md`：添加 gmapping 编译/运行命令
- `TODO.md`：标记 GMapping 为已完成



## WORK 完成后必须更新
1) `PJINFO.md`：只更新【自动】段落；“重要命令”只增不删  
2) `AGENTLOG.md`： 在仓库根目录 `AGENTLOG.md` 末尾追加本次工作日志，格式：
```
---
## Entry N: <Title>

- Date: YYYY-MM-DD
- Agent: <Codex/Claude Code/...>
- Summary:
  - <要点，3-8条>
- Modified/Created files:
  - `<file_path>`: <what changed>
- How to run:
  - `<command>`
- Notes / Next:
  - <下一步建议>
```
3) `CLAUDE.md`：更新最新关键点
4) `TODO.md`
