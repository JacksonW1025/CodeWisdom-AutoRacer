# PROMPT.md

> 使用方式：  
> - `do 1 @PROMPT.md` = WORK：按“本次任务/步骤”执行  
> - `xxx（task）do 2 @PROMPT.md` = UPDATE：只更新“本次任务/步骤”（其余不动）

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

将 LIO-SAM（LiDAR-Inertial Odometry via Smoothing and Mapping）集成到 AutoRacer 工作空间，实现 3D LiDAR+IMU 紧耦合 SLAM 建图与定位。

## 关键参考

- **Wheeltec LIO-SAM**: `reference/wheeltec_ros2/src/wheeltec_robot_slam/LIO-SAM-ROS2/`
- **默认配置**: `config/params.yaml`（N_SCAN=32, Velodyne 格式）
- **Wheeltec 适配配置**: `config/wheeltec_params.yaml`（N_SCAN=16, GPS 集成）
- **Launch 文件**: `run.launch.py`（基础）、`run_gnss.launch.py`（含 GPS）
- **论文**: Shan et al., IROS 2020

## 任务步骤 （UPDATE 会具体细化，需要理解我的口头表述，将一个复杂的任务拆解）

### Step 0: 前置依赖检查与安装
- 检查 GTSAM 是否已安装：`dpkg -l | grep gtsam` 或 `find /usr -name "GTSAMConfig.cmake" 2>/dev/null`
- 如未安装，**仅通过 apt 安装** GTSAM（LIO-SAM 核心依赖，因子图优化库）：
  - `sudo apt install ros-humble-gtsam` 或 `sudo apt install libgtsam-dev`
  - **不从源码编译**：如果 apt 安装失败，立即停止任务并向用户反馈问题，等待用户决策
- 检查其他依赖：PCL、OpenCV、Eigen（大概率已安装）

### Step 1: 复制 LIO-SAM 到 src/ 目录
- 创建 `src/autoracer_robot_slam/` 目录（对应 Wheeltec 的 `wheeltec_robot_slam/`，方便后续加入 Cartographer、SLAM Toolbox 等其他建图算法）
- 从 `reference/wheeltec_ros2/src/wheeltec_robot_slam/LIO-SAM-ROS2/` 复制到 `src/autoracer_robot_slam/LIO-SAM-ROS2/`
- 按照命名原则：保持原名 `lio_sam`（非 wheeltec 专有命名，无需改名）
- 排除不必要的文件：`config/doc/`（文档图片）

### Step 2: 创建 AutoRacer 专用配置文件
- 创建 `config/autoracer_params.yaml`，基于 `params.yaml` 适配 AutoRacer 硬件：
  - **LiDAR 配置**：
    - `pointCloudTopic: "/point_cloud_raw"`（lslidar_driver 发布的话题）
    - `sensor: velodyne`（C32 使用类 Velodyne 点云格式，需验证）
    - `N_SCAN: 32`（C32 = 32 通道）
    - `Horizon_SCAN: 1500`（待确认 C32 水平分辨率）
    - `lidarMinRange: 1.0`、`lidarMaxRange: 200.0`
  - **IMU 配置**：
    - `imuTopic: "/imu/data_raw"`（N300 Pro 发布的话题）
    - IMU 噪声参数（从 N300 Pro / HI13 数据手册获取，或使用 Wheeltec 默认值先跑通）
    - `imuGravity: 9.80511`
  - **Frame ID 配置**：
    - `lidarFrame: "laser"`
    - `baselinkFrame: "base_footprint"`
    - `odometryFrame: "odom"`
    - `mapFrame: "map"`
  - **IMU→LiDAR 外参**：
    - 根据 AutoRacer TF 树计算：IMU(gyro_link) → LiDAR(laser) 的变换
    - gyro_link 位于 base_footprint (0,0,0)，laser 位于 base_link (X=+0.24, Z=+0.39+0.11=+0.50)
    - `extrinsicTrans: [0.24, 0.0, 0.50]`（从 IMU 到 LiDAR 的平移）
    - `extrinsicRot` 和 `extrinsicRPY`：需考虑 LiDAR yaw=-90° 旋转
  - **性能配置**：
    - `numberOfCores: 4`（Orin NX 多核）
    - `mappingProcessInterval: 0.15`

### Step 3: 验证 LiDAR 点云字段兼容性（关键！）
- LIO-SAM **必须**要求点云包含 `time`（每个点的相对时间戳）和 `ring`（通道编号）字段
- 检查 lslidar_driver 发布的 PointCloud2 消息字段：
  - 命令：`ros2 topic echo /point_cloud_raw --field fields --once`
- 如果缺少 `time` 或 `ring` 字段：
  - 检查 `lslidar_cx.yaml` 是否有相关配置选项
  - 可能需要修改 LIO-SAM 的 `imageProjection.cpp` 适配 lslidar 点云格式
  - 或编写一个简单的点云格式转换节点

### Step 4: 编译 LIO-SAM
- 命令：`colcon build --packages-select lio_sam --symlink-install --parallel-workers 2`
- 编译自定义消息 `CloudInfo.msg` 和服务 `SaveMap.srv`
- 预期：编译通过，生成 5 个可执行节点

### Step 5: 创建 AutoRacer LIO-SAM Launch 文件
- 创建 `launch/autoracer_run.launch.py`：
  - 加载 `autoracer_params.yaml`
  - 启动 4 个核心节点：imageProjection、featureExtraction、imuPreintegration、mapOptimization
  - 添加 map→odom 静态 TF（初始阶段）
  - **不包含**机器人 bringup 和传感器驱动（由外部分别启动）
- 后续可创建 `autoracer_run_gnss.launch.py`（G90 GNSS 集成后）

### Step 6: 集成测试
- **测试流程**（需多个终端）：
  1. 终端 1：`ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py`（底盘 + URDF + TF + IMU）
  2. 终端 2：`ros2 launch lslidar_driver lslidar_cx_launch.py`（LiDAR 驱动）
  3. 终端 3：`ros2 launch lio_sam autoracer_run.launch.py`（LIO-SAM）
  4. 终端 4：`rviz2`（可视化地图、轨迹、点云）
- **验证要点**：
  - LIO-SAM 节点全部正常启动，无 crash
  - IMU 预积分正常（检查 `/odometry/imu` 话题）
  - 地图优化正常（检查 `/lio_sam/mapping/map_global` 话题）
  - TF 树：map → odom → base_footprint 链完整

### 验证要点
- [ ] GTSAM 安装成功，`lio_sam` 包编译通过
- [ ] lslidar C32 点云包含 `time` 和 `ring` 字段（或已适配）
- [ ] IMU→LiDAR 外参配置正确（extrinsicTrans/Rot/RPY）
- [ ] LIO-SAM 5 个节点正常启动，无段错误
- [ ] `/lio_sam/mapping/odometry` 话题正常发布
- [ ] RViz2 中可视化建图效果（点云地图 + 轨迹）

### 已知风险与注意事项
- **LiDAR 硬件一致**: Wheeltec 原车使用的 LiDAR 型号与 AutoRacer 完全一致（镭神 C32），因此 LiDAR 硬件层面无差异，驱动和点云格式可直接复用，Wheeltec 的 LIO-SAM 配置对 LiDAR 部分具有高参考价值
- **IMU 采样率**: N300 Pro ~100Hz，LIO-SAM 推荐 200Hz+，可能影响去畸变精度，需观察效果
- **GTSAM 版本**: 确保 GTSAM 版本与 LIO-SAM 兼容（推荐 4.0.3 或 4.1.x）
- **内存占用**: LIO-SAM 在 Jetson 上可能内存紧张，注意监控 `free -h`



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
