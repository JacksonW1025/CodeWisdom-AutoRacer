# LIO-SAM AutoRacer TODO

> 记录 LIO-SAM 在 AutoRacer 平台上的已知问题、调参方向和后续优化计划。
> 更新日期：2026-03-03

## 1. 已知问题

### 1.1 面特征不足 (surfFeatureMinValidNum)

- **现象**：`mapOptimization` 持续报警 `Not enough features! Only ~100 edge and ~90 planar features available.`
- **当前阈值**：`surfFeatureMinValidNum: 100`，实际面特征 ~86-100，频繁低于阈值导致帧被丢弃
- **原因**：2026-03-03 首次测试在狭小杂乱的室内环境，不符合 LIO-SAM 算法对结构化环境的预期
- **影响**：大量帧被跳过，建图和里程计精度下降
- **解决方向**：
  - [ ] 到结构化环境（走廊、停车场、室外道路）重新测试，确认特征数是否充足
  - [ ] 如结构化环境仍不足，考虑降低 `surfFeatureMinValidNum`（100 → 50）和 `edgeFeatureMinValidNum`（10 → 5）
  - [ ] 调整 `surfThreshold`（当前 0.1）或 `edgeThreshold`（当前 1.0）以提取更多特征

### 1.2 静止漂移

- **现象**：车辆静止时 LIO-SAM 里程计仍有漂移（修正外参后：x=-0.08m, y=-0.39m, z=0.27m）
- **原因**：
  1. 室内杂乱环境导致扫描匹配不稳定
  2. N300 Pro IMU ~100Hz 低于 LIO-SAM 推荐的 200Hz+，预积分精度受限
  3. IMU 噪声参数使用 Wheeltec 默认值，未针对 N300 Pro 实际标定
- **解决方向**：
  - [ ] 到结构化环境重新测试，排除环境因素后再评估漂移量
  - [ ] 如漂移仍然显著，进行 IMU Allan 方差标定，更新噪声参数（`imuAccNoise`、`imuGyrNoise`、`imuAccBiasN`、`imuGyrBiasN`）
  - [ ] 考虑使用 [imu_utils](https://github.com/gaowenliang/imu_utils) 或静态数据统计法标定

### 1.3 Madgwick 滤波器未接收数据

- **现象**：`imu_filter_madgwick` 持续报 `Still waiting for data on topics imu/data_raw and imu/mag...`
- **原因**：可能是话题命名空间/remapping 问题，Madgwick 订阅的相对话题 `imu/data_raw` 与 N300 Pro 发布的绝对话题 `/imu/data_raw` 未匹配
- **影响**：不影响 LIO-SAM（LIO-SAM 直接订阅 `/imu/data_raw`），但 EKF 融合链路（Madgwick → `/imu/data` → EKF）不工作
- **解决方向**：
  - [ ] 检查 `turn_on_autoracer_robot.launch.py` 中 Madgwick 节点的话题 remapping 配置
  - [ ] 确认 Madgwick 是否订阅了被 remap 到 `/imu/data_board` 的 STM32 IMU 话题，而非 N300 Pro 的 `/imu/data_raw`

## 2. 已修正问题

### 2.1 IMU→LiDAR 旋转外参（2026-03-03 已修正）

- **问题**：`extrinsicRot` 和 `extrinsicRPY` 使用单位阵，但 IMU 和 LiDAR 坐标系之间存在 +90° yaw 旋转
- **修正**：

  | 坐标系 | +X | +Y | +Z |
  |--------|----|----|-----|
  | IMU N300 Pro | 前方 | 左侧 | 上方 |
  | C32 LiDAR (`coordinate_opt=false`) | 右方 | 前方 | 上方 |

  ```yaml
  # 修正前（单位阵）
  extrinsicRot: [1, 0, 0, 0, 1, 0, 0, 0, 1]

  # 修正后（绕 Z 轴 +90°）
  extrinsicRot: [0, -1, 0, 1, 0, 0, 0, 0, 1]
  ```
- **效果**：静止 Z 方向漂移从 0.75m 降至 0.27m

## 3. 后续优化计划

### 3.1 环境测试

- [ ] 在走廊/停车场等结构化环境中测试建图质量
- [ ] 驱动车辆移动（需连接电机控制器），测试动态建图效果
- [ ] 录制 rosbag 用于离线回放调参

### 3.2 IMU 标定

- [ ] 采集 N300 Pro 静止状态数据（>2h），计算 Allan 方差
- [ ] 更新 `autoracer_params.yaml` 中的 IMU 噪声参数
- [ ] 参考工具：[imu_utils](https://github.com/gaowenliang/imu_utils)、[kalibr_allan](https://github.com/ori-drs/allan_variance_ros)

### 3.3 LiDAR-IMU 外参精标定

- [ ] 当前平移外参 `extrinsicTrans: [0.24, 0.0, 0.50]` 基于手工测量，精度有限
- [ ] 旋转外参基于坐标系理论推导，可能存在微小安装偏差
- [ ] 考虑使用 [lidar_imu_calib](https://github.com/chennuo0125-HIT/lidar_imu_calib) 进行自动标定

### 3.4 GNSS 融合

- [ ] 集成 G90 GNSS+RTK 后，启用 LIO-SAM GPS 因子（`gpsTopic`）
- [ ] 使用 `launch/run_gnss.launch.py` 或在 `autoracer_run.launch.py` 中添加 GPS odometry 节点
- [ ] 调整 `gpsCovThreshold` 和 `poseCovThreshold` 控制 GPS 因子频率

### 3.5 参数调优备忘

| 参数 | 当前值 | 说明 | 调整方向 |
|------|--------|------|----------|
| `surfFeatureMinValidNum` | 100 | 面特征最小数量 | 如特征不足可降至 50 |
| `edgeFeatureMinValidNum` | 10 | 边特征最小数量 | 一般不需调整 |
| `mappingProcessInterval` | 0.15s | 建图间隔 | Jetson 性能不足时增大 |
| `surroundingkeyframeAddingDistThreshold` | 1.0m | 关键帧距离阈值 | 小场景可降至 0.5 |
| `surroundingkeyframeAddingAngleThreshold` | 0.2rad | 关键帧角度阈值 | 小场景可降至 0.1 |
| `historyKeyframeFitnessScore` | 0.3 | 回环检测 ICP 阈值 | 越小越严格 |
| `imuRPYWeight` | 0.01 | IMU 姿态权重 | IMU 质量好可增大 |
