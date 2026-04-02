# AutoRacer vs Wheeltec 全仓迁移阶段性 Review 报告

- 日期: 2026-04-01
- 分支: `nav`
- 评审方式: 静态代码对比 + 关键包定向编译 + Python launch 语法校验 + URDF 生成校验

## 1. 总体结论

本次 review 只覆盖仓库中**已经迁移或已经接入运行链路**的模块，不把 `TODO.md` 中明确尚未迁移的功能直接算作实现错误。

结论如下:

1. **没有发现“整体路线严重跑偏、需要推倒重做”的大偏差。** AutoRacer 当前已迁移的核心栈，整体仍沿着 Wheeltec 的实现思路在收敛，尤其是串口底盘、LiDAR、IMU、URDF、LIO-SAM、SLAM Toolbox、GMapping、Nav2、ZED 这几条主线，目录结构和接口命名基本可追溯到参考实现。
2. **存在若干会影响默认运行路径的高风险偏差。** 其中最关键的是 `turn_on_autoracer_robot` 的默认 bringup 没有把 EKF 接入主链路，Madgwick 参数文件与节点名不匹配，IMU 滤波输出链路与 EKF 输入链路描述不一致。
3. **存在一批残留 Wheeltec 旧入口。** 这些旧入口不一定影响当前默认 AutoRacer 入口，但会导致同一 package 内出现“新入口可用、旧入口仍指向 `turn_on_wheeltec_robot`”的混合状态，尤其在 `lio_sam` 包中最明显。
4. **当前阶段更准确的状态应为: 架构方向基本正确，但默认集成完成度不足。** 如果把“是否已完全达到 Wheeltec 同等级可直接使用状态”作为标准，当前答案是否定的。

## 2. 覆盖范围

本次实际覆盖的已迁移/已接入模块如下:

| 模块 | 参考来源 | 结论 |
|---|---|---|
| `turn_on_autoracer_robot` | `turn_on_wheeltec_robot` | **存在高风险偏差** |
| `autoracer_interfaces` | `wheeltec_ultrasonic_avoid/robot_interfaces` | 基本一致 |
| `autoracer_keyboard` | `wheeltec_robot_keyboard` | 基本一致，做了合理精简 |
| `depend/serial_ros2` | `depend/serial_ros2` | 未见明显偏差 |
| `autoracer_lidar_ros2/lslidar_ros` | `wheeltec_lidar_ros2/lslidar_ros` | 基本一致，主要是配置改写 |
| `hipnuc_imu` | `wheeltec_imu/hipnuc_imu` | 基本一致，已适配 AutoRacer 话题/设备名 |
| `autoracer_imu_tf_broadcaster` | `WHEELTEC_N300Pro/imu_tf_broadcaster` | 基本一致，且本地实现更稳健 |
| `autoracer_robot_urdf` | `wheeltec_robot_urdf` | 方向正确，无明显结构性错误 |
| `lio_sam` | `wheeltec_robot_slam/LIO-SAM-ROS2` | 默认入口已适配，但保留失效旧入口 |
| `autoracer_slam_toolbox` | `wheeltec_slam_toolbox` | 主链路已接通，RViz 选择不理想 |
| `openslam_gmapping` | `wheeltec_robot_slam/openslam_gmapping` | 基本一致 |
| `slam_gmapping` | `wheeltec_robot_slam/slam_gmapping` | 主链路已接通，RViz 选择不理想 |
| `autoracer_robot_nav2` | `wheeltec_robot_nav2` | 已做 AutoRacer 定制，未见明显大偏差 |
| `zed-ros2-wrapper` | `reference/zed-ros2-wrapper` | 与官方参考基本一致 |
| `zed_display_rviz2` | `reference/zed-ros2-examples/zed_display_rviz2` | 与参考一致 |

明确**不计入本次错误项**的内容:

- `TODO.md` 中仍标记为未迁移/未完成的 GNSS、超声避障、手柄、Cartographer、RTAB-Map、ORB-SLAM2、LeGO-LOAM 等功能。
- 参考目录中存在、但 AutoRacer 当前并未承诺已完成的 Wheeltec 扩展功能。

## 3. 对比矩阵与阶段性判断

### 3.1 已确认与参考实现基本一致的部分

1. `autoracer_interfaces` 与 `robot_interfaces` 的消息/服务体定义实质一致，`Supersonic.msg`、`SetRgb.srv` 的差异仅为换行和包层包装，不构成功能偏差。
2. `autoracer_keyboard` 是对 Wheeltec 键盘控制的定向精简版，主要去掉了全向底盘模式，改为更贴合 Ackermann/小车场景的控制逻辑，属于合理适配，不是偏差错误。
3. `autoracer_lidar_ros2/lslidar_ros` 与 Wheeltec 的 `lslidar_ros` 差异主要集中在 `package.xml` 和 `lslidar_cx.yaml`，未看到明显实现分叉。
4. `zed_display_rviz2` 与 `reference/zed-ros2-examples/zed_display_rviz2` 对比无差异。
5. `zed-ros2-wrapper` 与 `reference/zed-ros2-wrapper` 的差异仅见于上游附带的文档/素材目录，源码侧没有明显本地偏移。
6. `openslam_gmapping` 与 Wheeltec 参考实现保持一致；`slam_gmapping` 核心源码只做了 `odom_frame_` 从 `odom_combined` 改到 `odom` 的 AutoRacer 适配，这与项目说明一致，不构成偏差。

### 3.2 已适配成功、但集成层仍有明显缺口的部分

1. `turn_on_autoracer_robot` 已完成串口节点、IMU remap、URDF/TF 接入，但默认 bringup 没有真正把 EKF 融合接入主链路。
2. `lio_sam` 已新增 `autoracer_run.launch.py` 和 `autoracer_params.yaml`，默认 AutoRacer 入口基本成型，但包内仍混杂多个旧 Wheeltec launch。
3. `autoracer_slam_toolbox`、`slam_gmapping` 都已完成 3D 点云转 2D `/scan` 的链路拼装，但一键启动使用的 RViz 配置没有切到专门的 `slam.rviz`。
4. `autoracer_robot_nav2` 已改成 AutoRacer 风格的一体化 launch，支持 `slam`/定位双模式，未见重大设计偏差。

## 4. Findings

### P0

#### P0-1 默认 bringup 未把 EKF 融合接入主链路

风险判断:

- `TODO.md` 与 `PJINFO.md` 都把 “IMU + EKF 融合完成” 作为已完成能力描述；
- 但默认入口 `ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py` 并不会启动 `robot_localization` 的 EKF 节点；
- 这意味着默认 bringup 只有底盘里程计、外置 IMU、Madgwick、URDF/TF，而没有融合后的 `/odom_combined`。

证据:

- `src/turn_on_autoracer_robot/launch/turn_on_autoracer_robot.launch.py:205-228`
  当前 `LaunchDescription` 只加入了串口节点、`hipnuc_imu`、Madgwick、URDF、静态 TF，没有任何 `autoracer_ekf.launch.py` 或 `ekf_node`。
- `src/turn_on_autoracer_robot/launch/autoracer_ekf.launch.py:21-40`
  EKF 入口单独存在，但没有被默认 bringup 引用。
- `PJINFO.md:133-145`
  当前文档仍把完整启动入口描述为主 bringup。

影响:

- 默认主入口无法产出文档所宣称的融合里程计链路；
- 依赖 `/odom_combined` 的后续链路只能靠用户手动补启动；
- 当前实现状态和“已完成 EKF 集成”的说法不一致。

#### P0-2 Madgwick 参数文件根节点名与实际节点名不匹配

风险判断:

- 当前 launch 中启动的节点名是 `imu_filter_madgwick`；
- 但参数文件根节点名写的是 `imu_filter_madgwick_node`；
- 在 ROS 2 参数文件按 node name 绑定的常规用法下，这会导致配置文件可能无法命中当前节点。

证据:

- `src/turn_on_autoracer_robot/launch/turn_on_autoracer_robot.launch.py:110-123`
  节点启动为 `name='imu_filter_madgwick'`，参数直接加载 `config/imu.yaml`。
- `src/turn_on_autoracer_robot/config/imu.yaml:6-12`
  参数文件根键是 `imu_filter_madgwick_node:`。

影响:

- `fixed_frame`、`use_mag`、`publish_tf`、`world_frame`、`orientation_stddev` 等预期参数可能未实际生效；
- 该问题不会在编译阶段暴露，但会在运行时静默退回默认参数，属于高风险集成错误。

#### P0-3 IMU 滤波输出链路与 EKF 输入链路自相矛盾

风险判断:

- `imu.yaml` 注释说明 Madgwick 输出 `/imu/data`；
- `ekf.yaml` 却配置 `imu0: /imu/data_raw`；
- 再叠加 P0-1，当前默认 bringup 实际上既没有完整 EKF 主链路，也没有把滤波后的 IMU 明确接给 EKF。

证据:

- `src/turn_on_autoracer_robot/config/imu.yaml:4-5`
  注释明确写“输出: `/imu/data`”。
- `src/turn_on_autoracer_robot/config/ekf.yaml:47-60`
  EKF 输入配置为 `imu0: /imu/data_raw`。
- `src/turn_on_autoracer_robot/launch/turn_on_autoracer_robot.launch.py:100-123`
  只启动了 `hipnuc_imu` 和 Madgwick，没有任何把滤波输出接入 EKF 的主链路动作。

影响:

- 当前 IMU 滤波节点的存在价值在默认入口中并不闭环；
- 文档层表达的是“滤波后用于 EKF”，实际实现不是这样。

### P1

#### P1-1 `lio_sam` 包内仍保留多个直接指向 Wheeltec 的旧 launch 入口

风险判断:

- 默认 AutoRacer 入口 `autoracer_run.launch.py` 已经做了本地化适配；
- 但同一 package 内仍保留多个会直接引用 `turn_on_wheeltec_robot`、`wheeltec_params.yaml`、`wheeltec_sensors_liosam.launch.py` 的 launch；
- 这些入口对当前仓库来说属于**显式失效入口**。

证据:

- `src/autoracer_robot_slam/LIO-SAM-ROS2/launch/run.launch.py:24-31`
  直接 `get_package_share_directory('turn_on_wheeltec_robot')`。
- `src/autoracer_robot_slam/LIO-SAM-ROS2/launch/run_gnss.launch.py:16-29`
  默认参数仍是 `wheeltec_params.yaml`，并 include `wheeltec_sensors_liosam.launch.py`。
- `src/autoracer_robot_slam/LIO-SAM-ROS2/launch/include/turn_on_wheeltec_robot_lio.launch.py:18-24`
  继续引用 `turn_on_wheeltec_robot` 和其配置目录。

影响:

- 对熟悉上游 LIO-SAM 的用户来说，包内同时存在“可用的新入口”和“会直接失败的旧入口”；
- 这不是默认 AutoRacer 主链路失效，但属于明显未清理完的迁移残留。

结论:

- 该问题应认定为 **残留旧入口未清理**，不是 LIO-SAM 核心算法本身跑偏。

#### P1-2 2D SLAM 一键入口使用的 RViz 配置不匹配 `/scan` 链路

风险判断:

- `autoracer_slam_toolbox` 和 `slam_gmapping` 都在 launch 中把 `/point_cloud_raw` 转成 `/scan`；
- 但两者启动的 RViz 配置文件是 `autoracer.rviz`，而不是专门的 `slam.rviz`；
- `autoracer.rviz` 的 LaserScan 订阅仍是 `/scan_raw`，这会让 2D SLAM 一键启动时的默认可视化不准确。

证据:

- `src/autoracer_robot_slam/autoracer_slam_toolbox/launch/slam.launch.py:85-95`
  RViz 指向 `autoracer_robot_urdf/rviz/autoracer.rviz`。
- `src/autoracer_robot_slam/slam_gmapping/launch/slam_gmapping.launch.py:79-89`
  同样指向 `autoracer_robot_urdf/rviz/autoracer.rviz`。
- `src/autoracer_robot_urdf/rviz/autoracer.rviz:58-68`
  LaserScan 显示订阅 `/scan_raw`。
- `src/autoracer_robot_urdf/rviz/slam.rviz:59-69`
  专门的 SLAM 视图订阅 `/scan`。

影响:

- 不影响 `/scan` 生成和 SLAM 节点本身运行；
- 但会导致“一键启动后默认 RViz 视图与实际 SLAM 主观测链路不一致”，属于可用性偏差。

### P2

#### P2-1 `autoracer_params.yaml` 当前是文档宣称的配置入口，但实际上未被主链路消费

风险判断:

- `PJINFO.md` 把 `src/turn_on_autoracer_robot/config/autoracer_params.yaml` 列为配置入口；
- 但两个主 launch 都直接以内联参数方式启动节点，没有加载该 YAML；
- 节点源码也只声明了少量串口和 frame 参数，没有消费 YAML 中额外的里程计标定/底盘几何参数。

证据:

- `src/turn_on_autoracer_robot/config/autoracer_params.yaml:4-26`
  文件中定义了串口、frame、标定和 Ackermann 参数。
- `src/turn_on_autoracer_robot/launch/autoracer_serial.launch.py:52-64`
  仅以内联参数启动节点。
- `src/turn_on_autoracer_robot/launch/turn_on_autoracer_robot.launch.py:89-97` 与 `:132-137`
  同样以内联参数启动。
- `src/turn_on_autoracer_robot/src/autoracer_robot.cpp:333-345`
  节点只声明/读取 `usart_port_name`、`serial_baud_rate`、`odom_frame_id`、`robot_frame_id`、`gyro_frame_id`。

影响:

- 当前 `autoracer_params.yaml` 更像“预备配置文件”而不是实际生效配置入口；
- 文档说法与代码真实行为不完全一致。

#### P2-2 项目文档对“已完成度”的表述略超前于默认运行链路真实状态

风险判断:

- `TODO.md` 和 `PJINFO.md` 已把 EKF、LIO-SAM、2D SLAM、Nav2 写入“完成/可用”状态；
- 从代码看，这些能力大方向确实都已经落地；
- 但默认入口、滤波闭环、RViz 视图、旧入口清理仍存在明显未完成项。

结论:

- 这更像“阶段性里程碑文档写早了一步”，不是核心设计错误；
- 但如果继续对外宣称“已基本对齐 Wheeltec”，容易造成使用预期偏差。

## 5. 参考残留但暂不判定为错误的部分

以下内容在仓库里仍然能看到 Wheeltec 痕迹，但本次**不直接判定为实现错误**:

1. `lio_sam` 包中保留 `params.yaml`、`wheeltec_params.yaml`、若干旧 launch。
   原因: AutoRacer 已经另建 `autoracer_run.launch.py` 和 `autoracer_params.yaml` 作为新入口；旧文件更像迁移残留和参考保留。
2. 注释/README/PJINFO 中大量出现 “参考 Wheeltec”。
   原因: 参考来源说明本身不是问题，关键看默认入口是否真正完成 AutoRacer 适配。
3. `slam_gmapping` 核心源码把 `odom_frame_` 改为 `odom`。
   原因: 这与当前 AutoRacer 默认里程计链路一致，也符合项目内既有说明。

## 6. 与 Wheeltec 的一致性结论

### 一致性较高

- 串口协议总体形态仍沿用 Wheeltec/C63A 风格，底盘节点接口保持 `cmd_vel`/`odom`/`imu` 的主线一致。
- `lslidar_ros`、`openslam_gmapping`、`zed_display_rviz2`、`zed-ros2-wrapper` 基本保持参考实现形态。
- `hipnuc_imu`、`autoracer_imu_tf_broadcaster` 的本地改写没有表现出方向性错误，反而比原参考实现更贴近当前仓库使用方式。

### 一致性不足

- 默认 `turn_on_autoracer_robot` bringup 没有把 Wheeltec 风格的 IMU+滤波+EKF 主链路闭合起来。
- `lio_sam` 包内仍然处于“AutoRacer 新入口 + Wheeltec 旧入口并存”的半迁移状态。
- 2D SLAM 的 launch 与 RViz 视图没有完全切换到 `/scan` 这条 AutoRacer 新链路。

## 7. 已执行验证命令

以下命令已执行，结果用于支撑本报告:

```bash
colcon build --packages-select \
  turn_on_autoracer_robot \
  autoracer_robot_urdf \
  autoracer_imu_tf_broadcaster \
  hipnuc_imu \
  openslam_gmapping \
  slam_gmapping \
  autoracer_slam_toolbox \
  lio_sam \
  autoracer_robot_nav2
```

结果:

- 9 个目标包全部编译通过。
- `lio_sam` 与 `slam_gmapping` 有 warning，但没有 build failure。

```bash
python3 -m py_compile $(rg --files src/turn_on_autoracer_robot src/autoracer_robot_urdf src/autoracer_imu_tf_broadcaster src/autoracer_robot_slam -g '*.py')
```

结果:

- 相关 Python launch/node 语法校验通过。

```bash
xacro src/autoracer_robot_urdf/urdf/autoracer.urdf.xacro >/tmp/autoracer.urdf
```

结果:

- URDF 可正常展开。

除此之外，还执行了多组 `diff -rq`、`diff -u`、`rg -n` 来确认:

- AutoRacer 与 Wheeltec 对应模块的目录差异；
- 是否残留 `turn_on_wheeltec_robot`、`wheeltec_*` 入口；
- RViz 配置是否匹配 `/scan`、`/scan_raw`、`/point_cloud_raw` 等实际链路；
- `zed_display_rviz2` 与 `reference/zed-ros2-examples/zed_display_rviz2` 是否一致；
- `zed-ros2-wrapper` 与 `reference/zed-ros2-wrapper` 是否存在源码级明显偏移。

## 8. 建议后续动作

建议按下面顺序收敛:

1. 先修 `turn_on_autoracer_robot` 默认 bringup:
   把 EKF 接入主链路，统一 IMU filter 输入/输出与 EKF 的 topic 设计。
2. 修正 `imu.yaml` 根节点名:
   保证 Madgwick 参数文件必然命中当前 launch 启动的节点。
3. 清理 `lio_sam` 旧入口:
   至少把仍会直接引用 `turn_on_wheeltec_robot` 的 launch 标为 deprecated，或直接改成 AutoRacer 版本。
4. 把 2D SLAM 一键入口的 RViz 配置切到 `slam.rviz`:
   让默认可视化与 `/scan` 主链路一致。
5. 再处理文档对齐:
   包括 `PJINFO.md`、`TODO.md`、README、配置入口说明。

---

## 阶段性结论

**AutoRacer 当前迁移工作不存在“实现方式严重偏离 Wheeltec”的总体性错误；问题主要集中在默认入口闭环、参数绑定、旧入口清理和可视化细节。**

如果以“方向是否正确”判断，本项目当前回答是 **正确**；
如果以“是否已经达到可直接替代 Wheeltec 默认使用体验”判断，当前回答是 **还没有**。
