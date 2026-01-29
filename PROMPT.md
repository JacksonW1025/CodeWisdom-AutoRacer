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
**LiDAR C32 点云与车辆朝向 180° 旋转错误修复**

现象：运行 URDF/TF + LiDAR 驱动后，在 RViz2 中观察到车辆后方对应的是实际前方的点云（即点云整体旋转了 180°）。需要排查 LiDAR 坐标系定义与 TF 中 yaw 旋转角的配置是否正确。

## 任务步骤 （UPDATE 会具体细化，需要理解我的口头表述，将一个复杂的任务拆解）

### 步骤 1：阅读镭神 C32 用户手册，确认 LiDAR 自身坐标系定义
- 阅读 `reference/docs/1.镭神智能_C32_用户手册_V2.7.8_20241015.pdf`
- 重点关注：坐标系定义（X/Y/Z 轴方向）、0° 角方向（通常为线缆出口方向或前面板方向）
- 记录 LiDAR 原生坐标系与 ROS REP-103 标准坐标系（X前 Y左 Z上）的差异

### 步骤 2：检查 LiDAR 驱动的坐标系输出配置
- 阅读 `src/autoracer_lidar_ros2/lslidar_ros/lslidar_driver/` 驱动代码
- 检查 `params/lslidar_cx.yaml` 中的 frame_id、角度偏移等配置参数
- 确认驱动是否已对原始数据做了坐标系变换（部分驱动会在内部完成旋转）

### 步骤 3：检查当前 TF 配置中 LiDAR 的 yaw 旋转
- 阅读 `src/autoracer_robot_urdf/urdf/autoracer.urdf.xacro` 中 `laser` link 的 joint 定义
- 当前配置：base_link → laser，yaw=+90°（+1.5708 rad）
- 分析：当前 yaw=+90° 的假设是 LiDAR 坐标系为"X左 Y后 Z上"，如果手册显示不同则需修正

### 步骤 4：确定正确的旋转角度并修复
- 根据步骤 1-3 的分析结果，计算正确的 yaw 旋转值
- 可能的情况：
  - 如果 LiDAR 0° 方向指向线缆方向（后方），可能需要 yaw=0° 或 yaw=180°
  - 如果驱动已做了坐标变换，可能 yaw=+90° 本身多余
- 修改 URDF 中 laser joint 的 rpy 参数
- 同步修改 `turn_on_autoracer_robot.launch.py` 中 `use_urdf:=false` 回退路径的静态 TF

### 步骤 5：验证修复结果
- 重新编译 `autoracer_robot_urdf` 包
- 启动 LiDAR 驱动 + URDF/TF，在 RViz2 中检查点云是否与车辆朝向一致
- 验证方式：面对车辆前方的障碍物应在 RViz2 中显示在车辆模型前方, wait user to check. ask the user if can continue

### 步骤 6：更新项目文档
- 更新 `PJINFO.md`、`AGENTLOG.md`、`CLAUDE.md`、`TODO.md`

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
