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

## 本次任务
P0 Phase D: URDF 模型创建

## 任务步骤
1. 研究 Wheeltec URDF 参考：
   - 阅读 `reference/wheeltec_ros2/src/wheeltec_robot_urdf/` 结构
   - 分析 `urdf/top_akm_bs_robot.urdf`（Ackermann 底盘模型）
   - 理解 `robot_mode_description.launch.py` 的 robot_state_publisher 集成方式

2. 创建 `autoracer_robot_urdf` 包：
   - 使用 ament_cmake 构建
   - 包含 `urdf/`, `meshes/`（如需要）, `launch/` 目录
   - 参考 Wheeltec 包结构

3. 创建 AutoRacer URDF 模型（`autoracer.urdf.xacro`）：
   - 底盘：box（0.85m×0.50m×0.40m）
   - 4 个车轮：cylinder（半径 0.11m）
   - base_footprint → base_link（Z=+0.11m）
   - base_link → [front_left/right_wheel, rear_left/right_wheel]
   - 传感器 link（laser, zed_camera_link）可选择内联或引用主 launch 的静态 TF
   - 使用 Xacro 宏定义车轮，减少重复代码

4. 参数配置：
   - 使用 CLAUDE.md 中的 Ackermann 参数：wheelbase=0.54m, track_width=0.48m, wheel_radius=0.11m
   - 使用测量的车辆尺寸：长=0.85m, 宽=0.50m, 高=0.40m
   - 前轴距车头=0.15m, 后轴距车尾=0.16m

5. 创建 Launch 文件（`robot_description.launch.py`）：
   - 加载 URDF 到参数服务器
   - 启动 robot_state_publisher
   - 启动 joint_state_publisher（带 GUI 选项）

6. 集成到主 bringup launch：
   - 更新 `turn_on_autoracer_robot.launch.py`
   - 添加 `use_urdf` 参数（默认 true）
   - 从静态 TF 迁移到 URDF（传感器 link）

7. 验证：
   - `ros2 run tf2_tools view_frames` 检查 TF 树
   - 在 RViz2 中添加 RobotModel 显示
   - 检查车轮、传感器位置是否正确

8. 更新文档：
   - PJINFO.md（已完成列表、重要命令）
   - CLAUDE.md（Run Commands、Development Status）
   - TODO.md（标记 Task 5 完成）
   - AGENTLOG.md（追加日志）

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
