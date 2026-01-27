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
在 RViz2 中验证 ZED X 摄像头的可视化输出（RGB 图像、深度图、点云、IMU、Visual Odometry），参照 reference/zed-ros2-wrapper/README.md 中的 RViz visualization 章节。

## 任务步骤
1) 将 `reference/zed-ros2-examples/zed_display_rviz2/` 复制到 `src/` 下（不需要整个 zed-ros2-examples 仓库）
   ```bash
   cp -r ~/CodeWisdom-AutoRacer/reference/zed-ros2-examples/zed_display_rviz2 ~/CodeWisdom-AutoRacer/src/
   ```
2) 检查 `zed_display_rviz2` 的依赖，按需修改 `package.xml` / `CMakeLists.txt`（确保引用 `src/zed-ros2-wrapper` 中的包）
3) 构建 `zed_display_rviz2` 包
   ```bash
   cd ~/CodeWisdom-AutoRacer
   colcon build --packages-select zed_display_rviz2 --symlink-install --parallel-workers 2
   source install/setup.bash
   ```
4) 启动 ZED X 摄像头节点（如尚未运行）
   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
   ```
5) 启动 RViz2 预配置显示（新终端）
   ```bash
   ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx
   ```
   - 若构建失败，回退方案：手动 `rviz2`，手动添加以下 Display：
     - Image → topic: `/zed/zed_node/left/image_rect_color`
     - Image → topic: `/zed/zed_node/depth/depth_registered`
     - PointCloud2 → topic: `/zed/zed_node/point_cloud/cloud_registered`
     - Imu → topic: `/zed/zed_node/imu/data`
     - Odometry → topic: `/zed/zed_node/odom`
     - TF
6) 逐项验证各 topic 在 RViz2 中正常显示：
   - [ ] 左目 RGB 图像
   - [ ] 深度图
   - [ ] 点云
   - [ ] IMU 数据
   - [ ] Visual Odometry
   - [ ] TF 树完整性
7) 记录验证结果（截图/文字），更新 PJINFO.md / AGENTLOG.md / CLAUDE.md / TODO.md

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
