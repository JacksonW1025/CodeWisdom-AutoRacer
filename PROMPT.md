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
集成 N300 Pro IMU，参考 `reference/wheeltec_ros2` 的实现方式，替代 STM32 板载 MPU6050。

## 任务步骤
1. 复制 hipnuc_imu 驱动包到 `src/`：
   ```bash
   cp -r reference/wheeltec_ros2/src/wheeltec_imu/hipnuc_imu src/
   ```
2. 创建 udev 规则 `/etc/udev/rules.d/autoracer_imu.rules`：
   ```
   KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="autoracer_imu"
   ```
3. 修改 `hipnuc_imu/config/hipnuc_config.yaml`：
   ```yaml
   IMU_publisher:
       ros__parameters:
           serial_port: "/dev/autoracer_imu"
           baud_rate: 115200
           frame_id: "gyro_link"
           imu_topic: "/imu/data_raw"
   ```
4. 修改 `turn_on_autoracer_robot` launch 文件，添加 IMU topic remapping：
   - STM32 IMU: `/imu/data_raw` → `/imu/data_board` (remapped away)
   - N300 Pro: 发布到 `/imu/data_raw`
5. 添加 imu_filter_madgwick 节点（参考 wheeltec 的 imu.yaml）
6. 配置 EKF 融合（参考 wheeltec 的 ekf.yaml）
7. 编译测试：
   ```bash
   colcon build --packages-select hipnuc_imu --symlink-install
   ros2 launch hipnuc_imu imu_spec_msg.launch.py
   ros2 topic echo /imu/data_raw
   ```

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
