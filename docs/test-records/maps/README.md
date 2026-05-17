# 阶段 3 地图保留说明

本目录当前保留的正式阶段 3 地图为:

- `stage3-final-floor2-loop-20260517-101119.yaml`
- `stage3-final-floor2-loop-20260517-101119.pgm`
- `stage3-final-floor2-loop-20260517-101119.posegraph`
- `stage3-final-floor2-loop-20260517-101119.data`
- `stage3-final-floor2-loop-20260517-101119-preview.png`

这组地图是阶段 3 验收通过后的二楼环形通道固定地图，也是阶段 4 固定地图定位、Nav2 规划和后续路径跟踪测试的默认输入。

默认不要删除、改名或替换这组文件。只有在重新完成阶段 3 扫图验收，并同步更新阶段 3 验收记录、阶段 4 启动参数或地图引用后，才允许用新的正式地图替代它。

临时扫图、失败扫图和大体积 rosbag 不放在这里作为长期交付物；rosbag 保留在本地 `docs/test-records/rosbags/`，默认不提交 git。
