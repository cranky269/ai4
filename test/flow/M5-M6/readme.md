# M5 → M6 小联调

## 结论与数据流

**PASS（真实 M5 → 真实 M6）**。本次合格链路为：

```text
受控 /test/detections (uav_tracking/Detections)
+ 受控 320×240 /test/camera/image_raw (sensor_msgs/Image)
→ 真实 visual_tracker_node.py
→ /test/visual_tracks (uav_tracking/VisualTrackArray)
→ 真实 pid_controller_node.py
→ /test/ctrl_visual (mavros_msgs/PositionTarget)
```

验收基于 develop `431c049408f323580a351d6e3532daba8c6742f5`。Detection
和图像由接收方脚本控制，但脚本不直接发布 VisualTrackArray；bbox、ID、
area_ratio 来自真实 M5，控制命令来自真实 M6。

这不是实时 M4 Detector 链的 PASS。当前机器上的正式 M4 640 ONNX CPU 推理
约需 19–23 秒，而正式结果最大允许年龄为 1.5 秒，结果持续过期，未产生可用
`/detections`。因此实时 M0→M4→M5→M6 尚未通过，不能表述为完整视觉链成功。

## 环境、启动与复现

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
/usr/bin/python3 -B test/flow/M5-M6/python/run_real_m5_m6.py \
  --run-dir test/flow/M5-M6/log/<NEW_RUN> \
  --image-dir test/flow/M5-M6/image/<NEW_RUN>
```

脚本创建随机端口的独立 ROS master 和临时 `ROS_HOME`，只启动真实 M5、真实
M6 和审计节点。M5 使用 320×240 Image、`track_buffer=3`；M6 使用 50 Hz、
0.2 秒超时、`image_width=320`、`image_height=240`、阈值 0.35。进程以有界
阶段运行，并在 `finally` 中只停止本轮记录的 PID。动态桌面展示入口为
`python/live_panel.py`，透明/故障中继为 `python/live_relay.py`。

## 实际结果与统计

- 真实 M5 各有效阶段约 19.98 Hz；真实 M6 为 49.9979 Hz。
- M6 始终 `coordinate_frame=9`、`type_mask=1479`、`velocity.y=0`。
- M5 输出 bbox 为 `[x,y,w,h]`，按 320×240 图像计算 area_ratio；阶段内 ID
  稳定，清空超过 track_buffer 后恢复时合理创建新 ID。
- 右下远目标产生 `velocity.x>0`、`velocity.z<0`、`yaw_rate<0`。
- float32 的 0.35 实测为 `0.3499999940`，修复后前进速度为 0；0.3499 未
  提前触发，0.3501 正确进入末端分支。
- 空 Detections 先进入 HOLD，距最后有效航迹约 0.216 秒进入全零 STOP；完全
  静默保持零输出，恢复输入后重新跟踪。
- `/ctrl/visual` 和 `/mavros/setpoint_raw/local` 均无发布者。

## 图片与日志

- [真实 M5 正常跟踪](image/20261116_real_m5_run3/01_real_m5_normal_tracking.png)
- [右下远目标与控制方向](image/20261116_real_m5_run3/02_right_down_far_direction.png)
- [0.35 float32 边界](image/20261116_real_m5_run3/03_float32_terminal_boundary.png)
- [HOLD、0.2 秒超时和 STOP](image/20261116_real_m5_run3/04_empty_silence_stop.png)
- [完整阶段时间线](image/20261116_real_m5_run3/05_full_timeline.png)
- [动态演示联系表](image/M5-M6_visual_servo_dynamic_contact_sheet.png)

最终运行目录为 `log/20261116_real_m5_run3/`，其中：

- `summary.json`：逐项 PASS/FAIL、频率、边界和超时统计；
- `session.json`、`node_list.log`、`topic_graph.json`：节点、PID 和安全边界；
- `topic_info.log`、`message_types.log`、`frequency.log`：类型、连接和频率；
- `sample_detections.log`、`sample_visual_tracks.log`、
  `sample_ctrl_visual.log`、`timeline.csv`：关键样本和完整时间线；
- `cleanup_confirmation.log`：本轮进程均已停止。

外部离线回放视频 `M5-M6_visual_servo_dynamic.mp4` 未加入 Git；
`log/video-index.json` 保留文件名、SHA-256、数据来源和对应 run。视频忠实读取
上述真实 M5/M6 时间线，不是实时 M4 YOLO 链路。

## 已知限制与停止方法

本次只证明 M5→M6 接口和 M6 控制响应；受控 Detections/Image 不能替代
M0→正式 M4 的实时性能验收。M6 提供 ROS body FLU 数值，不手动进行
FLU→FRD 或 ENU→NED 转换。未录制 rosbag，也不提交 M4 模型、隔离环境或
大批性能诊断日志。

停止时只停止本轮随机 master、M5、M6 和审计进程。不得启动或清理
control_mux、state_machine、MAVROS、PX4、OFFBOARD，不调用飞控服务，不发布
`/mavros/setpoint_raw/local`。
