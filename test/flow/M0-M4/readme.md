# M0 → M4 联调与复现

本目录对应团队固定 e6a5328 的接收方 M4。原检测器和跟踪器代码沿用已验证版本，单线程入口来自44c89c8。现有内容分为 python/ 辅助脚本、image/ 真实截图、log/ 测试结果和本说明。

## 真实联调启动

先按 M0 使用文档启动 Gazebo。已有会话直接复用。进入本工程根目录，用 M0 本次运行生成的 connection.bash 启动 M4：

```bash
bash test/flow/M0-M4/run.sh \
  --connection /本次M0运行目录/connection.bash \
  --model /绝对路径/yolo11s-drone.onnx
```

该命令检查同一Master的相机发布者，并启动原版visual_tracker与单线程detector。模型和依赖准备、预检、参数及可视化方法见 `design_module/M4/模块使用文档.md`。若消息复用其他已编译工作空间，设置 M4_ROS_SETUP。M0相机已有发布者时，无需启动camera_node。

## 历史真实数据的离线 ROS 回放

已执行下列命令。它回放真实Gazebo原图，不等于新一轮Gazebo测试。脚本创建独立Master，覆盖离场前、无目标停留和重入。消息由本交付中的检测器和原跟踪器重新产生，Ctrl+C可中止。

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
/usr/bin/python3 test/flow/M0-M4/python/replay-acceptance.py \
  --bag /绝对路径/20261110原始离场重入/live.bag \
  --model /绝对路径/yolo11s-drone.onnx --ort-deps "$PWD/lib/m4_onnx"
```

本次回放使用原包偏移110秒、时长50秒，实际结果见 log/replay-summary.json。收到原图356张，检测、航迹、调试图各129条，含108条空检测、113条空航迹，drone类编号0，全部调试图对应已接收原图，最大图像时龄约0.646秒仿真时间。回放使用真实模型，转换失败和推理失败会由节点报错，测试不会将其计为正常空目标。

## 真实证据

- 用户2026-11-10在 /home/demo/ai4-m4-integration-e6a5328 人工联调，代码a1ffd9e。两段原始录像与6个关键文件的哈希核验保留在附件。image/manual-detection.png、manual-tracks.png 是这两段人工录屏的截图，不用于证明后续新启动脚本已实测。
- 原始单线程 A/B 在 /home/demo/m4-ort1-ab/，0.985→2.135Hz，推理中位979.41→459.21ms。
- 单线程离场/重入包 /home/demo/m4-ort1-exit-reentry.NeAllp/live.bag，SHA-256 139009ec869407a1871a4b2b0fa8f1f7420b6bd68b054d06d301320e0af912ea。无目标222张原图及83条检测/航迹均为空，重入建立新ID。该数据为原跟踪器与单线程方案，不混用优先级跟踪实验。
- 2026-11-11完整ABBA补采 /home/demo/m4-ort-live-recovery-abba.JMeDfU/。A1/A2均为单线程，B1/B2为两线程。单线程2.575Hz，推理中位/P95 379.8/458.7ms，图像时龄413/532ms仿真时间，相机6.125Hz，RTF0.856，VM总CPU75.2%。两线程2.308Hz及94.6%VM CPU，未采用。log/live-abba-metrics.json 保留四组原始统计，CPU总利用率以VM四核为分母。

原始视频、模型和bag作为独立附件交接，不放入Git。具体文件与哈希见 log/evidence-manifest.json。人工录像证明用户操作版本；本次回放证明新交付入口，来源分别记录。

## 已知限制

本场景存在漏检、空航迹和ID重建。检出统计使用真值投影与像素参考框，未提供人工标注精度。输出跟随推理节拍，故不满足逐相机帧输出的字面节拍要求，待组长确认。两路数组无Header符合既有消息结构，跨帧配对需参考调试图和时序日志。没有最低检测Hz或零ID切换的明确验收门槛，不能据此把性能限制写成接口失败；也不能将链路跑通写成稳定身份跟踪已通过。
