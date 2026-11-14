# M0→M4 第二周接收方联调

本目录为2026-11-14新Gazebo/人工验收证据，运行依赖PR #18的c421ec7，不含重复核心代码。python/是本次已使用的辅助脚本，image/是真实截图/视频抽帧，log/为原始日志和派生统计。

新实测相机6.495Hz、检测2.734Hz；人工录制01另有254原图/89检测/89debug，不混用不同运行的计数。视频由成员群聊交付，详见../../../../design_module/M4/模块说明文档/week2-20261114/video-index.json。

脚本保留现场绝对Ubuntu部署路径以便核对，不保证任意新机器零配置运行。首次启动前须安装PR18规定依赖、构建Catkin、设置DISPLAY=:0；使用本次M0生成的connection.bash，不额外起roscore。manual_m4_prepare.py可启动M0及原M4并等待人工开始；manual_m4_view.py只展示，manual_m4_item1_armed.py按独立会话路径启动后等待start.request，窗口10秒倒计时后执行可见/离场/恢复。历史week2-live-check.py是限时自动检查，已有gzserver会拒绝重复启动，不能用于覆盖队友会话。

现场脚本参数/时序和安全停止细节见本脚本。独立隔离演示入口见test/detector/week2/manual/readme.md；下游正式flow代码仍由对应接收方维护。

现场复现的完整准备命令（由维护者确认没有其他Gazebo实例后执行；不是要求现在重跑）：

```bash
source /opt/ros/noetic/setup.bash
source /home/demo/ai4-m4-integration-e6a5328/catkin_ws/devel/setup.bash
export DISPLAY=:0
cp python/manual_m4_prepare.py python/manual_m4_view.py python/manual_m4_item1_armed.py /tmp/
/usr/bin/python3 -B -u python/manual_m4_prepare.py
```

程序打印SESSION目录并启动准备窗口，人工准备好OBS后创建该SESSION/start.request触发；会话已运行时优先复用已打印的connection，不重新运行启动命令。首次准备用到临时脚本/tmp，不属于模型或依赖安装。原始c421部署路径需真实存在，否则先按PR18部署。
