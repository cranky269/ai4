# M4发送侧人工验收辅助

03/04均明确为隔离受控节点级测试。复用原M8/PID接收类，不改其算法，不等价于接收方正式flow或飞行验收。02则复用当前真实M0和原M4/M5。

现场Ubuntu准备：source /opt/ros/noetic/setup.bash；source /home/demo/ai4-m4-integration-e6a5328/catkin_ws/devel/setup.bash；export DISPLAY=:0。

在本目录运行 `/usr/bin/python3 -B -u python/manual_receiver_demo.py 3`，或在第三项完成并关闭后用参数4。程序自动选独立master并打印SESSION目录，点击窗口开始按钮，10秒内启动用户OBS；结束提示后由用户停止录屏，关闭窗口清理本项进程。04只在隔离节点临时320×240。不要把本master的受控数据发布到真实M0/control链。

02的track_view以单个真实会话目录为参数，仅订阅；item2_armed以同目录为参数，等待start.request后自动录包并暂停/移动/恢复target_uav，必须连接该次M0的connection.bash，结束由用户确认。监督器/停止细节见已执行脚本，不应盲目在他人运行的目标场景执行。节点源码路径仍指向已验证/home/demo/ai4-m4-acceptance-c421ec7；换机器需由维护者明确适配路径，不可宣称仅克隆本证据分支就能运行。

log/对应已执行原始记录；image/为OBS原视频的抽帧拼图；完整视频由成员群聊交付，索引见design_module/M4/模块说明文档/week2-20261114/video-index.json。bag不入库。

展示版本说明：03录像运行于同一脚本加窗口开始按钮之前，04录像使用带按钮版本；新增按钮只触发start.request，不改变原接收类或消息时序。两种已运行版本及实际输出以截图和原始日志为准，不用04录像宣称新算法通过。
