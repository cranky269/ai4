# 传感器观测用8字运动学目标

启用：bash catkin_ws/src/uav_sim/scripts/start_static_scene.sh dynamic_target:=true markers:=false
默认dynamic_target=false、markers=true，保留原静态摄像头标志物模式。

| launch参数 | 默认 | 单位/含义 |
| --- | --- | --- |
| target_center_x / target_center_y | 4 / 0 | world中心米 |
| target_height | 0.6 | 模型原点相对地面高度米 |
| target_scale_x / target_scale_y | 0.8 / 0.9 | 两方向半尺度米 |
| target_period | 12 | 仿真秒 |
| target_rate | 20 | 每仿真秒最多更新次数，实际频率受服务和VM限制 |

相位theta=2*pi*(clock-epoch)/period，x=cx+ax*sin(theta)，y=cy+ay*sin(2theta)，z=height；模型方向为单位四元数，无姿态/速度动力学。有效/clock>0后启动相位，墙钟轮询检查clock，不使用墙钟计算运动；clock不变则不设位姿/不重复真值，时钟回退重新建立epoch并回到中心。

模型target_uav独立命名，资源ai4_target_iris/model.sdf复用本地Iris网格，无传感器和飞控插件。static=true关闭重力和刚体飞行动力学，但SetModelState可改变其位姿。本任务是运动学场景目标，不是真实飞行或追踪控制。

真值：/target/ground_truth/pose，geometry_msgs/PoseStamped，world坐标系，pose及时间戳取自Gazebo GetModelState响应header；只有查询成功才发布，不能将脚本期望位置充作真值。
参考：/target/expected_pose，同消息类型，world，时间戳为生成命令时收到的/clock。实际响应时间可能稍晚，分析使用消息发布序号关联命令与实际读回，同时记录两种仿真stamp。更新按Gazebo实际服务速度执行，不补发漏掉的旧命令。

默认范围x=3.2..4.8、y=-0.9..0.9、z=0.6米，观测机位于(0,0,0.12)。目标机体最低约z0.533米，两机中心距离至少3.2米，包络不相交。节点拒绝x最小值<1米或高度<0.2米的配置；改变模型/观测机几何后须重新核验安全距离。

ROS节点figure_eight_target，等待Gazebo服务60秒；模型加载/设置/查询失败立即报错，不发布伪真值。失败由launch required=true停止本次场景。
停止：启动终端Ctrl-C，只关闭本次launch的master/Gazebo/辅助节点；不全局pkill。
摄像头15Hz实测受VM限制、rqt显示/退出问题保持已知状态，本轮不修复。

单独暂停/恢复目标（保持整个仿真和相机运行，相位连续）：`rosservice call /target/pause "data: true"` / `rosservice call /target/pause "data: false"`；须source本轮connection.bash。
