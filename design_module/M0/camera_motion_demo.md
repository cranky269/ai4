# 机载相机位姿驱动演示

相机原本就是 `static_uav` 模型内的 `camera_link/front_camera`，无需拆装或移动独立相机。相对机体 `base_link` 的位置为(0.20,0,0.12)m、姿态单位旋转，朝+X；ROS图像 `/uav/front_camera/image_raw`，optical frame原静态安装TF不变。

演示通过已有 Gazebo `/gazebo/set_model_state` 移动**整个static_uav模型**，维持static=true，以固定高度0.12m实现运动学展示；不是飞控闭环、空气动力或MAVLink飞行验收。只给该模型发送位姿，不移动红蓝标志物。机体TF从原固定发布改为实际GetModelState读回位姿发布，安装TF不变。观察相机在录制前设好，演示全程不由代码移动它。

场景已运行时，直接执行：

```bash
# 异步触发约26秒：静止3s、左右/回中8s、靠近/退回6s、偏航/恢复5s、停止4s。
/usr/bin/python3 catkin_ws/src/uav_sim/scripts/camera_motion_command.py run
# 立即取消后续位姿命令，停在当前位姿，不关闭场景：
/usr/bin/python3 catkin_ws/src/uav_sim/scripts/camera_motion_command.py stop
# 若中途停止，3秒缓慢回初始位置，然后可再次run：
/usr/bin/python3 catkin_ws/src/uav_sim/scripts/camera_motion_command.py home
/usr/bin/python3 catkin_ws/src/uav_sim/scripts/camera_motion_command.py status
# 关闭本入口启动的场景和相机：
/usr/bin/python3 catkin_ws/src/uav_sim/scripts/gazebo_static.py stop
```

CLI读取当前唯一实例active.json，核对PID启动时间，从本次scene.log开头获取实际ROS master；不会另启仿真。必须在本机桌面终端执行，容器PID命名空间中的身份核对可能看不到主机进程。
只有无运行实例时才使用原 `gazebo_static.py start`。launch新增的Python节点保持名称world_uav_tf，默认只发布真实位姿TF并待机，不自动运行运动。相机/雷达参数、world、网格和旧目标运动脚本不变。

演示范围：x=0..0.45m，y=-0.25..0.25m，z=0.12m，yaw=0..0.12rad（6.88°）；平移和转向用三次平滑插值，墙钟约20Hz发送位姿。模型包络与x≈2m的静态标志物保持明显间距，底面仍在地面之上。home仅接受该有限区域内的姿态，run仅接受接近原点的初始位姿；无规划、避障、识别或跟踪功能。

ROS控制服务（std_srvs/Trigger）：/camera_motion_demo/run、stop、home、status。
/camera_motion_demo/actual_pose 是Gazebo GetModelState读回实际位姿（非期望指令），/camera_motion_demo/state 记录阶段/期望和读回值，供本任务验收。新增动态 world→uav_base_link TF与已有两段静态安装TF串联。
