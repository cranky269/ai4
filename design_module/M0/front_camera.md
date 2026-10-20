# 固定前视摄像头参数（单位集中记录）

下文命令与资源路径均相对仓库根目录。

实际传感器配置：uav_description/models/ai4_static_iris/model.sdf；实际TF：uav_sim/launch/static_uav.launch。修改安装位姿时必须同时调整SDF camera_link与mount TF，不能仅改frame_id。

| 参数 | 数值 | 单位 / 约定 |
| --- | --- | --- |
| 无人机world位姿 | 0,0,0.12；RPY=0,0,0 | 米；弧度 |
| camera_link相对base | 0.20,0,0.12；RPY=0,0,0 | 米；弧度，+X前/+Y左/+Z上 |
| optical相对camera_link | 位移0；四元数(-0.5,0.5,-0.5,0.5) | xyzw；光学+Z前/+X右/+Y下 |
| 分辨率 | 640×480 | 像素 |
| 水平视场角 | 1.0471975512（60） | 弧度（度） |
| sensor update_rate | 15 | 仿真时间Hz，性能不足时实测可能偏低 |
| 插件updateRate | 0 | 不二次节流，跟随sensor |
| 裁剪范围 | 0.05～30 | 米 |
| SDF像素格式 / ROS编码 | R8G8B8 / rgb8 | 三通道8位 |
| 畸变K1,K2,K3,T1,T2 | 全0 | 无畸变针孔；CameraInfo plumb_bob |
| ROS namespace / cameraName | /uav / front_camera | 最终见下方话题 |
| image / info | /uav/front_camera/image_raw；/uav/front_camera/camera_info | sensor_msgs/Image、CameraInfo |
| frame_id | uav_camera_optical_frame | TF已实际建立 |
| 标志物 | 中心(2,0,0.24)，尺寸0.04×0.6×0.4 | 米，静态红色；蓝侧块在机体+Y侧，图像应位于左边 |

Gazebo相机沿camera_link的+X采集图像，插件消息使用已建立TF的optical frame。旋转矩阵列对应optical轴在camera_link的方向：X=(0,-1,0)，Y=(0,0,-1)，Z=(1,0,0)。

启动：bash catkin_ws/src/uav_sim/scripts/start_static_scene.sh。停止：启动终端Ctrl-C。
图像查看：另开终端，source /opt/ros/noetic/setup.bash；将ROS_MASTER_URI设为启动脚本打印的地址，再执行rqt_image_view /uav/front_camera/image_raw。必须使用同一master。也可用系统图像查看器打开实际接收的PNG，结果路径见本轮报告。
