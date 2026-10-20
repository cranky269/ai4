# 平面Ray/Lidar距离传感器

插件：本机Gazebo11/ROS Noetic的libgazebo_ros_laser.so，GazeboRosLaser继承RayPlugin，发布sensor_msgs/LaserScan。不是完整毫米波雷达，也不产生假目标数据。

| 参数 | 数值 | 单位/约定 |
| --- | --- | --- |
| lidar | false默认 | 统一入口显式启用 |
| lidar_calibration | false默认 | 启用固定碰撞标定板 |
| lidar_noise_mean | 0.0 | 米，Gazebo原生ray高斯均值 |
| lidar_noise_stddev | 0.0默认，验收0.02 | 米，非负 |
| 平台原点/传感器安装位姿 | world (0,-2,1)，RPY=0 | 米/弧度，固定平台；link与sensor无附加偏移 |
| TF | world→lidar_link，平移(0,-2,1)，xyzw=(0,0,0,1) | X前、Y左、Z上 |
| 水平扫描 | -pi/4..pi/4，181点，resolution=1 | 弧度；总90度；角间隔0.5度 |
| 垂直扫描 | 0..0，1点，resolution=1 | 单层平面；不是3D扫描 |
| 距离范围/分辨率配置 | 0.1..10 / 0.001 | 米；不假定该参数一定导致输出毫米量化 |
| update_rate | 10 | 仿真Hz；实际另测 |
| 话题 | /lidar/scan | 原始真实LaserScan，保留ranges/intensities |
| frame_id | lidar_link | sensor原点已建立真实TF |
| 噪声位置 | ray/noise/type=gaussian、mean、stddev | 本机SDF1.6 Ray schema要求type子元素，非type属性 |

数据完全由Gazebo Ray碰撞测距和内置噪声生成，spawn_planar_lidar.py仅加载模型与参数，不发布扫描、fake_radar或真值替代输出。无需PointCloud2，未设置转换节点；平面ranges只能表示该扫描平面。

标定板：实体lidar_calibration_plane，中心(5.1,-2,1)米，box厚度X=0.2、宽Y=4、高Z=2米，实际前面x=5米。传感器(0,-2,1)，固定射线index90角度0，法向命中y=-2,z=1，参考r=(5-0)/cos(theta)≈5米；距左右边缘2米，上下边缘1米，远离0.1/10米截断。外围超过板宽的射线会无回波，不能用其作中心测距噪声统计。

启用标定：bash catkin_ws/src/uav_sim/scripts/start_static_scene.sh lidar:=true lidar_calibration:=true dynamic_target:=false markers:=false lidar_noise_stddev:=0.02
查看：另开终端source /opt/ros/noetic/setup.bash，export ROS_MASTER_URI为启动脚本打印的地址，rostopic echo -n 1 /lidar/scan或rostopic hz /lidar/scan。停止：本次启动终端Ctrl-C。
默认不加lidar参数仍为原场景。非零mean/stddev需记录单位，统计有限样本不要求恰好配置值。LaserScan scan_time/time_increment如插件报告0，不能杜撰成扫描周期，用header.stamp实测频率。
