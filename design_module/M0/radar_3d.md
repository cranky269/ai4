# 三维 Ray 雷达简化模型

Gazebo Classic 11 CPU Ray，ROS Noetic；原静态机体、相机和world原位复用，另加固定观测平台，沿用原平面雷达的安装点 world=(0,-2,1)m、单位旋转。`radar_link` 轴为X前、Y左、Z上。不是电磁毫米波模型；仅将原生径向白高斯测距噪声作为本任务的简化杂波，不生成额外假点。

首次构建（已有依赖，不自动安装）：

```bash
cd catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
```

启动：

```bash
bash catkin_ws/src/uav_sim/scripts/start_radar_3d.sh
# 默认开GUI；无GUI测量、五高度静止标定靶：
bash catkin_ws/src/uav_sim/scripts/start_radar_3d.sh gui:=false calibration:=true
# 无噪声基准，其他参数一致：
bash catkin_ws/src/uav_sim/scripts/start_radar_3d.sh gui:=false calibration:=true noise_stddev:=0
```

在启动终端Ctrl-C，只停止这次roslaunch；不要全局pkill。脚本打印独立ROS_MASTER_URI、GAZEBO_MASTER_URI及日志位置 `reports/radar-3d-runs/<时间>-<PID>/`。另一终端读取话题须先source ROS并设同一ROS_MASTER_URI。改变噪声/采样参数请停止后重启；本次实测传感器删除重建会中断已有transport订阅，不支持热替换。

`config/radar_3d.yaml` 是主参数文件，可复制后以 `config:=/绝对路径/config.yaml` 传入；可用非空 `noise_stddev:=数值` 覆盖YAML，仅验收基准设0，默认0.02m。安装点通过 `x:=... y:=... z:=...` 同时传入spawn和静态TF；旋转固定单位姿态。姿态不是光学frame，勿把ROS光学Z前公式用于这里。

| 参数 | 默认 | 单位 |
| --- | --- | --- |
| 水平角/采样 | -45..45 / 181 | 度 / 条；配置角为弧度 |
| 垂直角/采样 | -10..10 / 5 | 度 / 层；配置角为弧度 |
| 扫描resolution | 水平1、垂直1 | 固定，905条物理射线，无插值 |
| 量程/分辨率 | 0.1..10 / 0.001 | 米；Ray几何精度与此声明分辨率不同，不额外量化 |
| 更新率 | 5 | 仿真Hz；实际频率看验收数据 |
| 噪声mean/stddev | 0 / 0.02 | 米；bias_mean/bias_stddev/dynamic_bias_stddev全0 |

原生Ray在每帧的每条有效射线上单独调用GaussianNoiseModel，计算r+N(0,sigma²)，不使用整帧偏移。桥接不调用随机数，不对XYZ再次加噪，不重置随机种子。独立性为伪随机采样设计和有限样本统计支持，不是数学证明。

点云 `/radar/points`：`sensor_msgs/PointCloud2`，organized width=181/height=5（随配置变化），step=24字节；字段x/y/z/range(float32米)、ring/column(uint16)、status(uint8)。垂直为行、水平为列，均从最小角递增。
`x=r cos(v)cos(h), y=r cos(v)sin(h), z=r sin(v)`；stamp原样使用Gazebo LaserScanStamped测量时间，frame=radar_link，TF world→radar_link=(0,-2,1)/单位四元数。

状态：0=无回波或非有限无效值、1=量程内部、2=低于最小量程（含原生负无穷）、3=最小量程边界、4=最大量程边界。原生真值>=max为+inf，<=min为-inf，噪声只加在内部有效回波上，噪声越界原生clamp到[min,max]。+/-inf和NaN的XYZ均为NaN，range保留原非有限值；不会把无回波变成最大量程平面。有限噪声clamp点保留边界XYZ并标记3/4，因此临近边界误差分布不再是未截断Gaussian。Ray物理射线从min开始，完全位于盲区内的几何不能据此保证被检测；不把盲区无回波伪造为近距离点。

验收：`source /opt/ros/noetic/setup.bash` 后用系统Python运行 `test/uav_sim/radar_3d/run_validation.py`，每组独立启动同一场景，依次采集zero60/noisy600/boundary120帧。然后 `python3 test/uav_sim/radar_3d/analyze.py <输出目录>`。输出原始PointCloud2字节、结构化range/XYZ/索引/状态、stamp、接收墙钟、有效SDF、TF/位姿、误差CSV及相关性矩阵；不需要PCL、额外ROS包或软件安装。
