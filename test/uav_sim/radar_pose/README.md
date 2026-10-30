# M0 逐帧世界位姿回归

从项目根目录构建正式 catkin 工程后，使用系统 Python，source ROS、catkin_ws/devel/setup.bash 和 lib/venv/env.bash。

```bash
cmake -S test/uav_sim/radar_pose -B /tmp/ai4-pose-audit-build
cmake --build /tmp/ai4-pose-audit-build -j2
RUN_DIR="data/radar/pose_validation_$(date +%Y%m%d_%H%M%S)"
/usr/bin/python3 test/uav_sim/radar_pose/run_gazebo_check.py "$RUN_DIR" /tmp/ai4-pose-audit-build/libmodel_pose_audit.so
/usr/bin/python3 test/uav_sim/radar_pose/compare_model_pose.py "$RUN_DIR"
```

测试使用独立随机端口，复用 mounted_radar.launch，关闭 GUI。仅在本次生成世界中改变机体相对模型及雷达相对机体的六自由度固定安装，并插入独立 world plugin。插件每个物理步读取 Gazebo static_uav 模型位姿，按秒/纳秒记录；核验脚本只按完全相同的仿真时间及重置 epoch 配对，不取最近帧。控制阶段涵盖平移、roll/pitch/yaw 及 reset_simulation 后新位姿。

`collect_pairs.py OUTPUT --seconds 45` 也可连接统一启动入口分配的 master；先 source 本次 data/radar/<运行目录>/connection.bash。核验点云布局及 world frame_id、完全一致的时间戳、有限数值和单位四元数。ROS 两个话题并非原子传输，启动/退出边界的未匹配帧单独记录。下游应在输入消息或 /clock 时间回退时清空同步缓存，避免跨 epoch 使用重复的秒/纳秒。

运行产物位于指定目录：pose_pairs.json、model_pose.csv、phases.json、comparison.json、console.log。测试还会通过真实 Gazebo transport 注入非法四元数、NaN 坐标、目标机父框架以及时间回退，要求非法位姿整帧拒收，仅 2 帧合法点云及 2 帧同帧位姿发布，结果写入 invalid_pose.json。测试不代表 GUI 画面或飞控动力学验收。

重置阶段持续 24 墙钟秒，超过既有 20 秒停滞阈值，要求启动看门狗不把仍在更新的相机/雷达误判为失败。`reset_watchdog.json` 保存当时的真实年龄和时钟追赶标志；`test_exit.json` 要求所属 roslaunch 正常退出。

## 新终端连接与显式录制

`test_active_run.py` 覆盖启动器多赋值 export、错误 master 身份、死亡发布者、无实例提示以及多实例显式选择。直接以系统 Python 运行即可。真实新终端入口为 `sensor_live.sh pose` 和 `sensor_live.sh record`；二者自动加载活动实例，无需 source 或填写目录。`check_live_bag.py` 接受实际 bag 文件路径，可用 `--output` 保存 JSON，回读验证两个话题的消息数、原始采样时间及分段配对。

本轮世界点云检查还核验每点到同帧雷达世界原点的距离与原 range 一致。故障注入检查三轴旋转与平移的真实 XYZ、range 不变、无效点 NaN，非法位姿整帧拒收。地面 z 稳定仅覆盖地面回波，不能替代独立固定物体全场景验收。
