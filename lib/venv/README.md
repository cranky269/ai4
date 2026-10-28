# M0 隔离依赖

此处集中管理 M0 Python 隔离依赖及版本记录。目录名沿用本轮约定；这里是 `pip --target` 环境，不是 virtualenv，没有独立解释器或 activate 脚本。按项目 ROS 约束使用系统 `/usr/bin/python3`，系统依赖不复制、不升级。

- `sensor_python/`：本机已安装的隔离包，Git 忽略。
- `requirements.txt`：迁移前已安装版本的精确清单，不替换根目录的全局依赖要求。
- `env.bash`：动态定位并添加隔离包目录。先 source ROS 和 catkin，再 source 此文件；仿真入口已自动处理。

从项目根目录检查或首次重建：

```bash
bash scripts/冲突库下载/install_m0_deps.sh --check
bash scripts/冲突库下载/install_m0_deps.sh
```

安装脚本仅在隔离包目录不存在时按版本清单安装；已有目录只检查，不覆盖。不会安装相机、雷达查看器或系统依赖，也不会启动仿真。若包不完整，先核对现有目录再处理，脚本不会自行删除它。

迁移来源是 `catkin_ws/build/sensor_python`。无 virtualenv 路径绑定；现有 f2py 入口绑定系统解释器，二进制依赖使用包内相对路径。已在新位置验证 ROS 导入、DBSCAN 和 SciPy 计算后清理旧副本。未来可清理 catkin 构建目录而不丢失此环境。

不要在此保存系统 ROS/Gazebo、M4 的依赖、模型或运行数据。不要提交 `sensor_python/`。
