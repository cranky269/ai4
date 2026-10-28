# M0 双机机载雷达文档导航

本文件保留原有入口名称，使用说明统一维护在 design_module/M0/模块使用文档.md，避免两份启动步骤产生冲突。

- 使用、依赖、启动/停止与当前状态：design_module/M0/模块使用文档.md。
- 职责拆分、调用关系、模型安装及验证边界：design_module/M0/模块说明文档/拆分文档.md。
- 启动参数、函数、ROS服务与话题：design_module/M0/模块说明文档/函数接口文档.md。
- 位姿、状态JSON、PointCloud2布局、配置及记录格式：design_module/M0/模块说明文档/数据类型文档.md。

路径相对于ai4项目根目录。Gazebo正常来自用户确认，本轮仅文档与依赖核对，保留当前场景和运动。M0隔离包实际位于lib/venv/sensor_python，使用系统Python加PYTHONPATH，不是独立virtualenv；管理方法与检查结果见模块使用文档。相机、雷达画面及记录未列入本轮验收。
