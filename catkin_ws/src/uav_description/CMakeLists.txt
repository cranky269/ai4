cmake_minimum_required(VERSION 3.0.2)
project(uav_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
  gazebo_ros
)

catkin_package()

## ------------------------------------------------------------------
## 安装模型资源（文件就位后取消注释）
## ------------------------------------------------------------------
# install(DIRECTORY urdf models meshes
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
