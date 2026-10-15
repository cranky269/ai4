cmake_minimum_required(VERSION 3.0.2)
project(uav_tracking)

## 编译类型默认 Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
  message_generation
)

## ------------------------------------------------------------------
## 自定义消息（新增 .msg 时把文件名追加进 FILES）
## ------------------------------------------------------------------
add_message_files(FILES
  CentroidArray.msg
  DetectedTarget.msg
  Detections.msg
  VisualTrack.msg
  VisualTrackArray.msg
  Track.msg
  TrackArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs mavros_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

## ------------------------------------------------------------------
## Python ROS 节点
## ------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/tracker_node.py
  scripts/navigator_node.py
  scripts/pid_controller_node.py
  ## ---- M4 视觉链（相机驱动 / 检测 / 跟踪）----
  scripts/camera_node.py
  scripts/detector_node.py
  scripts/visual_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## ------------------------------------------------------------------
## Launch、参数和 RViz 资源
## ------------------------------------------------------------------
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
