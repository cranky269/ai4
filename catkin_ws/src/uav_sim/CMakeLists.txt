cmake_minimum_required(VERSION 3.0.2)
project(uav_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  gazebo_ros
  xacro
)

catkin_package()

find_package(gazebo REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_executable(ray_cloud_bridge src/ray_cloud_bridge.cpp)
target_compile_features(ray_cloud_bridge PRIVATE cxx_std_17)
target_link_libraries(ray_cloud_bridge ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
install(TARGETS ray_cloud_bridge RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## ------------------------------------------------------------------
## 安装仿真资源（文件就位后取消注释）
## ------------------------------------------------------------------
# install(DIRECTORY worlds launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/prepare_mounted_radar.py scripts/start_mounted_radar.sh scripts/record_radar.py scripts/check_radar_run.py scripts/repeat_camera_motion.py scripts/start_static_scene.sh scripts/figure_eight_target.py scripts/spawn_planar_lidar.py scripts/spawn_radar_3d.py scripts/start_radar_3d.sh scripts/camera_motion_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sensor_source_audit src/sensor_source_audit.cpp)
target_compile_features(sensor_source_audit PRIVATE cxx_std_17)
target_link_libraries(sensor_source_audit ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
install(TARGETS sensor_source_audit RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/sensor_interfaces.py scripts/sensor_session.py scripts/start_sensor_clock.py scripts/sensor_viewer.py scripts/open_sensor_run.sh scripts/check_sensor_run.py scripts/find_sensor_run.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/audited_cluster.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
