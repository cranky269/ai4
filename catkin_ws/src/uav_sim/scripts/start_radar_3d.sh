#!/usr/bin/env bash
set -eo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
RUN="$REPO/reports/radar-3d-runs/$(date +%Y%m%d_%H%M%S)-$$"
mkdir -p "$RUN"
exec > >(tee -i "$RUN/launch.log") 2>&1
trap 'echo "启动失败或退出异常，请查看：$RUN/launch.log"' ERR
source /opt/ros/noetic/setup.bash
source "$REPO/catkin_ws/devel/setup.bash"
test -x "$REPO/catkin_ws/devel/lib/uav_sim/ray_cloud_bridge" || {
  echo "缺少ray_cloud_bridge，请先在catkin_ws运行catkin_make；日志：$RUN"; exit 1;
}
read -r RADAR_ROS_PORT RADAR_GZ_PORT < <(/usr/bin/python3 - <<'PY'
import socket
with socket.socket() as a, socket.socket() as b:
    a.bind(('127.0.0.1',0)); b.bind(('127.0.0.1',0))
    print(a.getsockname()[1],b.getsockname()[1])
PY
)
export ROS_MASTER_URI="http://127.0.0.1:$RADAR_ROS_PORT"
export GAZEBO_MASTER_URI="http://127.0.0.1:$RADAR_GZ_PORT"
export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 GAZEBO_IP=127.0.0.1
export ROS_HOME="$RUN/ros_home" ROS_LOG_DIR="$RUN/ros"
export GAZEBO_MODEL_DATABASE_URI=""
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
echo "日志：$RUN；本终端Ctrl-C只停止本次roslaunch。"
exec roslaunch -p "$RADAR_ROS_PORT" uav_sim radar_3d.launch "$@"
