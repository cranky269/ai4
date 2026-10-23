#!/usr/bin/env bash
set -eo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$REPO/catkin_ws/devel/setup.bash"
if [[ -z "${AI4_CLUSTER_PYTHON:-}" ]]; then
  if [[ -x "$REPO/.venv-sensors/bin/python3" ]]; then
    export AI4_CLUSTER_PYTHON="$REPO/.venv-sensors/bin/python3"
  else
    export AI4_CLUSTER_PYTHON=/usr/bin/python3
  fi
fi
if ! "$AI4_CLUSTER_PYTHON" -c 'import rospy, sklearn, scipy, numpy; from uav_tracking.msg import CentroidArray' ; then
  echo "cluster依赖不可用：$AI4_CLUSTER_PYTHON。请按design_module/M0/传感器记录与回放操作说明.md配置.venv-sensors或AI4_CLUSTER_PYTHON。" >&2
  exit 1
fi
mkdir -p "$REPO/data/radar"
exec 9>"$REPO/data/radar/instance.lock"
flock -n 9 || { echo '已有移动雷达演示运行中' >&2; exit 1; }
if pgrep -x gzserver >/dev/null; then
  echo '已有gzserver；请先停止所属实例' >&2; exit 1
fi
RUN_DIR="$REPO/data/radar/$(date +%Y%m%d_%H%M%S)_$(cat /proc/sys/kernel/random/uuid)"
mkdir "$RUN_DIR"
export ROS_HOME="$RUN_DIR/ros_home" ROS_LOG_DIR="$RUN_DIR/ros"
trap 'printf "%s\n" "$?" > "$RUN_DIR/launcher_exit_code.txt"' EXIT
CONFIG="${RADAR_CONFIG:-$REPO/catkin_ws/src/uav_sim/config/radar_3d.yaml}"
PREPARE_ARGS=()
for RADAR_ARG in "$@"; do
  case "$RADAR_ARG" in markers:=true) PREPARE_ARGS=(--markers) ;; esac
done
/usr/bin/python3 "$REPO/catkin_ws/src/uav_sim/scripts/prepare_mounted_radar.py" "$REPO" "$RUN_DIR" "$CONFIG" "${PREPARE_ARGS[@]}"
echo "雷达/相机数据目录：$RUN_DIR；停止：Ctrl+C"
# All failures, including startup, are retained in this unique run.
exec > >(tee -i "$RUN_DIR/console.log") 2>&1
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
printf 'export ROS_MASTER_URI=%q\nexport GAZEBO_MASTER_URI=%q\nexport ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1\n' "$ROS_MASTER_URI" "$GAZEBO_MASTER_URI" > "$RUN_DIR/connection.bash"
/usr/bin/python3 "$REPO/catkin_ws/src/uav_sim/scripts/sensor_session.py" "$RUN_DIR" roslaunch -p "$RADAR_ROS_PORT" uav_sim mounted_radar.launch world_file:="$RUN_DIR/mounted.world" run_dir:="$RUN_DIR" "$@"
