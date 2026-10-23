#!/usr/bin/env bash
# 前台运行本轮静态场景；Ctrl-C仅停止本次launch及其子进程。
set -eo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
# radar:=true 显式启用机载三维Ray与逐帧记录；保留原入口默认行为。
RADAR_ENABLED=false
RADAR_DYNAMIC=false
RADAR_ARGS=()
for ARG in "$@"; do
  case "$ARG" in
    radar:=*) RADAR_ENABLED="${ARG#radar:=}" ;;
    *) RADAR_ARGS+=("$ARG") ;;
  esac
  case "$ARG" in dynamic_target:=true) RADAR_DYNAMIC=true ;; esac
done
if [[ "$RADAR_ENABLED" == true && "$RADAR_DYNAMIC" == true ]]; then
  exec bash "$REPO/catkin_ws/src/uav_sim/scripts/start_mounted_radar.sh" "${RADAR_ARGS[@]}"
fi
set -- "${RADAR_ARGS[@]}"
source /opt/ros/noetic/setup.bash
source "$REPO/catkin_ws/devel/setup.bash"
# 同一入口的并发启动在分配端口之前拒绝；锁随本次launch释放。
mkdir -p "$REPO/reports/static-scene-runs"
exec 9>"$REPO/reports/static-scene-runs/instance.lock"
if ! flock -n 9; then
  echo '已有场景入口正在运行，请复用现有窗口；未启动新实例。' >&2
  exit 1
fi
# 从实际桌面执行时也拒绝另一个入口已启动的Gazebo，不擅自停止它。
if pgrep -x gzserver >/dev/null; then
  echo '检测到已有gzserver，请核对并复用当前场景；未启动新实例。' >&2
  exit 1
fi
SCENE_GUI=true
for SCENE_ARG in "$@"; do
  case "$SCENE_ARG" in gui:=*) SCENE_GUI="${SCENE_ARG#gui:=}" ;; esac
done
if [[ "$SCENE_GUI" == true || "$SCENE_GUI" == True || "$SCENE_GUI" == 1 ]]; then
  if ! xdpyinfo >/dev/null 2>&1; then
    echo '无法连接当前X桌面；未启动场景。请从实际桌面终端执行。' >&2
    exit 1
  fi
fi
read -r SCENE_ROS_PORT SCENE_GAZEBO_PORT < <(/usr/bin/python3 - <<'PY'
import socket
with socket.socket() as a, socket.socket() as b:
 a.bind(('127.0.0.1',0)); b.bind(('127.0.0.1',0))
 print(a.getsockname()[1],b.getsockname()[1])
PY
)
export ROS_MASTER_URI="http://127.0.0.1:$SCENE_ROS_PORT"
export GAZEBO_MASTER_URI="http://127.0.0.1:$SCENE_GAZEBO_PORT"
export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 GAZEBO_IP=127.0.0.1
SCENE_LOG_DIR="$REPO/reports/static-scene-runs/$(date +%Y%m%d_%H%M%S)-$$"
mkdir -p "$SCENE_LOG_DIR"
export ROS_HOME="$SCENE_LOG_DIR/ros_home" ROS_LOG_DIR="$SCENE_LOG_DIR/ros"
printf 'export ROS_MASTER_URI=%q\nexport GAZEBO_MASTER_URI=%q\nexport ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1\n' "$ROS_MASTER_URI" "$GAZEBO_MASTER_URI" > "$SCENE_LOG_DIR/connection.bash"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
echo "静态场景日志：$SCENE_LOG_DIR；停止：本终端Ctrl-C"
exec roslaunch -p "$SCENE_ROS_PORT" uav_sim static_uav.launch "${@}"
