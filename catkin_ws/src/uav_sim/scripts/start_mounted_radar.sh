#!/usr/bin/env bash
set -eo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$REPO/catkin_ws/devel/setup.bash"
SCENE_ONLY=false
CHECK_ENV=false
SENSOR_PROFILE=desktop
export AI4_RECORDING=false
FORWARD_ARGS=()
for ARG in "$@"; do
  case "$ARG" in
    scene_only:=*) SCENE_ONLY="${ARG#scene_only:=}" ;;
    --check-env) CHECK_ENV=true ;;
    sensor_profile:=*) SENSOR_PROFILE="${ARG#sensor_profile:=}" ;;
    record:=true|record:=True|record:=1) export AI4_RECORDING=true; FORWARD_ARGS+=("$ARG") ;;
    *) FORWARD_ARGS+=("$ARG") ;;
  esac
done
set -- "${FORWARD_ARGS[@]}"
source "$REPO/lib/venv/env.bash"
if [[ -z "${AI4_CLUSTER_PYTHON:-}" ]]; then
  export AI4_CLUSTER_PYTHON=/usr/bin/python3
fi
if [[ "$SCENE_ONLY" != true || "$CHECK_ENV" == true ]] && ! "$AI4_CLUSTER_PYTHON" -c 'import rospy, sklearn, scipy, numpy; from uav_tracking.msg import CentroidArray' ; then
  echo "cluster依赖不可用：$AI4_CLUSTER_PYTHON。请执行 scripts/冲突库下载/install_m0_deps.sh，说明见 design_module/M0/模块使用文档.md。" >&2
  exit 1
fi
if [[ "$CHECK_ENV" == true ]]; then
  "$AI4_CLUSTER_PYTHON" -c 'import sys, numpy, scipy, sklearn; print("Python:", sys.executable); print("numpy:", numpy.__version__, numpy.__file__); print("scipy:", scipy.__version__, scipy.__file__); print("sklearn:", sklearn.__version__, sklearn.__file__)'
  exit 0
fi
mkdir -p "$REPO/data/radar"
exec 9>"$REPO/data/radar/instance.lock"
flock -n 9 || { echo '已有移动雷达演示运行中' >&2; exit 1; }
if pgrep -x gzserver >/dev/null; then
  echo '已有gzserver；请先停止所属实例' >&2; exit 1
fi
RUN_DIR="$REPO/data/radar/$(date +%Y%m%d_%H%M%S)_$(cat /proc/sys/kernel/random/uuid)"
mkdir "$RUN_DIR"
export AI4_VIEW_PLUGIN="$REPO/catkin_ws/devel/lib/libai4_scene_view.so"
export AI4_VIEW_STATUS="$RUN_DIR/spectator_view.txt"
if [[ "$SCENE_ONLY" != true && ! -f "$AI4_VIEW_PLUGIN" ]]; then
  echo '缺少旁观视角插件，请先构建catkin_ws（见模块使用文档首次准备条件）。' >&2
  exit 1
fi
export ROS_HOME="$RUN_DIR/ros_home" ROS_LOG_DIR="$RUN_DIR/ros"
trap 'printf "%s\n" "$?" > "$RUN_DIR/launcher_exit_code.txt"' EXIT
CONFIG="${RADAR_CONFIG:-$REPO/catkin_ws/src/uav_sim/config/radar_3d.yaml}"
PREPARE_ARGS=(--sensor-profile "$SENSOR_PROFILE")
for RADAR_ARG in "$@"; do
  case "$RADAR_ARG" in markers:=true) PREPARE_ARGS+=(--markers) ;; esac
done
if [[ "$SCENE_ONLY" == true ]]; then PREPARE_ARGS+=(--scene-only); fi
/usr/bin/python3 "$REPO/catkin_ws/src/uav_sim/scripts/prepare_mounted_radar.py" "$REPO" "$RUN_DIR" "$CONFIG" "${PREPARE_ARGS[@]}"
echo "雷达/相机数据目录：$RUN_DIR；停止：Ctrl+C"
# All failures, including startup, are retained in this unique run.
exec > >(tee -i "$RUN_DIR/console.log" 9>&-) 2>&1
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
printf 'export ROS_MASTER_URI=%q\nexport GAZEBO_MASTER_URI=%q\nexport ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 GAZEBO_IP=127.0.0.1\n' "$ROS_MASTER_URI" "$GAZEBO_MASTER_URI" > "$RUN_DIR/connection.bash"
if [[ "$SCENE_ONLY" == true ]]; then
  echo "仅场景模式：保留机载雷达外壳；本次世界禁用相机/Ray传感器，不启动记录、处理及数据窗口。"
  echo "$$" > "$RUN_DIR/launcher.pid"
  exec roslaunch -p "$RADAR_ROS_PORT" uav_sim static_uav.launch world_file:="$RUN_DIR/mounted.world" dynamic_target:=true continuous_observer:=true target_height:=1.42 camera_view:=false wait_for_radar_gui:=false markers:=false "$@"
fi
/usr/bin/python3 "$REPO/catkin_ws/src/uav_sim/scripts/sensor_session.py" "$RUN_DIR" roslaunch -p "$RADAR_ROS_PORT" uav_sim mounted_radar.launch world_file:="$RUN_DIR/mounted.world" run_dir:="$RUN_DIR" "$@"
