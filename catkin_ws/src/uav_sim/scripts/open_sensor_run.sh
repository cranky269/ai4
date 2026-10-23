#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$ROOT/catkin_ws/devel/setup.bash"
if [[ $# -lt 1 ]]; then echo "用法：$0 /绝对路径/运行目录 [--self-test]" >&2; exit 2; fi
PORT=$(/usr/bin/python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1
export ROS_HOME="$1/replay_ros_home"
roscore -p "$PORT" > "$1/replay_master.log" 2>&1 &
MASTER_PID=$!
trap 'kill -INT "$MASTER_PID" 2>/dev/null || true; wait "$MASTER_PID" 2>/dev/null || true' EXIT
for i in {1..50}; do rosparam set use_sim_time false >/dev/null 2>&1 && break; sleep .1; done
/usr/bin/python3 "$ROOT/catkin_ws/src/uav_sim/scripts/sensor_viewer.py" --offline --run "$@"
