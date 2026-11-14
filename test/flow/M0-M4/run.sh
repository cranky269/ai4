#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
source /opt/ros/noetic/setup.bash
setup=${M4_ROS_SETUP:-"$root/catkin_ws/devel/setup.bash"}
[[ -f "$setup" ]] || { echo "Catkin 消息尚未构建，或请设置 M4_ROS_SETUP 指向已编译工作空间" >&2; exit 1; }
source "$setup"
if [[ ${1:-} == --connection ]]; then
  [[ $# -ge 2 && -f "$2" ]] || { echo '缺少本次 M0 的 connection.bash' >&2; exit 2; }
  source "$2"
  shift 2
fi
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 -B -u "$root/test/flow/M0-M4/python/run-m4.py" "$@"
