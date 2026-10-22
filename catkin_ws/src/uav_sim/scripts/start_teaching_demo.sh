#!/usr/bin/env bash
# 源码树入口：只管理本入口创建的两机教学演示。
set -eo pipefail
AI4_DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
if [[ ! -f "$AI4_DEMO_ROOT/catkin_ws/devel/setup.bash" ]]; then
  echo "工作区尚未构建：$AI4_DEMO_ROOT/catkin_ws/devel/setup.bash" >&2
  echo '请按 design_module/M0/仿真使用说明.md 首次准备后重试。' >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
source "$AI4_DEMO_ROOT/catkin_ws/devel/setup.bash"
exec /usr/bin/python3 "$AI4_DEMO_ROOT/catkin_ws/src/uav_sim/scripts/teaching_demo.py" "$@"
