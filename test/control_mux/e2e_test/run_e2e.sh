#!/usr/bin/env bash
# 独立本机 ROS E2E；断言保留在原检查器中。
set -eo pipefail
E2E="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd -- "$E2E/../../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$REPO/catkin_ws/devel/setup.bash"
export PATH="$REPO/.venv/bin:$PATH"
exec "$REPO/.venv/bin/python" "$REPO/test/e2e/run_isolated.py" control_mux
