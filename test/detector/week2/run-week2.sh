#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/demo/ai4-m4-integration-e6a5328/catkin_ws/devel/setup.bash
export DISPLAY=${DISPLAY:-:0}
export PYTHONDONTWRITEBYTECODE=1
base=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
/usr/bin/python3 -B -u "$base/test/detector/week2/sender-replay-audit.py"
/usr/bin/python3 -B -u "$base/test/flow/M0-M4/week2-20261114/python/week2-live-check.py"
