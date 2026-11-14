#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/demo/ai4-m4-integration-e6a5328/catkin_ws/devel/setup.bash
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
/usr/bin/python3 -B -u "$here/week2_receiver_checks.py"
/usr/bin/python3 -B -u "$here/week2_mux_check.py"
