#!/usr/bin/env bash
set -e
WORLD_EXEC=false
if [[ "${1:-}" == --wait-world-exec ]]; then
  WORLD_EXEC=true
  shift
  export AI4_WAIT_RADAR_GUI=false
fi
# Starting the native GUI before world creation can leave an empty scene.
# Wait for this run's actual sensor stream, then connect to the ready world.
trap 'echo "Gazebo world/model readiness failed (exit $?)" >&2' ERR
timeout 180 /usr/bin/python3 - <<'PY'
import os, time, rospy
from sensor_msgs.msg import PointCloud2
from gazebo_msgs.srv import GetWorldProperties
rospy.init_node('gazebo_gui_ready_wait', anonymous=True)
rospy.wait_for_service('/gazebo/get_world_properties', timeout=120)
get = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
needed = {'static_uav'}
if os.environ.get('AI4_WAIT_TARGET_GUI') == 'true': needed.add('target_uav')
deadline = time.monotonic() + 120
while not rospy.is_shutdown():
    if needed.issubset(set(get().model_names)): break
    if time.monotonic() > deadline: raise RuntimeError('Gazebo GUI: models not ready: ' + str(needed))
    time.sleep(.1)
if os.environ.get('AI4_WAIT_RADAR_GUI') == 'true':
    rospy.wait_for_message('/radar/points', PointCloud2, timeout=120)
PY
if [[ "$WORLD_EXEC" == true ]]; then exec "$@"; fi
if [[ -n "${AI4_VIEW_PLUGIN:-}" && -f "$AI4_VIEW_PLUGIN" ]]; then
  exec gzclient --gui-client-plugin "$AI4_VIEW_PLUGIN" "$@"
fi
exec gzclient "$@"
