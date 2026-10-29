#!/usr/bin/env bash
# New-terminal entry: locate the live master, then check real samples before use.
set -eo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ACTION="${1:-}"
if [[ "$ACTION" != pose && "$ACTION" != record && "$ACTION" != list ]]; then
  echo '用法：bash catkin_ws/src/uav_sim/scripts/sensor_live.sh pose|record|list [--run 实例名]' >&2
  exit 2
fi
shift
source /opt/ros/noetic/setup.bash
source "$REPO/catkin_ws/devel/setup.bash"
if [[ "$ACTION" == list ]]; then
  exec /usr/bin/python3 "$REPO/catkin_ws/src/uav_sim/scripts/find_active_sensor_run.py" --list "$@"
fi
RUN_DIR=$(/usr/bin/python3 "$REPO/catkin_ws/src/uav_sim/scripts/find_active_sensor_run.py" "$@")
source "$RUN_DIR/connection.bash"
echo "已连接活动实例：$RUN_DIR；master=$ROS_MASTER_URI" >&2
echo "正在核验点云与位姿实际采样，请等待。" >&2
/usr/bin/python3 - <<'PY'
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
try:
    rospy.init_node('sensor_live_readiness', anonymous=True, disable_signals=True)
    for topic, msg_type in [('/radar/points', PointCloud2),
                            ('/radar/uav_pose_world', PoseStamped)]:
        message = rospy.wait_for_message(topic, msg_type, timeout=15)
        if not message.header.stamp.to_nsec():
            raise RuntimeError(topic+' has no valid sampling time')
except KeyboardInterrupt:
    print('已取消连接检查。', file=sys.stderr)
    sys.exit(130)
except Exception as exc:
    print('活动实例未实际发布所需数据（可能暂停或已停止）：'+str(exc), file=sys.stderr)
    sys.exit(1)
finally:
    rospy.signal_shutdown('readiness check complete')
PY
if [[ "$ACTION" == pose ]]; then
  exec rostopic echo /radar/uav_pose_world
fi
mkdir -p "$REPO/data/radar"
BAG="$REPO/data/radar/radar_pose_$(date +%Y%m%d_%H%M%S_%N).bag"
echo "同时录制点云与位姿：$BAG；按 Ctrl+C 保存并停止录制。" >&2
exec rosbag record -O "$BAG" /radar/points /radar/uav_pose_world
