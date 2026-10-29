#!/usr/bin/env python3
"""Collect exact-stamp pairs; JSON retains epochs across simulation resets."""
import argparse
import json
import math
import threading
import time
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock

p = argparse.ArgumentParser()
p.add_argument('output'); p.add_argument('--seconds', type=float, default=45)
a = p.parse_args()
rospy.init_node('radar_pose_pair_check', anonymous=True)
lock = threading.RLock()
epochs = {"cloud": 0, "pose": 0}
last_stamps = {}
last_clock = None
clouds = {}; poses = {}; rows = []; errors = []; resets = []
def key(m, kind):
    stamp = m.header.stamp.to_nsec()
    if kind in last_stamps and stamp < last_stamps[kind]:
        epochs[kind] += 1
        resets.append(dict(stream=kind, stamp=stamp))
    last_stamps[kind] = stamp
    return (epochs[kind], m.header.stamp.secs, m.header.stamp.nsecs)
def match(k):
    if k not in clouds or k not in poses: return
    c, m = clouds.pop(k), poses.pop(k)
    xyz = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
    q = [m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]
    if m.header.frame_id != 'world' or not all(math.isfinite(v) for v in xyz+q): errors.append('invalid pose')
    if abs(sum(v*v for v in q)-1) > 1e-6: errors.append('nonunit quaternion')
    expected = [('x',0,7),('y',4,7),('z',8,7),('range',12,7),('ring',16,4),('column',18,4),('status',20,2)]
    if ([(f.name,f.offset,f.datatype) for f in c.fields] != expected or
        c.header.frame_id != 'radar_link' or c.point_step != 24 or
        c.row_step != c.width*24 or len(c.data) != c.row_step*c.height): errors.append('cloud schema changed')
    rows.append(dict(epoch=k[0], sec=k[1], nsec=k[2], xyz=xyz, quaternion_xyzw=q,
                     cloud_frame=c.header.frame_id, width=c.width, height=c.height))
def cloud(m):
    with lock:
        k = key(m, "cloud"); clouds[k] = m; match(k)
def pose(m):
    with lock:
        k = key(m, "pose"); poses[k] = m; match(k)
def clock(m):
    global last_clock
    with lock:
        t = m.clock.to_nsec()
        if last_clock is not None and t < last_clock:
            resets.append(dict(stream="clock", stamp=t))
        last_clock = t
subs = [rospy.Subscriber('/radar/points',PointCloud2,cloud,queue_size=200),
        rospy.Subscriber('/radar/uav_pose_world',PoseStamped,pose,queue_size=200),
        rospy.Subscriber('/clock',Clock,clock,queue_size=1000)]
end = time.monotonic()+a.seconds
while time.monotonic()<end and not rospy.is_shutdown(): time.sleep(.1)
with lock:
    result = dict(pairs=rows, errors=errors, time_resets=resets,
                  unmatched_clouds=list(clouds), unmatched_poses=list(poses))
    Path(a.output).write_text(json.dumps(result,indent=2))
print(json.dumps(dict(pairs=len(rows),errors=errors,resets=len(resets))))
if len(rows)<5 or errors: raise SystemExit(1)
