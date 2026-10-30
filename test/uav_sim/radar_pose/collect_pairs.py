#!/usr/bin/env python3
"""Collect exact-stamp pairs; JSON retains epochs across simulation resets."""
import argparse
import json
import math
import threading
import time
from pathlib import Path
import numpy as np
import yaml
from tf.transformations import quaternion_matrix, euler_matrix
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock

p = argparse.ArgumentParser()
p.add_argument('output'); p.add_argument('--seconds', type=float, default=45)
a = p.parse_args()
mount_values = yaml.safe_load((Path(a.output).parent/'radar_pose.yaml').read_text())['body_to_sensor']
mount = euler_matrix(*mount_values[3:]); mount[:3,3] = mount_values[:3]
range_errors=[]; ground_errors=[]; wall_errors=[]; wall_means=[]
config=yaml.safe_load((Path(a.output).parent/'radar.yaml').read_text())
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
        c.header.frame_id != 'world' or c.point_step != 24 or
        c.row_step != c.width*24 or len(c.data) != c.row_step*c.height): errors.append('cloud schema changed')
    body=quaternion_matrix(q);body[:3,3]=xyz
    sensor=body.dot(mount)
    dtype=np.dtype({'names':['x','y','z','range'],'formats':['<f4']*4,'offsets':[0,4,8,12],'itemsize':24})
    raw=np.frombuffer(c.data,dtype=dtype)
    points=np.column_stack([raw[n] for n in ('x','y','z')]);valid=np.isfinite(points).all(axis=1)
    ranges=raw['range'][valid]; points=points[valid]
    # Analytic independent ray/fixture intersection: use known fixed wall x=7.9,
    # select rays predicted to strike above all UAVs, excluding edges/occlusion.
    az=np.linspace(config['horizontal_min_angle'],config['horizontal_max_angle'],c.width)
    el=np.linspace(config['vertical_min_angle'],config['vertical_max_angle'],c.height)
    az,el=np.meshgrid(az,el)
    directions=np.column_stack([(np.cos(el)*np.cos(az)).ravel(),(np.cos(el)*np.sin(az)).ravel(),np.sin(el).ravel()]).dot(sensor[:3,:3].T)
    with np.errstate(divide='ignore',invalid='ignore'):
        distance=(7.9-sensor[0,3])/directions[:,0]
        expected=sensor[:3,3]+distance[:,None]*directions
    mask=(distance>config['range_min'])&(distance<config['range_max']-.1)&(expected[:,2]>2.5)&(expected[:,2]<7.8)&(abs(expected[:,1])<19)
    original=np.column_stack([raw[n] for n in ('x','y','z')])
    selected=original[mask]
    if len(selected)>5:
        if not np.isfinite(selected).all():errors.append('missing predicted fixed wall returns')
        else:
            residual=float(np.max(abs(selected[:,0]-7.9)))
            wall_errors.append(residual);wall_means.append(float(np.mean(selected[:,0])))
            if residual>.10:errors.append('stationary wall moved in world')
    if len(points):
        error=float(np.max(abs(np.linalg.norm(points-sensor[:3,3],axis=1)-ranges)))
        range_errors.append(error)
        if error>2e-5:errors.append('world distance inconsistent with sensor range')
        ground=points[abs(points[:,2])<.03,2]
        if len(ground)>10:ground_errors.append(float(np.max(abs(ground))))
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
Path(a.output).with_suffix('.ready.json').write_text(json.dumps({'ready_monotonic':time.monotonic()}))
end = time.monotonic()+a.seconds
while time.monotonic()<end and not rospy.is_shutdown(): time.sleep(.1)
for sub in subs: sub.unregister()
rospy.signal_shutdown('pair collection complete')
with lock:
    result = dict(fixed_wall_frames=len(wall_errors),fixed_wall_max_x_error_m=max(wall_errors,default=None),fixed_wall_mean_x_extent_m=float(np.ptp(wall_means)) if wall_means else None,world_range_error_max_m=max(range_errors,default=None), ground_z_error_max_m=max(ground_errors,default=None), ground_observed_frames=len(ground_errors), pairs=rows, errors=errors, time_resets=resets,
                  unmatched_clouds=list(clouds), unmatched_poses=list(poses))
    Path(a.output).write_text(json.dumps(result,indent=2))
print(json.dumps(dict(pairs=len(rows),errors=errors,resets=len(resets))))
if len(rows)<5 or errors: raise SystemExit(1)
