#!/usr/bin/env python3
"""Replay measured world clouds through unchanged pure M1/M2/M3; no truth as input."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import rosbag
root=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(root/'code'))
from cluster import filter_ground, dbscan_cluster
from tracker import KalmanTracker, track_state_to_track
from navigator import waypoints_from_track, select_track
from datatypes import Point3D
p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();run=Path(a.run)
tracker=KalmanTracker();frames=0;outputs=[];last_stamp=None;expected_by_stamp={}
for file in sorted((run/'frames').glob('*.npz'))[::5]:
 with np.load(file) as cloud:
  assert str(cloud['frame_id'])=='world', 'Legacy local cloud rejected'
  xyz=np.column_stack([cloud[k].ravel() for k in ('x','y','z')]);xyz=xyz[np.isfinite(xyz).all(axis=1)]
  points=[Point3D(*point) for point in xyz]
  stamp=int(cloud['stamp_sec'])+int(cloud['stamp_nsec'])*1e-9
  if last_stamp is not None and stamp<=last_stamp:tracker=KalmanTracker();last_stamp=None
  dt=stamp-last_stamp if last_stamp is not None else .2
  last_stamp=stamp
  clusters=dbscan_cluster(filter_ground(points,.3),.5,5)
  expected_by_stamp[(int(cloud['stamp_sec']),int(cloud['stamp_nsec']))]=[list(c.centroid) for c in clusters]
  tracks=[track_state_to_track(t) for t in tracker.step([np.array(c.centroid) for c in clusters],dt)]
  selected=select_track(tracks)
  if selected:
   wp=waypoints_from_track(selected)[0]
   assert np.allclose(wp.position,selected.position)
   assert np.linalg.eigvalsh(np.array(selected.covariance).reshape(6,6)).min()>-1e-8
   outputs.append(dict(file=file.name,clusters=[list(c.centroid) for c in clusters],track_position=selected.position,world_velocity=selected.velocity,waypoint=wp.position))
  frames+=1
assert frames>2 and outputs
matched_centroids=0
with rosbag.Bag(str(run/'raw.bag')) as bag:
 for _,message,_ in bag.read_messages(topics=['/radar/centroid']):
  if not message.centroids:continue
  assert all(point.header.frame_id=='world' for point in message.centroids)
  stamp=message.centroids[0].header.stamp
  key=(stamp.secs,stamp.nsecs)
  if key not in expected_by_stamp:continue
  actual=[[point.point.x,point.point.y,point.point.z] for point in message.centroids]
  np.testing.assert_allclose(actual,expected_by_stamp[key],atol=1e-10)
  matched_centroids+=1
assert matched_centroids>2
result=dict(actual_ros_centroid_frames_matched=matched_centroids,status='passed',scope='Measured world NPZ -> pure cluster/tracker/world waypoint; dt from sampling stamps; not live ROS timing or flight boundary validation',frames=frames,outputs=outputs)
(run/'world_business_replay.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(frames=frames,outputs=len(outputs))))
