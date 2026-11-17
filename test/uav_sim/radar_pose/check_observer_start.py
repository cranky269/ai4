#!/usr/bin/env python3
"""Live observer evidence collector. Source run connection.bash first.
Set VALIDATION_OUT and optional SECONDS_CHECK (default 40 wall seconds).
"""
import rospy,time,json,math,os
import numpy as np
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2,Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from tf.transformations import quaternion_matrix
from pathlib import Path
rospy.init_node('observer_validation',anonymous=True)
rows=[];poses={};clouds=[];cams=[];pending={}
def model(m):
 t=rospy.Time.now().to_sec();d={n:[p.position.x,p.position.y,p.position.z] for n,p in zip(m.name,m.pose)}
 rows.append([t,d])
def pose(m):
 poses[m.header.stamp.to_nsec()]=m.pose
 if len(poses)>2000:poses.pop(next(iter(poses)))
 waiting=pending.pop(m.header.stamp.to_nsec(),None)
 if waiting is not None:cloud(waiting)
def cloud(m):
 if len(clouds)>=100:return
 p=poses.get(m.header.stamp.to_nsec())
 if p is None:
  pending[m.header.stamp.to_nsec()]=m
  if len(pending)>200:pending.pop(next(iter(pending)))
  return
 xyz=np.array(list(point_cloud2.read_points(m,field_names=('x','y','z','range','ring','column'),skip_nans=True)))
 if not len(xyz):return
 q=p.orientation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);origin=np.array([p.position.x,p.position.y,p.position.z])+T[:3,:3]@np.array([.3,0,.12])
 import yaml
 cfg=yaml.safe_load((Path(__file__).resolve().parents[3]/'catkin_ws/src/uav_sim/config/radar_3d.yaml').read_text())
 az=cfg['horizontal_min_angle']+xyz[:,5]*(cfg['horizontal_max_angle']-cfg['horizontal_min_angle'])/(cfg['horizontal_samples']-1)
 el=cfg['vertical_min_angle']+xyz[:,4]*(cfg['vertical_max_angle']-cfg['vertical_min_angle'])/(cfg['vertical_samples']-1)
 local=xyz[:,3,None]*np.stack([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)],axis=1)
 expected=local@T[:3,:3].T+origin
 xyz_error=float(np.linalg.norm(xyz[:,:3]-expected,axis=1).max())
 err=np.abs(np.linalg.norm(xyz[:,:3]-origin,axis=1)-xyz[:,3]);clouds.append(dict(stamp=m.header.stamp.to_sec(),points=len(xyz),xyz_error_max=xyz_error,range_geometry_max=float(err.max()),ground_z_min=float(xyz[:,2].min())))
def cam(m):
 if len(cams)<100:cams.append([m.header.stamp.to_sec(),sum(m.data[::97])])
subs=[rospy.Subscriber('/gazebo/model_states',ModelStates,model),rospy.Subscriber('/radar/uav_pose_world',PoseStamped,pose),rospy.Subscriber('/radar/points',PointCloud2,cloud),rospy.Subscriber('/camera/image_raw',Image,cam)]
start=time.monotonic()
while time.monotonic()-start<float(os.environ.get('SECONDS_CHECK','40')):time.sleep(.2)
Path(os.environ['VALIDATION_OUT']).write_text(json.dumps(dict(rows=rows,clouds=clouds,cameras=cams)))
