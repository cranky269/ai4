#!/usr/bin/env python3
"""Offline checks against native scans and actual Gazebo poses; never renders truth as echoes."""
import csv,json,sys,bisect
from pathlib import Path
import numpy as np
import rosbag
from PIL import Image
r=Path(sys.argv[1]); poses={k:[] for k in ('observer','target')}
with rosbag.Bag(str(r/'raw.bag')) as b:
 for topic,m,t in b.read_messages(topics=['/camera_motion_demo/actual_pose','/target/ground_truth/pose']):
  key='observer' if topic.startswith('/camera_motion') else 'target'
  p=m.pose;poses[key].append([m.header.stamp.to_sec(),p.position.x,p.position.y,p.position.z,2*np.arctan2(p.orientation.z,p.orientation.w)])
poses={k:np.array(v) for k,v in poses.items()}
def nearest(a,t):return a[np.argmin(abs(a[:,0]-t))]
native=list(csv.DictReader((r/'source_sensor_stamps.csv.poses.csv').open()))
frames=list(csv.DictReader((r/'frames.csv').open())); errors=[]; echoes=[];separations=[];native_xyz=[]
for row,n in zip(frames,native):
 t=int(row['stamp_sec'])+int(row['stamp_nsec'])*1e-9
 o=nearest(poses['observer'],t); target=nearest(poses['target'],t)
 xyz=np.array([float(n[k]) for k in ('x','y','z')]); yaw=2*np.arctan2(float(n['qz']),float(n['qw']))
 c,s=np.cos(yaw),np.sin(yaw);rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])
 expected=o[1:4]+np.array([.30*np.cos(o[4]),.30*np.sin(o[4]),.12])
 errors.append([float(np.linalg.norm(xyz-expected)),float(abs(yaw-o[4]))]);native_xyz.append(xyz)
 assert row['frame_id']=='world', 'Legacy local cloud is not a world cloud'
 a=np.load(r/row['file']);cloud=np.stack([a[k].ravel() for k in ('x','y','z')],axis=1); cloud=cloud[np.isfinite(cloud).all(axis=1)]
 reference=target[1:4];d=np.linalg.norm(cloud-reference,axis=1)
 hit=cloud[d<.5];echoes.append([t,len(hit),float(np.mean(hit[:,1])) if len(hit) else None,float(reference[1])]);separations.append(float(np.linalg.norm(target[1:4]-o[1:4])))
cam=list(csv.DictReader((r/'camera_frames.csv').open()));green=[];changes=[];prev=None
for row in cam[::10]:
 a=np.asarray(Image.open(r/row['file'])).astype(float);mask=(a[:,:,1]>100)&(a[:,:,1]>a[:,:,0]*1.5)&(a[:,:,1]>a[:,:,2]*1.5)
 green.append(int(mask.sum()))
 if prev is not None:changes.append(float(np.mean(abs(a-prev))))
 prev=a
hits=[e for e in echoes if e[1]>0];o=poses['observer'];t=poses['target'];u=(t[:,1]-4)/.8;v=t[:,2]/.9
result=dict(duration_sim_s=float(o[-1,0]-o[0,0]),observer_xyz_extent_m=np.ptp(o[:,1:4],axis=0).tolist(),target_xyz_extent_m=np.ptp(t[:,1:4],axis=0).tolist(),eight_implicit_residual_max=float(np.max(abs(v*v-4*u*u*(1-u*u)))),minimum_separation_m=min(separations),native_radar_world_extent_m=np.ptp(native_xyz,axis=0).tolist(),mount_position_error_max_m=max(e[0] for e in errors),mount_yaw_error_max_rad=max(e[1] for e in errors),radar_frames=len(frames),frames_with_target_echo=len(hits),echo_world_y_extent_m=float(np.ptp([e[2] for e in hits])) if hits else 0,echo_truth_y_correlation=float(np.corrcoef([e[2] for e in hits],[e[3] for e in hits])[0,1]) if len(hits)>2 else None,camera_frames=len(cam),sampled_camera_target_visible=sum(g>10 for g in green),camera_samples=len(green),camera_pixel_change_mean=float(np.mean(changes)),sensor_frame='world')
result['passed']=result['duration_sim_s']>=30 and min(result['observer_xyz_extent_m'])>.1 and result['minimum_separation_m']>2 and result['mount_position_error_max_m']<.12 and result['frames_with_target_echo']>len(frames)*.75 and result['echo_world_y_extent_m']>1 and result['echo_truth_y_correlation']>.9 and result['sampled_camera_target_visible']>len(green)*.75
(r/'mounted_verification.json').write_text(json.dumps(result,indent=2));(r/'target_echo_check.csv').write_text('stamp,measured_target_points,measured_mean_y,reference_world_y\n'+'\n'.join(','.join(map(str,e)) for e in echoes));print(json.dumps(result,indent=2));sys.exit(0 if result['passed'] else 1)
