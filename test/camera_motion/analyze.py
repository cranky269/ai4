#!/usr/bin/python3
"""独立复算Gazebo真实位姿与实际像素，不用期望命令代替读回值。"""
import sys,json,math,hashlib,csv
from pathlib import Path
import numpy as np
from PIL import Image
out=Path(sys.argv[1])
def load(n):return [json.loads(l) for l in (out/(n+'.jsonl')).read_text().splitlines()]
rows=load('图像逐帧');poses=load('无人机实际位姿');mounts=load('物理相机安装关系');markers=load('标志物实际位姿')
selected=json.loads((out/'原始图像索引.json').read_text());K=json.loads((out/'相机标定.json').read_text())['K'];f,cx,cy=K[0],K[2],K[5]
checks={};details={}
def check(n,b,d=None):checks[n]={'pass':bool(b),'detail':d}
p=np.array([d['pose'] for d in poses]);t=np.array([d['stamp'] for d in poses]);order=np.argsort(t,kind='stable');t=t[order];p=p[order]
unique=np.r_[np.diff(t)>0,True];t=t[unique];p=p[unique]
def actual_at(stamp):return np.array([np.interp(stamp,t,p[:,j]) for j in range(7)])
def projection(pose):
 x,y,z,qx,qy,qz,qw=pose;yaw=2*math.atan2(qz,qw);co,si=math.cos(yaw),math.sin(yaw)
 pixels=[]
 # Entire fixed red box, including the front/back corner silhouette at nonzero yaw.
 for px in [1.98,2.02]:
  for py in [-.3,.3]:
   for pz in [.04,.44]:
    X=co*(px-x)+si*(py-y)-.20;Y=-si*(px-x)+co*(py-y);Z=pz-z-.12
    pixels.append([cx-f*Y/X,cy-f*Z/X])
 a=np.array(pixels);return [a[:,0].min(),a[:,1].min(),a[:,0].max(),a[:,1].max()]
errors=[];associations=[]
for row in rows:
 if row['red_bbox'] is None:continue
 pose=actual_at(row['stamp']);pred=projection(pose);delta=np.array(row['red_bbox'])-pred
 errors.append(delta)
 associations.append({'stamp':row['stamp'],'phase':row['phase_at_receipt'],'actual_pose_interpolated':pose.tolist(),'observed_red_bbox':row['red_bbox'],'predicted_red_bbox':pred,'bbox_error_px':delta.tolist()})
(out/'位姿与图像投影复算.json').write_text(json.dumps(associations,ensure_ascii=False,indent=2))
error=np.array(errors);details['projection_mean_abs_px']=float(np.abs(error).mean());details['projection_max_abs_px']=float(np.abs(error).max())
check('image_projection_matches_actual_body_motion',np.abs(error).mean()<5 and np.quantile(np.abs(error),.95)<12,{'mean_abs_px':details['projection_mean_abs_px'],'max_abs_px':details['projection_max_abs_px'],'p95_abs_px':float(np.quantile(np.abs(error),.95))})
check('image_payload_format',all(r['width']==640 and r['height']==480 and r['encoding']=='rgb8' and r['frame_id']=='uav_camera_optical_frame' for r in rows))
check('both_fixed_markers_remain_visible',all(r['red_bbox'] is not None and r['blue_bbox'] is not None for r in rows))
check('all_markers_uncropped',all(0<r['red_bbox'][0]<r['red_bbox'][2]<639 and 0<r['red_bbox'][1]<r['red_bbox'][3]<479 for r in rows if r['red_bbox']))
check('image_content_changes_with_motion',len(set(r['data_sha256'] for r in rows))>30,len(set(r['data_sha256'] for r in rows)))
mount=np.array([r['relative_camera_pose'] for r in mounts]);err=np.max(np.abs(mount-[.2,0,.12,0,0,0,1]))
check('physical_camera_mount_rigid',err<1e-9,{'samples':len(mounts),'max_component_error':float(err)})
for name,initial in [('camera_marker',[2,0,.24,0,0,0,1]),('marker_blue_left',[1.975,.18,.24,0,0,0,1])]:
 a=np.array([m[name] for m in markers]);check(name+'_stays_fixed',np.max(np.abs(a-initial))<1e-9,{'samples':len(a),'max_change':float(np.max(np.abs(a-initial)))})
byphase={d['selected_phase']:d for d in selected}
def feature(phase):
 a=byphase[phase]['red_bbox'];return {'center_x':(a[0]+a[2])/2,'width':a[2]-a[0]+1,'height':a[3]-a[1]+1}
features={k:feature(k) for k in byphase};details['selected_image_features']=features
base=features['initial_hold'];left=features['translate_left'];right=features['translate_right'];near=features['approach'];far=features['retreat'];yaw=features['yaw_left'];end=features['after_stop']
check('lateral_translation_explains_pixel_shift',left['center_x']-base['center_x']>50 and base['center_x']-right['center_x']>50,{'initial_u':base['center_x'],'positive_y_u':left['center_x'],'negative_y_u':right['center_x']})
check('approach_increases_apparent_size',near['width']/base['width']>1.25,{'initial_width_px':base['width'],'near_width_px':near['width'],'ratio':near['width']/base['width']})
check('retreat_restores_apparent_size',abs(far['width']-base['width'])<=3,{'retreat_width_px':far['width']})
check('yaw_changes_view_direction',yaw['center_x']-base['center_x']>50,{'yaw_u':yaw['center_x'],'initial_u':base['center_x']})
check('after_stop_matches_initial_image',byphase['after_stop']['data_sha256']==byphase['initial_hold']['data_sha256'])
stable=[r for r in rows if r['phase_at_receipt'] in ['final_hold','idle'] and r['demo_elapsed_at_receipt'] is not None and r['demo_elapsed_at_receipt']>24]
check('final_picture_stable',len(stable)>=10 and len(set(r['data_sha256'] for r in stable))==1,{'frames':len(stable),'unique_images':len(set(r['data_sha256'] for r in stable))})
yaw_all=2*np.arctan2(p[:,5],p[:,6]);details['actual_pose_min']=p.min(0).tolist();details['actual_pose_max']=p.max(0).tolist();details['yaw_range_rad']=[float(yaw_all.min()),float(yaw_all.max())]
check('entire_uav_pose_actually_changes',p[:,0].max()>.44 and p[:,1].min()<-.24 and p[:,1].max()>.24 and yaw_all.max()>.115)
check('safe_pose_envelope',p[:,0].max()<=.451 and np.max(np.abs(p[:,1]))<=.251 and np.max(np.abs(p[:,2]-.12))<1e-9)
check('final_model_pose_home',np.max(np.abs(p[-1]-[0,0,.12,0,0,0,1]))<1e-9)
check('fresh_image_timestamps',np.all(np.diff([r['stamp'] for r in rows])>0))
# Independently deserialize each saved raw ROS image and verify PNG pixels.
sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages');from sensor_msgs.msg import Image as ROSImage
same=[]
for d in selected:
 m=ROSImage();m.deserialize((out/'原始图像'/d['rosmsg']).read_bytes())
 same.append(Image.open(str(out/'原始图像'/d['png'])).tobytes()==bytes(m.data))
check('all_saved_pngs_are_original_ros_pixels',all(same),len(same))
result={'pass':all(c['pass'] for c in checks.values()),'checks':checks,'details':details,'sampling':json.loads((out/'采样结果.json').read_text())}
(out/'随动验收结果.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps(result,ensure_ascii=False,indent=2));sys.exit(0 if result['pass'] else 1)
