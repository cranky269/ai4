#!/usr/bin/env python3
"""运行中的同一静态场景：无噪声/噪声/截断边界；所有数据来自真实PointCloud2。"""
import os,time,json,threading
from pathlib import Path
from collections import deque
import numpy as np
import rospy,tf2_ros
from sensor_msgs.msg import PointCloud2
from gazebo_msgs.srv import GetModelState

OUT=Path(os.environ['RADAR_AUDIT_OUT']);OUT.mkdir(parents=True,exist_ok=True)
rospy.init_node('radar_3d_acceptance',anonymous=True)
lock=threading.Lock();received=deque(maxlen=1000)
def callback(msg):
 with lock:received.append((time.monotonic(),msg))
sub=rospy.Subscriber('/radar/points',PointCloud2,callback,queue_size=30)
tfbuf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(tfbuf)
# ROS node initialization and all waits also bounded by the outer wall-clock timeout.
def wait(predicate,seconds=90):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  value=predicate()
  if value:return value
  if rospy.is_shutdown():raise RuntimeError('ROS shutdown')
  time.sleep(.05)
 raise TimeoutError('wall-clock wait timed out')
wait(lambda: len(received)>3)
xml=rospy.get_param('/spawn_radar_3d/effective_sdf')
phase=os.environ.get('RADAR_AUDIT_PHASE','zero')
(OUT/(phase+'_effective.sdf')).write_text(xml)

def collect(name,count):
 with lock:received.clear()
 frames=[];wall=[]
 deadline=time.monotonic()+max(90,count*.7)
 while len(frames)<count and time.monotonic()<deadline:
  with lock:
   batch=list(received);received.clear()
  for at,msg in batch:
   if frames and msg.header.stamp<=frames[-1].header.stamp:continue
   frames.append(msg);wall.append(at)
   if len(frames)==1 or len(frames)%100==0:print(name,'received',len(frames),flush=True)
   if len(frames)==count:break
  time.sleep(.02)
 if len(frames)!=count:raise TimeoutError(name+': '+str(len(frames))+' frames')
 assert all(m.header.frame_id=='radar_link' and m.width==181 and m.height==5 for m in frames)
 m=frames[0]
 dtype=np.dtype({'names':['x','y','z','range','ring','column','status'],
                 'formats':['<f4','<f4','<f4','<f4','<u2','<u2','u1'],
                 'offsets':[0,4,8,12,16,18,20],'itemsize':24})
 assert not m.is_bigendian and m.point_step==24 and m.row_step==181*24
 data=np.stack([np.frombuffer(f.data,dtype=dtype).reshape(5,181) for f in frames])
 stamps=np.array([m.header.stamp.to_sec() for m in frames]);wall=np.array(wall)
 raw=np.stack([np.frombuffer(m.data,dtype=np.uint8) for m in frames])
 np.savez_compressed(str(OUT/(name+'.npz')),points=data,stamps=stamps,wall=wall,raw_pointcloud2_data=raw)
 (OUT/(name+'_header.json')).write_text(json.dumps({'frame_id':m.header.frame_id,'width':m.width,'height':m.height,'row_step':m.row_step,'point_step':m.point_step,'is_dense':m.is_dense,'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in m.fields],'frames':len(frames),'sim_hz':(count-1)/(stamps[-1]-stamps[0]),'wall_hz':(count-1)/(wall[-1]-wall[0])},indent=2))
 print(name,count,'frames',flush=True)
 return data,stamps

phase=os.environ.get('RADAR_AUDIT_PHASE','zero')
counts={'zero':60,'noisy':600,'boundary':120}
points,stamps=collect(phase,counts[phase])
transform=tfbuf.lookup_transform('world','radar_link',rospy.Time.from_sec(stamps[-1]),rospy.Duration(5))
(OUT/'tf_and_pose.json').write_text(json.dumps({'parent':transform.header.frame_id,'child':transform.child_frame_id,'translation':[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z],'rotation':[transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w],'sensor_pose':str(rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)('radar_3d','world').pose)},indent=2))
assert np.allclose([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z],[0,-2,1])
assert transform.transform.rotation.w==1

(OUT/(phase+'_complete.json')).write_text(json.dumps({'phase':phase,'frames':counts[phase],'finished_wall':time.time()},indent=2))
print('COLLECTION COMPLETE '+phase,flush=True)
