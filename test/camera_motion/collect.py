#!/usr/bin/python3
"""本次ROS实例的原始图像/真实模型位姿/物理安装关系验收记录器。"""
import os,sys,json,time,threading,hashlib,io,math
from pathlib import Path
import numpy as np
from PIL import Image as PILImage
out=Path(sys.argv[1]);connection=json.loads((out/'当前实例.json').read_text())
os.environ.update(ROS_MASTER_URI=connection['ROS_MASTER_URI'],ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_HOME=str(out/'probe_home'),ROS_LOG_DIR=str(out/'probe_logs'))
sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
import rospy
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetLinkState
from std_srvs.srv import Trigger
from std_msgs.msg import String
rospy.init_node('moving_camera_acceptance',anonymous=True)
lock=threading.RLock();rows=[];poses=[];states=[];markers=[];mounts=[];candidates={};latest={};enabled=[False];last_image=[None];last_model=[0.]
def pd(p):return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
def state_cb(m):
 d=json.loads(m.data);d['received_monotonic']=time.monotonic()
 with lock:latest.clear();latest.update(d);states.append(d)
def pose_cb(m):
 with lock:poses.append({'stamp':m.header.stamp.to_sec(),'received_monotonic':time.monotonic(),'pose':pd(m.pose)})
def models_cb(m):
 t=time.monotonic()
 if not enabled[0] or t-last_model[0]<.2:return
 last_model[0]=t
 d={'received_monotonic':t,'sim_receipt_stamp':rospy.Time.now().to_sec()}
 for name in ['camera_marker','marker_blue_left']:
  if name in m.name:d[name]=pd(m.pose[m.name.index(name)])
 with lock:markers.append(d)
def bbox(mask):
 y,x=np.where(mask)
 return None if not len(x) else [int(x.min()),int(y.min()),int(x.max()),int(y.max())]
def image_cb(m):
 at=time.monotonic();last_image[0]=at
 if not enabled[0]:return
 assert m.encoding=='rgb8'
 a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 red=bbox((a[:,:,0]>200)&(a[:,:,1]<60)&(a[:,:,2]<60));blue=bbox((a[:,:,2]>200)&(a[:,:,0]<60)&(a[:,:,1]<60))
 with lock:
  phase=latest.get('phase','unknown')
  row={'received_monotonic':at,'stamp':m.header.stamp.to_sec(),'seq':m.header.seq,'width':m.width,'height':m.height,'encoding':m.encoding,'step':m.step,'frame_id':m.header.frame_id,'phase_at_receipt':phase,'demo_elapsed_at_receipt':latest.get('elapsed_wall_s'),'red_bbox':red,'blue_bbox':blue,'data_sha256':hashlib.sha256(bytes(m.data)).hexdigest()}
  rows.append(row)
  candidates[phase]=(m,row)
  if latest.get('mode')=='idle' and latest.get('elapsed_wall_s',0)>=26:candidates['after_stop']=(m,row)
subs=[rospy.Subscriber('/camera_motion_demo/state',String,state_cb,queue_size=100),rospy.Subscriber('/camera_motion_demo/actual_pose',PoseStamped,pose_cb,queue_size=100),rospy.Subscriber('/gazebo/model_states',ModelStates,models_cb,queue_size=5),rospy.Subscriber('/uav/front_camera/image_raw',Image,image_cb,queue_size=20,buff_size=8*1024*1024,tcp_nodelay=True)]
get_link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
for s in ['/camera_motion_demo/run','/gazebo/get_link_state']:rospy.wait_for_service(s,timeout=30)
info=rospy.wait_for_message('/uav/front_camera/camera_info',CameraInfo,timeout=20)
(out/'相机标定.json').write_text(json.dumps({'width':info.width,'height':info.height,'K':list(info.K),'frame':info.header.frame_id},indent=2))
end=time.monotonic()+20
while (not latest or last_image[0] is None or not poses) and time.monotonic()<end:time.sleep(.03)
assert latest and last_image[0] and poses
(out/'collector_ready.json').write_text(json.dumps({'ready_monotonic':time.monotonic()}))
end=time.monotonic()+40
while not (out/'begin_demo.json').exists() and time.monotonic()<end:time.sleep(.02)
assert (out/'begin_demo.json').exists(),'未收到本次录屏就绪信号'
start=time.monotonic();enabled[0]=True
trigger=None;completed=None;last_mount=0
try:
 while time.monotonic()-start<36:
  now=time.monotonic()
  if trigger is None and now-start>=.6:
   result=rospy.ServiceProxy('/camera_motion_demo/run',Trigger)();assert result.success,result.message
   trigger=time.monotonic();(out/'触发结果.json').write_text(json.dumps({'wall_monotonic':trigger,'response':result.message},ensure_ascii=False,indent=2))
  if now-last_mount>.2:
   response=get_link('static_uav::camera_link','static_uav::base_link')
   assert response.success,response.status_message
   mounts.append({'received_monotonic':time.monotonic(),'stamp':rospy.Time.now().to_sec(),'relative_camera_pose':pd(response.link_state.pose)})
   last_mount=now
  if trigger and latest.get('mode')=='idle' and latest.get('elapsed_wall_s',0)>=26:
   if completed is None:completed=now
   if now-completed>1:break
  time.sleep(.015)
 assert completed is not None,'演示未在期限内完成'
finally:
 enabled[0]=False
 # 无论记录器是否失败，只停止本任务运动，绝不关闭Gazebo或相机。
 rospy.ServiceProxy('/camera_motion_demo/stop',Trigger)()
stop=time.monotonic()
for s in subs:s.unregister()
for name,data in [('图像逐帧',rows),('无人机实际位姿',poses),('运动状态',states),('标志物实际位姿',markers),('物理相机安装关系',mounts)]:
 with (out/(name+'.jsonl')).open('w') as f:
  for d in data:f.write(json.dumps(d,ensure_ascii=False)+'\n')
rawdir=out/'原始图像';rawdir.mkdir(exist_ok=True)
selected=[]
for phase in ['initial_hold','translate_left','translate_right','translate_center','approach','retreat','yaw_left','yaw_center','final_hold','after_stop']:
 if phase not in candidates:continue
 m,row=candidates[phase]
 PILImage.frombytes('RGB',(m.width,m.height),bytes(m.data),'raw','RGB',m.step,1).save(str(rawdir/(phase+'.png')))
 stream=io.BytesIO();m.serialize(stream);(rawdir/(phase+'.rosmsg')).write_bytes(stream.getvalue())
 nearest=min(poses,key=lambda p:abs(p['stamp']-row['stamp']))
 meta=dict(row);meta.update({'selected_phase':phase,'actual_uav_pose_nearest':nearest,'pose_time_difference_s':nearest['stamp']-row['stamp'],'png':phase+'.png','rosmsg':phase+'.rosmsg'})
 (rawdir/(phase+'.json')).write_text(json.dumps(meta,ensure_ascii=False,indent=2));selected.append(meta)
(out/'原始图像索引.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2))
receipt=[r['received_monotonic'] for r in rows];stamp=[r['stamp'] for r in rows]
r={'sample_wall_s':stop-start,'frames':len(rows),'wall_fps':(len(rows)-1)/(receipt[-1]-receipt[0]),'count_div_window_fps':len(rows)/(stop-start),'sim_stamp_span_s':stamp[-1]-stamp[0],'sim_stamp_fps':(len(rows)-1)/(stamp[-1]-stamp[0]),'max_receive_gap_s':max(np.diff(receipt)),'seq_gaps':sum(max(0,rows[i]['seq']-rows[i-1]['seq']-1) for i in range(1,len(rows))),'duplicate_or_backward_stamps':int(np.sum(np.diff(stamp)<=0)),'trigger_monotonic':trigger,'completion_monotonic':completed,'selected_phases':[r['selected_phase'] for r in selected],'topic':'/uav/front_camera/image_raw','master_uri':connection['ROS_MASTER_URI']}
(out/'采样结果.json').write_text(json.dumps(r,ensure_ascii=False,indent=2));print(json.dumps(r,ensure_ascii=False,indent=2),flush=True)
rospy.signal_shutdown('仅结束验收采样，机体停稳，保留Gazebo和image_view')
assert all(s in candidates for s in ['initial_hold','translate_left','translate_right','approach','yaw_left','after_stop'])
