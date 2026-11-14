"""Original PID node: five raw-pixel boxes, default and temporary test dimensions."""
import os,time,socket,tempfile,subprocess,signal,json
from pathlib import Path
root=Path('/home/demo/ai4-m4-acceptance-c421ec7');out=Path(tempfile.mkdtemp(prefix='m4-pid-geometry.'));print(out,flush=True)
with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_LOG_DIR=str(out/'ros'),PYTHONDONTWRITEBYTECODE='1')
ps=[];events=[];results=[];phase=['startup']
def start(cmd,n):
 with (out/(n+'.log')).open('w') as f:p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 ps.append(p);return p
def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
try:
 start(['roscore','-p',str(port)],'master');time.sleep(3)
 import rospy
 from uav_tracking.msg import VisualTrackArray,VisualTrack
 from mavros_msgs.msg import PositionTarget
 rospy.init_node('pid_geometry_audit',disable_signals=True)
 def cb(m):events.append(dict(phase=phase[0],wall=time.monotonic(),x=m.velocity.x,y=m.velocity.y,z=m.velocity.z,yaw=m.yaw_rate,frame=m.coordinate_frame,mask=m.type_mask))
 sub=rospy.Subscriber('/ctrl/visual',PositionTarget,cb,queue_size=1000)
 pub=rospy.Publisher('/visual/tracks',VisualTrackArray,queue_size=10)
 start(['rosbag','record','--lz4','-O',str(out/'controlled.bag'),'/visual/tracks','/ctrl/visual'],'record')
 for width,height,label in [(640,480,'original_defaults'),(320,240,'temporary_test_dimensions')]:
  args=[] if width==640 else ['_image_width:=320','_image_height:=240']
  node=start(['/usr/bin/python3','-B',str(root/'catkin_ws/src/uav_tracking/scripts/pid_controller_node.py')]+args,label)
  end=time.monotonic()+8
  while time.monotonic()<end and not pub.get_num_connections():time.sleep(.1)
  time.sleep(.3)
  for name,cx,cy in [('center',160,120),('left',120,120),('right',200,120),('up',160,90),('down',160,150)]:
   phase[0]=label+'/'+name;msg=VisualTrackArray();t=VisualTrack();t.id=1;t.bbox=[cx-10,cy-10,20,20];t.area_ratio=400/(320*240);msg.tracks=[t]
   begin=time.monotonic()
   for i in range(16):pub.publish(msg);time.sleep(.02)
   rows=[e for e in events if e['phase']==phase[0] and e['wall']>begin+.12]
   assert rows,'No PID output'
   last=rows[-1]
   results.append(dict(mode=label,dimensions=[width,height],case=name,bbox=list(t.bbox),actual_image_center=[160,120],PID_error_pixels=[cx-width/2,cy-height/2],output=last,source='original unmodified ROS node; temporary parameters only in isolated test'))
  stop(node);time.sleep(.3)
 (out/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results),flush=True)
finally:
 for p in reversed(ps):stop(p)
 (out/'events.json').write_text(json.dumps(events));print('EVIDENCE_DIR='+str(out),flush=True)
