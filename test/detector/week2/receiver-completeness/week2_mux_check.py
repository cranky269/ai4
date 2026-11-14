"""Controlled M8->mux transport timing, output remapped away from flight topics."""
import os,time,json,socket,tempfile,subprocess,signal
from pathlib import Path
root=Path('/home/demo/ai4-m4-acceptance-c421ec7');out=Path(tempfile.mkdtemp(prefix='m4-week2-mux.'));print(out,flush=True)
with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_LOG_DIR=str(out/'ros'),PYTHONDONTWRITEBYTECODE='1')
ps=[];events=[];phase=['startup'];results={}
def start(cmd,name):
 with (out/(name+'.log')).open('w') as f:p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 ps.append(p)
try:
 start(['roscore','-p',str(port)],'core');time.sleep(3)
 import rospy
 from uav_tracking.msg import Detections,DetectedTarget,VisualTrackArray,VisualTrack,TrackArray,Track
 from mavros_msgs.msg import PositionTarget
 from std_msgs.msg import String
 rospy.init_node('m4_mux_audit',disable_signals=True)
 def output(m):events.append({'kind':'output','wall':time.monotonic(),'x':m.velocity.x,'phase':phase[0]})
 def state(m):events.append({'kind':'state','wall':time.monotonic(),'state':m.data,'phase':phase[0]})
 subs=[rospy.Subscriber('/audit/setpoint',PositionTarget,output,queue_size=1000),rospy.Subscriber('/state',String,state,queue_size=100)]
 dp=rospy.Publisher('/detections',Detections,queue_size=10)
 vp=rospy.Publisher('/visual/tracks',VisualTrackArray,queue_size=10,latch=True)
 rp=rospy.Publisher('/radar/tracks',TrackArray,queue_size=10,latch=True)
 cp=[rospy.Publisher('/ctrl/radar',PositionTarget,queue_size=10),rospy.Publisher('/ctrl/visual',PositionTarget,queue_size=10)]
 start(['rosbag','record','--lz4','-O',str(out/'controlled.bag'),'/detections','/visual/tracks','/radar/tracks','/ctrl/radar','/ctrl/visual','/state','/audit/setpoint'],'record')
 start(['/usr/bin/python3','-B',str(root/'catkin_ws/src/uav_tracking/scripts/state_machine_node.py')],'m8')
 start(['/usr/bin/python3','-B',str(root/'catkin_ws/src/uav_tracking/scripts/control_mux_node.py'),'/mavros/setpoint_raw/local:=/audit/setpoint'],'mux')
 time.sleep(2)
 radar=TrackArray();t=Track();t.id=1;t.position=[3.,0.,1.];t.velocity=[0.,0.,0.];t.covariance=[0.]*36;radar.tracks=[t]
 vt=VisualTrackArray();v=VisualTrack();v.id=1;v.bbox=[100.,100.,160.,120.];v.area_ratio=.0625;vt.tracks=[v]
 good=Detections();d=DetectedTarget();d.class_id=0;d.confidence=.9;d.bbox=v.bbox;good.targets=[d]
 rp.publish(radar);vp.publish(vt);time.sleep(.3)
 def flow(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   for p,x in zip(cp,[11.,22.]):
    m=PositionTarget();m.velocity.x=x;p.publish(m)
   time.sleep(.02)
 phase[0]='radar_baseline';flow(.5)
 results['radar_routed']=any(e.get('x')==11 for e in events)
 for i in range(3):
  phase[0]='lock_'+str(i+1);stamp=time.monotonic();events.append({'kind':'detection','phase':phase[0],'wall':stamp});dp.publish(good);flow(.3)
 visual=[e for e in events if e.get('x')==22 and e['wall']>=stamp]
 results['N3_visual_routed']=bool(visual)
 results['third_detection_to_first_visual_ms']=(visual[0]['wall']-stamp)*1000 if visual else None
 results['controlled_transport_under_100ms']=bool(visual) and results['third_detection_to_first_visual_ms']<100
 vp.publish(VisualTrackArray());time.sleep(.1)
 for i in range(3):
  phase[0]='lost_'+str(i+1);stamp=time.monotonic();dp.publish(Detections());flow(.3)
 results['M3_radar_routed']=any(e.get('x')==11 and e['wall']>=stamp for e in events)
 results['scope']='Synthetic compliant input, original M8/mux ROS nodes, audit output only; not flight T4 acceptance. Commands are distinguishable markers, not safe flight commands.'
 (out/'summary.json').write_text(json.dumps(results,indent=2));print(json.dumps(results),flush=True)
finally:
 for p in reversed(ps):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 (out/'events.json').write_text(json.dumps(events))
 print('EVIDENCE_DIR='+str(out),flush=True)
