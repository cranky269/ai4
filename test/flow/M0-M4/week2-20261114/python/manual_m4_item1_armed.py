"""Preloaded manual demo. User START only creates start.request; no work after trigger."""
import os,sys,time,json,signal,subprocess,copy
from pathlib import Path
import rospy
from std_srvs.srv import SetBool
from gazebo_msgs.srv import GetModelState,SetModelState
from gazebo_msgs.msg import ModelState
from uav_tracking.msg import Detections
out=Path(sys.argv[1]);run=out/('manual-item1-'+time.strftime('%H%M%S'));run.mkdir();rows=[];phase=['ready'];paused=False;original=None;record=None
rospy.init_node('manual_item1_armed',disable_signals=True)
def mark(name,seconds=0):
 phase[0]=name;row=dict(phase=name,wall=time.time(),sim=rospy.Time.now().to_sec(),deadline=time.time()+seconds)
 rows.append(row);tmp=out/'manual-control.tmp';tmp.write_text(json.dumps(row));tmp.replace(out/'manual-control.json');print(json.dumps(row),flush=True)
def cb(m):
 if phase[0]!='ready':rows.append(dict(kind='detections',phase=phase[0],wall=time.time(),sim=rospy.Time.now().to_sec(),targets=[dict(confidence=t.confidence,bbox=list(t.bbox)) for t in m.targets]))
sub=rospy.Subscriber('/detections',Detections,cb,queue_size=100)
for n in ['/target/pause','/gazebo/get_model_state','/gazebo/set_model_state']:rospy.wait_for_service(n,timeout=10)
pause=rospy.ServiceProxy('/target/pause',SetBool);get=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState);put=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
def checked(r):
 if not r.success:raise RuntimeError(str(r))
 return r
def wait(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if (out/'stop.request').exists():raise RuntimeError('Stop requested')
  time.sleep(.05)
try:
 checked(get('target_uav','world'));mark('ready')
 (out/'armed.ready').write_text('Waiting for explicit user START; no target action yet')
 while not (out/(sys.argv[2] if len(sys.argv)>2 else 'start.request')).exists():
  if (out/'stop.request').exists():raise RuntimeError('Stop requested before start')
  time.sleep(.05)
 mark('countdown',10)
 with (run/'bag.log').open('w') as f:record=subprocess.Popen(['rosbag','record','--lz4','-O',str(run/'live.bag'),'/camera/image_raw','/camera/camera_info','/detections','/detector/debug_image','/clock','/target/ground_truth/pose','/gazebo/model_states'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 wait(10);mark('normal_visible',15);wait(15)
 checked(pause(True));paused=True;original=checked(get('target_uav','world'))
 saved=ModelState(model_name='target_uav',reference_frame='world',pose=original.pose,twist=original.twist)
 (run/'restore-state.yaml').write_text(str(saved));away=copy.deepcopy(saved);away.pose.position.x=-20.
 for v in [away.twist.linear,away.twist.angular]:v.x=v.y=v.z=0.
 checked(put(away));mark('target_away',15);wait(15)
 checked(put(saved));checked(pause(False));paused=False;mark('target_restored',15);wait(15)
 mark('finished')
except Exception as e:
 mark('error');(run/'error.txt').write_text(str(e));raise
finally:
 if paused:
  try:
   if original is not None:checked(put(ModelState(model_name='target_uav',reference_frame='world',pose=original.pose,twist=original.twist)))
  finally:checked(pause(False))
 if record is not None and record.poll() is None:
  os.killpg(record.pid,signal.SIGINT)
  try:record.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(record.pid,signal.SIGTERM);record.wait(timeout=5)
 (run/'events.json').write_text(json.dumps(rows,indent=2));(run/'verdict.json').write_text(json.dumps(dict(human_verdict=None,OBS_complete_confirmed=False)))
