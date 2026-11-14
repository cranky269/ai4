"""Prepare only manual item 1. Remain alive until a stop request, no target actions."""
import os,time,json,subprocess,signal,shlex,hashlib
from pathlib import Path
out=Path('/home/demo/m4-manual-acceptance-20261114-'+time.strftime('%H%M%S'));out.mkdir();print(out,flush=True)
formal=Path('/home/demo/ai4-formal-e6a5328');delivery=Path('/home/demo/ai4-m4-acceptance-c421ec7');ps=[]
def start(cmd,name,env=None):
 with (out/(name+'.log')).open('w') as f:p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
 ps.append((name,p));return p
try:
 reuse=os.environ.get('MANUAL_REUSE_CONNECTION')
 if not reuse and subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode==0:raise RuntimeError('Existing Gazebo found; refusing duplicate start. Inspect its connection before reuse.')
 env=dict(os.environ,DISPLAY=':0',M4_ROS_SETUP='/home/demo/ai4-m4-integration-e6a5328/catkin_ws/devel/setup.bash',PYTHONDONTWRITEBYTECODE='1')
 before=set((formal/'data/radar').glob('*/connection.bash'))
 m0=None if reuse else start(['bash',str(formal/'catkin_ws/src/uav_sim/scripts/start_mounted_radar.sh'),'gui:=true','camera_view:=false','observer_motion:=false'],'m0',env)
 end=time.monotonic()+70;connection=Path(reuse) if reuse else None
 while time.monotonic()<end:
  if connection is not None:break
  fresh=set((formal/'data/radar').glob('*/connection.bash'))-before
  if fresh:connection=next(iter(fresh));break
  if m0 is not None and m0.poll() is not None:raise RuntimeError('M0 exited')
  time.sleep(1)
 if connection is None:raise RuntimeError('No connection generated')
 raw=subprocess.check_output(['bash','-c','source '+shlex.quote(str(connection))+'; env -0'],env=env)
 for item in raw.split(b'\0'):
  if b'=' in item:
   k,v=item.split(b'=',1)
   if k.startswith((b'ROS_',b'GAZEBO_')):env[k.decode()]=v.decode()
 os.environ.update(env)
 import rosgraph
 master=rosgraph.Master('/manual_preflight');end=time.monotonic()+90
 while time.monotonic()<end:
  try:
   if '/camera/image_raw' in dict(master.getSystemState()[0]):break
  except Exception:pass
  if m0 is not None and m0.poll() is not None:raise RuntimeError('M0 exited')
  time.sleep(1)
 else:raise RuntimeError('Camera not registered')
 model=Path('/home/demo/m4-yolo11s-validation.8b694G/yolo11s-drone.onnx')
 assert hashlib.sha256(model.read_bytes()).hexdigest()=='320ec090ff91a333116d120479f2ad2a9f97a1893e5826e99cf2ad7fd2ca1bd7'
 start(['bash',str(delivery/'test/flow/M0-M4/run.sh'),'--connection',str(connection),'--model',str(model)],'m4',env)
 start(['/usr/bin/python3','-B','/tmp/manual_m4_view.py',str(out)],'viewer',env)
 start(['/usr/bin/python3','-B','/tmp/manual_m4_item1_armed.py',str(out)],'armed-controller',env)
 (out/'session.json').write_text(json.dumps(dict(item=1,state='waiting_for_user_start',connection=str(connection),master=env['ROS_MASTER_URI'],model=str(model),processes=[dict(name=n,pid=p.pid) for n,p in ps],automatic_target_actions=False,recording='User OBS; not started by Work'),indent=2))
 print('SESSION='+str(out),flush=True)
 while not (out/'stop.request').exists():
  failed=[n for n,p in ps if n!='armed-controller' and p.poll() is not None]
  if failed:raise RuntimeError('Exited: '+','.join(failed))
  time.sleep(1)
finally:
 for n,p in reversed(ps):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=35)
   except subprocess.TimeoutExpired:
    os.killpg(p.pid,signal.SIGTERM)
    try:p.wait(timeout=10)
    except subprocess.TimeoutExpired:(out/(n+'-cleanup-pending.txt')).write_text('Timeout; inspect owned process before further action')
 if reuse:
  (out/'reused-m0.json').write_text(json.dumps({'connection':reuse,'owned_previous_session':'/home/demo/m4-manual-acceptance-20261114-154957','M0_launcher_pid':13818,'M0_sensor_session_pid':13878,'note':'M0 reused, not stopped by new display supervisor; stop explicitly after user confirms item completion'}))
 (out/'stopped.txt').write_text('Owned processes stopped; no human pass/fail assigned.')
