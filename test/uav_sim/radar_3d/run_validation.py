#!/usr/bin/env python3
"""每组参数启动同一隔离静态场景；不依赖Gazebo传感器热替换。"""
import os,sys,time,subprocess,signal,xmlrpc.client,json
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[3]
OUT=Path(os.environ.get('RADAR_AUDIT_OUT',str(ROOT/'reports'/time.strftime('%Y%m%d_%H%M%S_三维Ray雷达验收'))))
OUT.mkdir(exist_ok=True);Path('/tmp/ai4-radar-output').write_text(str(OUT))
print(OUT,flush=True)
phases=sys.argv[1:] or ['zero','noisy','boundary']
returns={}
for phase in phases:
 if phase not in ['zero','noisy','boundary']:raise ValueError('unknown phase')
 config=yaml.safe_load((ROOT/'catkin_ws/src/uav_sim/config/radar_3d.yaml').read_text())
 config['noise_stddev']=0 if phase=='zero' else .02
 if phase=='boundary':config.update(range_min=4.99,range_max=5.01)
 cfg=OUT/(phase+'_config.yaml');cfg.write_text(yaml.safe_dump(config))
 logfile=OUT/(phase+'_launch_console.log')
 log=logfile.open('w')
 launch=subprocess.Popen(['bash',str(ROOT/'catkin_ws/src/uav_sim/scripts/start_radar_3d.sh'),'gui:=false','calibration:=true','config:='+str(cfg)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 collector=None
 try:
  for _ in range(120):
   if launch.poll() is not None:raise RuntimeError('launch exited, inspect '+str(logfile))
   text=logfile.read_text(errors='replace')
   uris=[line.split('=',1)[1] for line in text.splitlines() if line.startswith('ROS_MASTER_URI=')]
   if uris:
    try:
     if xmlrpc.client.ServerProxy(uris[0]).getPid('/radar_audit')[0]==1:break
    except OSError:pass
   time.sleep(1)
  else:raise TimeoutError('ROS master not ready')
  env=os.environ.copy();env.update(ROS_MASTER_URI=uris[0],ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',RADAR_AUDIT_OUT=str(OUT),RADAR_AUDIT_PHASE=phase,ROS_HOME=str(OUT/'collector_ros_home'),ROS_LOG_DIR=str(OUT/'collector_ros_logs'))
  with (OUT/(phase+'_collector.log')).open('w') as stream:
   collector=subprocess.Popen(['bash','-c','source /opt/ros/noetic/setup.bash; exec /usr/bin/python3 "$1"','bash',str(ROOT/'test/uav_sim/radar_3d/collect.py')],env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
   returns[phase]=collector.wait(timeout=600)
  print(phase+' collector return '+str(returns[phase]),flush=True)
 finally:
  for proc in [collector,launch]:
   if proc and proc.poll() is None:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=25)
    except subprocess.TimeoutExpired:
     os.killpg(proc.pid,signal.SIGTERM)
     try:proc.wait(timeout=10)
     except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  (OUT/(phase+'_process_cleanup.txt')).write_text('collector return='+str(collector.returncode if collector else None)+'\nlaunch return='+str(launch.returncode)+'\n本次进程已等待退出；未全局pkill\n')
 if returns[phase]:break
(OUT/'phase_returns.json').write_text(json.dumps(returns,indent=2))
sys.exit(0 if all(v==0 for v in returns.values()) else 1)
