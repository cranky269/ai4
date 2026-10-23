#!/usr/bin/env python3
from pathlib import Path
import subprocess,os,signal,time,json
ROOT=Path(__file__).resolve().parents[3];OUT=ROOT/'reports/sensor-recording-validation';OUT.mkdir(parents=True,exist_ok=True)
start_wall=time.time()
run=Path(json.loads((OUT/'live_result.json').read_text())['run']);log=(OUT/'replay.log').open('w')
p=subprocess.Popen(['bash',str(ROOT/'catkin_ws/src/uav_sim/scripts/open_sensor_run.sh'),str(run),'--self-test'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
proof=run/'viewer_evidence/offline_controls.json'
try:
 for _ in range(80):
  if p.poll() is not None:raise RuntimeError('Viewer exited early')
  if proof.exists() and proof.stat().st_mtime>=start_wall:
   data=json.loads(proof.read_text());assert all(data[k] for k in ['previous_next','play_advanced','paused','radar_step','rotation']);time.sleep(2);break
  time.sleep(1)
 else:raise TimeoutError('offline viewer test')
finally:
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=25)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 log.close()
assert p.returncode==0,p.returncode
(OUT/'replay_result.json').write_text(json.dumps(dict(run=str(run),returncode=p.returncode,controls=data),indent=2));print(data,flush=True)
