#!/usr/bin/env python3
import csv,json,os,signal,subprocess,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];OUT=ROOT/'reports/sensor-recording-validation';OUT.mkdir(parents=True,exist_ok=True)
assert (ROOT/'catkin_ws/devel/setup.bash').is_file(), 'Build this workspace first'
(ROOT/'data/radar').mkdir(parents=True,exist_ok=True)
before=set((ROOT/'data/radar').glob('*'))
command=['bash',str(ROOT/'catkin_ws/src/uav_sim/scripts/start_static_scene.sh'),'gui:=true','dynamic_target:=true','markers:=false','camera_view:=true','lidar:=false','radar:=true']
log=(OUT/'live.log').open('w');p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
run=None;reason='timeout'
duration=float(sys.argv[1]) if len(sys.argv)>1 else 12
try:
 for _ in range(180):
  paths=[x for x in (ROOT/'data/radar').glob('*') if x.is_dir() and x not in before]
  if paths:run=paths[0]
  if p.poll() is not None:reason='early_exit';break
  if run and (run/'frames.csv').exists():
   rows=list(csv.DictReader((run/'frames.csv').open()))
   if len(rows)>1:
    t=[int(r['stamp_sec'])+int(r['stamp_nsec'])*1e-9 for r in rows]
    if t[-1]-t[0]>=duration:reason='duration_reached';break
  time.sleep(1)
finally:
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=65)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=15)
 log.close()
result=dict(root=str(ROOT),run=str(run),reason=reason,returncode=p.returncode,command=command)
(OUT/'live_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(result,flush=True)
assert p.returncode==0 and reason=='duration_reached'
