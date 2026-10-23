#!/usr/bin/env python3
"""Own only this roslaunch; pause sensors and drain transport before Ctrl+C closure."""
import os,sys,signal,subprocess,threading,time,json
from pathlib import Path
root=Path(sys.argv[1])
stop=threading.Event()
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.set())
p=subprocess.Popen(sys.argv[2:],start_new_session=True)
while p.poll() is None and not stop.wait(.2):pass
if p.poll() is None:
 root=Path(sys.argv[1]);result={}
 (root/'stopping').touch()
 try:
  r=subprocess.run(['rosservice','call','/gazebo/pause_physics','{}'],capture_output=True,text=True,timeout=10)
  result['pause_returncode']=r.returncode;result['pause_output']=r.stdout+r.stderr
  # No sensor updates after pause; let all writers process in-flight callbacks.
  time.sleep(3)
 except Exception as e:result['error']=str(e)
 (root/'shutdown.json').write_text(json.dumps(result,indent=2))
 os.killpg(p.pid,signal.SIGINT)
try:code=p.wait(timeout=45)
except subprocess.TimeoutExpired:
 os.killpg(p.pid,signal.SIGTERM);code=p.wait(timeout=10)
try:
 stats=json.loads((root/'statistics.json').read_text())
 if stats.get('status')!='closed':code=code or 1
 manifest=json.loads((root/'run.json').read_text())
 manifest.update(status='closed_pending_verification' if code==0 else 'failed',
                 actual_start_utc=stats.get('start_utc'),actual_end_utc=stats.get('end_utc'),
                 radar_frames=stats.get('radar_frames'),camera_frames=stats.get('camera_png_frames'),exit_code=code)
 (root/'run.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
except Exception as exc:
 print('Session statistics unavailable: '+str(exc),file=sys.stderr);code=code or 1
sys.exit(code)
