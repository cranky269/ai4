"""Bounded M0/M4 check; owns only the processes started here."""
import os,sys,subprocess,time,signal,tempfile,json
from pathlib import Path
out=Path(tempfile.mkdtemp(prefix='m4-week2-live-20261114.'));print(out,flush=True)
root=Path('/home/demo/ai4-formal-e6a5328'); delivery=Path('/home/demo/ai4-m4-acceptance-c421ec7'); children=[]
def start(cmd,name,env=None):
    with (out/(name+'.log')).open('w') as f:p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
    children.append(p);return p
try:
    assert subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode!=0,'Existing Gazebo: refuse new instance'
    old=set((root/'data/radar').glob('*/connection.bash'))
    m0=start(['bash',str(root/'catkin_ws/src/uav_sim/scripts/start_mounted_radar.sh'),'gui:=false','camera_view:=false','observer_motion:=false'],'m0')
    end=time.monotonic()+45; connection=None
    while time.monotonic()<end:
        fresh=set((root/'data/radar').glob('*/connection.bash'))-old
        if fresh:connection=next(iter(fresh));break
        if m0.poll() is not None:raise RuntimeError('M0 exited; see m0.log')
        time.sleep(1)
    assert connection,'No connection.bash generated'
    import shlex
    env=dict(os.environ)
    raw=subprocess.check_output(['bash','-c','source '+shlex.quote(str(connection))+'; env -0'])
    for item in raw.split(b'\0'):
        if b'=' in item:
            k,v=item.split(b'=',1)
            if k.startswith((b'ROS_',b'GAZEBO_')):env[k.decode()]=v.decode()
    os.environ.update(env)
    (out/'connection.bash').write_bytes(connection.read_bytes())
    import rospy,rosgraph
    from sensor_msgs.msg import Image
    from uav_tracking.msg import Detections,VisualTrackArray
    master=rosgraph.Master('/week2_live')
    end=time.monotonic()+50
    while time.monotonic()<end:
        try:
            if '/camera/image_raw' in dict(master.getSystemState()[0]):break
        except Exception:pass
        time.sleep(1)
    else:raise RuntimeError('M0 camera publisher unavailable')
    env['M4_ROS_SETUP']='/home/demo/ai4-m4-integration-e6a5328/catkin_ws/devel/setup.bash'
    start(['bash',str(delivery/'test/flow/M0-M4/run.sh'),'--connection',str(connection),'--model','/home/demo/m4-yolo11s-validation.8b694G/yolo11s-drone.onnx'],'m4',env)
    rospy.init_node('week2_live_observer',disable_signals=True)
    events=[]; last=[None]
    def cb(msg,key):
        r=dict(kind=key,wall=time.monotonic(),sim=rospy.Time.now().to_sec())
        if key in ('camera','debug'):r.update(stamp=msg.header.stamp.to_sec(),width=msg.width,height=msg.height)
        if key=='debug':last[0]=msg
        if key=='det':r['targets']=[[x.class_id,x.confidence,list(x.bbox)] for x in msg.targets]
        if key=='track':r['tracks']=[[x.id,list(x.bbox),x.area_ratio] for x in msg.tracks]
        events.append(r)
    subs=[rospy.Subscriber(t,c,cb,callback_args=k,queue_size=50,buff_size=2**22) for t,c,k in [('/camera/image_raw',Image,'camera'),('/detector/debug_image',Image,'debug'),('/detections',Detections,'det'),('/visual/tracks',VisualTrackArray,'track')]]
    start(['rosbag','record','--lz4','-O',str(out/'live.bag'),'/camera/image_raw','/camera/camera_info','/detections','/detector/debug_image','/visual/tracks','/clock','/target/ground_truth/pose'],'record',env)
    time.sleep(60)
    (out/'graph.json').write_text(json.dumps(master.getSystemState(),indent=2))
    (out/'events.json').write_text(json.dumps(events))
    result={'connection':str(connection),'master':env['ROS_MASTER_URI'],'source':'new Gazebo M0 desktop, unchanged c421ec7 M4, ORT1', 'duration_wall':60}
    for key in ('camera','det','track','debug'):
        rows=[e for e in events if e['kind']==key];result[key+'_count']=len(rows)
        result[key+'_hz']=(len(rows)-1)/(rows[-1]['wall']-rows[0]['wall']) if len(rows)>1 else None
    result['empty_det']=sum(not e['targets'] for e in events if e['kind']=='det')
    result['empty_track']=sum(not e['tracks'] for e in events if e['kind']=='track')
    ages=sorted(e['sim']-e['stamp'] for e in events if e['kind']=='debug');result['debug_age_sim_max']=max(ages) if ages else None
    result['nonempty_det']=result['det_count']-result['empty_det']
    (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    if last[0]:
        import cv2
        from cv_bridge import CvBridge
        cv2.imwrite(str(out/'debug.png'),CvBridge().imgmsg_to_cv2(last[0],'bgr8'))
finally:
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    print('EVIDENCE_DIR='+str(out),flush=True)
