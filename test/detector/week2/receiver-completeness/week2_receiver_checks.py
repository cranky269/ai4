"""Isolated receiver tests. Synthetic logic inputs and real bag input stay separate."""
import os,time,json,socket,tempfile,subprocess,signal,threading
from pathlib import Path
ROOT=Path('/home/demo/ai4-m4-acceptance-c421ec7')
OUT=Path(tempfile.mkdtemp(prefix='m4-week2-receivers.'));print(OUT,flush=True)
with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_LOG_DIR=str(OUT/'ros'),PYTHONDONTWRITEBYTECODE='1')
children=[];events=[];checks=[];states=[];commands=[];mux=[];phase=['startup']
def launch(cmd,name):
    with (OUT/(name+'.log')).open('w') as f:p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    children.append(p);return p
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
def check(name,ok,detail=None):checks.append(dict(name=name,passed=bool(ok),detail=detail));print(name,bool(ok),flush=True)
try:
    launch(['roscore','-p',str(port)],'master');time.sleep(3)
    import rospy,rosgraph,rosbag,yaml
    from uav_tracking.msg import Detections,DetectedTarget,VisualTrackArray,VisualTrack,TrackArray,Track
    from std_msgs.msg import String
    from mavros_msgs.msg import PositionTarget
    rospy.init_node('m4_receiver_audit',disable_signals=True)
    def state_cb(m):states.append((time.monotonic(),m.data));events.append(dict(phase=phase[0],kind='state',wall=time.monotonic(),state=m.data))
    def cmd_cb(m,key):
        row=dict(phase=phase[0],kind=key,wall=time.monotonic(),x=m.velocity.x,z=m.velocity.z,yaw=m.yaw_rate,frame=m.coordinate_frame)
        events.append(row);(commands if key=='pid' else mux).append(row)
    subscribers=[rospy.Subscriber('/state',String,state_cb,queue_size=100),rospy.Subscriber('/ctrl/visual',PositionTarget,cmd_cb,callback_args='pid',queue_size=1000),rospy.Subscriber('/audit/setpoint',PositionTarget,cmd_cb,callback_args='mux',queue_size=1000)]
    dp=rospy.Publisher('/detections',Detections,queue_size=10)
    vp=rospy.Publisher('/visual/tracks',VisualTrackArray,queue_size=10,latch=True)
    rp=rospy.Publisher('/radar/tracks',TrackArray,queue_size=10,latch=True)
    radar=TrackArray();t=Track();t.id=1;t.position=[3.,0.,1.];t.velocity=[0.,0.,0.];t.covariance=[0.]*36;t.age=10;radar.tracks=[t]
    visual=VisualTrackArray();v=VisualTrack();v.id=1;v.bbox=[330.,250.,160.,120.];v.area_ratio=.0625;visual.tracks=[v]
    good=Detections();d=DetectedTarget();d.class_id=0;d.confidence=.9;d.bbox=v.bbox;good.targets=[d]
    low=Detections();d2=DetectedTarget();d2.class_id=0;d2.confidence=.7;d2.bbox=v.bbox;low.targets=[d2]
    empty=Detections()
    def beat(msg,label):
        phase[0]=label;events.append(dict(phase=label,kind='published_detection',wall=time.monotonic(),confidence=[t.confidence for t in msg.targets]));dp.publish(msg);time.sleep(.4)
    record=launch(['rosbag','record','--lz4','-O',str(OUT/'controlled.bag'),'/detections','/visual/tracks','/radar/tracks','/state','/ctrl/visual','/audit/setpoint'],'record')
    node=launch(['/usr/bin/python3','-B',str(ROOT/'catkin_ws/src/uav_tracking/scripts/state_machine_node.py')],'state-machine')
    end=time.monotonic()+10
    while time.monotonic()<end and not dp.get_num_connections():time.sleep(.1)
    # Initial /state is non-latched and may precede TCP subscription. Establish
    # the starting state through observed transitions, never assume reception.
    rp.publish(TrackArray());vp.publish(VisualTrackArray());time.sleep(.5)
    for i in range(5):beat(empty,'startup_establish_search')
    rp.publish(radar);time.sleep(.3);beat(empty,'startup_establish_radar')
    if not states:raise RuntimeError('No observed /state transition after controlled startup')
    rp.publish(radar);vp.publish(visual);time.sleep(.5)
    for i in range(4):beat(low,'below_threshold')
    check('low_conf_does_not_lock',bool(states) and states[-1][1]=='RADAR_GUIDE')
    beat(good,'lock_1');beat(good,'lock_2');check('N_minus_1_no_lock',states[-1][1]=='RADAR_GUIDE')
    beat(good,'lock_3');check('N3_VISUAL_LOCK',states[-1][1]=='VISUAL_LOCK')
    phase[0]='detector_silence';n=len(states);time.sleep(1.2);check('silence_freezes_state',len(states)==n and states[-1][1]=='VISUAL_LOCK')
    vp.publish(VisualTrackArray());time.sleep(.2)
    beat(empty,'lost_1');beat(empty,'lost_2');check('M_minus_1_remains_locked',states[-1][1]=='VISUAL_LOCK')
    beat(empty,'lost_3');check('M3_RADAR_GUIDE',states[-1][1]=='RADAR_GUIDE')
    rp.publish(TrackArray());time.sleep(.2)
    for i in range(4):beat(empty,'radar_lost_'+str(i+1))
    check('K_minus_1_no_search',states[-1][1]=='RADAR_GUIDE')
    beat(empty,'radar_lost_5');check('K5_SEARCH',states[-1][1]=='SEARCH/RTH')
    vp.publish(visual);time.sleep(.2)
    for i in range(3):beat(good,'visual_only_in_search')
    check('visual_alone_cannot_exit_search',states[-1][1]=='SEARCH/RTH')
    rp.publish(radar);time.sleep(.2);beat(empty,'radar_restored');check('radar_recovers_RADAR_GUIDE',states[-1][1]=='RADAR_GUIDE')
    vp.publish(visual);time.sleep(.2)
    for i in range(3):beat(good,'relock')
    check('relock_after_recovery',states[-1][1]=='VISUAL_LOCK')
    stop(node)
    # Component checks only: no flight services, no MAVROS or PX4.
    pid=launch(['/usr/bin/python3','-B',str(ROOT/'catkin_ws/src/uav_tracking/scripts/pid_controller_node.py')],'pid')
    time.sleep(1)
    phase[0]='pid_synthetic_right_down_far';vp.publish(visual);time.sleep(.12)
    active=[c for c in commands if c['phase']==phase[0]]
    check('pid_right_down_far_signs',any(c['yaw']>0 and c['z']>0 and c['x']>0 for c in active),active[-3:])
    phase[0]='pid_silence';start=time.monotonic();time.sleep(.6)
    recent=[c for c in commands if c['phase']==phase[0] and c['wall']>start+.3]
    check('pid_silence_zero_after_hold',bool(recent) and all(abs(c['x'])+abs(c['z'])+abs(c['yaw'])<1e-7 for c in recent))
    phase[0]='pid_recover';vp.publish(visual);time.sleep(.12);check('pid_recovers_on_tracks',any(c['phase']==phase[0] and c['x']>0 for c in commands))
    phase[0]='pid_empty';vp.publish(VisualTrackArray());time.sleep(.5)
    recent=[c for c in commands if c['phase']==phase[0]][-5:];check('pid_empty_settles_zero',bool(recent) and all(abs(c['x'])+abs(c['z'])+abs(c['yaw'])<1e-7 for c in recent))
    vp.unregister();rp.unregister();dp.unregister();time.sleep(.3)
    # Replay only original M4 outputs and camera. Tracker is recomputed, never old-track playback.
    config=yaml.safe_load((ROOT/'catkin_ws/src/uav_tracking/config/visual_tracker.yaml').read_text())
    for k,v in config.items():rospy.set_param('/visual_tracker/'+k,v)
    tracker=launch(['/usr/bin/python3','-B',str(ROOT/'catkin_ws/src/uav_tracking/scripts/visual_tracker_node.py')],'tracker')
    phase[0]='real_M4_bag_to_M5_PID'
    realrec=launch(['rosbag','record','--lz4','-O',str(OUT/'real-receiver.bag'),'/camera/image_raw','/detections','/visual/tracks','/ctrl/visual'],'real-record')
    time.sleep(2)
    player=launch(['rosbag','play','--start=100','--duration=20','/home/demo/m4-ort1-exit-reentry.NeAllp/live.bag','--topics','/camera/image_raw','/detections'],'real-player');player.wait(timeout=35);time.sleep(.5)
    real=[c for c in commands if c['phase']==phase[0]]
    check('real_M4_drives_PID_nonzero',any(abs(c['x'])+abs(c['yaw'])+abs(c['z'])>1e-7 for c in real),{'pid_commands':len(real),'nonzero':sum(abs(c['x'])+abs(c['z'])+abs(c['yaw'])>1e-7 for c in real),'PID_width':640,'camera_width':320,'geometry_consistency':'Mismatch: no parameter changes made'})
    (OUT/'graph.json').write_text(json.dumps(rosgraph.Master('/audit').getSystemState(),indent=2))
    (OUT/'events.json').write_text(json.dumps(events))
    (OUT/'summary.json').write_text(json.dumps({'checks':checks,'scope':'Isolated original ROS receiver nodes; synthetic state tests separate from original real M4 bag driving M5/PID; NOT T1/T2/T3/T4 flight acceptance','no_px4_mavros_started':True},indent=2))
    print(json.dumps(checks),flush=True)
finally:
    for p in reversed(children):stop(p)
    (OUT/'events.json').write_text(json.dumps(events))
    if not (OUT/'summary.json').exists():
        (OUT/'summary.json').write_text(json.dumps({'checks':checks,'completed':False},indent=2))
    print('EVIDENCE_DIR='+str(OUT),flush=True)
