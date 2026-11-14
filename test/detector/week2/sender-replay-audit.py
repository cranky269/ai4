"""M4 sender-side audit of real recorded detections; no receiver sources modified."""
import os, sys, json, time, socket, subprocess, signal, tempfile, threading
from pathlib import Path
ROOT=Path('/home/demo/ai4-m4-acceptance-c421ec7')
OUT=Path(tempfile.mkdtemp(prefix='m4-week2-20261114.'))
print(str(OUT),flush=True)
with socket.socket() as s:
    s.bind(('127.0.0.1',0)); port=s.getsockname()[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_LOG_DIR=str(OUT/'ros'),PYTHONDONTWRITEBYTECODE='1')
children=[]
def launch(cmd,name):
    f=open(OUT/(name+'.log'),'w'); p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True); f.close(); children.append(p); return p
try:
    launch(['roscore','-p',str(port)],'master'); time.sleep(4)
    import rospy, rosgraph, rosbag, yaml, importlib.util
    from uav_tracking.msg import Detections,VisualTrackArray
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    rospy.init_node('m4_week2_sender_audit',disable_signals=True)
    # Load unchanged receiver code; instrument its actual callback/advance only in memory.
    spec=importlib.util.spec_from_file_location('week2_original_m8',str(ROOT/'catkin_ws/src/uav_tracking/scripts/state_machine_node.py'))
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    node=m.StateMachineNode(); steps=[]; original=node.advance
    def advance():
        before=dict(node.counters); prior=node.state; original()
        steps.append(dict(wall=time.monotonic(),state_before=prior,state_after=node.state,before=before,after=dict(node.counters),detections=len(node._detections),visual_tracks=len(node._visual_tracks)))
    node.advance=advance
    config=yaml.safe_load((ROOT/'catkin_ws/src/uav_tracking/config/visual_tracker.yaml').read_text())
    for k,v in config.items():rospy.set_param('/visual_tracker/'+k,v)
    launch(['/usr/bin/python3','-B',str(ROOT/'catkin_ws/src/uav_tracking/scripts/visual_tracker_node.py')],'tracker')
    rows=[]; cameras=[0]; lastimage=[None]
    def camera(msg):cameras[0]+=1;lastimage[0]=msg
    def det(msg):rows.append(dict(kind='det',wall=time.monotonic(),boxes=[[t.confidence,list(t.bbox)] for t in msg.targets]))
    def tr(msg):rows.append(dict(kind='track',wall=time.monotonic(),tracks=[[t.id,list(t.bbox),t.area_ratio] for t in msg.tracks]))
    subs=[rospy.Subscriber('/camera/image_raw',Image,camera,queue_size=2),rospy.Subscriber('/detections',Detections,det,queue_size=100),rospy.Subscriber('/visual/tracks',VisualTrackArray,tr,queue_size=100)]
    record=launch(['rosbag','record','--lz4','-O',str(OUT/'receiver.bag'),'/camera/image_raw','/detections','/visual/tracks','/detector/debug_image','/state','/clock'],'record')
    time.sleep(2)
    (OUT/'ros-graph-before.json').write_text(json.dumps(rosgraph.Master('/audit').getSystemState(),indent=2))
    source='/home/demo/m4-ort1-exit-reentry.NeAllp/live.bag'
    player=launch(['rosbag','play','--clock','--start=100','--duration=100',source,'--topics','/camera/image_raw','/camera/camera_info','/detections','/detector/debug_image'],'player')
    player.wait(timeout=125);time.sleep(2)
    assert player.returncode==0
    (OUT/'events.json').write_text(json.dumps(rows));(OUT/'m8-steps.json').write_text(json.dumps(steps))
    ds=[r for r in rows if r['kind']=='det'];ts=[r for r in rows if r['kind']=='track']
    direct=gap=0;old=None;wasempty=False
    for t in ts:
        ids=tuple(x[0] for x in t['tracks'])
        if not ids:wasempty=True;continue
        if old is not None and ids!=old:
            if wasempty:gap+=1
            else:direct+=1
        old=ids;wasempty=False
    result=dict(provenance='Existing real Gazebo bag replay, NOT new Gazebo simulation; original M8 receiver instrumented in memory, no receiver source edits',source=source,offset=100,duration=100,camera_frames=cameras[0],detections=len(ds),nonempty_detections=sum(bool(d['boxes']) for d in ds),empty_detections=sum(not d['boxes'] for d in ds),tracks=len(ts),empty_tracks=sum(not t['tracks'] for t in ts),m8_callback_steps=len(steps),states=sorted({s['state_after'] for s in steps}),lock_conf_threshold=node.conf_thresh,lock_area_threshold=node.area_thresh,N=m.sm.N_VISUAL_LOCK,M=m.sm.M_VISUAL_LOST,K=m.sm.K_RADAR_LOST,direct_output_id_changes=direct,gap_output_id_changes=gap,identity_ground_truth_switches='Not established by ID sequence alone',area_ratio_valid=all(abs(x[2]-x[1][2]*x[1][3]/(320*240))<1e-5 for t in ts for x in t['tracks']),count_beats_equal=len(ds)==len(steps),N_seconds_at_2_to_3_hz=[m.sm.N_VISUAL_LOCK/3,m.sm.N_VISUAL_LOCK/2],M_seconds_at_2_to_3_hz=[m.sm.M_VISUAL_LOST/3,m.sm.M_VISUAL_LOST/2],K_seconds_at_2_to_3_hz=[m.sm.K_RADAR_LOST/3,m.sm.K_RADAR_LOST/2])
    result['real_detection_conf_over_08']=sum(any(b[0]>.8 for b in d['boxes']) for d in ds)
    (OUT/'summary.json').write_text(json.dumps(result,indent=2))
    if lastimage[0] is not None:
        from cv_bridge import CvBridge
        import cv2
        cv2.imwrite(str(OUT/'raw-replay-frame.png'),CvBridge().imgmsg_to_cv2(lastimage[0],'bgr8'))
    print(json.dumps(result),flush=True)
finally:
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    print('EVIDENCE_DIR='+str(OUT),flush=True)
