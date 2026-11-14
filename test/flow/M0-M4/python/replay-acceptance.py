"""Replay recorded M0 images through the delivered M4 in an isolated ROS master."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--model', required=True, type=Path)
    parser.add_argument('--ort-deps', required=True, type=Path)
    parser.add_argument('--start', type=float, default=110)
    parser.add_argument('--duration', type=float, default=50)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[4]
    output = Path(tempfile.mkdtemp(prefix='m4-acceptance-replay.'))
    print('REPLAY_EVIDENCE_DIR='+str(output),flush=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:' + str(port),
               ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
               ROS_HOME=str(output/'ros'), ROS_LOG_DIR=str(output/'ros/log'))
    os.environ.update({k:v for k,v in env.items() if k.startswith('ROS_')})
    children = []
    events = []
    camera = set()
    clock = [0.]
    lock = threading.Lock()

    def launch(command, name, child_env=env):
        with (output/(name+'.log')).open('w') as log:
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                     env=child_env, start_new_session=True)
        children.append(child)
        return child

    def until(predicate, seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if predicate():
                return
            time.sleep(.1)
        raise RuntimeError('Replay precondition timed out')

    try:
        core=launch(['roscore','-p',str(port)],'master')
        import rosgraph
        master=rosgraph.Master('/m4_replay_check')
        def master_ready():
            if core.poll() is not None:
                raise RuntimeError('Isolated master exited')
            try:
                master.getPid()
                return True
            except Exception:
                return False
        until(master_ready,15)
        subprocess.run(['rosparam','set','/use_sim_time','true'],env=env,check=True)
        import rospy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image
        from uav_tracking.msg import Detections, VisualTrackArray
        rospy.init_node('m4_replay_audit',anonymous=True,disable_signals=True)
        def cb(msg,key):
            with lock:
                row={'type':key,'arrival_sim':clock[0]}
                if key=='camera':
                    camera.add(msg.header.stamp.to_nsec())
                    return
                if key=='debug':
                    row['source_ns']=msg.header.stamp.to_nsec()
                elif key=='det':
                    row['boxes']=[[x.class_id,x.confidence,list(x.bbox)] for x in msg.targets]
                else:
                    row['tracks']=[[x.id,list(x.bbox),x.area_ratio] for x in msg.tracks]
                events.append(row)
        def tick(msg):
            clock[0]=msg.clock.to_sec()
        subs=[rospy.Subscriber(n,c,cb,callback_args=k,queue_size=20,buff_size=2**22)
              for n,c,k in [('/camera/image_raw',Image,'camera'),('/detector/debug_image',Image,'debug'),
                             ('/detections',Detections,'det'),('/visual/tracks',VisualTrackArray,'track')]]
        subs.append(rospy.Subscriber('/clock',Clock,tick,queue_size=20))
        player=launch(['rosbag','play','--clock','--delay=8','--start='+str(args.start),
                       '--duration='+str(args.duration),str(args.bag),'--topics',
                       '/camera/image_raw','/camera/camera_info'],'player')
        until(lambda:'/camera/image_raw' in dict(master.getSystemState()[0]),10)
        launcher=launch(['bash',str(root/'test/flow/M0-M4/run.sh'),'--model',str(args.model),
                         '--ort-deps',str(args.ort_deps)],'m4')
        until(lambda:bool(events),30)
        player.wait(timeout=args.duration+30)
        if player.returncode or launcher.poll() is not None:
            raise RuntimeError('Playback or M4 process failed')
        time.sleep(1)
        with lock:
            rows=list(events)
        d=[x for x in rows if x['type']=='det']
        t=[x for x in rows if x['type']=='track']
        g=[x for x in rows if x['type']=='debug']
        result=dict(source=str(args.bag),bag_sha256=hashlib.sha256(args.bag.read_bytes()).hexdigest(),
                    start_offset=args.start,duration_sim=args.duration,raw_camera_frames=len(camera),
                    detections=len(d),nonempty_detections=sum(bool(x['boxes']) for x in d),
                    empty_detections=sum(not x['boxes'] for x in d),
                    tracks=len(t),nonempty_tracks=sum(bool(x['tracks']) for x in t),
                    empty_tracks=sum(not x['tracks'] for x in t),debug_frames=len(g),
                    debug_source_found=sum(x['source_ns'] in camera for x in g),
                    class_ids=sorted({z[0] for x in d for z in x['boxes']}),
                    max_debug_age_sim=max((x['arrival_sim']-x['source_ns']/1e9 for x in g),default=None))
        assert result['nonempty_detections'] and result['nonempty_tracks'],result
        assert result['empty_detections']>=10 and result['empty_tracks']>=10,result
        assert len(d)==len(t)==len(g),result
        assert result['debug_source_found']==len(g),result
        assert result['class_ids']==[0],result
        assert result['max_debug_age_sim']<=1.6,result
        assert all(0 <= tr[2] and abs(tr[2]-tr[1][2]*tr[1][3]/(320*240))<1e-5
                   for row in t for tr in row['tracks']),result
        result['passed']=True
        (output/'events.json').write_text(json.dumps(rows))
        (output/'summary.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
        print('REPLAY_EVIDENCE_DIR='+str(output),flush=True)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    child.wait(timeout=5)


if __name__=='__main__':
    main()
