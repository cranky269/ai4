#!/usr/bin/env python3
"""Bounded startup and live data watchdog; never equate process presence with health."""
import json
import queue
import threading
import time
from pathlib import Path

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetPhysicsProperties
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
from std_srvs.srv import Empty


def bounded(call, timeout=10):
    result = queue.Queue(1)
    def work():
        try: result.put((True, call()))
        except Exception as exc: result.put((False, exc))
    threading.Thread(target=work, daemon=True).start()
    try: ok, value = result.get(timeout=timeout)
    except queue.Empty: raise RuntimeError('Gazebo service did not respond within %ss' % timeout)
    if not ok: raise value
    return value


if __name__ == '__main__':
    rospy.init_node('start_sensor_clock')
    root = Path(rospy.get_param('~run_dir'))
    lock = threading.Lock()
    stamps = {k: set() for k in ('clock', 'camera', 'radar')}
    last = {}; last_stamp = {}; positions = {}; moved = set()
    clock_reset_ceiling = None
    required_models = {'static_uav'}
    if rospy.get_param('~target_motion', True): required_models.add('target_uav')
    motion_models = set(required_models)
    if not rospy.get_param('~observer_motion', True): motion_models.discard('static_uav')
    def note(kind, stamp):
        global clock_reset_ceiling
        with lock:
            value = stamp.to_nsec()
            if kind == 'radar' and value < last_stamp.get(kind, value):
                # Gazebo ROS may suppress /clock after reset until simulation
                # catches the previous publish time. Keep actual ages intact;
                # only suspend the clock-only stall check during that interval.
                clock_reset_ceiling = last_stamp.get('clock')
            if kind == 'clock' and value < last_stamp.get(kind, value):
                clock_reset_ceiling = None
            if value != last_stamp.get(kind):
                last_stamp[kind] = value
                last[kind] = time.monotonic()
                # Only two distinct stamps are needed; last tracks subsequent progress.
                if len(stamps[kind]) < 2: stamps[kind].add(value)
    def models(msg):
        with lock:
            for name in required_models:
                if name not in msg.name: continue
                p = msg.pose[msg.name.index(name)].position
                xyz = (p.x, p.y, p.z)
                if name in positions and sum((a-b)**2 for a,b in zip(xyz, positions[name])) > 1e-6:
                    moved.add(name)
                positions[name] = xyz
    subs = [rospy.Subscriber('/clock', Clock, lambda m: note('clock', m.clock), queue_size=1),
            rospy.Subscriber('/camera/image_raw', Image, lambda m: note('camera', m.header.stamp), queue_size=1, buff_size=4*1024*1024),
            rospy.Subscriber('/radar/points', PointCloud2, lambda m: note('radar', m.header.stamp), queue_size=1),
            rospy.Subscriber('/gazebo/model_states', ModelStates, models, queue_size=1)]
    required = ['/cluster/ready', '/sensor_interfaces/ready', '/sensor_source_audit/ready']
    if rospy.get_param('~recorder_required', False): required.append('/radar_recorder/ready')
    if rospy.get_param('~viewer_required', False): required.append('/sensor_viewer/ready')
    try:
        deadline = time.monotonic() + 150
        while not rospy.is_shutdown():
            missing = [k for k in required if not rospy.get_param(k, False)]
            if not missing: break
            if time.monotonic() > deadline: raise RuntimeError('startup timed out, missing readiness: '+str(missing))
            rospy.rostime.wallsleep(.1)
        rospy.wait_for_service('/gazebo/unpause_physics', timeout=60)
        bounded(rospy.ServiceProxy('/gazebo/unpause_physics', Empty))
        deadline = time.monotonic() + 120
        healthy = False
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                fresh = {k: now-last.get(k, 0) for k in stamps}
                clock_catching_up = (clock_reset_ceiling is not None and
                                     last_stamp.get('radar', 0) < clock_reset_ceiling)
                watched_ages = [age for k, age in fresh.items()
                                if not (clock_catching_up and k == 'clock')]
                ready = all(len(v) >= 2 for v in stamps.values()) and motion_models <= moved
                payload = dict(status='running' if ready else 'starting', ages_wall_s=fresh,
                               actual_positions=positions.copy(), moving_models=sorted(moved),
                               recording=rospy.get_param('~recorder_required', False),
                               clock_reset_catching_up=clock_catching_up)
            if rospy.get_param('~gui_required', False):
                payload['gui_view_ready'] = (root/'spectator_view.txt').exists()
                ready = ready and payload['gui_view_ready']
            if rospy.get_param('~viewer_required', False):
                try:
                    displayed = json.loads((root/'viewer_health.json').read_text())['displayed']
                except (OSError, ValueError, KeyError): displayed = {}
                payload['viewer_frames'] = displayed
                ready = ready and all(displayed.get(k, 0) >= 2 for k in ('camera', 'radar'))
            payload['status'] = 'running' if ready else 'starting'
            (root/'startup_health.json').write_text(json.dumps(payload, indent=2))
            if not healthy:
                if ready:
                    healthy = True
                    rospy.loginfo('READY: simulation clock, both model poses, camera and mounted radar actually updating')
                elif now > deadline: raise RuntimeError('startup data/motion timeout: '+str(payload))
            elif max(watched_ages) > 20:
                physics = bounded(rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties))
                if physics.pause:
                    rospy.logwarn_throttle(10, 'Gazebo is paused; live watchdog waiting for resume')
                else: raise RuntimeError('unpaused simulation/data stream stalled: '+str(payload))
            rospy.rostime.wallsleep(.5)
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        (root/'startup_error.txt').write_text(str(exc)+'\n')
        rospy.logfatal('Full simulation health failed: %s', exc)
        raise
