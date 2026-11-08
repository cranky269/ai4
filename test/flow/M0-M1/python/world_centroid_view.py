#!/usr/bin/env python3
"""M0-owned world viewer; preserves the legacy local-coordinate viewer.

Requires sourced ROS/catkin environment. Selects the active M0 instance before
importing rospy. Truth overlay is diagnostic and only shown at an exact stamp.
No mounting offset or rotation is applied to world data.
"""
import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time


def world_xyz(msg):
    if msg.header.frame_id != 'world':
        raise ValueError('Expected world; refusing local or unknown coordinates')
    point = msg.pose.position if hasattr(msg, 'pose') else msg.point
    xyz = (point.x, point.y, point.z)
    if not all(math.isfinite(v) for v in xyz):
        raise ValueError('Nonfinite world position')
    return xyz


def centroid_values(msg):
    # CentroidArray has no outer header. Every member must share frame and stamp.
    points = [world_xyz(p) for p in msg.centroids]
    stamps = {p.header.stamp.to_nsec() for p in msg.centroids}
    if len(stamps) > 1:
        raise ValueError('Centroid frame contains mixed acquisition stamps')
    return (next(iter(stamps)) if stamps else None), points


def connect(run_name):
    root = Path(__file__).resolve().parents[4]
    scripts = root/'catkin_ws/src/uav_sim/scripts'
    command = [sys.executable, str(scripts/'find_active_sensor_run.py')]
    if run_name:
        command.extend(['--run', run_name])
    selected = subprocess.run(command, text=True, capture_output=True)
    if selected.returncode:
        raise RuntimeError(selected.stderr.strip())
    sys.path.insert(0, str(scripts))
    from find_active_sensor_run import inspect_run
    run = Path(selected.stdout.strip())
    os.environ.update(inspect_run(run))
    return run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run')
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--with-cloud', action='store_true')
    parser.add_argument('--show-truth', action='store_true')
    parser.add_argument('--max-cloud-points', type=int, default=500)
    parser.add_argument('--save', type=Path)
    args = parser.parse_args()
    if args.max_cloud_points < 1 or not math.isfinite(args.duration) or args.duration < 0:
        parser.error('positive point limit and finite nonnegative duration required')
    run = connect(args.run)
    import rospy
    import sensor_msgs.point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped
    from uav_tracking.msg import CentroidArray
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    rospy.init_node('m0_world_centroid_view', anonymous=True)
    lock = threading.Lock()
    state = dict(stamp=None, points=[], cloud=[], truth={}, frames=0, last={})

    def receive(msg, kind):
        try:
            if kind == 'centroid':
                stamp, points = centroid_values(msg)
            else:
                if msg.header.frame_id != 'world':
                    raise ValueError('Expected world '+kind+'; original data unchanged')
                stamp = msg.header.stamp.to_nsec()
                if kind == 'truth':
                    points = world_xyz(msg)
                else:
                    points = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
                    stride = max(1, math.ceil(len(points)/args.max_cloud_points))
                    points = points[::stride]
            with lock:
                if stamp is not None:
                    if kind in state['last'] and stamp < state['last'][kind]:
                        state.update(stamp=None, points=[], cloud=[], truth={})
                    state['last'][kind] = stamp
                if kind == 'truth':
                    state['truth'][stamp] = points
                    while len(state['truth']) > 128:
                        del state['truth'][next(iter(state['truth']))]
                elif kind == 'centroid':
                    state.update(stamp=stamp, points=points, frames=state['frames']+1)
                else:
                    state['cloud'] = points
        except ValueError as exc:
            with lock:
                if kind == 'centroid':state.update(stamp=None, points=[])
                elif kind == 'cloud':state['cloud']=[]
                else:state['truth']={}
            rospy.logerr_throttle(2, str(exc))

    subs = [rospy.Subscriber('/radar/centroid', CentroidArray, receive, callback_args='centroid', queue_size=1)]
    if args.with_cloud:
        subs.append(rospy.Subscriber('/radar/points', PointCloud2, receive, callback_args='cloud', queue_size=1))
    if args.show_truth:
        subs.append(rospy.Subscriber('/target/ground_truth/pose', PoseStamped, receive, callback_args='truth', queue_size=10))
    fig = plt.figure(figsize=(9,7));ax=fig.add_subplot(111, projection='3d')
    start = time.monotonic()
    try:
        while not rospy.is_shutdown() and plt.fignum_exists(fig.number):
            if args.duration and time.monotonic()-start >= args.duration:break
            with lock:
                stamp=state['stamp'];points=list(state['points']);cloud=list(state['cloud'])
                truth=state['truth'].get(stamp);frames=state['frames']
            ax.clear()
            for rows, color, label in [(cloud,'gray','world cloud (latest)'), (points,'red','world centroids'),
                                        ([truth] if truth else [],'orange','world truth (exact stamp)')]:
                if rows:ax.scatter(*zip(*rows), c=color, s=5 if color=='gray' else 45, label=label)
            ax.set(xlabel='world X [m]',ylabel='world Y [m]',zlabel='world Z [m]',
                   xlim=(-2,12),ylim=(-6,6),zlim=(-1,4))
            ax.set_title('%s | world | frames %d | stamp_ns %s'%(run.name,frames,stamp))
            if ax.collections:ax.legend()
            plt.pause(.1)
    finally:
        if args.save:fig.savefig(str(args.save),dpi=150)
        for sub in subs:sub.unregister()
        plt.close(fig);rospy.signal_shutdown('world viewer stopped')


if __name__ == '__main__':
    try:main()
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(str(exc), file=sys.stderr);raise SystemExit(1)
