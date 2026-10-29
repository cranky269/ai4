#!/usr/bin/env python3
"""Find live M0 instances by master identity and live publishers, never folder age."""
import argparse
from pathlib import Path
import shlex
import sys
from urllib.parse import urlparse
import xmlrpc.client

TOPICS = {'/radar/points': 'sensor_msgs/PointCloud2',
          '/radar/uav_pose_world': 'geometry_msgs/PoseStamped'}
CALLER = '/ai4_active_instance_check'


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.5
        return connection


def proxy(uri):
    return xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport())


def value(result):
    if result[0] != 1:
        raise ValueError(str(result[1]))
    return result[2]


def connection_env(path):
    """Accept only the literal export assignments produced by the launcher."""
    allowed = {'ROS_MASTER_URI', 'GAZEBO_MASTER_URI', 'ROS_IP', 'ROS_HOSTNAME', 'GAZEBO_IP'}
    env = {}
    for line in path.read_text().splitlines():
        tokens = shlex.split(line)
        if len(tokens) < 2 or tokens[0] != 'export' or any('=' not in token for token in tokens[1:]):
            raise ValueError('invalid connection file')
        for token in tokens[1:]:
            key, val = token.split('=', 1)
            if key not in allowed or any(c in val for c in ('$', '`', '\n', ';')):
                raise ValueError('unexpected connection assignment')
            env[key] = val
    uri = urlparse(env['ROS_MASTER_URI'])
    if uri.scheme != 'http' or uri.hostname not in ('127.0.0.1', 'localhost') or not uri.port:
        raise ValueError('not a local launcher master')
    return env


def inspect_run(run):
    env = connection_env(run/'connection.bash')
    master = proxy(env['ROS_MASTER_URI'])
    value(master.getPid(CALLER))
    identity = value(master.getParam(CALLER, '/sensor_interfaces/run_dir'))
    if Path(identity).resolve() != run.resolve():
        raise ValueError('master belongs to another run')
    advertised = dict(value(master.getPublishedTopics(CALLER, '/')))
    publishers = dict(value(master.getSystemState(CALLER))[0])
    for topic, msg_type in TOPICS.items():
        if advertised.get(topic) != msg_type or not publishers.get(topic):
            raise ValueError('required topic not published: '+topic)
        live = False
        for node in publishers[topic]:
            try:
                uri = value(master.lookupNode(CALLER, node))
                value(proxy(uri).getPid(CALLER))
                live = True
                break
            except (OSError, ValueError, xmlrpc.client.Error):
                continue
        if not live:
            raise ValueError('publisher is not alive: '+topic)
    return env


def discover(data_root):
    active, rejected = [], []
    for connection in sorted(data_root.glob('*/connection.bash')):
        run = connection.parent
        try:
            active.append((run.resolve(), inspect_run(run)))
        except (OSError, ValueError, KeyError, xmlrpc.client.Error) as exc:
            rejected.append((run.name, str(exc)))
    return active, rejected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', help='exact run directory name, as printed by --list')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[4]
    active, rejected = discover(root/'data/radar')
    if args.list:
        for run, env in active:
            print(run.name+'  '+env['ROS_MASTER_URI'])
        if active:
            return 0
    if not active:
        print('没有可用的活动仿真实例（master 身份、发布者和所需话题未全部就绪）。'
              '请先启动：bash catkin_ws/src/uav_sim/scripts/start_mounted_radar.sh；已启动则等待就绪。', file=sys.stderr)
        for name, why in rejected:
            print('  '+name+': '+why, file=sys.stderr)
        return 1
    selected = [item for item in active if item[0].name == args.run] if args.run else active
    if len(selected) != 1:
        print('存在多个活动实例或指定实例不可用；请选择以下活动实例的完整命令：', file=sys.stderr)
        for run, env in active:
            print('  '+run.name+'  '+env['ROS_MASTER_URI'], file=sys.stderr)
            for action in ('pose', 'record'):
                print('  bash catkin_ws/src/uav_sim/scripts/sensor_live.sh '+action+
                      ' --run '+shlex.quote(run.name), file=sys.stderr)
        return 2
    print(selected[0][0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
