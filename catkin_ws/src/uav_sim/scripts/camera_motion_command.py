#!/usr/bin/python3
"""对当前唯一Gazebo实例触发/停止演示；本脚本不会启动仿真。"""
import json
import os
from pathlib import Path
import sys

root=Path(__file__).resolve().parents[4]
action=sys.argv[1] if len(sys.argv)==2 else 'status'
if action not in ['run','stop','home','status']:
    sys.exit('用法：camera_motion_command.py [run|stop|home|status]')
state=json.loads((root/'reports/unified-static-runs/active.json').read_text())
if Path('/proc/'+str(state['pid'])+'/stat').read_text().rsplit(')',1)[1].split()[19]!=state['birth']:
    sys.exit('活动实例身份不匹配，拒绝连接')
lines=(Path(state['log'])/'scene.log').read_text(errors='replace').splitlines()
uri=next(line.split('=',1)[1] for line in lines if line.startswith('ROS_MASTER_URI='))
os.environ.update(ROS_MASTER_URI=uri,ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1')
sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
import rospy
from std_srvs.srv import Trigger
rospy.init_node('camera_demo_command',anonymous=True)
service='/camera_motion_demo/'+action
rospy.wait_for_service(service,timeout=10)
result=rospy.ServiceProxy(service,Trigger)()
print(result.message)
sys.exit(0 if result.success else 1)
