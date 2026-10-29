#!/usr/bin/env python3
"""Run controlled real Gazebo phases using the installed M0 launch.
Usage after sourcing ROS, catkin and lib/venv/env.bash:
  python3 test/uav_sim/radar_pose/run_gazebo_check.py RUN_DIR AUDIT_PLUGIN
Uses only its private ROS/Gazebo master and normally stops its process group.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import yaml
import numpy as np
from tf.transformations import euler_matrix, euler_from_matrix, quaternion_from_euler

p=argparse.ArgumentParser(); p.add_argument('run_dir'); p.add_argument('audit_plugin'); a=p.parse_args()
assert Path(a.audit_plugin).is_file() and os.access(Path(a.audit_plugin).with_name('invalid_pose_check'),os.X_OK), 'Build audit plugin and fault injector first'
root=Path(__file__).resolve().parents[3]; run=Path(a.run_dir).resolve(); run.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(root/'catkin_ws/src/uav_sim/scripts'))
from prepare_mounted_radar import prepare
prepare(root,run,root/'catkin_ws/src/uav_sim/config/radar_3d.yaml',sensor_profile='desktop')
world=ET.parse(str(run/'mounted.world')); w=world.getroot().find('world')
body=w.find("model[@name='static_uav']/link[@name='base_link']")
body_values=[.08,-.04,.06,.03,-.05,.07]
sensor_values=[.30,.02,.12,.10,-.20,.30]
body.find('pose').text=' '.join(map(str,body_values)); body.find("sensor[@name='ray_3d']/pose").text=' '.join(map(str,sensor_values))
def mat(v):
 m=euler_matrix(*v[3:]);m[:3,3]=v[:3];return m
mount=mat(body_values).dot(mat(sensor_values))
(run/'radar_pose.yaml').write_text(yaml.safe_dump(dict(body_to_sensor=mount[:3,3].tolist()+[float(x) for x in euler_from_matrix(mount)],scan_parent='static_uav::base_link')))
plugin=ET.SubElement(w,'plugin',name='independent_model_pose_audit',filename=str(Path(a.audit_plugin).resolve()))
ET.SubElement(plugin,'output').text=str(run/'model_pose.csv');world.write(str(run/'mounted.world'),encoding='utf-8',xml_declaration=True)
def port():
 with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]
ros_port=port();gz_port=port()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%ros_port,GAZEBO_MASTER_URI='http://127.0.0.1:%d'%gz_port,ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',ROS_HOME=str(run/'ros_home'),ROS_LOG_DIR=str(run/'ros'))
log=(run/'console.log').open('w')
launch=subprocess.Popen(['roslaunch','-p',str(ros_port),'uav_sim','mounted_radar.launch','world_file:='+str(run/'mounted.world'),'run_dir:='+str(run),'gui:=false','camera_view:=false','dynamic_target:=false','observer_motion:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
collector=None
try:
 import rospy
 from gazebo_msgs.srv import SetModelState,GetModelState
 from gazebo_msgs.msg import ModelState
 from std_srvs.srv import Empty
 rospy.init_node('controlled_radar_pose_validation',anonymous=True,disable_signals=True)
 deadline=time.monotonic()+160
 while not (run/'startup_health.json').exists():
  if launch.poll() is not None:raise RuntimeError('launch exited')
  if time.monotonic()>deadline:raise RuntimeError('startup timeout')
  time.sleep(.5)
 while json.loads((run/'startup_health.json').read_text())['status']!='running':
  if time.monotonic()>deadline:raise RuntimeError('health timeout')
  time.sleep(.5)
 collector=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).with_name('collect_pairs.py')),str(run/'pose_pairs.json'),'--seconds','56'])
 pause=rospy.ServiceProxy('/gazebo/pause_physics',Empty);unpause=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
 set_pose=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState);get_pose=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
 time.sleep(2)
 phases=[]
 for i,v in enumerate(([0,0,1.3,0,0,0],[1.2,-.7,2.1,0,0,0],[1.2,-.7,2.1,.18,-.12,1.1],[-.6,.8,1.7,-.15,.2,-.8])):
  pause()
  if i==3: rospy.ServiceProxy('/gazebo/reset_simulation',Empty)()
  m=ModelState(model_name='static_uav',reference_frame='world');m.pose.position.x,m.pose.position.y,m.pose.position.z=v[:3]
  q=quaternion_from_euler(*v[3:]);m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
  assert set_pose(m).success
  actual=get_pose('static_uav','world');assert actual.success
  phases.append(dict(index=i,reset=i==3,xyz=v[:3],quaternion_xyzw=q.tolist(),gazebo_service_xyz=[actual.pose.position.x,actual.pose.position.y,actual.pose.position.z]))
  unpause();time.sleep(24 if i==3 else 8)
  if i==3:
   (run/'reset_watchdog.json').write_text((run/'startup_health.json').read_text())
 collector.wait(timeout=10)
 assert not (run/'startup_error.txt').exists(), 'watchdog rejected live reset'
 assert launch.poll() is None, 'launch exited during reset validation'
 (run/'phases.json').write_text(json.dumps(phases,indent=2))
 rospy.set_param('/fixture_bridge/body_to_sensor',[.3,0.,.12,0.,0.,0.])
 rospy.set_param('/fixture_bridge/scan_parent','static_uav::base_link')
 fixture_bridge=subprocess.Popen(['rosrun','uav_sim','ray_cloud_bridge','__name:=fixture_bridge',
     '_publish_uav_pose:=true','_gazebo_topic:=/gazebo/base_scene/pose_fixture/scan',
     '_cloud_topic:=/fixture/points','_uav_pose_topic:=/fixture/pose'],stdout=log,stderr=subprocess.STDOUT)
 try:
  time.sleep(2)
  subprocess.run([str(Path(a.audit_plugin).resolve().with_name('invalid_pose_check')),
      '_output:='+str(run/'invalid_pose.json')],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
 finally:
  fixture_bridge.send_signal(signal.SIGINT);fixture_bridge.wait(timeout=10)
finally:
 if collector and collector.poll() is None:collector.terminate();collector.wait(timeout=10)
 if launch.poll() is None:os.killpg(launch.pid,signal.SIGINT)
 try: launch.wait(timeout=45)
 except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=15)
 log.close()
(run/'test_exit.json').write_text(json.dumps(dict(launch_returncode=launch.returncode)))
assert launch.returncode==0, 'launch did not stop normally'
print(run)
