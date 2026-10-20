#!/usr/bin/env python3
"""仅加载本地真实Ray传感器/标定碰撞平面，不计算传感器输出。"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

rospy.init_node('spawn_planar_lidar')
mean=float(rospy.get_param('~noise_mean',0.0))
sigma=float(rospy.get_param('~noise_stddev',0.0))
if not math.isfinite(mean) or not math.isfinite(sigma) or sigma<0:
    raise ValueError('高斯mean须有限，stddev须有限且非负')
sdf=ET.parse(rospy.get_param('~model_file'))
sdf.find('.//noise/mean').text=str(mean)
sdf.find('.//noise/stddev').text=str(sigma)
rospy.wait_for_service('/gazebo/spawn_sdf_model',timeout=60)
spawn=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
def load(name,xml,x,y,z):
    pose=Pose(); pose.position.x=x; pose.position.y=y; pose.position.z=z; pose.orientation.w=1
    result=spawn(name,xml,'/lidar',pose,'world')
    if not result.success: raise RuntimeError(name+': '+result.status_message)
load('planar_lidar',ET.tostring(sdf.getroot(),encoding='unicode'),0,-2,1)
if rospy.get_param('~calibration',False):
    board='<sdf version="1.6"><model name="lidar_calibration_plane"><static>true</static><link name="plane_link"><collision name="plane_collision"><geometry><box><size>0.2 4 2</size></box></geometry></collision><visual name="plane_visual"><geometry><box><size>0.2 4 2</size></box></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual></link></model></sdf>'
    load('lidar_calibration_plane',board,5.1,-2,1)
rospy.loginfo('已加载真实平面Ray：noise mean=%g m/stddev=%g m；标定=%s',mean,sigma,rospy.get_param('~calibration',False))
# 保持进程存活供launch错误监控；测距完全由Gazebo插件发布。
rospy.spin()
