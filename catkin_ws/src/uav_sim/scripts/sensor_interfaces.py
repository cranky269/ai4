#!/usr/bin/env python3
"""Documented camera/debug aliases; preserve native messages and account at relay publisher."""
import json,threading
from pathlib import Path
import rospy
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped
class Interfaces:
 def __init__(self):
  self.root=Path(rospy.get_param('~run_dir'));self.lock=threading.Lock()
  self.file=(self.root/'camera_publisher.csv').open('w',buffering=1)
  self.file.write('source_seq,stamp_sec,stamp_nsec,frame_id\n')
  self.pub=rospy.Publisher('/camera/image_raw',Image,queue_size=200)
  self.info=rospy.Publisher('/camera/camera_info',CameraInfo,queue_size=20)
  self.pose=rospy.Publisher('/target/pose',PoseStamped,queue_size=20)
  self.sub=rospy.Subscriber('/uav/front_camera/image_raw',Image,self.receive,queue_size=200,buff_size=64*1024*1024)
  self.others=[rospy.Subscriber('/uav/front_camera/camera_info',CameraInfo,self.info.publish,queue_size=20),rospy.Subscriber('/target/ground_truth/pose',PoseStamped,self.pose.publish,queue_size=20)]
  rospy.set_param('~ready',True)
 def receive(self,m):
  with self.lock:
   self.file.write('%d,%d,%d,%s\n'%(m.header.seq,m.header.stamp.secs,m.header.stamp.nsecs,m.header.frame_id))
   self.pub.publish(m)
 def close(self):
  self.sub.unregister()
  with self.lock:self.file.close()
if __name__=='__main__':
 rospy.init_node('sensor_interfaces');node=Interfaces()
 try:rospy.spin()
 finally:node.close()
