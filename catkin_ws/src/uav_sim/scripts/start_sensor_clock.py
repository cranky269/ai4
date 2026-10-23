#!/usr/bin/env python3
import time
import rospy
from std_srvs.srv import Empty
if __name__=='__main__':
 rospy.init_node('start_sensor_clock')
 deadline=time.monotonic()+90
 required=['/cluster/ready','/radar_recorder/ready','/sensor_interfaces/ready','/sensor_source_audit/ready']
 if rospy.get_param('~viewer_required',False):required+=['/sensor_viewer/ready']
 while not rospy.is_shutdown():
  if all(rospy.get_param(k,False) for k in required):break
  if time.monotonic()>deadline:raise RuntimeError('recorders/viewer not ready: '+str(required))
  time.sleep(.1)
 # Connection establishment is asynchronous; sensors remain paused during this grace period.
 time.sleep(2)
 rospy.wait_for_service('/gazebo/unpause_physics',timeout=30)
 rospy.ServiceProxy('/gazebo/unpause_physics',Empty)()
 rospy.loginfo('Recorders/interfaces/native source audit ready; simulation clock running')

 rospy.spin()
