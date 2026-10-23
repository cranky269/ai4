#!/usr/bin/env python3
"""Only trigger the existing 26 wall-second preset; preserve its motion implementation."""
import time
import rospy
from std_srvs.srv import Trigger

if __name__ == '__main__':
    rospy.init_node('repeat_camera_motion')
    rospy.wait_for_service('/camera_motion_demo/run',timeout=90)
    run = rospy.ServiceProxy('/camera_motion_demo/run',Trigger)
    # Service is advertised before Gazebo pose becomes available.
    deadline = time.monotonic()+90
    while not rospy.is_shutdown():
        result = run()
        if not result.success:
            if time.monotonic() > deadline:
                raise RuntimeError(result.message)
            time.sleep(.2)
            continue
        rospy.loginfo('Existing observer preset triggered: %s',result.message)
        until = time.monotonic()+28
        while not rospy.is_shutdown() and time.monotonic()<until:
            time.sleep(.1)
        deadline = time.monotonic()+5
