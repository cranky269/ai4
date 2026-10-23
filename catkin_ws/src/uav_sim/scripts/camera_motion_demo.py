#!/usr/bin/env python3
"""只用Gazebo SetModelState移动整个已有静态无人机；不实现飞控。"""
import json
import math
import threading
import time

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


def smooth(a, b, t):
    t = min(1., max(0., t))
    return a + (b - a) * t * t * (3 - 2 * t)


def path(t):
    # wall seconds, x/y metres, yaw radians; all commands remain well before x=2m boards.
    if t < 3: return 'initial_hold', (0., 0., .12, 0.)
    if t < 6: return 'translate_left', (0., smooth(0, .25, (t-3)/3), .12, 0.)
    if t < 9: return 'translate_right', (0., smooth(.25, -.25, (t-6)/3), .12, 0.)
    if t < 11: return 'translate_center', (0., smooth(-.25, 0, (t-9)/2), .12, 0.)
    if t < 14: return 'approach', (smooth(0, .45, (t-11)/3), 0., .12, 0.)
    if t < 17: return 'retreat', (smooth(.45, 0, (t-14)/3), 0., .12, 0.)
    if t < 20: return 'yaw_left', (0., 0., .12, smooth(0, .12, (t-17)/3))
    if t < 22: return 'yaw_center', (0., 0., .12, smooth(.12, 0, (t-20)/2))
    return 'final_hold', (0., 0., .12, 0.)


class Demo:
    def __init__(self):
        self.lock = threading.RLock()
        self.mode = 'idle'
        self.epoch = None
        self.home_start = None
        self.actual = None
        self.state = {}
        self.tf = tf2_ros.TransformBroadcaster()
        self.pose_pub = rospy.Publisher('/camera_motion_demo/actual_pose', PoseStamped, queue_size=20)
        self.state_pub = rospy.Publisher('/camera_motion_demo/state', String, queue_size=10, latch=True)
        self.get = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.Service('/camera_motion_demo/run', Trigger, self.run)
        rospy.Service('/camera_motion_demo/stop', Trigger, self.stop)
        rospy.Service('/camera_motion_demo/home', Trigger, self.home)
        rospy.Service('/camera_motion_demo/status', Trigger, self.status)

    def coordinates(self):
        p = self.actual.pose
        return p.position.x, p.position.y, p.position.z, 2*math.atan2(p.orientation.z, p.orientation.w)

    def run(self, _):
        with self.lock:
            if self.mode != 'idle' or self.actual is None:
                return TriggerResponse(False, '演示正在运行或Gazebo状态尚未就绪')
            p = self.coordinates()
            if max(abs(p[0]), abs(p[1]), abs(p[2]-.12), abs(p[3])) > .015:
                return TriggerResponse(False, '请先执行home缓慢回到初始位姿，再run')
            self.mode, self.epoch = 'demo', time.monotonic()
        return TriggerResponse(True, '已触发26秒整机位姿演示；不是飞控闭环飞行')

    def stop(self, _):
        with self.lock:
            self.mode = 'idle'
        return TriggerResponse(True, '已取消后续运动命令，保留当前位姿；Gazebo和相机保持运行')

    def home(self, _):
        with self.lock:
            if self.actual is None:
                return TriggerResponse(False, 'Gazebo状态尚未就绪')
            p = self.coordinates()
            if not (-.02 <= p[0] <= .5 and abs(p[1]) <= .3 and abs(p[2]-.12)<.02 and abs(p[3])<=.16):
                return TriggerResponse(False, '当前位置超出演示区域；不自动移动')
            self.mode, self.epoch, self.home_start = 'home', time.monotonic(), p
        return TriggerResponse(True, '3秒缓慢回初始位姿')

    def status(self, _):
        with self.lock:
            return TriggerResponse(True, json.dumps(self.state, ensure_ascii=False))

    def loop(self):
        while not rospy.is_shutdown():
            start = time.monotonic()
            # Single controller thread owns SetModelState; a stop cannot race a new command.
            with self.lock:
                elapsed = start-self.epoch if self.epoch is not None else 0.
                phase, desired = 'idle', None
                if self.mode == 'demo':
                    phase, desired = path(elapsed)
                    if elapsed >= 26:
                        self.mode, phase = 'idle', 'complete'
                elif self.mode == 'home':
                    phase = 'return_home'
                    desired = tuple(smooth(a,b,elapsed/3) for a,b in zip(self.home_start,(0,0,.12,0)))
                    if elapsed >= 3: self.mode = 'idle'
                if desired is not None:
                    x,y,z,yaw = desired
                    if not (-.02<=x<=.5 and abs(y)<=.3 and abs(z-.12)<.02 and abs(yaw)<=.16):
                        raise ValueError('pose outside fixed demonstration bounds')
                    command = ModelState(model_name='static_uav', reference_frame='world')
                    command.pose.position.x,command.pose.position.y,command.pose.position.z = x,y,z
                    command.pose.orientation.z,command.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
                    # Static kinematic model: no gravity/dynamics and no residual commanded twist.
                    try:
                        result = self.set(command)
                    except rospy.ServiceException as exc:
                        rospy.logwarn_throttle(5, '观测机位姿写入暂时失败，重试: %s', str(exc))
                        time.sleep(.1)
                        continue
                    if not result.success: raise RuntimeError(result.status_message)
            try:
                actual = self.get('static_uav','world')
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5, '读取观测机位姿暂时失败，等待重试: %s', str(exc))
                time.sleep(.1)
                continue
            if actual.success:
                with self.lock:
                    self.actual = actual
                    self.state = {'mode':self.mode,'phase':phase,'elapsed_wall_s':elapsed,
                                  'actual_xyz_yaw':self.coordinates(),'stamp':actual.header.stamp.to_sec(),
                                  'desired_xyz_yaw':desired,'method':'Gazebo SetModelState; static model'}
                    state_json = json.dumps(self.state)
                pose=PoseStamped();pose.header=actual.header;pose.header.frame_id='world';pose.pose=actual.pose
                self.pose_pub.publish(pose)
                t=TransformStamped();t.header=pose.header;t.child_frame_id='uav_base_link'
                t.transform.translation.x,t.transform.translation.y,t.transform.translation.z = actual.pose.position.x,actual.pose.position.y,actual.pose.position.z
                t.transform.rotation=actual.pose.orientation
                self.tf.sendTransform(t)
                self.state_pub.publish(String(data=state_json))
            time.sleep(max(.001,.05-(time.monotonic()-start)))


if __name__ == '__main__':
    rospy.init_node('world_uav_tf')
    rospy.wait_for_service('/gazebo/get_model_state',timeout=90)
    rospy.wait_for_service('/gazebo/set_model_state',timeout=90)
    Demo().loop()
