#!/usr/bin/env python3
"""传感器观测用运动学目标；真值仅来自 Gazebo GetModelState。"""
import math
import threading
from pathlib import Path

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel
from geometry_msgs.msg import Pose, PoseStamped
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool, SetBoolResponse


class FigureEightTarget:
    def __init__(self):
        self.name = rospy.get_param('~model_name', 'target_uav')
        self.cx = float(rospy.get_param('~center_x', 4.0))
        self.cy = float(rospy.get_param('~center_y', 0.0))
        self.height = float(rospy.get_param('~height', 0.6))
        self.ax = float(rospy.get_param('~scale_x', 0.8))
        self.ay = float(rospy.get_param('~scale_y', 0.9))
        self.period = float(rospy.get_param('~period', 12.0))
        self.hz = float(rospy.get_param('~update_rate', 20.0))
        values = [self.cx, self.cy, self.height, self.ax, self.ay, self.period, self.hz]
        if not all(math.isfinite(v) for v in values) or min(self.height, self.ax, self.ay, self.period, self.hz) <= 0:
            raise ValueError('轨迹参数须有限，尺度/高度/周期/频率必须为正')
        # 对本轮默认模型碰撞尺寸及观测机位置作保守安全检查。
        if self.cx - self.ax < 1.0 or self.height < 0.2:
            raise ValueError('目标须保持x>=1m、模型原点高度>=0.2m，避免观测机/地面相交')
        self.lock = threading.Lock()
        self.now = None
        self.epoch = None
        self.last = None
        self.reset_pending = False
        self.paused = False
        self.pause_clock = None
        self.pause_service = rospy.Service("/target/pause", SetBool, self.pause)
        self.subscription = rospy.Subscriber('/clock', Clock, self.clock, queue_size=1)
        self.truth = rospy.Publisher('/target/ground_truth/pose', PoseStamped, queue_size=10)
        # 期望值单独标明，用于误差分析，不能替代真值。
        self.expected = rospy.Publisher('/target/expected_pose', PoseStamped, queue_size=10)

    def pause(self, request):
        with self.lock:
            if request.data and not self.paused:
                self.pause_clock = self.now
            elif not request.data and self.paused:
                if self.epoch is not None and self.pause_clock is not None and self.now is not None:
                    self.epoch += max(0.0, self.now - self.pause_clock)
                self.last = None
            self.paused = request.data
        return SetBoolResponse(True, '目标已暂停' if request.data else '目标已恢复')

    def clock(self, msg):
        t = msg.clock.to_sec()
        with self.lock:
            if self.now is not None and t < self.now:
                self.reset_pending = True
            self.now = t

    def run(self):
        for service in ['/gazebo/spawn_sdf_model', '/gazebo/set_model_state', '/gazebo/get_model_state']:
            rospy.wait_for_service(service, timeout=60)
        spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.cx, self.cy, self.height
        pose.orientation.w = 1.0
        sdf = Path(rospy.get_param('~model_file')).read_text()
        response = spawn(self.name, sdf, '/target', pose, 'world')
        if not response.success:
            raise RuntimeError('目标加载失败：' + response.status_message)
        # 墙钟轮询不受暂停/时钟回退阻塞；相位与更新节拍只由/clock推进。
        while not rospy.is_shutdown():
            rospy.rostime.wallsleep(0.005)
            with self.lock:
                t = self.now
                if t is None or t <= 0 or self.paused:
                    continue
                if self.epoch is None or self.reset_pending or (self.last is not None and t < self.last):
                    self.epoch = t
                    self.last = None
                    self.reset_pending = False
                    rospy.loginfo('目标相位起点/时钟重置: %.6f', t)
                if self.last is not None and t - self.last < 1.0 / self.hz:
                    continue
                epoch = self.epoch
                self.last = t
            phase = 2 * math.pi * (t - epoch) / self.period
            command = ModelState(model_name=self.name, reference_frame='world')
            command.pose.position.x = self.cx + self.ax * math.sin(phase)
            command.pose.position.y = self.cy + self.ay * math.sin(2 * phase)
            command.pose.position.z = self.height
            command.pose.orientation.w = 1.0
            with self.lock:
                if self.paused:
                    continue
                try:
                    applied = set_state(command)
                except rospy.ServiceException as exc:
                    rospy.logwarn_throttle(5, '目标位姿写入暂时失败，重试: %s', str(exc))
                    continue
            if not applied.success:
                raise RuntimeError('设置目标位姿失败：' + applied.status_message)
            try:
                actual = get_state(self.name, 'world')
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5, '目标位姿读取暂时失败，重试: %s', str(exc))
                continue
            if not actual.success:
                raise RuntimeError('读取目标实际状态失败：' + actual.status_message)
            expected = PoseStamped()
            expected.header.frame_id = 'world'
            expected.header.stamp = rospy.Time.from_sec(t)
            expected.pose = command.pose
            truth = PoseStamped()
            truth.header = actual.header
            truth.header.frame_id = 'world'
            truth.pose = actual.pose
            self.expected.publish(expected)
            self.truth.publish(truth)


if __name__ == '__main__':
    rospy.init_node('figure_eight_target')
    FigureEightTarget().run()
