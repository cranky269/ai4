#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假检测源（E2E 测试用）：发布 ``/detections``（``uav_tracking/Detections``）。

在无相机 / 无 YOLO 模型时，直接构造检测消息，打通
``/detections → /visual_tracker → /visual/tracks`` 链路。

参数（rosparam，均可选）
------------------------
:~rate         float  发布频率 Hz           默认 10.0
:~with_target  bool   是否包含目标          默认 True
:~x / ~y       float  目标 bbox 左上角      默认 280.0 / 200.0
:~w / ~h       float  目标 bbox 宽 / 高     默认 64.0 / 48.0
:~conf         float  置信度                默认 0.9
:~class_id     int    类别 ID               默认 0
:~vx / ~vy     float  目标每帧位移（测 ID 稳定性用）默认 4.0 / 3.0
:~max_step     int    位移到多少帧后重置     默认 0（不重置）
:~noise        float  置信度抖动幅度          默认 0.0
:~drop_every   int    每 N 帧丢一帧（模拟漏检，0=不丢）默认 0
"""
from __future__ import annotations

import rospy
from uav_tracking.msg import DetectedTarget, Detections

TOPIC = "/detections"


class FakeDetections:
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.with_target = rospy.get_param("~with_target", True)
        self.x = rospy.get_param("~x", 280.0)
        self.y = rospy.get_param("~y", 200.0)
        self.w = rospy.get_param("~w", 64.0)
        self.h = rospy.get_param("~h", 48.0)
        self.conf = rospy.get_param("~conf", 0.9)
        self.class_id = rospy.get_param("~class_id", 0)
        self.vx = rospy.get_param("~vx", 4.0)
        self.vy = rospy.get_param("~vy", 3.0)
        self.max_step = int(rospy.get_param("~max_step", 0))
        self.noise = rospy.get_param("~noise", 0.0)
        self.drop_every = int(rospy.get_param("~drop_every", 0))

        self.pub = rospy.Publisher(TOPIC, Detections, queue_size=1)
        self.step = 0
        self.rng = __import__("random").Random(0)
        rospy.loginfo("[fake_detections] rate=%.1fHz with_target=%s bbox=(%.0f,%.0f,%.0f,%.0f)",
                      self.rate_hz, self.with_target, self.x, self.y, self.w, self.h)

    def _current_bbox(self):
        """按 vx/vy 逐步位移；max_step > 0 时走到头就复位（循环）。"""
        if self.max_step > 0:
            self.step = self.step % self.max_step
        bx = self.x + self.vx * self.step
        by = self.y + self.vy * self.step
        self.step += 1
        return bx, by

    def build_msg(self):
        msg = Detections()
        if not self.with_target:
            return msg                    # 「空也发」：仍返回合法空消息
        t = DetectedTarget()
        t.class_id = int(self.class_id)
        conf = float(self.conf)
        if self.noise:
            conf = max(0.0, min(1.0, conf + self.rng.uniform(-self.noise, self.noise)))
        t.confidence = conf
        bx, by = self._current_bbox()
        t.bbox = [float(bx), float(by), float(self.w), float(self.h)]
        msg.targets.append(t)
        return msg

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        n = 0
        while not rospy.is_shutdown():
            if self.drop_every and n % self.drop_every == 0:
                n += 1
                rate.sleep()
                continue                  # 丢一帧：验证 buffer 内 ID 不丢
            self.pub.publish(self.build_msg())
            n += 1
            rate.sleep()


def main():
    rospy.init_node("fake_detections")
    FakeDetections().spin()


if __name__ == "__main__":
    main()
