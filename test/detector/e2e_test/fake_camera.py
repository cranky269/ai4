#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假相机（E2E 测试用）：发布 ``/camera/image_raw``（``sensor_msgs/Image``）。

在无 Gazebo 相机插件、无真机相机时，为 ``/detector`` 提供可控图像源。

参数（rosparam，均可选）
------------------------
:~rate       float  发布频率 Hz            默认 10.0
:~width      int    画面宽                 默认 640
:~height     int    画面高                 默认 480
:~image_path str    若提供且可读，则循环发布该图片（真实检出用）
:~pattern    str    无图片时的合成图案：``blank``（纯灰）| ``rect``（灰底白框）
:~frame_id   str    坐标系                 默认 "camera_optical_frame"

说明
----
- ``blank`` 用于验证「空也发」与链路连通（**不需要模型能检出目标**）；
- ``rect`` 画一个白色矩形，仅供肉眼/调试查看，**不保证 YOLO 能检出**；
- 要验证「确实能检出」，请用 ``~image_path`` 指向一张含真实目标的图片/视频首帧。
"""
from __future__ import annotations

import os

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

TOPIC = "/camera/image_raw"


class FakeCamera:
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))
        self.image_path = rospy.get_param("~image_path", "")
        self.pattern = str(rospy.get_param("~pattern", "blank")).lower()
        self.frame_id = rospy.get_param("~frame_id", "camera_optical_frame")

        self.bridge = CvBridge()
        self.frame = self._build_frame()
        self.pub = rospy.Publisher(TOPIC, Image, queue_size=1)
        rospy.loginfo("[fake_camera] rate=%.1fHz %dx%d pattern=%s image=%s",
                      self.rate_hz, self.width, self.height, self.pattern,
                      self.image_path or "(none)")

    def _build_frame(self):
        if self.image_path:
            if os.path.isfile(self.image_path):
                import cv2
                img = cv2.imread(self.image_path)
                if img is not None:
                    return img
                rospy.logwarn("[fake_camera] 图片读取失败，回退合成图案: %s", self.image_path)
            else:
                rospy.logwarn("[fake_camera] 图片不存在，回退合成图案: %s", self.image_path)

        frame = np.full((self.height, self.width, 3), 127, dtype=np.uint8)
        if self.pattern == "rect":
            x0, y0 = int(self.width * 0.4), int(self.height * 0.4)
            x1, y1 = int(self.width * 0.6), int(self.height * 0.65)
            frame[y0:y1, x0:x1] = (255, 255, 255)
        return frame

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            out = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.frame_id
            self.pub.publish(out)
            rate.sleep()


def main():
    rospy.init_node("fake_camera")
    FakeCamera().spin()


if __name__ == "__main__":
    main()
