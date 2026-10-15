#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/camera_node 节点壳：相机驱动 → ``/camera/image_raw``。

职责对齐《无人机项目实施拆解文档》第二章节点表：

    ``/camera_node`` | 相机驱动 | —（无订阅） | ``/camera/image_raw``（``sensor_msgs/Image``）

仿真与真机的来源差异
--------------------
``design/函数调用关系图-flowchart.md``（1.1 版）已明确：**仿真下 Gazebo 相机插件
直接发布 ``/camera/image_raw``**，中间不需要本节点。因此本节点的定位是：

- **真机**：USB 相机 → 本节点驱动并发布 ``/camera/image_raw``；
- **仿真**：可用 ``source:=gazebo`` 做**话题中转**（把 Gazebo 发布在其它话题上的
  图像重映射到契约话题），也可完全不启动本节点；
- **过渡**：相机插件尚未配好时，用 ``source:=video`` / ``image`` 喂本地视频/图片，
  让整条视觉链（/detector → /visual_tracker → ...）**不依赖仿真环境即可跑通**。

四种数据源（``~source`` 参数）
------------------------------
============  ====================================================
``usb``       ``cv2.VideoCapture(~device_id)``，真机 USB 相机
``video``     ``cv2.VideoCapture(~video_path)``，可 ``~loop`` 循环
``image``     单张图片按 ``~fps`` 反复发布（静态场景联调）
``gazebo``    订阅 ``~gazebo_topic`` 中转（**无定时器**，事件驱动）
============  ====================================================

设计约定
--------
- **发布即契约**：只负责把图像送到 ``/camera/image_raw``，下游 ``/detector`` 无感；
- **参数外置**：分辨率、帧率、设备号、路径一律走 ``rosparam``，不写死；
- ``usb`` / ``video`` / ``image`` 三种主动源用 ``rospy.Timer`` 按 ``~fps`` 定频出图
  （采集端固有频率，与决策端的 20-50Hz 指令时钟无关）。
"""
from __future__ import annotations

import os
import sys

import rospy
from sensor_msgs.msg import Image

# 把 lib/ 加入 import 路径（与其他节点壳保持一致；本节点当前不直接用 lib/）
_LIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib")
if _LIB_DIR not in sys.path:
    sys.path.insert(0, _LIB_DIR)

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - 环境相关
    CvBridge = None

__all__ = ["FrameSource", "CameraNode"]

# --- 模块级常量（代码书写原则第 6 条：默认值 / 未指定参数集中定义）---
IMAGE_TOPIC = "/camera/image_raw"
QUEUE_SIZE = 1                     # 只保最新帧
DEFAULT_SOURCE = "video"           # 默认用视频源，便于无仿真环境时联调
DEFAULT_FPS = 30
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_DEVICE_ID = 0
SUPPORTED_SOURCES = ("usb", "video", "image", "gazebo")


class FrameSource:
    """主动型图像源（``usb`` / ``video`` / ``image``）的薄封装。

    统一 ``open`` / ``read`` / ``release`` 三个动作，使节点主循环不必区分来源。
    读取失败（文件结束、设备断开）时 ``read`` 返回 ``None``，由节点决定重试或停止。
    """

    def __init__(self, source, path=None, loop=False):
        self.source = source
        self.path = path
        self.loop = bool(loop)
        self.cap = None
        self._static_image = None

    def open(self):
        """打开数据源；失败抛 ``RuntimeError``（阶段校验由节点负责）。"""
        import cv2

        if self.source == "image":
            self._static_image = cv2.imread(self.path)
            if self._static_image is None:
                raise RuntimeError("[camera_node] 无法读取图片: %s" % (self.path,))
            return self

        target = self.path if self.source == "video" else int(self.path)
        self.cap = cv2.VideoCapture(target)
        if not self.cap.isOpened():
            raise RuntimeError("[camera_node] 无法打开 %s 源: %r" % (self.source, self.path))
        return self

    def read(self):
        """读一帧 BGR 图像；无可用帧时返回 ``None``。"""
        if self.source == "image":
            return self._static_image.copy() if self._static_image is not None else None

        ok, frame = self.cap.read()
        if not ok:
            if self.loop and self.source == "video":
                self.cap.set(1 if sys.platform == "win32" else 0, 0)  # 回到首帧
                ok, frame = self.cap.read()
            if not ok:
                return None
        return frame

    def release(self):
        """释放底层资源（幂等）。"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class CameraNode:
    """``/camera_node`` 节点：把所选数据源发布到 ``/camera/image_raw``。"""

    def __init__(self):
        self.source = str(rospy.get_param("~source", DEFAULT_SOURCE)).lower()
        self.fps = float(rospy.get_param("~fps", DEFAULT_FPS))
        self.width = rospy.get_param("~width", DEFAULT_WIDTH)
        self.height = rospy.get_param("~height", DEFAULT_HEIGHT)
        self.frame_id = rospy.get_param("~frame_id", "camera_optical_frame")
        self.image_topic = rospy.get_param("~image_topic", IMAGE_TOPIC)

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.image_topic, Image, queue_size=QUEUE_SIZE)
        self.timer = None
        self.src = None

        if self.source not in SUPPORTED_SOURCES:
            rospy.logerr("[camera_node] 不支持的 source=%r；可选 %s",
                         self.source, SUPPORTED_SOURCES)
            raise ValueError("unsupported source: %r" % (self.source,))

        if self.source == "gazebo":
            self._init_gazebo_relay()
        else:
            self._init_active_source()

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("[camera_node] ready: source=%s fps=%.1f out=%s",
                      self.source, self.fps, self.image_topic)

    # ------------------------------------------------------------------ #
    # 初始化：两种来源模式
    # ------------------------------------------------------------------ #
    def _init_active_source(self):
        """usb / video / image：主动按 fps 出图。"""
        if self.source == "video":
            path = rospy.get_param("~video_path", "")
        elif self.source == "image":
            path = rospy.get_param("~image_path", "")
        else:
            path = rospy.get_param("~device_id", DEFAULT_DEVICE_ID)

        self.src = FrameSource(
            self.source, path=path, loop=rospy.get_param("~loop", True))
        self.src.open()

        if self.source in ("usb", "video"):
            self.src.cap.set(3, int(self.width))   # CAP_PROP_FRAME_WIDTH
            self.src.cap.set(4, int(self.height))  # CAP_PROP_FRAME_HEIGHT

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.fps, 1e-3)),
                                 self.on_timer)

    def _init_gazebo_relay(self):
        """gazebo：订阅既有图像话题做中转，事件驱动、无定时器。"""
        gazebo_topic = rospy.get_param("~gazebo_topic", self.image_topic)
        if gazebo_topic == self.image_topic:
            rospy.logwarn(
                "[camera_node] gazebo_topic 与 image_topic 相同（%s）；"
                "此时无需本节点中转，建议直接不启动 /camera_node", gazebo_topic)
        rospy.Subscriber(gazebo_topic, Image, self.on_image, queue_size=QUEUE_SIZE)

    # ------------------------------------------------------------------ #
    # 事件层 / 时钟层
    # ------------------------------------------------------------------ #
    def on_timer(self, _event):
        """定时器回调（usb / video / image）：读一帧 → 发布。"""
        frame = self.src.read()
        if frame is None:
            rospy.logwarn_throttle(5.0, "[camera_node] %s 源暂无可用帧", self.source)
            return
        self._publish(frame)

    def on_image(self, msg):
        """中转回调（gazebo）：原样重发到契约话题，仅修正 header.frame_id。"""
        if not msg.header.frame_id:
            msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def _publish(self, frame):
        """BGR ndarray → ``sensor_msgs/Image`` 并发布（统一 encoding=bgr8）。"""
        try:
            out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except Exception as exc:
            rospy.logerr_throttle(5.0, "[camera_node] 图像打包失败: %s", exc)
            return
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id
        self.pub.publish(out)

    def shutdown(self):
        """退出时释放相机 / 视频句柄。"""
        if self.timer is not None:
            self.timer.shutdown()
        if self.src is not None:
            self.src.release()


def main():
    rospy.init_node("camera_node")
    CameraNode()
    rospy.spin()


if __name__ == "__main__":
    main()
