#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/visual_tracker 节点壳：``/detections`` → BoT-SORT 跟踪 → ``/visual/tracks``。

对应《python代码移植到ros平台方法整理.md》第五节与《design/函数接口.md》模块 5：

- **壳**（本文件）：订阅 / 发布 / 消息互转 / 日志，**不含跟踪算法**；
- **核**（``lib/visual_tracker.py``）：``BotSortTracker`` 纯逻辑，零改动。

设计约定（``design/函数接口.md`` 模块 5 与「决策端设计原则」）
--------------------------------------------------------
- **空也发**：输入为空时照常发布**空** ``VisualTrackArray``（``tracks=[]``），
  节拍跟 ``/detections`` 走、不静默——保证「空 = 没目标（业务正常）、
  静默 = 故障」语义成立，并为 ``/state_machine`` 的 N/M/K 帧计数提供节拍
  （设计原则第 5 条）；
- **同时订阅 ``/detections`` 与 ``/camera/image_raw``**：图像入参是
  **组长 2026-10-06 确认的补充**（与模块 4 ``detect`` 的入参同类型），
  用于取实际画面尺寸算 ``area_ratio``；GMC 属可选优化项，当前默认不开；
- **跨帧状态归节点持有**：跟踪器实例是本节点的成员变量 ``self.tracker``
  （《python代码移植到ros平台方法整理.md》8.1）；
- **画面尺寸的两级来源**：
  1. **运行期（优先）**：每帧把最新图像传给 ``track``，用 ``image.shape``
     取实际尺寸（相机换了分辨率也自动适配）；
  2. **兜底**：图像尚未到达时用 ``~width`` / ``~height`` 参数
     （默认等于模块常量 ``DEFAULT_FRAME_SIZE``）；
- **参数外置**：阈值、buffer 等一律走 ``rosparam``，不写死。
"""
from __future__ import annotations

import os
import sys

import rospy
from sensor_msgs.msg import Image

# 把 lib/ 加入 import 路径（纯逻辑与壳解耦；lib/ 内为指向 code/ 的软链）
_LIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib")
if _LIB_DIR not in sys.path:
    sys.path.insert(0, _LIB_DIR)

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - 环境相关
    CvBridge = None

from datatypes import DetectedTarget  # noqa: E402
from visual_tracker import DEFAULT_FRAME_SIZE  # noqa: E402
from visual_tracker import setup as setup_tracker  # noqa: E402  纯逻辑，零改动
from uav_tracking.msg import Detections, VisualTrack, VisualTrackArray  # noqa: E402

__all__ = [
    "detections_from_msg",
    "visual_tracks_to_msg",
    "VisualTrackerNode",
]

# --- 模块级常量（代码书写原则第 6 条：默认值 / 未指定参数集中定义）---
DETECTIONS_TOPIC = "/detections"
TRACKS_TOPIC = "/visual/tracks"
IMAGE_TOPIC = "/camera/image_raw"  # 图像入参来源（与 /detector 同源话题）
QUEUE_SIZE = 1                     # 只保最新
DEFAULT_WIDTH = DEFAULT_FRAME_SIZE[0]     # 画面宽兜底值（引用核的常量，不硬编码）
DEFAULT_HEIGHT = DEFAULT_FRAME_SIZE[1]    # 画面高兜底值


# --------------------------------------------------------------------------- #
# 纯转换函数（不依赖 ROS master，可单测）
# --------------------------------------------------------------------------- #
def detections_from_msg(msg):
    """``Detections`` 消息 → ``list[DetectedTarget]``（纯转换，可单测）。

    ``targets`` 为空时返回 ``[]``——「空 = 没目标」，不视为错误。
    """
    out = []
    for t in msg.targets:
        # bbox 在消息里是 float32[4]，解包为 (x, y, w, h) 元组
        x, y, w, h = t.bbox
        out.append(
            DetectedTarget(
                bbox=(float(x), float(y), float(w), float(h)),
                class_id=int(t.class_id),
                confidence=float(t.confidence),
            )
        )
    return out


def visual_tracks_to_msg(tracks):
    """``list[VisualTrack]`` → ``VisualTrackArray`` 消息（纯转换，可单测）。

    - ``id`` / ``bbox`` / ``area_ratio`` 三个字段与《数据类型.md》1.3 一一对应；
    - **空列表也构造出合法的空消息**，由调用方照常发布，落实「空也发」约定。

    注意：``VisualTrackArray.msg`` **无 header 字段**（《数据类型.md》1.4 照搬定义）。
    """
    msg = VisualTrackArray()
    for trk in tracks:
        item = VisualTrack()
        item.id = int(trk.id)
        x, y, w, h = trk.bbox
        item.bbox = [float(x), float(y), float(w), float(h)]
        item.area_ratio = float(trk.area_ratio)
        msg.tracks.append(item)
    return msg


# --------------------------------------------------------------------------- #
# 节点
# --------------------------------------------------------------------------- #
class VisualTrackerNode:
    """``/visual_tracker`` 节点：事件订阅 → ``BotSortTracker`` 跟踪 → 发布航迹。"""

    def __init__(self):
        # --- 参数一律走 rosparam（launch 可调，不写死）---
        self.track_high_thresh = rospy.get_param("~track_high_thresh", 0.5)
        self.track_low_thresh = rospy.get_param("~track_low_thresh", 0.1)
        self.new_track_thresh = rospy.get_param("~new_track_thresh", 0.6)
        self.track_buffer = rospy.get_param("~track_buffer", 30)
        self.match_thresh = rospy.get_param("~match_thresh", 0.8)
        self.min_area_ratio = rospy.get_param("~min_area_ratio", 0.0)
        self.width = rospy.get_param("~width", DEFAULT_WIDTH)
        self.height = rospy.get_param("~height", DEFAULT_HEIGHT)
        self.detections_topic = rospy.get_param("~detections_topic", DETECTIONS_TOPIC)
        self.image_topic = rospy.get_param("~image_topic", IMAGE_TOPIC)
        self.use_image = bool(rospy.get_param("~use_image", True))

        # --- 核：跟踪器实例 —— 跨帧状态由本节点持有（设计 8.1）---
        self.tracker = setup_tracker(
            track_high_thresh=self.track_high_thresh,
            track_low_thresh=self.track_low_thresh,
            new_track_thresh=self.new_track_thresh,
            track_buffer=self.track_buffer,
            match_thresh=self.match_thresh,
            min_area_ratio=self.min_area_ratio,
            frame_size=(self.width, self.height),   # 兜底；图像到达后按实际尺寸自动更新
        )

        # 最新一帧图像：只缓存引用，不做任何算法处理
        self.bridge = CvBridge() if CvBridge is not None else None
        self.latest_image = None

        self.pub = rospy.Publisher(TRACKS_TOPIC, VisualTrackArray, queue_size=QUEUE_SIZE)
        rospy.Subscriber(
            self.detections_topic, Detections, self.on_detections, queue_size=QUEUE_SIZE)
        if self.use_image:
            if self.bridge is None:
                rospy.logwarn(
                    "[visual_tracker] 无 cv_bridge，退回 rosparam 尺寸 %.0fx%.0f",
                    self.width, self.height)
            else:
                rospy.Subscriber(
                    self.image_topic, Image, self.on_image, queue_size=QUEUE_SIZE)

        rospy.loginfo(
            "[visual_tracker] ready: 兜底尺寸=%dx%d use_image=%s high=%.2f low=%.2f "
            "buffer=%d in=%s,%s out=%s",
            self.width, self.height, self.use_image, self.track_high_thresh,
            self.track_low_thresh, self.track_buffer,
            self.detections_topic, self.image_topic if self.use_image else "(未订阅)",
            TRACKS_TOPIC,
        )

    # ---- 事件层：缓存最新图像（供 area_ratio 取实际尺寸）----
    def on_image(self, msg):
        """缓存最新一帧图像；转换失败只告警，不影响跟踪链路。"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "[visual_tracker] 图像转换失败，沿用当前尺寸: %s", exc)

    # ---- 事件层：收到一帧检测处理一帧（本模块无定时器）----
    def on_detections(self, msg):
        """一帧检测结果 → 跟踪 → 发布。

        流程严格按「壳」的四件事：拆包 → 调核 → 装包 → 发布。
        异常时仍发布**空** ``VisualTrackArray``，不让节点静默。
        """
        try:
            detections = detections_from_msg(msg)
        except Exception as exc:
            rospy.logerr("[visual_tracker] 检测消息解析失败: %s", exc)
            self.pub.publish(visual_tracks_to_msg([]))
            return

        # 调核：这一行才是"算法"，其余全是搬运
        # 传入最新图像：核会用它的实际尺寸算 area_ratio（图像未到则为 None，用兜底尺寸）
        try:
            tracks = self.tracker.track(detections, self.latest_image)
        except Exception as exc:
            rospy.logerr("[visual_tracker] 跟踪失败: %s", exc)
            self.pub.publish(visual_tracks_to_msg([]))
            return

        # 装包发布（无航迹时 tracks=[] ——「空也发」）
        self.pub.publish(visual_tracks_to_msg(tracks))

        if not tracks:
            rospy.logdebug("[visual_tracker] 本帧无航迹（已发空 VisualTrackArray）")


def main():
    rospy.init_node("visual_tracker")
    VisualTrackerNode()
    rospy.spin()          # 回调驱动，不用写 while 循环


if __name__ == "__main__":
    main()
