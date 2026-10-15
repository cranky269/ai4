#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``/camera_node`` E2E 断言：订阅 ``/camera/image_raw`` 并检查契约。

检查项
------
1. **消息数量**达到下限 —— 话题在流（不是静默无输出）；
2. **发布频率**接近 ``~fps``（容差 ±30%）—— 定频出图生效；
3. ``encoding == bgr8`` —— 与 ``/detector`` 的 ``image_msg_to_array`` 约定一致；
4. **分辨率**等于 ``~width`` × ``~height``；
5. ``header.frame_id`` 等于 ``~frame_id``；
6. **图像内容非空**（``max > 0``，排除全黑/未初始化的空图）;
7. ``~mode:=video`` 时额外检查**帧确实在变**（多帧指纹不全相同）。

退出码：``0`` = 全通过；``1`` = 有失败项。

用法::

    python3 check_camera.py _mode:=video _duration:=6.0 _min_msgs:=20 _fps:=15.0
"""
import hashlib
import sys

import rospy
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - 环境相关
    CvBridge = None

DEFAULT_TOPIC = "/camera/image_raw"


def main():
    rospy.init_node("check_camera", anonymous=True, disable_signals=True)

    topic = rospy.get_param("~topic", DEFAULT_TOPIC)
    mode = str(rospy.get_param("~mode", "video")).lower()
    duration = float(rospy.get_param("~duration", 6.0))
    min_msgs = int(rospy.get_param("~min_msgs", 20))
    expect_fps = float(rospy.get_param("~fps", 15.0))
    expect_w = int(rospy.get_param("~width", 320))
    expect_h = int(rospy.get_param("~height", 240))
    expect_frame = str(rospy.get_param("~frame_id", "camera_optical_frame"))

    if CvBridge is None:
        print("FAIL: 无法 import cv_bridge")
        return 1
    bridge = CvBridge()

    stamps = []
    metas = []          # (w, h, encoding, frame_id, max_pixel, mean)
    fingerprints = []

    def on_image(msg):
        if len(stamps) >= 400:
            return
        stamps.append(rospy.get_time())
        arr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        metas.append((
            msg.width, msg.height, msg.encoding, msg.header.frame_id,
            int(arr.max()), round(float(arr.mean()), 2),
        ))
        fingerprints.append(hashlib.md5(arr.tobytes()).hexdigest())

    rospy.Subscriber(topic, Image, on_image, queue_size=50)
    print("[check] mode=%s 观察 %.1fs，话题 %s ..." % (mode, duration, topic))
    rospy.sleep(duration)

    failures = []
    n = len(stamps)

    # 1. 数量
    print("[check] 收到 %d 条 %s（期望 >= %d）" % (n, topic, min_msgs))
    if n < min_msgs:
        failures.append("消息数不足：%d < %d" % (n, min_msgs))

    # 2. 频率
    if n >= 2:
        span = stamps[-1] - stamps[0]
        rate = (n - 1) / span if span > 0 else 0.0
        lo, hi = expect_fps * 0.7, expect_fps * 1.3
        print("[check] 实测频率 %.2f Hz（期望 %.1f，容差 %.1f~%.1f）"
              % (rate, expect_fps, lo, hi))
        if not lo <= rate <= hi:
            failures.append("频率 %.2f Hz 超出 %.1f~%.1f" % (rate, lo, hi))
    else:
        failures.append("消息太少，无法测频率")

    if metas:
        # 3~6
        w, h, enc, fid, mx, mn = metas[0]
        print("[check] 首帧: %dx%d enc=%s frame_id=%s max=%d mean=%.2f"
              % (w, h, enc, fid, mx, mn))

        if enc != "bgr8":
            failures.append("encoding=%s，期望 bgr8" % enc)
        else:
            print("PASS: encoding=bgr8")

        if (w, h) != (expect_w, expect_h):
            failures.append("分辨率 %dx%d，期望 %dx%d" % (w, h, expect_w, expect_h))
        else:
            print("PASS: 分辨率 = %dx%d" % (w, h))

        if fid != expect_frame:
            failures.append("frame_id=%r，期望 %r" % (fid, expect_frame))
        else:
            print("PASS: frame_id = %s" % fid)

        mn_max = min(m[4] for m in metas)
        if mn_max <= 0:
            failures.append("存在全黑帧（min(max)=%d）" % mn_max)
        else:
            print("PASS: 图像内容非空（各帧 max 最小值 = %d）" % mn_max)

        # 7. video 模式下帧应在变化
        if mode == "video":
            uniq = len(set(fingerprints))
            if uniq <= 1:
                failures.append("video 模式但 %d 帧完全相同（视频没在播放）" % n)
            else:
                print("PASS: 视频帧在变化（%d 帧中 %d 种不同内容）" % (n, uniq))

    print("----------------------------------------")
    if failures:
        for f in failures:
            print("FAIL: %s" % f)
        print("[check] 共 %d 项失败" % len(failures))
        return 1
    print("[check] 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
