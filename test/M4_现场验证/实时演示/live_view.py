#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M4 实时画面窗口 —— 把「带检测框的画面」和「实时状态」画在同一个窗口里。

用法：
    python3 live_view.py                       # 默认 /detector/debug_image
    python3 live_view.py /camera/image_raw     # 只看原始画面

说明：
    * 右侧是 /detector 发布的调试图（**真实的检测框**）；
    * 左上角叠一块状态面板：当前状态 / 目标数 / 航迹 ID / area_ratio / 帧率；
    * 用系统 python3 + 系统 cv2(4.2.0)，**不要**用 lib/m4_deps 里的 cv2；
    * cv2.putText 不支持中文，所以面板文字用英文，避免显示成方块。
"""
import os
import subprocess
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from uav_tracking.msg import Detections, VisualTrackArray

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/detector/debug_image"
WIN = "M4 LIVE  " + TOPIC

IMG = {"v": None}
ST = {"state": "?"}
DET = {"n": 0, "conf": 0.0, "msgs": 0}
TRK = {"ids": [], "areas": [], "msgs": 0}


def on_image(msg):
    IMG["v"] = msg


def on_state(msg):
    ST["state"] = msg.data


def on_det(msg):
    DET["msgs"] += 1
    DET["n"] = len(msg.targets)
    DET["conf"] = max([t.confidence for t in msg.targets] or [0.0])


def on_trk(msg):
    TRK["msgs"] += 1
    TRK["ids"] = [t.id for t in msg.tracks]
    TRK["areas"] = [getattr(t, "area_ratio", 0.0) for t in msg.tracks]


def state_from_log():
    """``/state`` 只在变化时发布，晚订阅会收不到 —— 从节点日志兜底读。"""
    for path in ("/tmp/m4_live/state.log",
                 os.path.join(os.path.dirname(os.path.abspath(__file__)), "state.log")):
        try:
            with open(path) as fh:
                lines = fh.readlines()
        except OSError:
            continue
        for line in reversed(lines):
            if "->" in line:
                return line.split("->")[-1].split("(")[0].strip()
            if "state=" in line:
                return line.split("state=")[-1].strip()
    return None


def draw_panel(canvas, lines, x=12, y=28):
    """在图像左上角画半透明底板 + 文字（英文，cv2 不支持中文）。"""
    h = 24
    w = 0
    for text, _color in lines:
        (tw, _th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
        w = max(w, tw)
    pad = 14
    x2, y2 = x + w + pad * 2, y + h * len(lines) + pad
    overlay = canvas.copy()
    cv2.rectangle(overlay, (x - pad, y - h - 6), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.62, canvas, 0.38, 0, canvas)
    for i, (text, color) in enumerate(lines):
        cv2.putText(canvas, text, (x, y + i * h),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)


# --- 窗口尺寸/位置：按屏幕宽度放右侧 ---
ASPECT = 1080.0 / 810.0        # 调试图 810x1080（竖版）


def screen_size(default=(1152, 864)):
    """读当前屏幕分辨率。

    为什么要在循环里反复读：VMware 的「自动适应」会**跟着宿主机窗口尺寸
    改变 guest 分辨率**。分辨率一变，原来摆好的窗口就可能跑到屏幕外，
    看着像"程序没开"。所以这里持续探测、变了就重新摆。
    """
    try:
        out = subprocess.check_output(["xdpyinfo"], stderr=subprocess.DEVNULL).decode()
        dims = [l for l in out.splitlines() if "dimensions:" in l][0].split()[1]
        return tuple(int(v) for v in dims.split("x"))
    except Exception:
        return default


def layout(sw, sh):
    """按屏幕尺寸算出窗口的位置和大小。"""
    w = int(sw * 0.34)
    h = int(w * ASPECT)
    if h > sh - 80:
        h = sh - 80
        w = int(h / ASPECT)
    return max(sw - w - 10, 0), 40, w, h


SW, SH = screen_size()
POS = layout(SW, SH)

rospy.init_node("m4_live_view", anonymous=True, disable_signals=True)
bridge = CvBridge()
rospy.Subscriber(TOPIC, Image, on_image, queue_size=1)
rospy.Subscriber("/state", String, on_state, queue_size=1)
rospy.Subscriber("/detections", Detections, on_det, queue_size=1)
rospy.Subscriber("/visual/tracks", VisualTrackArray, on_trk, queue_size=1)

cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, POS[2], POS[3])
cv2.moveWindow(WIN, POS[0], POS[1])
print("screen=%dx%d  window=%dx%d@(%d,%d)  topic=%s"
      % (SW, SH, POS[2], POS[3], POS[0], POS[1], TOPIC))
sys.stdout.flush()

t0 = time.time()
last_check = 0.0
while not rospy.is_shutdown():
    # 每 2 秒看一次分辨率；变了就重新摆窗口（VMware 自动适应会改 guest 分辨率）
    if time.time() - last_check > 2.0:
        last_check = time.time()
        now_sw, now_sh = screen_size()
        if (now_sw, now_sh) != (SW, SH):
            SW, SH = now_sw, now_sh
            new_pos = layout(SW, SH)
            if new_pos != POS:
                POS = new_pos
                cv2.resizeWindow(WIN, POS[2], POS[3])
                cv2.moveWindow(WIN, POS[0], POS[1])
                print("screen changed -> %dx%d, window=%dx%d@(%d,%d)"
                      % (SW, SH, POS[2], POS[3], POS[0], POS[1]))
                sys.stdout.flush()

    msg = IMG["v"]
    if msg is None:
        canvas = cv2.UMat(400, 640, cv2.CV_8UC3).get()
        cv2.putText(canvas, "waiting for " + TOPIC, (40, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    else:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        state = ST["state"]
        if state == "?":
            fb = state_from_log()
            if fb:
                state = fb + " (from log)"
        lock_color = (0, 255, 0) if "VISUAL_LOCK" in state else (0, 200, 255)

        el = max(time.time() - t0, 0.001)
        lines = [
            ("STATE  : " + state, lock_color),
            ("DET    : %d target(s), max conf %.2f" % (DET["n"], DET["conf"]), (255, 255, 255)),
            ("TRACKS : %d  ids=%s" % (len(TRK["ids"]), TRK["ids"][:6]), (255, 255, 255)),
            ("AREA   : " + " ".join("%.3f" % a for a in TRK["areas"][:4]), (200, 200, 200)),
            ("RATE   : %.1f Hz  img=%s" % (DET["msgs"] / el, str(img.shape)),
             (200, 200, 200)),
        ]
        canvas = img.copy()
        draw_panel(canvas, lines)

    cv2.imshow(WIN, canvas)
    key = cv2.waitKey(30)

    # ---- 让窗口能被关掉 ----
    # 不检测的话，点了标题栏的 ✕ 之后，下一帧 imshow() 又会把窗口建回来，
    # 表现为"窗口关不掉"（实测踩到过）。这里主动检测窗口是否已被关闭。
    try:
        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            print("窗口已被关闭，退出。")
            break
    except cv2.error:
        print("窗口已被销毁，退出。")
        break
    if key == 27:                      # ESC 也能退
        print("按下 ESC，退出。")
        break

    rospy.sleep(0.02)

cv2.destroyAllWindows()
