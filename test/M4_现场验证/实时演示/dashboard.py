#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M4 实时状态面板 —— 在终端里刷新的可视化面板（给人看）。

用法：
    python3 dashboard.py

订阅 /state、/detections、/visual/tracks，每秒刷新一次。
"""
import os
import sys
import time

import rospy
from std_msgs.msg import String
from uav_tracking.msg import Detections, VisualTrackArray

STATE = {"v": "(尚未收到)"}
DET = {"n": 0, "msgs": 0, "conf": 0.0}
TRK = {"list": [], "msgs": 0}
STATS = {"det": 0.0, "det_n": 0, "trk": 0.0, "trk_n": 0}


def on_state(msg):
    STATE["v"] = msg.data


def state_from_log():
    """从状态机日志兜底取当前状态。

    为什么需要：``/state`` **只在状态变化时发布**（且没有 latch）。面板若在
    状态跃迁之后才订阅，就永远收不到 —— 必须从日志里兜底读，
    否则面板会一直显示"尚未收到"，看着像没跑起来。
    """
    for path in ("/tmp/m4_live/state.log",
                 "/home/demo/桌面/ai4/test/M4_现场验证/实时演示/state.log"):
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


def on_det(msg):
    DET["msgs"] += 1
    DET["n"] = len(msg.targets)
    if msg.targets:
        DET["conf"] = max(t.confidence for t in msg.targets)


def on_trk(msg):
    TRK["msgs"] += 1
    TRK["list"] = [(t.id, getattr(t, "area_ratio", None)) for t in msg.tracks]


rospy.init_node("m4_dashboard", anonymous=True, disable_signals=True)
rospy.Subscriber("/state", String, on_state, queue_size=1)
rospy.Subscriber("/detections", Detections, on_det, queue_size=1)
rospy.Subscriber("/visual/tracks", VisualTrackArray, on_trk, queue_size=1)

t0 = time.time()
while not rospy.is_shutdown():
    el = max(time.time() - t0, 0.001)
    os.system("clear")
    print("=" * 62)
    print("           M4 实时状态面板        %s" % time.strftime("%H:%M:%S"))
    print("=" * 62)
    print()
    shown = STATE["v"]
    src = "话题"
    if shown == "(尚未收到)":
        fb = state_from_log()
        if fb:
            shown, src = fb, "日志兜底"
    print("  当前状态 /state :  \033[1;33m%s\033[0m   (来源: %s)" % (shown, src))
    print("                    （RADAR_GUIDE → VISUAL_LOCK 表示视觉锁定成功）")
    print()
    print("  /detections     :  本帧 %d 个目标（最高置信度 %.2f）"
          % (DET["n"], DET["conf"]))
    print("  /visual/tracks  :  本帧 %d 条航迹  (已收 %d 帧)" % (len(TRK["list"]), TRK["msgs"]))
    for tid, ar in TRK["list"]:
        print("        id=%-3s  area_ratio=%s"
              % (tid, ("%.4f" % ar) if isinstance(ar, float) else ar))
    print()
    print("  ---- 接收统计 ----")
    print("  /detections 消息数 : %d    （约 %.1f Hz）" % (DET["msgs"], DET["msgs"] / el))
    print("  /visual/tracks 数  : %d" % TRK["msgs"])
    print()
    print("  停止：Ctrl+C；或另开终端 bash m4_live.sh stop")
    sys.stdout.flush()
    rospy.sleep(1.0)
