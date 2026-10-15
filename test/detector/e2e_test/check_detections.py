#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""断言节点（E2E 测试用）：订阅 ``/detections`` 并校验。

参数（rosparam）
----------------
:~mode       str    "empty" | "target" | "flow"
                    - empty ：话题**仍在流** 且 每条消息 ``targets`` 为空（空也发）
                    - target：话题在流 **且** 至少一条消息含目标，并校验 bbox 合法性
                    - flow  ：只要求话题在流（链路连通即可）
:~duration   float  观察时长（秒）            默认 6.0
:~min_msgs   int    最少消息数（证明在流）     默认 20
:~frame_w    int    画面宽（校验 bbox 越界）   默认 640
:~frame_h    int    画面高                     默认 480
:~conf_min   float  target 模式下要求的置信度下限 默认 0.0

bbox 合法性的三条断言（**专治 xyxy / xywh 搞混**）
--------------------------------------------------
1. ``w > 0 且 h > 0``        —— 若是 ``xywh`` 却按 ``xyxy`` 填，宽高会是负数；
2. ``x >= -eps 且 y >= -eps`` —— 左上角不应显著为负；
3. ``x + w <= frame_w * 1.2`` —— 右边界不应远超画面（``x2`` 误填成 ``x`` 时会超）。

退出码：0 = PASS，1 = FAIL（便于 run_e2e.sh 汇总）。
"""
from __future__ import annotations

import sys

import rospy
from uav_tracking.msg import Detections

TOPIC = "/detections"
EPS = 1.0


class Checker:
    def __init__(self):
        self.mode = str(rospy.get_param("~mode", "flow")).lower()
        self.duration = float(rospy.get_param("~duration", 6.0))
        self.min_msgs = int(rospy.get_param("~min_msgs", 20))
        self.frame_w = int(rospy.get_param("~frame_w", 640))
        self.frame_h = int(rospy.get_param("~frame_h", 480))
        self.conf_min = float(rospy.get_param("~conf_min", 0.0))

        self.count = 0
        self.nonempty = 0
        self.n_targets = 0
        self.bbox_bad = 0
        self.conf_bad = 0
        self.samples = []
        rospy.Subscriber(TOPIC, Detections, self.on_detections, queue_size=10)

    def _check_bbox(self, x, y, w, h):
        """返回 True 表示 bbox 合法。"""
        if w <= 0 or h <= 0:
            return False                                   # 断言 1
        if x < -EPS or y < -EPS:
            return False                                   # 断言 2
        if x + w > self.frame_w * 1.2 or y + h > self.frame_h * 1.2:
            return False                                   # 断言 3
        return True

    def on_detections(self, msg):
        self.count += 1
        if len(msg.targets) == 0:
            return
        self.nonempty += 1
        for t in msg.targets:
            self.n_targets += 1
            x, y, w, h = [float(v) for v in t.bbox]
            if not self._check_bbox(x, y, w, h):
                self.bbox_bad += 1
            if float(t.confidence) < self.conf_min:
                self.conf_bad += 1
            if len(self.samples) < 3:
                self.samples.append("cls%d conf=%.2f bbox=(%.0f,%.0f,%.0f,%.0f)"
                                    % (int(t.class_id), float(t.confidence), x, y, w, h))

    def run(self):
        rospy.loginfo("[check] mode=%s 观察 %.1fs ...", self.mode, self.duration)
        rospy.sleep(self.duration)

        print("[check] 收到 %d 条 %s（期望 >= %d）" % (self.count, TOPIC, self.min_msgs))
        failures = []

        if self.count < self.min_msgs:
            failures.append("话题未在流：仅 %d 条 < %d" % (self.count, self.min_msgs))

        if self.mode == "empty":
            if self.nonempty > 0:
                failures.append("mode=empty 却收到 %d 条含目标的消息" % self.nonempty)
            elif self.count >= self.min_msgs:
                print("PASS: empty 阶段 —— 话题在流且 targets 恒为空（空也发成立）")
            else:
                # ★ 不能在这里无脑打印 PASS：count 不足时下面会报 FAIL，
                #   两行自相矛盾会把排查方向带偏（实测踩到过：明明 0 条却打印
                #   "话题在流…PASS"，然后摘要又 FAIL）。
                print("[check] 收到 %d 条空消息，但少于 %d 条，不足以判定「话题在流」"
                      "（见下方 FAIL）" % (self.count, self.min_msgs))
        elif self.mode == "target":
            if self.nonempty == 0:
                failures.append("mode=target 但未收到任何目标（模型是否可检出该图？）")
            else:
                print("[check] 含目标消息 %d 条，检出目标共 %d 个" % (self.nonempty, self.n_targets))
                print("[check] 样例: %s" % ("; ".join(self.samples) or "-"))
                if self.bbox_bad:
                    failures.append(
                        "bbox 不合法（%d 个）—— 检查是否为 xywh=(x,y,w,h)："
                        "w/h 必须为正、x/y 不为负、不越界" % self.bbox_bad)
                else:
                    print("PASS: bbox 全部合法（xywh 约定，未越界）")
                if self.conf_bad:
                    failures.append("有 %d 个目标置信度 < %.2f" % (self.conf_bad, self.conf_min))
        else:
            print("PASS: flow 阶段 —— 话题在流")

        if failures:
            for f in failures:
                print("FAIL: " + f)
            return 1
        return 0


def main():
    rospy.init_node("check_detections")
    sys.exit(Checker().run())


if __name__ == "__main__":
    main()
