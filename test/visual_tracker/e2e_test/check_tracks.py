#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""断言节点（E2E 测试用）：订阅 ``/visual/tracks`` 并校验。

参数（rosparam）
----------------
:~mode       str    "target" | "empty"
                    - target：要求话题在流 **且** 收到航迹
                    - empty ：要求话题**仍在流** 且 每条消息 ``tracks`` 为空（空也发）
:~duration   float  观察时长（秒）          默认 6.0
:~min_msgs   int    最少消息数（证明在流）   默认 20
:~frame_w    int    画面宽（校验 area_ratio）默认 640
:~frame_h    int    画面高                   默认 480
:~area_tol   float  area_ratio 相对容差      默认 0.02（2%）
:~check_id_stable bool 是否校验 ID 跨帧稳定   默认 True
~check_move bool  是否校验航迹**确实在移动** 默认 False
~min_move   float 中心点最小跨度（像素）     默认 20.0

> **为什么要有 `~check_move`**：只查「ID 跨帧稳定」是有漏洞的 ——
> 若跟踪器偷懒**输出一个固定不变的框**，ID 稳定、area_ratio 自洽，**照样能过**。
> 打开 `~check_move` 后会要求「第一条航迹的中心点跨度 >= `~min_move`」，
> 才能证明**输入在动、输出也跟着动**（这才是"跟踪移动目标"）。

退出码：0 = PASS，1 = FAIL（便于 run_e2e.sh 汇总）。
"""
from __future__ import annotations

import sys

import rospy
from uav_tracking.msg import VisualTrackArray

TOPIC = "/visual/tracks"


class Checker:
    def __init__(self):
        self.mode = str(rospy.get_param("~mode", "target")).lower()
        self.duration = float(rospy.get_param("~duration", 6.0))
        self.min_msgs = int(rospy.get_param("~min_msgs", 20))
        self.frame_w = int(rospy.get_param("~frame_w", 640))
        self.frame_h = int(rospy.get_param("~frame_h", 480))
        self.area_tol = float(rospy.get_param("~area_tol", 0.02))
        self.check_id_stable = bool(rospy.get_param("~check_id_stable", True))
        self.check_move = bool(rospy.get_param("~check_move", False))
        self.min_move = float(rospy.get_param("~min_move", 20.0))

        self.count = 0
        self.nonempty = 0
        self.ids = set()
        self.id_seq = []
        self.area_bad = 0
        self.area_samples = []
        self.centers = []          # 每条含航迹消息里「第一条航迹」的中心点
        rospy.Subscriber(TOPIC, VisualTrackArray, self.on_tracks, queue_size=10)

    def on_tracks(self, msg):
        self.count += 1
        if len(msg.tracks) == 0:
            return
        self.nonempty += 1
        # 记一条航迹的中心点（用于「框到底动没动」的校验）
        _x, _y, _w, _h = [float(v) for v in msg.tracks[0].bbox]
        self.centers.append((_x + _w / 2.0, _y + _h / 2.0))
        for t in msg.tracks:
            self.ids.add(int(t.id))
            self.id_seq.append(int(t.id))
            x, y, w, h = [float(v) for v in t.bbox]
            expect = (w * h) / float(self.frame_w * self.frame_h)
            got = float(t.area_ratio)
            self.area_samples.append(got)
            if expect > 0 and abs(got - expect) / expect > self.area_tol:
                self.area_bad += 1

    def run(self):
        rospy.loginfo("[check] mode=%s 观察 %.1fs ...", self.mode, self.duration)
        rospy.sleep(self.duration)

        print("[check] 收到 %d 条 %s（期望 >= %d）" % (self.count, TOPIC, self.min_msgs))
        failures = []

        if self.count < self.min_msgs:
            failures.append("话题未在流：仅 %d 条 < %d" % (self.count, self.min_msgs))

        if self.mode == "empty":
            if self.nonempty > 0:
                failures.append("mode=empty 却收到 %d 条含航迹的消息" % self.nonempty)
            else:
                print("PASS: empty 阶段 —— 话题在流且 tracks 恒为空（空也发成立）")
        else:
            if self.nonempty == 0:
                failures.append("mode=target 但未收到任何航迹")
            else:
                print("[check] 航迹消息 %d 条，ID 集合 = %s" % (self.nonempty, sorted(self.ids)))
                if self.area_bad:
                    failures.append("area_ratio 与 bbox/画面面积 不一致（%d 条超差）"
                                    % self.area_bad)
                else:
                    print("PASS: area_ratio 与 bbox/(%dx%d) 一致（样本 %s）"
                          % (self.frame_w, self.frame_h,
                             ["%.4f" % v for v in self.area_samples[:3]]))
                if self.check_id_stable:
                    if len(self.ids) != 1:
                        failures.append("ID 跨帧不稳定：出现 %d 个不同 ID %s"
                                        % (len(self.ids), sorted(self.ids)))
                    else:
                        print("PASS: ID 跨帧稳定（全程 id=%d，共 %d 帧）"
                              % (self.id_seq[0], len(self.id_seq)))
                if self.check_move:
                    if len(self.centers) < 2:
                        failures.append("样本不足（%d 条），无法校验航迹是否移动"
                                        % len(self.centers))
                    else:
                        xs = [c[0] for c in self.centers]
                        ys = [c[1] for c in self.centers]
                        span = max(max(xs) - min(xs), max(ys) - min(ys))
                        if span >= self.min_move:
                            print("PASS: 航迹中心确实在移动（跨度 %.1f 像素 >= %.1f）"
                                  "—— 不是固定框" % (span, self.min_move))
                        else:
                            failures.append(
                                "航迹中心几乎没动：跨度仅 %.1f 像素 < %.1f"
                                "（固定不变的框也能骗过 ID 稳定检查）"
                                % (span, self.min_move))

        if failures:
            for f in failures:
                print("FAIL: " + f)
            return 1
        return 0


def main():
    rospy.init_node("check_tracks")
    sys.exit(Checker().run())


if __name__ == "__main__":
    main()
