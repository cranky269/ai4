#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/detector 节点壳：``/camera/image_raw`` → YOLO 检测 → ``/detections``。

对应《python代码移植到ros平台方法整理.md》第五节与《design/函数接口.md》模块 4：

- **壳**（本文件）：订阅 / 发布 / 图像转换 / 绘制调试图 / 日志，**不含检测算法**；
- **核**（``lib/detector.py``）：``YoloDetector`` / ``detect`` 纯逻辑，零改动。

设计约定（``design/函数接口.md`` 模块 4 与「决策端设计原则」）
--------------------------------------------------------
- **空也发**：无目标时发布**空** ``Detections``（``targets=[]``），不静默——
  保证「空 = 没目标（业务正常）、静默 = 故障」语义成立，同时为 ``/state_machine``
  的 N/M/K 帧计数提供节拍（设计原则第 5 条）；
- **队列策略**：图像是大消息、旧帧无意义 → ``queue_size=1`` 只保最新。
  配合 rospy 单线程 spinner，慢推理期间到达的帧会覆盖队列中的旧帧，
  即「处理尽可能快、永远吃最新帧」，天然抑制延迟累积；
- **参数外置**：模型路径、阈值、类别等一律走 ``rosparam``，不写死；
- **可视化交付物**：方案书要求「在画面上绘制 Bounding Box 并输出坐标数据」，
  故发布调试图 ``/detector/debug_image``（可关）。

视觉依赖隔离（组长 2026-10-06 确认）
------------------------------------
``ultralytics`` 要求 ``numpy>=1.18.5``，而项目基线是系统 numpy 1.17.4
（见 ``requirements.txt``）。故视觉依赖装在**工程内**的独立目录::

    <工程根>/lib/m4/

**严格隔离**：只有**本节点**把这个目录加进 ``sys.path``：

- 其他节点壳只把 ``lib/`` 加进 ``sys.path``；解释器查找 ``import numpy``
  时只会去看 ``lib/numpy``（不存在），**不会**进 ``lib/m4/numpy``；
- ``lib/m4`` 不在任何默认搜索路径上，也不会被写进 ``.pth`` / ``PYTHONPATH``；
- 本节点启动时会打印实际加载的 numpy 版本与路径，便于核对。

下载/安装命令见 ``M4交付说明/组长确认冲突库下载说明文档.md``。
"""
from __future__ import annotations

import os
import sys
import traceback
import threading
import time

# --------------------------------------------------------------------------- #
# ★★ 路径注入必须发生在 `import rospy` 之前 ★★
#
# rospy / sensor_msgs 在导入时就会 `import numpy`。若等 `import rospy` 之后再
# 改 sys.path，numpy 已经进了 sys.modules，隔离目录将**完全失效**：
# 实测会静默退回系统 numpy 1.17.4，并导致 ultralytics 的 letterbox 抛
#   TypeError: integer argument expected, got float
# （1.17.4 太老，边界值算成了 float，cv2.copyMakeBorder 拒收）。
# --------------------------------------------------------------------------- #
# 把 lib/ 加入 import 路径（纯逻辑与壳解耦；lib/ 内为指向 code/ 的软链）
_LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib"))
if _LIB_DIR not in sys.path:
    sys.path.insert(0, _LIB_DIR)

# --- 视觉依赖隔离目录 ---
# 仓库根 lib/m4（lib/ 下按模块建子目录，由 scripts/冲突库下载/install_m4_deps.sh 安装）。
_M4_DEPS = os.path.abspath(
    os.path.join(_LIB_DIR, "..", "..", "..", "..", "lib", "m4"))
if os.path.isdir(_M4_DEPS) and _M4_DEPS not in sys.path:
    sys.path.insert(0, _M4_DEPS)

import rospy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import numpy as np  # noqa: E402

from detector import setup as setup_detector  # noqa: E402  纯逻辑，零改动
from uav_tracking.msg import DetectedTarget, Detections  # noqa: E402

try:  # 调试图需要 cv_bridge；缺失时不影响主链路
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - 环境相关
    CvBridge = None

__all__ = [
    "image_msg_to_array",
    "detections_to_msg",
    "draw_detections",
    "DetectorNode",
]

# --- 模块级常量（代码书写原则第 6 条：默认值 / 未指定参数集中定义）---
IMAGE_TOPIC = "/camera/image_raw"
DETECTIONS_TOPIC = "/detections"
DEBUG_IMAGE_TOPIC = "/detector/debug_image"
QUEUE_SIZE = 1
IMAGE_BUFFER_SIZE = 2 ** 24
MAX_RESULT_AGE = 1.5
DEFAULT_MODEL_PATH = "yolov8n.pt"
DEBUG_COLOR = (0, 255, 0)          # BGR，全项目统一配色（代码书写原则第 5 条）
DEBUG_FONT_SCALE = 0.5
DEBUG_THICKNESS = 2


# --------------------------------------------------------------------------- #
# 纯转换函数（不依赖 ROS master，可单测）
# --------------------------------------------------------------------------- #
def image_msg_to_array(msg, bridge=None):
    """``sensor_msgs/Image`` → BGR ``np.ndarray``（纯转换，可单测）。

    统一产出 ``bgr8``，与 ``lib/detector.py`` 的图像约定一致：
    灰度图会被堆叠成 3 通道，``rgb8`` 会被转换通道序。

    参数
    ----
    msg : sensor_msgs/Image
        输入图像消息。
    bridge : cv_bridge.CvBridge | None
        未提供时自动新建一个。

    返回
    ----
    np.ndarray
        形状 ``(H, W, 3)`` 的 BGR 数组。
    """
    if bridge is None:
        bridge = CvBridge()
    # desired_encoding="bgr8" 让 cv_bridge 顺手完成 mono8 / rgb8 的归一化
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def detections_to_msg(detections):
    """``list[DetectedTarget]`` → ``Detections`` 消息（纯转换，可单测）。

    - ``bbox`` 按 ``(x, y, w, h)`` 原样填入 ``float32[4]``；
    - **空列表也构造出合法的空消息**（``targets`` 为空数组），
      由调用方照常发布，落实「空也发」约定。

    注意：``Detections.msg`` **无 header 字段**（《数据类型.md》1.2 照搬定义），
    故此处不设置时间戳；节点若需时间戳对齐，改用 ``rospy.Time.now()`` 打日志。
    """
    msg = Detections()
    for det in detections:
        target = DetectedTarget()
        target.class_id = int(det.class_id)
        target.confidence = float(det.confidence)
        x, y, w, h = det.bbox
        target.bbox = [float(x), float(y), float(w), float(h)]
        msg.targets.append(target)
    return msg


def draw_detections(image, detections):
    """在图像副本上绘制 bbox + 类别 + 置信度（方案书要求的可视化交付物）。

    参数
    ----
    image : np.ndarray
        BGR 原图（函数内部复制，不修改入参）。
    detections : list[DetectedTarget]
        检测结果。

    返回
    ----
    np.ndarray
        绘制后的图像；``detections`` 为空时返回原图副本（便于观察"确实没目标"）。
    """
    import cv2

    canvas = image.copy()
    for det in detections:
        x, y, w, h = (int(round(v)) for v in det.bbox)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), DEBUG_COLOR, DEBUG_THICKNESS)
        label = "cls%d %.2f" % (det.class_id, det.confidence)
        cv2.putText(canvas, label, (x, max(y - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, DEBUG_FONT_SCALE, DEBUG_COLOR, 1)
    return canvas


# --------------------------------------------------------------------------- #
# 节点
# --------------------------------------------------------------------------- #
class DetectorNode:
    """``/detector`` 节点：事件订阅 → ``YoloDetector`` 推理 → 发布检测与调试图。"""

    def __init__(self):
        # --- 参数一律走 rosparam（launch 可调，不写死）---
        self.model_path = rospy.get_param("~model_path", DEFAULT_MODEL_PATH)
        self.conf_thres = rospy.get_param("~conf_thres", 0.5)
        self.iou_thres = rospy.get_param("~iou_thres", 0.45)
        self.imgsz = rospy.get_param("~imgsz", 640)
        self.device = rospy.get_param("~device", "cpu")   # 仿真机无 CUDA
        self.classes = rospy.get_param("~classes", [])    # 空 = 全部类别
        self.max_det = rospy.get_param("~max_det", 20)
        self.publish_debug = rospy.get_param("~publish_debug", True)
        self.image_topic = rospy.get_param("~image_topic", IMAGE_TOPIC)

        # --- 核：检测器实例（模型惰性加载，故此处调用很轻）---
        self.detector = setup_detector(
            model_path=self.model_path,
            conf_thres=self.conf_thres,
            iou_thres=self.iou_thres,
            imgsz=self.imgsz,
            device=self.device,
            classes=self.classes,
            max_det=self.max_det,
        )

        self.bridge = CvBridge() if CvBridge is not None else None
        self.pub = rospy.Publisher(DETECTIONS_TOPIC, Detections, queue_size=QUEUE_SIZE)
        self.pub_debug = (
            rospy.Publisher(DEBUG_IMAGE_TOPIC, Image, queue_size=QUEUE_SIZE)
            if self.publish_debug else None
        )
        self._condition = threading.Condition()
        self._pending = None
        self._last_stamp = None
        self._last_unstamped_seq = None
        self._last_clock = None
        self._epoch = 0
        self._sim_time = rospy.get_param("use_sim_time", False)
        self._stopping = False
        self._timing_enabled = os.environ.get("M4_TIMING_LOG") == "1"
        self._worker = threading.Thread(target=self._run_worker, name="m4-inference", daemon=True)
        self._subscriber = rospy.Subscriber(
            self.image_topic, Image, self.on_image,
            queue_size=QUEUE_SIZE, buff_size=IMAGE_BUFFER_SIZE,
        )
        self._worker.start()
        rospy.on_shutdown(self._stop_worker)

        rospy.loginfo(
            "[detector] ready: model=%s conf=%.2f imgsz=%d device=%s in=%s out=%s",
            self.model_path, self.conf_thres, self.imgsz, self.device,
            self.image_topic, DETECTIONS_TOPIC,
        )
        # 隔离自检：确认 numpy 真的来自隔离目录。
        # 这里必须"会喊"而不是只打印 —— 隔离一旦失效，推理会在 ultralytics
        # 内部以 `integer argument expected, got float` 这种晦涩方式崩掉。
        np_dir = os.path.dirname(os.path.abspath(np.__file__))
        if os.path.isdir(_M4_DEPS):
            if os.path.realpath(np_dir).startswith(os.path.realpath(_M4_DEPS)):
                rospy.loginfo(
                    "[detector] numpy %s @ %s（隔离生效，目录 %s）",
                    np.__version__, np_dir, _M4_DEPS)
            else:
                rospy.logerr(
                    "[detector] numpy 隔离失效！实际加载 %s @ %s，期望在 %s 之下。"
                    "请检查是否有模块在本文件顶部的 sys.path 注入之前就 import 了 numpy。",
                    np.__version__, np_dir, _M4_DEPS)
        else:
            rospy.logwarn(
                "[detector] 未找到隔离目录 %s，改用系统 numpy %s。"
                "若推理报 'integer argument expected, got float'，请按 "
                "《组长确认冲突库下载说明文档.md》安装视觉依赖。",
                _M4_DEPS, np.__version__)

    def _clock_locked(self):
        """Detect a ROS clock rewind before accepting or publishing a frame."""
        now = rospy.Time.now()
        if self._last_clock is not None and now < self._last_clock:
            self._epoch += 1
            self._pending = None
            self._last_stamp = None
            self._last_unstamped_seq = None
        self._last_clock = now
        return now

    def on_image(self, msg):
        received = time.monotonic()
        stamp = msg.header.stamp
        with self._condition:
            if self._stopping:
                return
            now = self._clock_locked()
            receive_age = self._source_age(now, stamp)
            if self._sim_time and (now.is_zero() or stamp.is_zero()):
                return  # /clock 未就绪或输入缺失可用于仿真的时间戳
            if receive_age is not None and not 0 <= receive_age <= MAX_RESULT_AGE:
                return
            if not stamp.is_zero():
                if self._last_stamp is not None and stamp <= self._last_stamp:
                    return
                self._last_stamp = stamp
            elif msg.header.seq:
                if msg.header.seq == self._last_unstamped_seq:
                    return
                self._last_unstamped_seq = msg.header.seq
            self._pending = (msg, received, receive_age, now, self._epoch)
            self._condition.notify()

    def _stop_worker(self):
        with self._condition:
            self._stopping = True
            self._pending = None
            self._condition.notify_all()
        self._subscriber.unregister()
        if self._worker is not threading.current_thread():
            self._worker.join(timeout=MAX_RESULT_AGE)
            if self._worker.is_alive():
                rospy.logwarn("[detector] 停止时推理仍在执行，等待进程退出")

    @staticmethod
    def _source_age(now, stamp):
        if now.is_zero() or stamp.is_zero():
            return None
        return (now - stamp).to_sec()

    def _run_worker(self):
        while True:
            with self._condition:
                ready = self._condition.wait_for(
                    lambda: self._stopping or self._pending is not None,
                    timeout=MAX_RESULT_AGE,
                )
                if self._stopping:
                    return
                item = self._pending
                self._pending = None
            if not ready:
                rospy.logwarn_throttle(5., "[detector] 没有可处理的新相机帧，检测输出暂停")
                continue
            self._process_image(*item)

    def _process_image(self, msg, received, receive_age, receive_clock, epoch):
        started = time.monotonic()
        with self._condition:
            now = self._clock_locked()
            source_age = self._source_age(now, msg.header.stamp)
            if self._stopping or epoch != self._epoch:
                return
        if started - received > MAX_RESULT_AGE or (
                source_age is not None and not 0 <= source_age <= MAX_RESULT_AGE):
            rospy.logwarn_throttle(5., "[detector] 跳过过期的相机帧")
            return
        try:
            image = image_msg_to_array(msg, self.bridge)
        except Exception:
            rospy.logerr_throttle(5., "[detector] 图像转换失败: %s", traceback.format_exc())
            return
        converted = time.monotonic()
        try:
            detections = self.detector.detect(image)
        except Exception:
            rospy.logerr_throttle(5., "[detector] 推理失败: %s", traceback.format_exc())
            return
        inferred = time.monotonic()
        with self._condition:
            now = self._clock_locked()
            source_age = self._source_age(now, msg.header.stamp)
            if self._stopping or epoch != self._epoch or rospy.is_shutdown():
                return
            expired = inferred - received > MAX_RESULT_AGE or (
                source_age is not None and not 0 <= source_age <= MAX_RESULT_AGE)
            paused = self._sim_time and now == receive_clock
            if not expired and not paused:
                self.pub.publish(detections_to_msg(detections))
        if expired or paused:
            rospy.logwarn_throttle(5., "[detector] 图像结果过期或仿真时钟未推进，跳过发布")
            return
        published = time.monotonic()
        if self._timing_enabled:
            rospy.loginfo("M4_TIMING %.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.9f",
                          msg.header.stamp.to_sec(), receive_age if receive_age is not None else -1.,
                          source_age if source_age is not None else -1., started-received,
                          converted-started, inferred-converted, published-inferred, published-received, published)
        if self.pub_debug is not None and not rospy.is_shutdown():
            self._publish_debug(image, detections, msg)

    def _publish_debug(self, image, detections, src_msg):
        """发布带 bbox 的调试图（不参与逻辑，仅供联调复盘）。"""
        try:
            canvas = draw_detections(image, detections)
            out = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            out.header = src_msg.header       # 继承相机时间戳/坐标系
            self.pub_debug.publish(out)
        except Exception as exc:  # 调试图失败不影响主链路
            rospy.logwarn_throttle(5.0, "[detector] 调试图发布失败: %s", exc)


def main():
    rospy.init_node("detector")
    DetectorNode()
    rospy.spin()          # 回调驱动，不用写 while 循环


if __name__ == "__main__":
    main()
