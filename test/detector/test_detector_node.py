"""模块 4 节点壳（``/detector``）单元测试。

覆盖 ``scripts/detector_node.py`` 中**可测的纯转换函数**（不依赖 ROS master）：

- ``detections_to_msg``：``list[DetectedTarget]`` → ``Detections``
  （含「空也发」——空列表产出合法空消息、字段数值、多目标）；
- ``draw_detections``：画框不修改入参、空检测返回原图副本（方案书要求的可视化交付物）；
- ``image_msg_to_array``：``sensor_msgs/Image`` → BGR ``np.ndarray``（bgr8 / rgb8 / mono8）。

运行前提：已 ``source /opt/ros/noetic/setup.bash`` 与
``source catkin_ws/devel/setup.bash``（需能 import ``rospy`` 与生成的
``uav_tracking.msg``）；否则本文件整体跳过，不影响纯逻辑测试。
"""
import os
import sys

import pytest

# 将 catkin 包 scripts/ 加入 import 路径，使 detector_node 可被 import。
_SCRIPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "catkin_ws", "src", "uav_tracking", "scripts",
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# 依赖 ROS（rospy + 生成的 uav_tracking.msg + cv_bridge），缺失则本文件整体跳过
pytest.importorskip("rospy")
pytest.importorskip("uav_tracking.msg")
pytest.importorskip("cv_bridge")

import numpy as np  # noqa: E402

from cv_bridge import CvBridge  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from uav_tracking.msg import Detections  # noqa: E402

import detector_node  # noqa: E402
from datatypes import DetectedTarget  # noqa: E402  纯逻辑层的数据结构


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def det(class_id=0, confidence=0.9, bbox=(10.0, 20.0, 30.0, 40.0)):
    """构造一条纯逻辑层检测结果（与 /detector 输出类型一致）。"""
    return DetectedTarget(class_id=class_id, confidence=confidence, bbox=bbox)


# ---------------------------------------------------------------------------
# detections_to_msg
# ---------------------------------------------------------------------------
def test_detections_to_msg_empty_still_produces_valid_message():
    """「空也发」：空列表必须产出**合法**的 Detections（targets 为空数组）。"""
    msg = detector_node.detections_to_msg([])
    assert isinstance(msg, Detections)
    assert len(msg.targets) == 0


def test_detections_to_msg_fields_roundtrip():
    msg = detector_node.detections_to_msg([det(class_id=2, confidence=0.75,
                                               bbox=(1.0, 2.0, 3.0, 4.0))])
    assert len(msg.targets) == 1
    t = msg.targets[0]
    assert int(t.class_id) == 2
    assert float(t.confidence) == pytest.approx(0.75)
    assert [float(v) for v in t.bbox] == pytest.approx([1.0, 2.0, 3.0, 4.0])


def test_detections_to_msg_multiple_targets_preserve_order():
    msg = detector_node.detections_to_msg([
        det(class_id=0, confidence=0.9, bbox=(0.0, 0.0, 10.0, 10.0)),
        det(class_id=1, confidence=0.8, bbox=(50.0, 50.0, 20.0, 20.0)),
    ])
    assert [int(t.class_id) for t in msg.targets] == [0, 1]


def test_detections_to_msg_bbox_is_xywh_not_xyxy():
    """bbox 约定为 (x, y, w, h)，不得被误当成 (x1,y1,x2,y2)。"""
    msg = detector_node.detections_to_msg([det(bbox=(10.0, 20.0, 30.0, 40.0))])
    x, y, w, h = [float(v) for v in msg.targets[0].bbox]
    assert (x, y, w, h) == pytest.approx((10.0, 20.0, 30.0, 40.0))


# ---------------------------------------------------------------------------
# draw_detections
# ---------------------------------------------------------------------------
def test_draw_detections_does_not_modify_input():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    before = image.copy()
    detector_node.draw_detections(image, [det(bbox=(10.0, 10.0, 40.0, 30.0))])
    assert np.array_equal(image, before), "draw_detections 不得修改入参图像"


def test_draw_detections_returns_copy_when_empty():
    image = np.full((60, 80, 3), 7, dtype=np.uint8)
    out = detector_node.draw_detections(image, [])
    assert out.shape == image.shape
    assert np.array_equal(out, image)


def test_draw_detections_draws_something():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    out = detector_node.draw_detections(image, [det(bbox=(20.0, 20.0, 50.0, 40.0))])
    assert out.shape == image.shape
    assert out.sum() > 0, "画框后应出现非零像素"


# ---------------------------------------------------------------------------
# image_msg_to_array
# ---------------------------------------------------------------------------
def test_image_msg_to_array_bgr8():
    bridge = CvBridge()
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    frame[:, :, 0] = 200                      # B 通道
    msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    out = detector_node.image_msg_to_array(msg, bridge)
    assert out.shape == (48, 64, 3)
    assert int(out[0, 0, 0]) == 200, "B 通道应保留在 index 0（BGR 约定）"


def test_image_msg_to_array_mono8_becomes_3ch():
    """灰度图应被归一化为 3 通道 BGR，供 detector 统一处理。"""
    bridge = CvBridge()
    mono = np.full((30, 40), 128, dtype=np.uint8)
    msg = bridge.cv2_to_imgmsg(mono, encoding="mono8")
    out = detector_node.image_msg_to_array(msg, bridge)
    assert out.ndim == 3 and out.shape[2] == 3


def test_image_msg_to_array_creates_bridge_when_none():
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(np.zeros((16, 16, 3), dtype=np.uint8), encoding="bgr8")
    out = detector_node.image_msg_to_array(msg)     # bridge 参数省略
    assert out.shape == (16, 16, 3)
