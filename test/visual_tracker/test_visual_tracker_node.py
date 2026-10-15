"""模块 5 节点壳（``/visual_tracker``）单元测试。

覆盖 ``scripts/visual_tracker_node.py`` 中**可测的纯转换函数**（不依赖 ROS master）：

- ``detections_from_msg``：``Detections`` → ``list[DetectedTarget]``（含空消息、字段数值）；
- ``visual_tracks_to_msg``：``list[VisualTrack]`` → ``VisualTrackArray``
  （含「空也发」——空列表产出合法空消息）；
- 一次端到端：``Detections`` → 跟踪器 → ``VisualTrackArray``，ID 与
  ``area_ratio`` 数值一致。

运行前提：已 ``source /opt/ros/noetic/setup.bash`` 与
``source catkin_ws/devel/setup.bash``；否则本文件整体跳过，不影响纯逻辑测试。
"""
import os
import sys

import pytest

# 将 catkin 包 scripts/ 加入 import 路径，使 visual_tracker_node 可被 import。
_SCRIPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "catkin_ws", "src", "uav_tracking", "scripts",
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# 依赖 ROS（rospy + 生成的 uav_tracking.msg），缺失则本文件整体跳过
pytest.importorskip("rospy")
pytest.importorskip("uav_tracking.msg")

from uav_tracking.msg import Detections  # noqa: E402

import visual_tracker  # noqa: E402
import visual_tracker_node as vtn  # noqa: E402

# 自定义消息与 datatypes 同名，起别名避免冲突（同 state_machine_node.py 的做法）
from datatypes import DetectedTarget as DetectedTargetData  # noqa: E402
from datatypes import VisualTrack as VisualTrackData  # noqa: E402
from uav_tracking.msg import DetectedTarget as DetectedTargetMsg  # noqa: E402


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def make_detections_msg(items):
    """按 ``(class_id, confidence, bbox)`` 列表构造 ``Detections`` 消息。"""
    msg = Detections()
    for class_id, confidence, bbox in items:
        t = DetectedTargetMsg()
        t.class_id = int(class_id)
        t.confidence = float(confidence)
        t.bbox = [float(v) for v in bbox]
        msg.targets.append(t)
    return msg


# ---------------------------------------------------------------------------
# detections_from_msg
# ---------------------------------------------------------------------------
def test_detections_from_msg_empty_returns_empty_list():
    """空消息 → []（「空 = 没目标」，非错误）。"""
    assert vtn.detections_from_msg(Detections()) == []


def test_detections_from_msg_converts_fields():
    msg = make_detections_msg([(1, 0.85, (10.0, 20.0, 30.0, 40.0))])
    out = vtn.detections_from_msg(msg)
    assert len(out) == 1
    d = out[0]
    assert isinstance(d, DetectedTargetData)      # 纯逻辑层类型，不是 ROS 消息
    assert d.class_id == 1
    assert d.confidence == pytest.approx(0.85)
    assert d.bbox == pytest.approx((10.0, 20.0, 30.0, 40.0))


def test_detections_from_msg_multiple_preserve_order():
    msg = make_detections_msg([
        (0, 0.9, (0.0, 0.0, 10.0, 10.0)),
        (2, 0.7, (50.0, 50.0, 20.0, 20.0)),
    ])
    out = vtn.detections_from_msg(msg)
    assert [d.class_id for d in out] == [0, 2]


# ---------------------------------------------------------------------------
# visual_tracks_to_msg
# ---------------------------------------------------------------------------
def test_visual_tracks_to_msg_empty_still_produces_valid_message():
    """「空也发」：空列表必须产出**合法**的 VisualTrackArray（tracks 为空数组）。"""
    msg = vtn.visual_tracks_to_msg([])
    assert len(msg.tracks) == 0


def test_visual_tracks_to_msg_fields_roundtrip():
    tracks = [VisualTrackData(id=7, bbox=(1.0, 2.0, 3.0, 4.0), area_ratio=0.0123)]
    msg = vtn.visual_tracks_to_msg(tracks)
    assert len(msg.tracks) == 1
    t = msg.tracks[0]
    assert int(t.id) == 7
    assert [float(v) for v in t.bbox] == pytest.approx([1.0, 2.0, 3.0, 4.0])
    assert float(t.area_ratio) == pytest.approx(0.0123)


def test_visual_tracks_to_msg_keeps_all_tracks():
    tracks = [
        VisualTrackData(id=1, bbox=(0.0, 0.0, 10.0, 10.0), area_ratio=0.01),
        VisualTrackData(id=2, bbox=(50.0, 50.0, 10.0, 10.0), area_ratio=0.02),
    ]
    msg = vtn.visual_tracks_to_msg(tracks)
    assert [int(t.id) for t in msg.tracks] == [1, 2]


# ---------------------------------------------------------------------------
# 端到端（消息 → 跟踪 → 消息）
# ---------------------------------------------------------------------------
def test_end_to_end_msg_to_tracks_msg():
    """整条壳内链路：Detections → detections_from_msg → track → visual_tracks_to_msg。"""
    visual_tracker.reset()
    visual_tracker.setup(frame_size=(640, 480))

    msg = make_detections_msg([(0, 0.95, (280.0, 200.0, 64.0, 48.0))])
    dets = vtn.detections_from_msg(msg)
    tracks = visual_tracker.track(dets)
    out = vtn.visual_tracks_to_msg(tracks)

    assert len(out.tracks) == 1
    t = out.tracks[0]
    assert int(t.id) >= 1
    # area_ratio 应与 (w*h)/(640*480) 一致
    assert float(t.area_ratio) == pytest.approx((64.0 * 48.0) / (640.0 * 480.0))

    visual_tracker.reset()


def test_end_to_end_empty_input_still_publishes_empty_array():
    """端到端「空也发」：无检测进 → 空消息出（不得抛异常）。"""
    visual_tracker.reset()
    visual_tracker.setup(frame_size=(640, 480))

    dets = vtn.detections_from_msg(Detections())
    tracks = visual_tracker.track(dets)
    out = vtn.visual_tracks_to_msg(tracks)

    assert dets == []
    assert tracks == []
    assert len(out.tracks) == 0

    visual_tracker.reset()
