"""模块 5（``/visual_tracker``）单元测试。

覆盖：

- ``iou_matrix`` / ``greedy_match``：IoU 计算与贪心分配的边界情况；
- ``_cxcywh_to_xywh`` / ``_detection_to_cxcywh``：坐标约定转换；
- ``BotSortTracker``：空输入、ID 跨帧稳定、``area_ratio`` 计算与**画面尺寸的
  两级来源**（图像入参优先 / ``set_frame_size`` 与模块常量兜底）、
  丢失—恢复保 ID、``track_buffer`` 淘汰、**两阶段关联**（低分帧保 ID）、
  多目标不串号；
- ``BotSortTracker.set_frame_size``：非法参数（dev/onboard 两阶段）；
- ``track`` / ``setup`` / ``is_ready`` / ``reset``：模块级入口的状态机。

说明
----
本文件**不需要** ROS 与 ``ultralytics``：跟踪算法只用 ``numpy`` + 纯 Python。
真实视频流验证放在集成阶段（见 ``code/visual_tracker.ipynb``）。

阶段开关通过 monkeypatch 修改 ``visual_tracker.ONBOARD`` 模拟。
"""
import numpy as np
import pytest

import visual_tracker
from datatypes import DetectedTarget, VisualTrack
from visual_tracker import BotSortTracker


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def det(x, y, w=60.0, h=60.0, conf=0.9, cls=0):
    """按文档 ``bbox = (x, y, w, h)`` 构造一个检测。"""
    return DetectedTarget(bbox=(float(x), float(y), float(w), float(h)),
                          class_id=cls, confidence=float(conf))


def img(width, height):
    """构造一张满足 ``track(..., image)`` 的图像替身（只需 shape）。"""
    return np.zeros((height, width, 3), np.uint8)


def tracker(**kwargs):
    kwargs.setdefault("frame_size", (640, 480))
    return BotSortTracker(**kwargs)


# ---------------------------------------------------------------------------
# 坐标转换
# ---------------------------------------------------------------------------
def test_cxcywh_to_xywh():
    assert visual_tracker._cxcywh_to_xywh((50.0, 50.0, 20.0, 10.0)) == (40.0, 45.0, 20.0, 10.0)


def test_detection_to_cxcywh():
    assert visual_tracker._detection_to_cxcywh(det(10.0, 20.0, 40.0, 60.0)) == [30.0, 50.0, 40.0, 60.0]


# ---------------------------------------------------------------------------
# iou_matrix / greedy_match
# ---------------------------------------------------------------------------
def test_iou_matrix_identical_boxes_is_one():
    box = np.array([[10.0, 10.0, 4.0, 4.0]])
    assert visual_tracker.iou_matrix(box, box)[0, 0] == pytest.approx(1.0)


def test_iou_matrix_disjoint_boxes_is_zero():
    a = np.array([[0.0, 0.0, 2.0, 2.0]])
    b = np.array([[100.0, 100.0, 2.0, 2.0]])
    assert visual_tracker.iou_matrix(a, b)[0, 0] == pytest.approx(0.0)


def test_iou_matrix_empty_input_shape():
    a = np.zeros((0, 4))
    b = np.array([[0.0, 0.0, 2.0, 2.0]])
    assert visual_tracker.iou_matrix(a, b).shape == (0, 1)


def test_greedy_match_picks_best_then_removes():
    # 2 航迹 × 2 检测：对角 IoU 高
    iou = np.array([[0.9, 0.1],
                    [0.1, 0.8]])
    pairs = visual_tracker.greedy_match(iou, match_thresh=0.5)
    assert pairs == [(0, 0), (1, 1)]


def test_greedy_match_rejects_below_threshold():
    iou = np.array([[0.2]])
    assert visual_tracker.greedy_match(iou, match_thresh=0.5) == []


def test_greedy_match_empty_matrix():
    assert visual_tracker.greedy_match(np.zeros((0, 0)), match_thresh=0.5) == []


# ---------------------------------------------------------------------------
# BotSortTracker：基本行为
# ---------------------------------------------------------------------------
def test_empty_input_returns_empty():
    """空即正常：无检测 → []（节点据此发空 VisualTrackArray）。"""
    assert tracker().track([]) == []


def test_none_input_returns_empty():
    assert tracker().track(None) == []


def test_single_detection_creates_one_track():
    out = tracker().track([det(100.0, 100.0)])
    assert len(out) == 1
    assert isinstance(out[0], VisualTrack)
    assert out[0].id == 1


def test_id_stable_across_moving_frames():
    """跟踪的核心意义：目标连续移动时 ID 保持不变。"""
    t = tracker()
    ids = []
    for i in range(8):
        out = t.track([det(100.0 + i * 12.0, 200.0 + i * 8.0)])
        ids.append(out[0].id if out else None)
    assert ids == [1] * 8


def test_area_ratio_matches_definition():
    """area_ratio = bbox 面积 / 画面总面积。"""
    t = tracker(frame_size=(640, 480))
    out = t.track([det(0.0, 0.0, w=64.0, h=48.0)])
    assert out[0].area_ratio == pytest.approx((64.0 * 48.0) / (640.0 * 480.0))


def test_frame_size_injection_changes_area_ratio():
    """无图像时，画面尺寸由 set_frame_size 注入。"""
    t = tracker(frame_size=(640, 480))
    t.set_frame_size(320, 240)
    out = t.track([det(0.0, 0.0, w=64.0, h=48.0)])
    assert out[0].area_ratio == pytest.approx((64.0 * 48.0) / (320.0 * 240.0))


# ---------------------------------------------------------------------------
# 画面尺寸来源之二：图像入参（组长 2026-10-06 确认的补充）
# ---------------------------------------------------------------------------
def test_track_image_overrides_frame_size():
    """传入图像时，用图像实际尺寸算 area_ratio（覆盖 setup 的兜底尺寸）。"""
    t = tracker(frame_size=(640, 480))
    out = t.track([det(0.0, 0.0, w=64.0, h=48.0)], img(320, 240))
    assert out[0].area_ratio == pytest.approx((64.0 * 48.0) / (320.0 * 240.0))


def test_track_image_shape_convention_is_hw():
    """图像形状约定为 ``(高, 宽, 通道)``，不能把宽高弄反。"""
    t = tracker(frame_size=(640, 480))
    out = t.track([det(0.0, 0.0, w=64.0, h=48.0)], img(320, 240))
    assert t.frame_size == (320, 240)          # (宽, 高) = (320, 240)


def test_track_without_image_keeps_previous_size():
    """不传图像（None）时沿用已设定尺寸，行为与改动前一致。"""
    t = tracker(frame_size=(640, 480))
    out = t.track([det(0.0, 0.0, w=64.0, h=48.0)], None)
    assert out[0].area_ratio == pytest.approx((64.0 * 48.0) / (640.0 * 480.0))


def test_track_image_size_persists_across_frames():
    """图像尺寸一旦取到，后续无图像的帧也沿用（不会退回旧值）。"""
    t = tracker(frame_size=(640, 480))
    t.track([det(0.0, 0.0, w=64.0, h=48.0)], img(320, 240))
    out = t.track([det(0.0, 0.0, w=64.0, h=48.0)])          # 这一帧不带图
    assert out[0].area_ratio == pytest.approx((64.0 * 48.0) / (320.0 * 240.0))


def test_track_bad_image_shape_is_tolerated(monkeypatch):
    """图像形状非法时只告警、沿用当前尺寸，不打断跟踪（onboard 阶段）。"""
    monkeypatch.setattr(visual_tracker, "ONBOARD", True)
    t = tracker(frame_size=(640, 480))
    out = t.track([det(0.0, 0.0, w=64.0, h=48.0)], object())   # 无 shape
    assert t.frame_size == (640, 480)
    assert out[0].area_ratio == pytest.approx((64.0 * 48.0) / (640.0 * 480.0))


def test_min_area_ratio_filters_small_boxes():
    t = tracker(frame_size=(640, 480), min_area_ratio=0.5)
    assert t.track([det(0.0, 0.0, w=10.0, h=10.0)]) == []


def test_track_count_reflects_alive_tracks():
    t = tracker()
    t.track([det(100.0, 100.0)])
    assert t.track_count == 1


# ---------------------------------------------------------------------------
# BotSortTracker：丢失 / 恢复 / 淘汰
# ---------------------------------------------------------------------------
def test_lost_then_recovered_keeps_id():
    """丢失一帧不输出；恢复后 ID 不变（buffer 内）。"""
    t = tracker(track_buffer=30)
    first = t.track([det(100.0, 100.0)])
    lost = t.track([])
    second = t.track([det(104.0, 102.0)])
    assert lost == []
    assert second and first[0].id == second[0].id


def test_track_evicted_after_buffer_new_target_gets_new_id():
    """超过 track_buffer 帧未更新 → 淘汰；新目标拿新 ID。"""
    t = tracker(track_buffer=3)
    first = t.track([det(100.0, 100.0)])
    for _ in range(6):
        t.track([])
    second = t.track([det(100.0, 100.0)])
    assert first[0].id != second[0].id


def test_lost_track_is_not_reported_even_within_buffer():
    """buffer 内也不输出丢失航迹（避免下游追幽灵目标）。"""
    t = tracker(track_buffer=5)
    t.track([det(100.0, 100.0)])
    assert t.track([]) == []
    assert t.track([]) == []


# ---------------------------------------------------------------------------
# BotSortTracker：两阶段关联（BoT-SORT 的关键特性）
# ---------------------------------------------------------------------------
def test_low_score_detection_keeps_id_via_second_stage():
    """分数掉到高分阈值以下（模拟遮挡）仍能保住 ID —— 阶段二关联。"""
    t = tracker(track_high_thresh=0.5, track_low_thresh=0.1)
    high = t.track([det(100.0, 100.0, conf=0.9)])
    low = t.track([det(102.0, 101.0, conf=0.3)])   # 0.1 <= 0.3 < 0.5
    assert high and low and high[0].id == low[0].id


def test_below_low_thresh_is_ignored():
    """低于低分阈值 → 既不关联也建不了新航迹。"""
    t = tracker(track_high_thresh=0.5, track_low_thresh=0.4, new_track_thresh=0.6)
    t.track([det(100.0, 100.0, conf=0.9)])
    out = t.track([det(102.0, 101.0, conf=0.2)])
    assert out == []


def test_new_track_requires_new_track_thresh():
    """高分但低于 new_track_thresh 的检测不建新航迹。"""
    t = tracker(track_high_thresh=0.5, new_track_thresh=0.9)
    assert t.track([det(100.0, 100.0, conf=0.6)]) == []
    assert t.track([det(100.0, 100.0, conf=0.95)]) != []


# ---------------------------------------------------------------------------
# BotSortTracker：多目标
# ---------------------------------------------------------------------------
def test_two_targets_get_distinct_ids_and_do_not_swap():
    t = tracker()
    first = t.track([det(50.0, 50.0, w=40.0, h=40.0), det(400.0, 300.0, w=40.0, h=40.0)])
    assert len(first) == 2
    assert first[0].id != first[1].id

    second = t.track([det(55.0, 52.0, w=40.0, h=40.0), det(405.0, 302.0, w=40.0, h=40.0)])
    assert len(second) == 2
    # 位置先按 x 排序检查：左仍左、右仍右，ID 未串
    assert second[0].bbox[0] < 200 and second[1].bbox[0] > 200
    assert {d.id for d in second} == {d.id for d in first}
    assert second[0].id == first[0].id and second[1].id == first[1].id


# ---------------------------------------------------------------------------
# 参数校验（dev / onboard 两阶段）
# ---------------------------------------------------------------------------
def test_init_bad_track_buffer_raises_in_dev(monkeypatch):
    monkeypatch.setattr(visual_tracker, "ONBOARD", False)
    with pytest.raises(ValueError):
        BotSortTracker(track_buffer="30")


def test_init_bad_thresh_onboard_falls_back(monkeypatch):
    monkeypatch.setattr(visual_tracker, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        t = BotSortTracker(match_thresh=None)
    assert t.match_thresh == visual_tracker.DEFAULT_MATCH_THRESH


def test_init_bad_frame_size_raises_in_dev(monkeypatch):
    monkeypatch.setattr(visual_tracker, "ONBOARD", False)
    with pytest.raises(ValueError):
        BotSortTracker(frame_size=640)


def test_set_frame_size_bad_value_raises_in_dev(monkeypatch):
    monkeypatch.setattr(visual_tracker, "ONBOARD", False)
    t = tracker()
    with pytest.raises(ValueError):
        t.set_frame_size(0, 480)


def test_set_frame_size_bad_value_onboard_keeps_old(monkeypatch):
    monkeypatch.setattr(visual_tracker, "ONBOARD", True)
    t = tracker(frame_size=(640, 480))
    with pytest.warns(RuntimeWarning):
        t.set_frame_size(-1, 480)
    assert t.frame_size == (640, 480)


def test_bool_is_not_a_number(monkeypatch):
    monkeypatch.setattr(visual_tracker, "ONBOARD", False)
    with pytest.raises(ValueError):
        BotSortTracker(track_buffer=True)


# ---------------------------------------------------------------------------
# 模块级入口
# ---------------------------------------------------------------------------
def test_track_before_setup_raises_runtime_error():
    visual_tracker.reset()
    with pytest.raises(RuntimeError):
        visual_tracker.track([det(1.0, 1.0)])


def test_setup_returns_instance_and_marks_ready():
    visual_tracker.reset()
    inst = visual_tracker.setup(track_buffer=10, frame_size=(640, 480))
    assert isinstance(inst, BotSortTracker)
    assert inst.track_buffer == 10
    assert visual_tracker.is_ready() is True
    visual_tracker.reset()
    assert visual_tracker.is_ready() is False


def test_module_level_track_delegates_to_instance():
    visual_tracker.reset()
    visual_tracker.setup(frame_size=(640, 480))
    out = visual_tracker.track([det(100.0, 100.0)])
    assert len(out) == 1 and out[0].id == 1
    visual_tracker.reset()


def test_module_level_track_accepts_image():
    """模块级入口同样支持图像入参（与实例方法一致）。"""
    visual_tracker.reset()
    visual_tracker.setup(frame_size=(640, 480))
    out = visual_tracker.track([det(0.0, 0.0, w=64.0, h=48.0)], img(320, 240))
    assert out[0].area_ratio == pytest.approx((64.0 * 48.0) / (320.0 * 240.0))
    visual_tracker.reset()


def test_default_frame_size_constant_is_used_as_spec_default():
    """兜底尺寸来自模块常量，而不是散落的字面量（组长指出的问题）。"""
    assert visual_tracker.DEFAULT_FRAME_SIZE == (640, 480)
    assert BotSortTracker().frame_size == visual_tracker.DEFAULT_FRAME_SIZE
