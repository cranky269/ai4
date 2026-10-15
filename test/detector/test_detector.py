"""模块 4（``/detector``）单元测试。

覆盖：

- ``_xyxy_to_xywh``：ultralytics ``xyxy`` → 文档约定 ``bbox=[x, y, w, h]``；
- ``YoloDetector.to_detections``：正常多框、无框、``boxes`` 缺失、结果为空；
- ``YoloDetector.__init__``：非法参数（dev/onboard 两阶段）；
- ``detect``：未 ``setup`` 抛 ``RuntimeError``；非法图像（dev/onboard 两阶段）；
- ``setup`` / ``is_ready`` / ``reset``：模块级入口的状态机。

说明
----
本文件**不需要** ``ultralytics`` 与 ROS：模型加载是惰性的，测试只覆盖
参数校验与「推理结果 → 数据类型」的纯转换（用假的 ultralytics 结果对象）。
真实推理的验证放在集成阶段（见 ``code/detector.ipynb``）。

阶段开关通过 monkeypatch 修改 ``detector.ONBOARD`` 模拟，不依赖真实环境变量。
"""
import numpy as np
import pytest

import detector
from datatypes import DetectedTarget


# ---------------------------------------------------------------------------
# 辅助：模拟 ultralytics 的 Results / Boxes 结构
# ---------------------------------------------------------------------------
class _FakeTensor:
    """模拟 torch.Tensor 的 ``.cpu().numpy()`` 链式调用。"""

    def __init__(self, array):
        self._array = np.asarray(array, dtype=float)

    def cpu(self):
        return self

    def numpy(self):
        return self._array


class _FakeBoxes:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = _FakeTensor(xyxy)
        self.conf = _FakeTensor(conf)
        self.cls = _FakeTensor(cls)

    def __len__(self):
        return len(self.xyxy.numpy())


class _FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


def _image(h=480, w=640):
    """构造一张合成的 BGR 图像（内容不重要，只测转换与校验）。"""
    return np.zeros((h, w, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# _xyxy_to_xywh
# ---------------------------------------------------------------------------
def test_xyxy_to_xywh_basic():
    assert detector._xyxy_to_xywh(10.0, 20.0, 60.0, 100.0) == (10.0, 20.0, 50.0, 80.0)


def test_xyxy_to_xywh_zero_size_box():
    assert detector._xyxy_to_xywh(5.0, 5.0, 5.0, 5.0) == (5.0, 5.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# YoloDetector.to_detections（纯转换）
# ---------------------------------------------------------------------------
def test_to_detections_converts_boxes_to_documented_bbox():
    res = _FakeResult(_FakeBoxes(
        xyxy=[[10.0, 20.0, 60.0, 100.0]],
        conf=[0.9],
        cls=[0.0],
    ))
    out = detector.YoloDetector.to_detections([res])
    assert len(out) == 1
    det = out[0]
    assert isinstance(det, DetectedTarget)
    assert det.bbox == (10.0, 20.0, 50.0, 80.0)   # x, y, w, h
    assert det.class_id == 0
    assert det.confidence == pytest.approx(0.9)


def test_to_detections_multiple_boxes_and_classes():
    res = _FakeResult(_FakeBoxes(
        xyxy=[[0.0, 0.0, 10.0, 10.0], [100.0, 100.0, 130.0, 150.0]],
        conf=[0.95, 0.80],
        cls=[0.0, 2.0],
    ))
    out = detector.YoloDetector.to_detections([res])
    assert len(out) == 2
    assert [d.class_id for d in out] == [0, 2]
    assert out[1].bbox == (100.0, 100.0, 30.0, 50.0)


def test_to_detections_no_boxes_returns_empty():
    """无目标 → 返回 []（空即正常，不是错误）。"""
    res = _FakeResult(_FakeBoxes(xyxy=[], conf=[], cls=[]))
    assert detector.YoloDetector.to_detections([res]) == []


def test_to_detections_boxes_none_returns_empty():
    assert detector.YoloDetector.to_detections([_FakeResult(None)]) == []


def test_to_detections_empty_results_returns_empty():
    assert detector.YoloDetector.to_detections([]) == []


def test_to_detections_batch_of_images():
    """ultralytics 对一批输入返回结果列表，逐张展开。"""
    r1 = _FakeResult(_FakeBoxes([[0.0, 0.0, 10.0, 10.0]], [0.9], [0]))
    r2 = _FakeResult(_FakeBoxes([[1.0, 1.0, 5.0, 5.0]], [0.7], [0]))
    out = detector.YoloDetector.to_detections([r1, r2])
    assert len(out) == 2


# ---------------------------------------------------------------------------
# YoloDetector.__init__ 参数校验（dev / onboard 两阶段）
# ---------------------------------------------------------------------------
def test_init_defaults_are_sane():
    d = detector.YoloDetector()
    assert d.model_path == detector.DEFAULT_MODEL_PATH
    assert d.conf_thres == detector.DEFAULT_CONF_THRES
    assert d.classes is None          # 空 = 全部类别
    assert d.device == "cpu"          # 仿真机无 CUDA


def test_init_bad_model_path_raises_in_dev(monkeypatch):
    monkeypatch.setattr(detector, "ONBOARD", False)
    with pytest.raises(ValueError):
        detector.YoloDetector(model_path="")


def test_init_bad_model_path_onboard_falls_back(monkeypatch):
    monkeypatch.setattr(detector, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        d = detector.YoloDetector(model_path=None)
    assert d.model_path == detector.DEFAULT_MODEL_PATH


def test_init_bad_conf_thres_raises_in_dev(monkeypatch):
    monkeypatch.setattr(detector, "ONBOARD", False)
    with pytest.raises(ValueError):
        detector.YoloDetector(conf_thres=None)


def test_init_bad_imgsz_onboard_falls_back(monkeypatch):
    monkeypatch.setattr(detector, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        d = detector.YoloDetector(imgsz=0)
    assert d.imgsz == detector.DEFAULT_IMGSZ


def test_init_bool_is_not_a_number(monkeypatch):
    """bool 不算数值：conf_thres=True 应被拒绝。"""
    monkeypatch.setattr(detector, "ONBOARD", False)
    with pytest.raises(ValueError):
        detector.YoloDetector(conf_thres=True)


# ---------------------------------------------------------------------------
# detect：模块级入口的状态机与输入校验
# ---------------------------------------------------------------------------
def test_detect_before_setup_raises_runtime_error():
    detector.reset()
    with pytest.raises(RuntimeError):
        detector.detect(_image())


def test_setup_returns_instance_and_marks_ready():
    detector.reset()
    inst = detector.setup(model_path="yolov8n.pt", conf_thres=0.4)
    assert isinstance(inst, detector.YoloDetector)
    assert inst.conf_thres == 0.4
    assert detector.is_ready() is True
    detector.reset()
    assert detector.is_ready() is False


def test_detect_non_ndarray_raises_in_dev(monkeypatch):
    detector.reset()
    detector.setup()
    monkeypatch.setattr(detector, "ONBOARD", False)
    with pytest.raises(ValueError):
        detector.detect([[1, 2, 3]])          # list 而非 ndarray
    detector.reset()


def test_detect_non_ndarray_onboard_returns_empty(monkeypatch):
    detector.reset()
    detector.setup()
    monkeypatch.setattr(detector, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = detector.detect([[1, 2, 3]])
    assert out == []
    detector.reset()


def test_detect_empty_image_onboard_returns_empty(monkeypatch):
    """空数组 → []（上机阶段不崩）。"""
    detector.reset()
    detector.setup()
    monkeypatch.setattr(detector, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = detector.detect(np.zeros((0, 0, 3), dtype=np.uint8))
    assert out == []
    detector.reset()


def test_detect_bad_ndim_onboard_returns_empty(monkeypatch):
    detector.reset()
    detector.setup()
    monkeypatch.setattr(detector, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = detector.detect(np.zeros((4,), dtype=np.uint8))
    assert out == []
    detector.reset()
