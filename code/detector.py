"""模块 4：视觉检测（``/detector``）。

对应 ``design/函数接口.md`` 模块 4，含一个函数（签名严格照搬文档，不增删参数）：

- :func:`detect` —— YOLOv8/v11 目标检测。

设计约束
--------
- **空即正常**：图中无目标时返回 ``[]``，不视为错误——对应全链路
  「空 = 没目标（业务正常）、静默 = 故障」约定（设计原则第 5 条）。
  节点壳据此发布**空** ``Detections``（不静默）。
- **阶段化参数校验**：运行阶段统一在 ``config.py`` 读取（环境变量 ``UAV_STAGE``，默认 ``dev``）：
  - ``dev``     —— 参数非法直接抛 ``ValueError``，问题尽早暴露；
  - ``onboard`` —— 参数非法仅打印告警并返回 ``[]``，上机不因脏数据崩溃。
- **单帧无状态**：``detect`` 只处理当前帧，跨帧状态不驻留本模块
  （见《python代码移植到ros平台方法整理.md》8.1）。

范围说明
--------
本模块只提供**纯逻辑**（不依赖 ROS/``rospy``），便于单元测试；节点壳
（订阅 / 发布 / ``sensor_msgs/Image`` ↔ ``np.ndarray`` 转换）在
``scripts/detector_node.py`` 中完成。

模型加载
--------
``detect`` 的签名（文档定义）不含模型路径等参数，故模型等资源由
:func:`setup` 注入**:class:`YoloDetector` 实例**；模块级 :func:`detect`
是文档签名的便捷入口，委托给最近一次 ``setup`` 的实例。
节点应在 ``__init__`` 中 ``setup(...)`` 一次。

图像约定
--------
输入为 **BGR** 顺序的 ``np.ndarray``（H×W×3 或 H×W），与 ROS
``sensor_msgs/Image`` 的 ``bgr8`` 编码一致；ultralytics 原生接受该布局，
无需额外色彩空间转换。输出 ``bbox`` 统一为 ``(x, y, w, h)``（左上角 + 宽高），
而 ultralytics 原始输出为 ``xyxy``，本模块内部完成转换。
"""
from __future__ import annotations

import logging
import warnings
from typing import Optional

import numpy as np

from config import STAGE, ONBOARD  # noqa: F401  (STAGE 供使用方展示运行阶段)
from datatypes import DetectedTarget  # 统一用组长的类型名（见 M4交付说明/03 第 2 条）

__all__ = [
    "YoloDetector",
    "setup",
    "detect",
    "is_ready",
    "reset",
]

_LOG = logging.getLogger(__name__)

# --- 运行阶段开关统一定义在 config.py ---

# --- 默认参数（代码书写原则第 6 条：默认值集中定义）---
DEFAULT_MODEL_PATH = "yolov8n.pt"
DEFAULT_CONF_THRES = 0.5   # 低于状态机 0.8 判据，保证 0.8~1.0 区间不被截断
DEFAULT_IOU_THRES = 0.45
DEFAULT_IMGSZ = 640
DEFAULT_DEVICE = "cpu"     # 仿真机无 CUDA，只能 CPU 推理
DEFAULT_MAX_DET = 20


def _reject(message: str) -> bool:
    """参数非法时的统一处理（同 ``cluster._reject``）。

    - 开发阶段（默认）：抛出 ``ValueError``；
    - 上机阶段（``UAV_STAGE=onboard``）：发出 ``RuntimeWarning`` 并返回 ``True``，
      由调用方返回 ``[]`` 兜底。

    返回 ``True`` 表示调用方应立即返回空结果；抛出异常时本函数不返回。
    """
    if ONBOARD:
        warnings.warn(
            "[detector] %s；上机阶段按空结果处理（返回 []）" % message,
            RuntimeWarning,
            stacklevel=2,
        )
        return True
    raise ValueError("[detector] " + message)


def _is_number(value: object) -> bool:
    """判断是否为实数（``bool`` 不算）。"""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _xyxy_to_xywh(x1: float, y1: float, x2: float, y2: float):
    """``(x1, y1, x2, y2)`` → ``(x, y, w, h)``（左上角 + 宽高）。

    ultralytics 输出为 ``xyxy``，而《数据类型.md》1.1/2.4 约定
    ``bbox = [x, y, w, h]``，故在此统一转换。
    """
    return (float(x1), float(y1), float(x2) - float(x1), float(y2) - float(y1))


class YoloDetector:
    """YOLOv8/v11 检测器（持有模型资源）。

    模型在**首次推理时**才真正加载（惰性），使本模块在未安装 ``ultralytics``
    时依然可以被 ``import``（单元测试可只测转换逻辑）。

    参数
    ----
    model_path : str
        权重路径或 ultralytics 可识别的模型名（如 ``"yolov8n.pt"``）。
    conf_thres : float
        置信度阈值。
    iou_thres : float
        NMS 的 IoU 阈值。
    imgsz : int
        推理输入边长。
    device : str
        ``"cpu"`` 或 ``"cuda"``。
    classes : list[int] | None
        只保留的类别 ID；``None`` / 空表示全部类别。
    max_det : int
        单帧最大检出数。
    """

    def __init__(
        self,
        model_path: str = DEFAULT_MODEL_PATH,
        conf_thres: float = DEFAULT_CONF_THRES,
        iou_thres: float = DEFAULT_IOU_THRES,
        imgsz: int = DEFAULT_IMGSZ,
        device: str = DEFAULT_DEVICE,
        classes: Optional[list[int]] = None,
        max_det: int = DEFAULT_MAX_DET,
    ) -> None:
        if not isinstance(model_path, str) or not model_path:
            if _reject("YoloDetector: model_path 必须为非空 str，实际为 %r" % (model_path,)):
                model_path = DEFAULT_MODEL_PATH
        if not _is_number(conf_thres):
            if _reject("YoloDetector: conf_thres 必须为数值，实际为 %r" % (conf_thres,)):
                conf_thres = DEFAULT_CONF_THRES
        if not _is_number(iou_thres):
            if _reject("YoloDetector: iou_thres 必须为数值，实际为 %r" % (iou_thres,)):
                iou_thres = DEFAULT_IOU_THRES
        if not isinstance(imgsz, int) or isinstance(imgsz, bool) or imgsz <= 0:
            if _reject("YoloDetector: imgsz 必须为正整数，实际为 %r" % (imgsz,)):
                imgsz = DEFAULT_IMGSZ

        self.model_path = model_path
        self.conf_thres = float(conf_thres)
        self.iou_thres = float(iou_thres)
        self.imgsz = int(imgsz)
        self.device = device
        self.classes = list(classes) if classes else None
        self.max_det = int(max_det)
        self._model = None  # 惰性加载

    # ------------------------------------------------------------------ #
    # 模型加载
    # ------------------------------------------------------------------ #
    def _ensure_model(self):
        """惰性加载 ultralytics 模型（只加载一次）。"""
        if self._model is None:
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                raise RuntimeError(
                    "[detector] 未安装 ultralytics，无法进行检测；"
                    "请先安装视觉依赖（见 requirements.txt）"
                ) from exc
            self._model = YOLO(self.model_path)
            _LOG.info("[detector] 模型已加载: %s", self.model_path)
        return self._model

    # ------------------------------------------------------------------ #
    # 主逻辑
    # ------------------------------------------------------------------ #
    def detect(self, image: np.ndarray) -> list[DetectedTarget]:
        """对单帧图像做目标检测。

        参数
        ----
        image : np.ndarray
            BGR 图像，形状 ``(H, W, 3)``（``(H, W)`` 灰度亦可）。

        返回
        ----
        list[DetectedTarget]
            检测结果；无目标时返回 ``[]``（空即正常，非错误）。
        """
        if not isinstance(image, np.ndarray):
            if _reject("detect: image 必须为 np.ndarray，实际为 %r" % (type(image),)):
                return []
        if image.size == 0:
            if _reject("detect: image 为空数组"):
                return []
        if image.ndim not in (2, 3):
            if _reject("detect: image 维数应为 2 或 3，实际为 %d" % image.ndim):
                return []

        model = self._ensure_model()
        results = model.predict(
            source=image,
            conf=self.conf_thres,
            iou=self.iou_thres,
            imgsz=self.imgsz,
            device=self.device,
            classes=self.classes,
            max_det=self.max_det,
            verbose=False,
        )
        return self.to_detections(results)

    @staticmethod
    def to_detections(results) -> list[DetectedTarget]:
        """ultralytics 推理结果 → ``list[DetectedTarget]``（纯转换，可单测）。

        逐框做 ``xyxy → xywh`` 转换与类型规整；无框时返回 ``[]``。
        """
        detections: list[DetectedTarget] = []
        if not results:
            return detections
        for res in results:
            boxes = getattr(res, "boxes", None)
            if boxes is None or len(boxes) == 0:
                continue
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy()
            for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, classes):
                detections.append(
                    DetectedTarget(
                        bbox=_xyxy_to_xywh(x1, y1, x2, y2),
                        class_id=int(cls_id),
                        confidence=float(conf),
                    )
                )
        return detections


# --------------------------------------------------------------------------- #
# 模块级便捷入口（文档定义签名）
# --------------------------------------------------------------------------- #
_detector: Optional[YoloDetector] = None


def setup(**kwargs) -> YoloDetector:
    """配置并返回检测器实例；节点应在 ``__init__`` 中调用一次。

    支持直接传 :class:`YoloDetector` 的构造参数::

        from detector import setup, detect
        setup(model_path="models/uav.pt", conf_thres=0.4)

    返回实例便于节点自行持有（设计 8.1：跨帧状态放节点成员变量）。
    """
    global _detector
    _detector = YoloDetector(**kwargs)
    return _detector


def is_ready() -> bool:
    """模块级 :func:`detect` 是否可用（即是否已 ``setup``）。"""
    return _detector is not None


def reset() -> None:
    """释放模块级实例（主要用于单元测试）。"""
    global _detector
    _detector = None


def detect(image: np.ndarray) -> list[DetectedTarget]:
    """YOLO 目标检测（``design/函数接口.md`` 4.1，文档定义，签名照搬）。

    模块级便捷入口，委托给最近一次 :func:`setup` 创建的 :class:`YoloDetector`。
    节点可改为自行持有实例并调用 ``instance.detect(image)``，二者等价。

    参数
    ----
    image : np.ndarray
        输入图像（BGR）。

    返回
    ----
    list[DetectedTarget]
        检测结果列表；无目标时返回 ``[]``。

    异常
    ----
    RuntimeError
        未调用 :func:`setup`，或未安装 ``ultralytics``。
    """
    if _detector is None:
        raise RuntimeError(
            "[detector] 检测器未配置；请先调用 detector.setup(...)（节点在 __init__ 中完成）"
        )
    return _detector.detect(image)
