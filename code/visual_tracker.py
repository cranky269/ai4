"""模块 5：视觉跟踪（``/visual_tracker``）。

对应 ``design/函数接口.md`` 模块 5，含一个函数（签名严格照搬文档，不增删参数）：

- :func:`track` —— BoT-SORT 目标跟踪（吃检测器输出）。

算法说明
--------
本实现覆盖 **BoT-SORT 的核心思想**（ReID 关闭、GMC 为可插拔钩子）：

1. **卡尔曼滤波**（匀速模型，状态 ``[cx, cy, w, h, vcx, vcy, vw, vh]``）逐航迹预测；
2. **两阶段关联**（BoT-SORT 的标志性特征）：
   - 阶段一：高分检测（``conf >= track_high_thresh``）与全部航迹做 IoU 匹配；
   - 阶段二：低分检测（``track_low_thresh <= conf < track_high_thresh``）与阶段一
     未匹配的航迹再匹配——目标被遮挡、分数掉下来时借此**保住 ID**，
     而不是当成新目标（这是相对原版 SORT 的主要收益）；
3. 匹配代价用 IoU，采用**贪心分配**（决策文档第八章第 5 条：前期用最近邻，
   真机或高机动场景再换匈牙利分配）；
4. 未匹配航迹进入 buffer，连续 ``track_buffer`` 帧未更新才删除。

关于 GMC（可选优化）
--------------------
BoT-SORT 原版的 GMC（全局运动补偿）需要**原始图像**估计背景运动，依赖
``boxmot`` / ``ultralytics`` 的 ``update(dets, img)`` 接口。而本模块签名
``track(detections)`` 无图像入参，且 ``/visual_tracker`` 按节点图谱**只订阅
``/detections``**、不订阅图像。故 GMC **默认关闭**，仅保留 :meth:`set_gmc`
钩子——这与决策文档第九章第 3 条「GMC 列为可选优化项；若不上 BoT-SORT，
纯 SORT 即可」一致，即：**守住签名与图谱，不为此新增订阅**。

设计约束
--------
- **空即正常**：输入为空时返回 ``[]``（不视为错误）——对应全链路
  「空 = 没目标（业务正常）、静默 = 故障」约定（设计原则第 5 条）。
  节点壳据此发布**空** ``VisualTrackArray``（不静默），同时为 ``/state_machine``
  的 N/M/K 帧计数提供节拍。
- **阶段化参数校验**：运行阶段统一在 ``config.py`` 读取（环境变量 ``UAV_STAGE``，默认 ``dev``）：
  - ``dev``     —— 参数非法直接抛 ``ValueError``，问题尽早暴露；
  - ``onboard`` —— 参数非法仅打印告警并用默认值兜底，上机不因脏数据崩溃。
- **跨帧状态归节点持有**（见《python代码移植到ros平台方法整理.md》8.1）：
  跟踪器是**有状态**的，状态住在 :class:`BotSortTracker` 实例里，
  由 ``/visual_tracker`` 节点的成员变量持有；:func:`track` 为文档签名的
  便捷入口，委托给最近一次 :func:`setup` 的实例。

范围说明
--------
本模块只提供**纯逻辑**（不依赖 ROS/``rospy``），便于单元测试；节点壳
（订阅 ``/detections``、发布 ``/visual/tracks``、消息互转）在
``scripts/visual_tracker_node.py`` 中完成。``VisualTrack`` 用
``datatypes`` 中的纯数据结构，不用 ROS 消息，以保证算法层可离线测试。
"""
from __future__ import annotations

import logging
import warnings
from typing import Optional

import numpy as np

from config import STAGE, ONBOARD  # noqa: F401  (STAGE 供使用方展示运行阶段)
from datatypes import DetectedTarget, VisualTrack  # 统一用组长的类型名（见 M4交付说明/03 第 2 条）

__all__ = [
    "BotSortTracker",
    "setup",
    "track",
    "is_ready",
    "reset",
]

_LOG = logging.getLogger(__name__)

# --- 运行阶段开关统一定义在 config.py ---

# --- 默认参数（代码书写原则第 6 条：默认值集中定义）---
DEFAULT_TRACK_HIGH_THRESH = 0.5   # 阶段一：高分检测阈值
DEFAULT_TRACK_LOW_THRESH = 0.1    # 阶段二：低分检测阈值
DEFAULT_NEW_TRACK_THRESH = 0.6    # 新建航迹所需的最低分数
DEFAULT_TRACK_BUFFER = 30         # 航迹丢失后保留帧数（对应状态机 M 帧熔断）
DEFAULT_MATCH_THRESH = 0.8        # IoU 距离阈值，> 该值视为不匹配
DEFAULT_MIN_AREA_RATIO = 0.0      # 过滤过小框，0 = 不过滤
DEFAULT_FRAME_SIZE = (640, 480)   # (宽, 高)，用于计算 area_ratio


def _reject(message: str) -> bool:
    """参数非法时的统一处理（同 ``cluster._reject``）。

    - 开发阶段（默认）：抛出 ``ValueError``；
    - 上机阶段（``UAV_STAGE=onboard``）：发出 ``RuntimeWarning`` 并返回 ``True``，
      由调用方用默认值兜底或返回 ``[]``。

    返回 ``True`` 表示调用方应走兜底分支；抛出异常时本函数不返回。
    """
    if ONBOARD:
        warnings.warn(
            "[visual_tracker] %s；上机阶段按默认值/空结果处理" % message,
            RuntimeWarning,
            stacklevel=2,
        )
        return True
    raise ValueError("[visual_tracker] " + message)


def _is_number(value: object) -> bool:
    """判断是否为实数（``bool`` 不算）。"""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _cxcywh_to_xywh(box) -> tuple[float, float, float, float]:
    """``(cx, cy, w, h)`` → ``(x, y, w, h)``（左上角 + 宽高）。

    对外输出统一用《数据类型.md》的 ``bbox = [x, y, w, h]`` 约定。
    """
    cx, cy, w, h = box
    return (float(cx - w / 2.0), float(cy - h / 2.0), float(w), float(h))


def _detection_to_cxcywh(det: DetectedTarget) -> list[float]:
    """``DetectedTarget.bbox``（x, y, w, h）→ 卡尔曼观测 ``[cx, cy, w, h]``。"""
    x, y, w, h = det.bbox
    return [x + w / 2.0, y + h / 2.0, w, h]


def iou_matrix(boxes_a: np.ndarray, boxes_b: np.ndarray) -> np.ndarray:
    """两批 ``cxcywh`` 框之间的 IoU 矩阵，形状 ``(len(a), len(b))``。

    任一输入为空时返回相应形状的全零矩阵（便于直接参与匹配）。
    """
    if len(boxes_a) == 0 or len(boxes_b) == 0:
        return np.zeros((len(boxes_a), len(boxes_b)), dtype=float)

    a = boxes_a[:, None, :]
    b = boxes_b[None, :, :]
    a_xy1, a_xy2 = a[..., :2] - a[..., 2:] / 2.0, a[..., :2] + a[..., 2:] / 2.0
    b_xy1, b_xy2 = b[..., :2] - b[..., 2:] / 2.0, b[..., :2] + b[..., 2:] / 2.0

    inter_wh = np.clip(np.minimum(a_xy2, b_xy2) - np.maximum(a_xy1, b_xy1), 0.0, None)
    inter = inter_wh[..., 0] * inter_wh[..., 1]
    union = a[..., 2] * a[..., 3] + b[..., 2] * b[..., 3] - inter
    return np.where(union > 0, inter / union, 0.0)


def greedy_match(iou: np.ndarray, match_thresh: float) -> list[tuple[int, int]]:
    """贪心 IoU 匹配（文档第八章第 5 条：前期用最近邻）。

    代价定义为 ``1 - IoU``，代价 ``> match_thresh`` 视为不匹配——与 BoT-SORT
    的 ``match_thresh`` 语义一致。按 IoU 从大到小依次认领，行列均不重复。

    参数
    ----
    iou : np.ndarray
        形状 ``(航迹数, 检测数)`` 的 IoU 矩阵。
    match_thresh : float
        匹配阈值。

    返回
    ----
    list[tuple[int, int]]
        ``(航迹下标, 检测下标)`` 列表。
    """
    pairs: list[tuple[int, int]] = []
    if iou.size == 0:
        return pairs

    used_row: set[int] = set()
    used_col: set[int] = set()
    for flat in np.argsort(-iou, axis=None):
        r, c = int(flat // iou.shape[1]), int(flat % iou.shape[1])
        if r in used_row or c in used_col:
            continue
        if (1.0 - iou[r, c]) > match_thresh:
            break  # IoU 已按降序，后续只会更小
        pairs.append((r, c))
        used_row.add(r)
        used_col.add(c)
    return pairs


class KalmanBoxFilter:
    """针对 bbox 中心/尺寸的匀速卡尔曼滤波（8 维状态 / 4 维观测）。

    状态量 ``x = [cx, cy, w, h, vcx, vcy, vw, vh]``，观测
    ``z = [cx, cy, w, h]``。过程噪声对速度分量给更大不确定度
    （速度不可直接观测）。

    属性
    ----
    x : np.ndarray
        状态量，形状 ``(8, 1)``。
    P : np.ndarray
        状态协方差，形状 ``(8, 8)``。
    """

    DIM_X = 8
    DIM_Z = 4

    def __init__(self, cx: float, cy: float, w: float, h: float) -> None:
        self.x = np.zeros((self.DIM_X, 1), dtype=float)
        self.x[:4, 0] = [cx, cy, w, h]

        self.F = np.eye(self.DIM_X, dtype=float)
        self.H = np.zeros((self.DIM_Z, self.DIM_X), dtype=float)
        self.H[:4, :4] = np.eye(4)

        self.Q = np.eye(self.DIM_X, dtype=float) * 1e-2
        self.Q[4:, 4:] *= 1e-1
        self.R = np.eye(self.DIM_Z, dtype=float) * 1e-1

        self.P = np.eye(self.DIM_X, dtype=float) * 10.0
        self.P[4:, 4:] *= 100.0

    def predict(self, dt: float = 1.0) -> None:
        """状态预测：``x = F x``，``P = F P Fᵀ + Q``。"""
        for i in range(4):
            self.F[i, i + 4] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement: np.ndarray) -> None:
        """量测更新：标准卡尔曼增益修正。"""
        z = np.asarray(measurement, dtype=float).reshape((self.DIM_Z, 1))
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(self.DIM_X) - K @ self.H) @ self.P

    @property
    def box(self) -> np.ndarray:
        """当前估计框 ``[cx, cy, w, h]``。"""
        return self.x[:4, 0].copy()


class _Track:
    """单条内部航迹（不属于对外数据类型，故不放入 ``datatypes.py``）。"""

    __slots__ = ("track_id", "kf", "hits", "age", "time_since_update", "last_score")

    def __init__(self, track_id: int, kf: KalmanBoxFilter, score: float) -> None:
        self.track_id = track_id
        self.kf = kf
        self.hits = 1
        self.age = 0
        self.time_since_update = 0
        self.last_score = score


class BotSortTracker:
    """BoT-SORT 跟踪器纯逻辑（有状态）。

    对应 ``/visual_tracker`` 节点的跨帧状态（设计 8.1）——**由节点持实例**，
    而不是模块级全局变量。

    参数
    ----
    track_high_thresh : float
        阶段一高分检测阈值。
    track_low_thresh : float
        阶段二低分检测阈值。
    new_track_thresh : float
        新建航迹所需的最低分数。
    track_buffer : int
        航迹丢失后保留帧数。
    match_thresh : float
        IoU 距离阈值。
    frame_size : tuple[int, int]
        ``(宽, 高)``，``area_ratio`` 的**兜底**计算基准，默认模块常量
        :data:`DEFAULT_FRAME_SIZE`。运行时若 :meth:`track` 收到图像，
        会自动改用图像实际尺寸（见 :meth:`set_frame_size`）。
    min_area_ratio : float
        过滤过小框，``0`` 表示不过滤。
    """

    def __init__(
        self,
        track_high_thresh: float = DEFAULT_TRACK_HIGH_THRESH,
        track_low_thresh: float = DEFAULT_TRACK_LOW_THRESH,
        new_track_thresh: float = DEFAULT_NEW_TRACK_THRESH,
        track_buffer: int = DEFAULT_TRACK_BUFFER,
        match_thresh: float = DEFAULT_MATCH_THRESH,
        frame_size: tuple[int, int] = DEFAULT_FRAME_SIZE,
        min_area_ratio: float = DEFAULT_MIN_AREA_RATIO,
    ) -> None:
        numeric = {
            "track_high_thresh": track_high_thresh,
            "track_low_thresh": track_low_thresh,
            "new_track_thresh": new_track_thresh,
            "match_thresh": match_thresh,
            "min_area_ratio": min_area_ratio,
        }
        fallback = {
            "track_high_thresh": DEFAULT_TRACK_HIGH_THRESH,
            "track_low_thresh": DEFAULT_TRACK_LOW_THRESH,
            "new_track_thresh": DEFAULT_NEW_TRACK_THRESH,
            "match_thresh": DEFAULT_MATCH_THRESH,
            "min_area_ratio": DEFAULT_MIN_AREA_RATIO,
        }
        for name, value in numeric.items():
            if not _is_number(value):
                if _reject(
                    "BotSortTracker: %s 必须为数值，实际为 %r" % (name, value)
                ):
                    numeric[name] = fallback[name]

        if not isinstance(track_buffer, int) or isinstance(track_buffer, bool):
            if _reject(
                "BotSortTracker: track_buffer 必须为整数，实际为 %r" % (track_buffer,)
            ):
                track_buffer = DEFAULT_TRACK_BUFFER

        self.track_high_thresh = float(numeric["track_high_thresh"])
        self.track_low_thresh = float(numeric["track_low_thresh"])
        self.new_track_thresh = float(numeric["new_track_thresh"])
        self.match_thresh = float(numeric["match_thresh"])
        self.min_area_ratio = float(numeric["min_area_ratio"])
        self.track_buffer = int(track_buffer)

        self._frame_size: tuple[int, int] = DEFAULT_FRAME_SIZE
        if isinstance(frame_size, (list, tuple)) and len(frame_size) == 2:
            self.set_frame_size(*frame_size)
        else:
            _reject("BotSortTracker: frame_size 应为 (宽, 高)，实际为 %r" % (frame_size,))

        self._tracks: list[_Track] = []
        self._next_id = 1
        self._gmc = None

    # ------------------------------------------------------------------ #
    # 配置
    # ------------------------------------------------------------------ #
    @property
    def frame_size(self) -> tuple[int, int]:
        """当前画面尺寸 ``(宽, 高)``。"""
        return self._frame_size

    def set_frame_size(self, width: int, height: int) -> None:
        """注入画面尺寸——``area_ratio`` 的计算基准。

        画面尺寸有两个来源，**前者优先**：

        1. **图像入参**：``track(detections, image)`` 会从 ``image.shape`` 自动取，
           无需手工调用本方法（节点走这条路径）；
        2. **本方法 / 构造参数** ``frame_size``：直接库调用（无图像）时使用，
           默认值为模块常量 :data:`DEFAULT_FRAME_SIZE`。

        非法值在 ``dev`` 阶段抛错，``onboard`` 阶段告警并沿用当前值。
        """
        if not isinstance(width, int) or isinstance(width, bool) or width <= 0:
            if _reject("set_frame_size: width 必须为正整数，实际为 %r" % (width,)):
                return
        if not isinstance(height, int) or isinstance(height, bool) or height <= 0:
            if _reject("set_frame_size: height 必须为正整数，实际为 %r" % (height,)):
                return
        self._frame_size = (int(width), int(height))

    def _apply_image_size(self, image) -> None:
        """从图像取画面尺寸并更新 ``area_ratio`` 的计算基准。

        ``numpy`` 图像形状为 ``(高, 宽[, 通道])``，故取 ``shape[1]`` 为宽、
        ``shape[0]`` 为高。形状非法时告警并沿用当前尺寸，不影响跟踪主流程。
        """
        shape = getattr(image, "shape", None)
        if not shape or len(shape) < 2:
            _reject("track: image 形状非法，实际为 %r" % (shape,))
            return
        height, width = int(shape[0]), int(shape[1])
        if width > 0 and height > 0:
            self.set_frame_size(width, height)

    def set_gmc(self, fn) -> None:
        """注册全局运动补偿钩子（**可选优化项**，见模块 docstring「关于 GMC」）。

        钩子签名建议 ``fn(prev_homography) -> np.ndarray``（3×3 单应矩阵）。

        ``track(..., image)`` 已能拿到原始图像，GMC 具备启用条件；
        但 GMC 属《拆解文档》第九章第 3 条的**可选优化**，
        故当前**默认不启用**，留此钩子以便需要时接入。
        """
        self._gmc = fn

    @property
    def track_count(self) -> int:
        """当前存活的航迹条数（含暂时丢失、仍在 buffer 内的）。"""
        return len(self._tracks)

    # ------------------------------------------------------------------ #
    # 主逻辑
    # ------------------------------------------------------------------ #
    def track(
        self,
        detections: list[DetectedTarget],
        image: np.ndarray | None = None,
    ) -> list[VisualTrack]:
        """对单帧检测结果做跟踪，输出本帧的视觉航迹。

        参数
        ----
        detections : list[DetectedTarget]
            本帧检测结果（模块 4 ``detect`` 的输出）。
        image : np.ndarray | None
            本帧原始图像（``bgr8``），**与模块 4 ``detect`` 的入参同类型**。

            - **传入**：用它取实际画面尺寸（``image.shape``）作为 ``area_ratio``
              的计算基准；同时为 GMC（全局运动补偿）提供输入。
            - **不传**（``None``）：沿用 :meth:`set_frame_size` 或模块常量
              :data:`DEFAULT_FRAME_SIZE` 设定的尺寸，仍可正常工作。

        返回
        ----
        list[VisualTrack]
            **本帧更新到**的航迹（丢失中的航迹不输出，避免下游追"幽灵目标"）；
            输入为空或无有效航迹时返回 ``[]``。
        """
        if detections is None:
            detections = []
        if not isinstance(detections, (list, tuple)):
            if _reject("track: detections 必须为序列，实际为 %r" % (type(detections),)):
                return []

        # 0) 画面尺寸：有图像就用图像的实际尺寸（更准），否则沿用已设定值
        if image is not None:
            self._apply_image_size(image)

        # 1) 预测：全部航迹推进一步，丢失计数 +1
        for trk in self._tracks:
            trk.kf.predict(1.0)
            trk.age += 1
            trk.time_since_update += 1

        # 2) 按分数分组（两阶段关联的前提）
        high = [d for d in detections if d.confidence >= self.track_high_thresh]
        low = [
            d for d in detections
            if self.track_low_thresh <= d.confidence < self.track_high_thresh
        ]

        matched_dets: set[int] = set()
        matched_tracks = self._associate(high, None, matched_dets, None)

        # 阶段二：低分检测 ←→ 阶段一未匹配的航迹（保住被遮挡目标的 ID）
        unmatched = [i for i in range(len(self._tracks)) if i not in matched_tracks]
        self._associate(low, unmatched, matched_dets, matched_tracks)

        # 3) 未匹配的高分检测 → 新建航迹
        for det in high:
            if id(det) not in matched_dets and det.confidence >= self.new_track_thresh:
                self._tracks.append(
                    _Track(self._next_id, KalmanBoxFilter(*_detection_to_cxcywh(det)), det.confidence)
                )
                self._next_id += 1

        # 4) 淘汰超期航迹
        self._tracks = [
            t for t in self._tracks if t.time_since_update <= self.track_buffer
        ]

        return self._to_outputs()

    # ------------------------------------------------------------------ #
    # 内部：单阶段关联
    # ------------------------------------------------------------------ #
    def _associate(
        self,
        dets: list[DetectedTarget],
        track_pool: Optional[list[int]],
        matched_dets: set[int],
        matched_tracks: Optional[set[int]],
    ) -> set[int]:
        """把 ``dets`` 与 ``track_pool``（``None`` 表示全部航迹）做一次匹配。

        匹配上的航迹做量测更新并清零丢失计数；返回本次匹配上的航迹下标集合。
        """
        pool = list(range(len(self._tracks))) if track_pool is None else track_pool
        matched: set[int] = set()
        if not dets or not pool:
            return matched

        t_boxes = np.array([self._tracks[i].kf.box for i in pool], dtype=float)
        d_boxes = np.array([_detection_to_cxcywh(d) for d in dets], dtype=float)
        iou = iou_matrix(t_boxes, d_boxes)

        for r, c in greedy_match(iou, self.match_thresh):
            trk = self._tracks[pool[r]]
            det = dets[c]
            trk.kf.update(np.array(_detection_to_cxcywh(det), dtype=float))
            trk.hits += 1
            trk.time_since_update = 0
            trk.last_score = det.confidence
            matched.add(pool[r])
            matched_dets.add(id(det))
            if matched_tracks is not None:
                matched_tracks.add(pool[r])
        return matched

    # ------------------------------------------------------------------ #
    # 内部：组装输出
    # ------------------------------------------------------------------ #
    def _to_outputs(self) -> list[VisualTrack]:
        """把本帧更新到的航迹转成对外 ``VisualTrack`` 列表。"""
        width, height = self._frame_size
        total_px = float(width * height)
        outs: list[VisualTrack] = []
        for trk in self._tracks:
            if trk.time_since_update != 0:
                continue  # 本帧未更新：不输出（空 = 没目标）
            x, y, w, h = _cxcywh_to_xywh(trk.kf.box)
            area_ratio = (w * h) / total_px if total_px > 0 else 0.0
            if area_ratio < self.min_area_ratio:
                continue
            outs.append(VisualTrack(id=trk.track_id, bbox=(x, y, w, h), area_ratio=area_ratio))
        return outs


# --------------------------------------------------------------------------- #
# 模块级便捷入口（文档定义签名）
# --------------------------------------------------------------------------- #
_tracker: Optional[BotSortTracker] = None


def setup(**kwargs) -> BotSortTracker:
    """配置并返回跟踪器实例；节点应在 ``__init__`` 中调用一次。

    返回实例便于节点自行持有（设计 8.1：跨帧状态放节点成员变量）::

        from visual_tracker import DEFAULT_FRAME_SIZE, setup, track

        setup(track_buffer=30, frame_size=DEFAULT_FRAME_SIZE)   # 引用常量，不硬编码字面量

    ``frame_size`` 是 ``area_ratio`` 的**兜底基准**：节点若在
    ``track(detections, image)`` 里传了图像，运行时会自动改用图像实际尺寸。
    """
    global _tracker
    _tracker = BotSortTracker(**kwargs)
    return _tracker


def is_ready() -> bool:
    """模块级 :func:`track` 是否可用（即是否已 ``setup``）。"""
    return _tracker is not None


def reset() -> None:
    """释放模块级实例（主要用于单元测试）。"""
    global _tracker
    _tracker = None


def track(
    detections: list[DetectedTarget],
    image: np.ndarray | None = None,
) -> list[VisualTrack]:
    """BoT-SORT 目标跟踪（``design/函数接口.md`` 5.1 + 组长确认补充的图像入参）。

    模块级便捷入口，委托给最近一次 :func:`setup` 创建的 :class:`BotSortTracker`。
    节点可改为自行持有实例并调用 ``instance.track(dets, image)``，二者等价。

    参数
    ----
    detections : list[DetectedTarget]
        检测结果列表（模块 4 ``detect`` 的输出）。
    image : np.ndarray | None
        本帧原始图像，**类型与模块 4 ``detect`` 的入参一致**（``np.ndarray``）。
        传入时用它取实际画面尺寸计算 ``area_ratio``；默认 ``None`` 时
        沿用 ``setup(frame_size=...)`` 或模块常量 :data:`DEFAULT_FRAME_SIZE`。

    返回
    ----
    list[VisualTrack]
        视觉航迹列表；输入为空时返回 ``[]``。

    异常
    ---
    RuntimeError
        未调用 :func:`setup`。

    注解说明
    --------
    签名写作 ``list[...]``（PEP 585 内置泛型）而非 ``typing.List``，
    与组长既有代码（``cluster.py`` / ``state_machine.py`` 等）风格一致——
    本模块顶部已 ``from __future__ import annotations``，
    故 Python 3.8 下也不会在函数定义时求值报错。
    """
    if _tracker is None:
        raise RuntimeError(
            "[visual_tracker] 跟踪器未配置；请先调用 visual_tracker.setup(...)（节点在 __init__ 中完成）"
        )
    return _tracker.track(detections, image)
