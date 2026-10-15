"""项目自定义数据类型定义（全项目唯一入口）。

约定
----
- 本项目所有**自定义数据类型**统一在本文件定义，其余 py 文件一律 ``import`` 引用，
  不再重复定义。
- 字段严格照搬 ``design/数据类型.md``，不新增、不虚构。

已定义（按 ``design/数据类型.md`` 第二章章节归类）：
- 2.1 / 2.2 · 模块 1 点迹聚类：:class:`Point3D`、:class:`Cluster`；
- 2.3 · 模块 2 跟踪：:class:`TrackState`（卡尔曼内部滤波状态）；
- 2.4 · 消息镜像 dataclass（与 ``.msg`` 同名、字段一致；节点壳负责
  ``.msg`` ↔ 纯 dataclass 拆装包）：:class:`Track`、:class:`VisualTrack`、
  :class:`DetectedTarget`；
- 2.7 / 2.8 / 2.9 · 模块 3 导航与模块 6 PID：:class:`Waypoint`、
  :class:`Pose`、:class:`PositionCommand`。

新增类型时在本文件统一定义，不在其他 py 重复定义。

Python 3.8 兼容说明
-------------------
为在 3.8 下直接使用 PEP 585 泛型（``list[...]`` / ``tuple[...]``）作为注解，
本模块启用 ``from __future__ import annotations``（注解延迟求值为字符串）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

__all__ = [
    "Point3D",
    "Cluster",
    "DetectedTarget",
    "VisualTrack",
    "Track",
    "TrackState",
    "Waypoint",
    "Pose",
    "PositionCommand",
]


@dataclass
class Point3D:
    """雷达点云中的一个三维点（``design/数据类型.md`` 2.1）。"""

    x: float
    y: float
    z: float
    intensity: float = 0.0


@dataclass
class Cluster:
    """DBSCAN 聚类结果（``design/数据类型.md`` 2.2）。

    属性
    ----
    id : int
        聚类 ID（从 0 起按出现顺序编号）。
    centroid : tuple[float, float, float]
        质心 ``(x, y, z)``。
    points : list[Point3D]
        簇内点集。
    size : int
        簇内点数，恒等于 ``len(points)``。
    """

    id: int
    centroid: Tuple[float, float, float]
    points: List[Point3D]
    size: int


@dataclass
class DetectedTarget:
    """一次目标检测结果（``design/数据类型.md`` 2.4.3，消息 ``DetectedTarget.msg``）。

    ``/detections`` 话题元素。字段与消息定义一一对应：
    ``class_id`` / ``confidence`` / ``bbox=[x, y, w, h]``。
    """

    class_id: int
    confidence: float
    bbox: Tuple[float, float, float, float]


@dataclass
class VisualTrack:
    """视觉航迹（``design/数据类型.md`` 2.4.2，消息 ``VisualTrack.msg``）。

    ``/visual/tracks`` 话题元素。``area_ratio`` 为 bbox 面积 / 画面总面积。
    """

    id: int
    bbox: Tuple[float, float, float, float]
    area_ratio: float


@dataclass
class Track:
    """雷达航迹（``design/数据类型.md`` 2.4.1，消息 ``Track.msg``）。

    ``/radar/tracks`` 话题元素。注意与模块 2 的**内部滤波结构**
    :class:`TrackState`（state/cov）区分——本类为消息镜像、发布用的结构，
    ``covariance`` 为 6x6 协方差展平的 36 维序列。
    """

    id: int
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    covariance: Tuple[float, ...]
    age: int


@dataclass
class TrackState:
    """内部卡尔曼航迹状态（``design/数据类型.md`` 2.3）。

    ``state`` 的六个元素依次为 ``[x, y, z, vx, vy, vz]``，``cov`` 为对应
    的 6x6 协方差矩阵。该结构只供纯算法层使用，不是 ROS 消息。
    """

    id: int
    state: np.ndarray
    cov: np.ndarray
    age: int
    last_update: float


@dataclass
class Waypoint:
    """纯导航航点，字段与 develop 正式接口一致。"""

    position: Tuple[float, float, float]
    yaw: float = 0.0


@dataclass(frozen=True)
class Pose:
    """纯位姿类型，镜像 ``geometry_msgs/Pose`` 的位置和四元数。"""

    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass(frozen=True)
class PositionCommand:
    """纯控制指令，发布前由 ROS 节点转换为 ``PositionTarget``。"""

    coordinate_frame: int
    type_mask: int
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    yaw_rate: float = 0.0

    @property
    def velocity_x(self) -> float:
        return self.velocity[0]

    @property
    def velocity_y(self) -> float:
        return self.velocity[1]

    @property
    def velocity_z(self) -> float:
        return self.velocity[2]
