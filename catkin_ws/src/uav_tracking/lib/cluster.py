"""模块 1：点迹聚类（``/cluster``）。

对应 ``design/函数接口.md`` 模块 1，含两个函数（签名严格照搬文档，不增删参数）：

- :func:`filter_ground`  —— 按高度阈值滤除地面杂波点；
- :func:`dbscan_cluster` —— 官方 ``sklearn.cluster.DBSCAN`` 聚类并取质心。

调用顺序：``filter_ground`` → ``dbscan_cluster``（先滤地、再聚类）。

设计约束
--------
- **空即正常**：输入为空（无点）时返回 ``[]``，不视为错误——对应全链路
  「空 = 没目标（业务正常）、静默 = 故障」约定（设计原则第 5 条）。
- **阶段化参数校验**：运行阶段统一在 ``config.py`` 读取（环境变量 ``UAV_STAGE``，默认 ``dev``）：
  - ``dev``     —— 参数非法直接抛 ``ValueError``，问题尽早暴露；
  - ``onboard`` —— 参数非法仅打印告警并返回 ``[]``，上机不因脏数据崩溃。

范围说明
--------
本模块只提供纯函数与数据类，**不涉及 ROS 节点**。``sensor_msgs/PointCloud2`` →
``list[Point3D]`` 的转换属于节点封装，在后续 ``/cluster`` 节点中完成。
"""
from __future__ import annotations

import warnings

import numpy as np
from sklearn.cluster import DBSCAN

from config import STAGE, ONBOARD  # noqa: F401  (STAGE 供使用方展示运行阶段)
from datatypes import Cluster, Point3D

__all__ = ["filter_ground", "dbscan_cluster"]

# 运行阶段开关（``dev`` 严格 / ``onboard`` 宽松）统一定义在 ``config.py``。


def _reject(message: str) -> bool:
    """参数非法时的统一处理。

    - 开发阶段（默认）：抛出 ``ValueError``；
    - 上机阶段（``UAV_STAGE=onboard``）：发出 ``RuntimeWarning`` 并返回 ``True``，
      由调用方返回 ``[]`` 兜底。

    返回 ``True`` 表示调用方应立即返回空结果；抛出异常时本函数不返回。
    """
    if ONBOARD:
        warnings.warn(
            "[cluster] %s；上机阶段按空结果处理（返回 []）" % message,
            RuntimeWarning,
            stacklevel=2,
        )
        return True
    raise ValueError("[cluster] " + message)


def _is_number(value: object) -> bool:
    """判断是否为实数（``bool`` 不算）。"""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def filter_ground(points: list[Point3D], z_thresh: float) -> list[Point3D]:
    """滤除地面杂波点（``design/函数接口.md`` 1.1，文档定义）。

    保留高度 ``z > z_thresh`` 的点，丢弃 ``z <= z_thresh`` 的地面点
    （地面点 z 约为 0，位于阈值上的边界点 ``z == z_thresh`` 亦丢弃）。

    参数
    ----
    points : list[Point3D]
        原始点集。
    z_thresh : float
        地面高度阈值。

    返回
    ----
    list[Point3D]
        滤地后的点集；``points`` 为空时返回 ``[]``。
    """
    if not points:
        return []
    if not _is_number(z_thresh):
        if _reject("filter_ground: z_thresh 必须为数值，实际为 %r" % (z_thresh,)):
            return []
    return [p for p in points if p.z > z_thresh]


def dbscan_cluster(
    points: list[Point3D], eps: float, min_samples: int
) -> list[Cluster]:
    """DBSCAN 聚类并取质心（``design/函数接口.md`` 1.2，文档定义）。

    使用官方 ``sklearn.cluster.DBSCAN``（3D 欧氏距离）。噪声点（label ``-1``）
    不进入任何簇。每个簇的 ``id`` 按出现顺序从 0 重新编号，``centroid`` 为簇内
    点坐标均值，``size`` 为簇内点数。

    参数
    ----
    points : list[Point3D]
        滤地后的点集。
    eps : float
        邻域半径（须 > 0）。
    min_samples : int
        核心点的最小样本数（含点自身，sklearn 语义；须为正整数）。

    返回
    ----
    list[Cluster]
        聚类结果；``points`` 为空时返回 ``[]``。
    """
    if not points:
        return []

    # --- 参数校验（dev 抛错 / onboard 告警返回 []）---
    if not _is_number(eps):
        if _reject("dbscan_cluster: eps 必须为数值，实际为 %r" % (eps,)):
            return []
    elif eps <= 0:
        if _reject("dbscan_cluster: eps 必须 > 0，实际为 %r" % (eps,)):
            return []

    if not isinstance(min_samples, int) or isinstance(min_samples, bool):
        if _reject(
            "dbscan_cluster: min_samples 必须为整数，实际为 %r" % (min_samples,)
        ):
            return []
    if min_samples <= 0:
        if _reject(
            "dbscan_cluster: min_samples 必须 > 0，实际为 %r" % (min_samples,)
        ):
            return []

    # --- 聚类 ---
    coords = np.array([[p.x, p.y, p.z] for p in points], dtype=float)
    labels = DBSCAN(eps=float(eps), min_samples=int(min_samples)).fit(coords).labels_

    # --- 按 label 分组，丢弃噪声（-1），组装 Cluster ---
    clusters: list[Cluster] = []
    for new_id, label in enumerate(sorted(set(int(v) for v in labels) - {-1})):
        indices = np.where(labels == label)[0]
        centroid = tuple(float(v) for v in coords[indices].mean(axis=0))
        clusters.append(
            Cluster(
                id=new_id,
                centroid=centroid,
                points=[points[i] for i in indices],
                size=int(indices.size),
            )
        )
    return clusters
