"""模块 8：状态机（``/state_machine``）。

对应 ``design/函数接口.md`` 模块 8，含三个函数（签名照搬文档 8.1–8.3）：

- :func:`evaluate_visual_lock` —— 视觉锁定进入条件判定（单帧）；
- :func:`evaluate_radar_lost`  —— 雷达丢失判定（单帧）；
- :func:`transition`          —— 按第四章规则执行状态转移（含 N/M/K 计数）。

职责（文档）：订阅 ``/detections`` ``/visual/tracks`` ``/radar/tracks``，
发布 ``/state``（``std_msgs/String``）。三态：``RADAR_GUIDE`` / ``VISUAL_LOCK``
/ ``SEARCH/RTH``。

设计约束
--------
- **无发布时钟**：``/state`` 按变化发布，N/M/K 帧计数以数据消息到达为节拍
  （设计原则第 4 条、主文档第八章第 7 条）。
- **空即正常**：空数组到达（「空 = 没目标（业务正常）」）照常参与判定，
  不为错误（设计原则第 5 条）。
- **优先级**：``RADAR_GUIDE`` 下同时满足视觉锁定与雷达丢失时，**视觉锁定优先**。
- **计数原地更新**：``transition`` 只返回新状态，N/M/K 计数在传入的
  ``counters`` 字典上**原地**更新（键见 :data:`COUNTER_KEYS`）；状态切换时
  无关计数清零。
- **阶段化校验**：运行阶段由 ``config.ONBOARD`` 控制——``dev``（默认）非法输入
  抛 ``ValueError``；``onboard`` 告警并走安全默认分支（不崩溃）。

范围说明：本模块只提供纯逻辑（不依赖 ROS/``rospy``），节点壳（订阅 / 发布 /
切换日志）在集成阶段封装；输入的 ``DetectedTarget`` / ``VisualTrack`` /
``Track`` 均为**消息镜像 dataclass**（纯类型，字段与同名 ``.msg`` 一致），
由节点壳负责 ``.msg`` ↔ 纯 dataclass 拆装包（见 ``datatypes``）。
"""
from __future__ import annotations

import warnings

from config import STAGE, ONBOARD  # noqa: F401  (STAGE 供使用方展示运行阶段)
from datatypes import DetectedTarget, Track, VisualTrack

__all__ = [
    "STATE_RADAR_GUIDE",
    "STATE_VISUAL_LOCK",
    "STATE_SEARCH_RTH",
    "STATES",
    "COUNTER_KEYS",
    "N_VISUAL_LOCK",
    "M_VISUAL_LOST",
    "K_RADAR_LOST",
    "evaluate_visual_lock",
    "evaluate_radar_lost",
    "transition",
]

# --- 状态值（design/数据类型.md 三、状态枚举，与 /control_mux 的取值一致）---
STATE_RADAR_GUIDE = "RADAR_GUIDE"
STATE_VISUAL_LOCK = "VISUAL_LOCK"
STATE_SEARCH_RTH = "SEARCH/RTH"

#: 合法状态集合（``transition`` 输入边界）。
STATES = (STATE_RADAR_GUIDE, STATE_VISUAL_LOCK, STATE_SEARCH_RTH)

# --- N/M/K 连续帧阈值（design/函数接口.md 模块 8 第四章切换规则）---
#: 文档未给出数值，此处为默认值，测试/联调可按需覆盖（模块级常量）。
N_VISUAL_LOCK = 3  # 进入 VISUAL_LOCK：视觉锁定条件连续 N 帧保持
M_VISUAL_LOST = 3  # 退出 VISUAL_LOCK：连续 M 帧丢失目标 → RADAR_GUIDE
K_RADAR_LOST = 5   # 进入 SEARCH/RTH：RADAR_GUIDE 下连续 K 帧无雷达航迹

#: N/M/K 计数器键（``transition`` 在 ``counters`` 上原地更新这三项）。
COUNTER_KEYS = ("visual_lock", "visual_lost", "radar_lost")


def _reject(message: str) -> bool:
    """参数非法时的统一处理（同 ``cluster._reject`` / ``control_mux._reject``）。

    - 开发阶段（默认）：抛出 ``ValueError``；
    - 上机阶段（``UAV_STAGE=onboard``）：发出 ``RuntimeWarning`` 并返回 ``True``，
      由调用方走安全默认分支。

    返回 ``True`` 表示调用方应走默认分支；抛出异常时本函数不返回。
    """
    if ONBOARD:
        warnings.warn(
            "[state_machine] %s；上机阶段按默认值处理" % message,
            RuntimeWarning,
            stacklevel=2,
        )
        return True
    raise ValueError("[state_machine] " + message)


def _is_number(value: object) -> bool:
    """判断是否为实数（``bool`` 不算）。"""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _reset(counters: dict) -> None:
    """将 N/M/K 计数全部清零（状态切换后调用）。"""
    for key in COUNTER_KEYS:
        counters[key] = 0


def evaluate_visual_lock(
    detections: list[DetectedTarget],
    visual_tracks: list[VisualTrack],
    conf_thresh: float = 0.8,
    area_thresh: float = 0.02,
) -> bool:
    """视觉锁定进入条件判定（``design/函数接口.md`` 8.1，文档推导）。

    判定当前帧是否满足「置信度 > 阈值 **且** BBox 面积比 > 阈值」的视觉锁定
    进入条件（文档第四章）。采用**存在性判定**（单目标场景）：

    - 置信度条件：存在任一 ``detection`` 使 ``confidence > conf_thresh``；
    - 面积条件：存在任一 ``visual_track`` 使 ``area_ratio > area_thresh``；
    - 二者同时成立才返回 ``True``。

    阈值比较均为**严格大于**（等于阈值不满足）。

    参数
    ----
    detections : list[DetectedTarget]
        当前帧检测结果（``/detections``，置信度来源）。
    visual_tracks : list[VisualTrack]
        当前帧视觉航迹（``/visual/tracks``，面积比来源）。
    conf_thresh : float
        置信度阈值，默认 ``0.8``（对应文档「> 80%」）。
    area_thresh : float
        面积比阈值，默认 ``0.02``（对应文档「> 2% 画面」）。

    返回
    ----
    bool
        是否满足视觉锁定进入条件；输入为空（无目标）时返回 ``False``。
    """
    if not isinstance(detections, list):
        if _reject("evaluate_visual_lock: detections 必须为 list，实际为 %r" % (detections,)):
            return False
    if not isinstance(visual_tracks, list):
        if _reject("evaluate_visual_lock: visual_tracks 必须为 list，实际为 %r" % (visual_tracks,)):
            return False
    if not _is_number(conf_thresh):
        if _reject("evaluate_visual_lock: conf_thresh 必须为数值，实际为 %r" % (conf_thresh,)):
            return False
    if not _is_number(area_thresh):
        if _reject("evaluate_visual_lock: area_thresh 必须为数值，实际为 %r" % (area_thresh,)):
            return False

    conf_ok = any(d.confidence > conf_thresh for d in detections)
    area_ok = any(t.area_ratio > area_thresh for t in visual_tracks)
    return conf_ok and area_ok


def evaluate_radar_lost(radar_tracks: list[Track]) -> bool:
    """雷达丢失判定（``design/函数接口.md`` 8.2，文档推导）。

    判定当前帧雷达是否无有效航迹（用于进入 / 退出 ``SEARCH/RTH``）。

    参数
    ----
    radar_tracks : list[Track]
        当前帧雷达航迹（``/radar/tracks``）。

    返回
    ----
    bool
        ``True`` 表示雷达无有效航迹（丢失）；``radar_tracks`` 为空即丢失。
    """
    if not isinstance(radar_tracks, list):
        if _reject("evaluate_radar_lost: radar_tracks 必须为 list，实际为 %r" % (radar_tracks,)):
            return True  # onboard：无法判断，按丢失（安全）
    return len(radar_tracks) == 0


def transition(
    state: str,
    visual_lock: bool,
    radar_lost: bool,
    counters: dict,
) -> str:
    """按第四章状态机规则执行状态转移（``design/函数接口.md`` 8.3，文档推导）。

    切换规则（文档第四章）：

    - 进入 ``VISUAL_LOCK``：``RADAR_GUIDE`` 下视觉锁定条件连续 N 帧保持；
    - 退出 ``VISUAL_LOCK``：连续 M 帧丢失目标 → ``RADAR_GUIDE``；
    - 进入 ``SEARCH/RTH``：``RADAR_GUIDE`` 下连续 K 帧无雷达航迹；
    - 退出 ``SEARCH/RTH``：雷达恢复航迹 → ``RADAR_GUIDE``。

    计数规则：本函数在传入的 ``counters``（键见 :data:`COUNTER_KEYS`）
    上**原地**累加 / 清零，只返回新状态：

    - ``RADAR_GUIDE``：``visual_lock`` 与 ``radar_lost`` **各自独立**连续计数；
      达阈值即切换，且 **视觉锁定优先于雷达丢失**（二者同时达阈时进
      ``VISUAL_LOCK``）。
    - ``VISUAL_LOCK``：``visual_lock`` 满足则视觉丢失计数清零并保持；
      连续 M 帧不满足 → ``RADAR_GUIDE``。
    - ``SEARCH/RTH``：``radar_lost`` 为假（雷达恢复）即 → ``RADAR_GUIDE``。

    任一状态切换发生时，N/M/K 计数全部清零（新一轮计数从 0 开始）。

    参数
    ----
    state : str
        当前状态（:data:`STATES` 之一）。
    visual_lock : bool
        本帧视觉锁定条件是否满足（:func:`evaluate_visual_lock` 结果）。
    radar_lost : bool
        本帧雷达是否丢失（:func:`evaluate_radar_lost` 结果）。
    counters : dict
        N/M/K 连续帧计数器（原地更新）。

    返回
    ----
    str
        新状态。
    """
    if not isinstance(state, str) or state not in STATES:
        if _reject("transition: state 必须为 %r 之一，实际为 %r" % (STATES, state)):
            return state  # onboard：无法解释，保持原状态、不转移
    if not isinstance(visual_lock, bool):
        if _reject("transition: visual_lock 必须为 bool，实际为 %r" % (visual_lock,)):
            visual_lock = bool(visual_lock)
    if not isinstance(radar_lost, bool):
        if _reject("transition: radar_lost 必须为 bool，实际为 %r" % (radar_lost,)):
            radar_lost = bool(radar_lost)
    if not isinstance(counters, dict):
        if _reject("transition: counters 必须为 dict，实际为 %r" % (counters,)):
            return state  # onboard：无计数容器，保持原状态、不转移

    if state == STATE_RADAR_GUIDE:
        counters["visual_lock"] = (counters.get("visual_lock", 0) + 1) if visual_lock else 0
        counters["radar_lost"] = (counters.get("radar_lost", 0) + 1) if radar_lost else 0
        if counters["visual_lock"] >= N_VISUAL_LOCK:  # 视觉锁定优先
            _reset(counters)
            return STATE_VISUAL_LOCK
        if counters["radar_lost"] >= K_RADAR_LOST:
            _reset(counters)
            return STATE_SEARCH_RTH
        return STATE_RADAR_GUIDE

    if state == STATE_VISUAL_LOCK:
        counters["visual_lost"] = 0 if visual_lock else (counters.get("visual_lost", 0) + 1)
        if counters["visual_lost"] >= M_VISUAL_LOST:
            _reset(counters)
            return STATE_RADAR_GUIDE
        return STATE_VISUAL_LOCK

    # SEARCH/RTH：雷达恢复航迹即回 RADAR_GUIDE（文档第四章）
    if not radar_lost:
        _reset(counters)
        return STATE_RADAR_GUIDE
    return STATE_SEARCH_RTH
