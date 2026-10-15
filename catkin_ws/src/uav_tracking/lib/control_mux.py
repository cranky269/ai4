"""模块 7：控制仲裁（``/control_mux``）。

对应 ``design/函数接口.md`` 模块 7。职责（文档）：

    订阅 ``/state``、``/ctrl/radar``、``/ctrl/visual``，按状态只放行一路，
    转发到 ``/mavros/setpoint_raw/local``，其余丢弃——唯一向飞控发指令的出口。

设计约束（见 ``design/函数接口.md`` 模块 7 与「决策端设计原则」）：

- **只转发不生成**：薄节点，收到哪路原样转哪路，不生成、不修改指令；
- **事件转发、无定时器**：收到即转；指令流连续性由上游 ``/navigator`` /
  ``/pid_controller`` 的固定时钟（20-50Hz）保证；
- **每路只留最新一条**（``queue_size=1``），避免积压旧指令；
- **不做 ``/state`` 超时检测**（方案 B）：状态机挂死时由 ``/pid_controller`` 的
  ``stop_setpoint`` 与 PX4 OFFBOARD failsafe 两层兜底。

本模块只提供**纯逻辑**（不依赖 ROS/``rospy``），便于单元测试；节点壳
（订阅 / 发布 / 日志）在集成阶段封装。mux 为纯转发节点，指令
``mavros_msgs/PositionTarget`` 在此按**不透明载荷**处理（不生成、不解释指令内容），
**不涉及** ``PositionCommand``——后者仅存在于 ``/navigator`` / ``/pid_controller``
的纯逻辑侧（见 ``design/文档修改历史.md`` 1.1）。

阶段化校验（代码书写原则第 1 条）：由 ``config.ONBOARD`` 控制——``dev``
（默认）非法输入抛 ``ValueError``；``onboard`` 告警并丢弃 / 忽略。
"""
from __future__ import annotations

import logging
import warnings
from typing import Any, Optional

from config import STAGE, ONBOARD  # noqa: F401  (STAGE 供使用方展示运行阶段)

__all__ = [
    "STATE_RADAR_GUIDE",
    "STATE_VISUAL_LOCK",
    "CHANNEL_RADAR",
    "CHANNEL_VISUAL",
    "select_channel",
    "ControlMux",
]

_LOG = logging.getLogger(__name__)

# --- 状态值（design/数据类型.md 三、状态枚举）---
STATE_RADAR_GUIDE = "RADAR_GUIDE"
STATE_VISUAL_LOCK = "VISUAL_LOCK"

# --- 仲裁通道（话题名，design/函数接口.md 模块 7）---
CHANNEL_RADAR = "/ctrl/radar"
CHANNEL_VISUAL = "/ctrl/visual"

#: 合法通道集合（mux 输入边界）。
CHANNELS = (CHANNEL_RADAR, CHANNEL_VISUAL)


def _reject(message: str) -> bool:
    """参数非法时的统一处理（同 ``cluster._reject``）。

    - 开发阶段（默认）：抛 ``ValueError``；
    - 上机阶段（``UAV_STAGE=onboard``）：发 ``RuntimeWarning`` 并返回 ``True``，
      由调用方走默认分支。

    返回 ``True`` 表示调用方应走默认分支；抛异常时本函数不返回。
    """
    if ONBOARD:
        warnings.warn(
            "[control_mux] %s；上机阶段按默认值处理" % message,
            RuntimeWarning,
            stacklevel=2,
        )
        return True
    raise ValueError("[control_mux] " + message)


def select_channel(state: str) -> str:
    """按仲裁规则选择放行通道（``design/函数接口.md`` 7.1，文档推导）。

    规则（文档 3.7 仲裁表）：

    ==========================  ====================
    ``/state`` 值                放行通道
    ==========================  ====================
    ``RADAR_GUIDE``             ``/ctrl/radar``
    ``VISUAL_LOCK``             ``/ctrl/visual``
    其他值（含 ``SEARCH/RTH``）   ``/ctrl/radar``（回退）
    ==========================  ====================

    参数
    ----
    state : str
        最近一次收到的状态值。

    返回
    ----
    str
        放行通道：``"/ctrl/radar"`` 或 ``"/ctrl/visual"``。
    """
    if not isinstance(state, str):
        if _reject("select_channel: state 必须为 str，实际为 %r" % (state,)):
            return CHANNEL_RADAR  # onboard：回退雷达
    if state == STATE_VISUAL_LOCK:
        return CHANNEL_VISUAL
    return CHANNEL_RADAR  # RADAR_GUIDE / SEARCH/RTH / 其他值 → 回退雷达


class ControlMux:
    """控制仲裁纯逻辑（事件转发）。

    持有「最近一次 ``/state``」与「每路最新一条指令缓存」，收到指令时按当前状态
    决定放行（返回该指令）或丢弃（返回 ``None``）。对应 ``/control_mux`` 节点的
    事件层逻辑。

    属性
    ----
    state : str
        最近一次收到的状态值（初始 ``RADAR_GUIDE``）。
    channel : str
        当前放行通道（``select_channel(state)`` 的结果）。
    """

    def __init__(self, initial_state: str = STATE_RADAR_GUIDE) -> None:
        if not isinstance(initial_state, str):
            if _reject(
                "ControlMux: initial_state 必须为 str，实际为 %r" % (initial_state,)
            ):
                initial_state = STATE_RADAR_GUIDE  # onboard：默认雷达态
        self._state = initial_state
        self._latest = {CHANNEL_RADAR: None, CHANNEL_VISUAL: None}

    @property
    def state(self) -> str:
        """最近一次收到的状态值。"""
        return self._state

    @property
    def channel(self) -> str:
        """当前放行通道。"""
        return select_channel(self._state)

    def latest(self, channel: str) -> Optional[Any]:
        """返回某通道缓存的最新指令（未收到过则为 ``None``）。

        即 ``queue_size=1`` 语义：每路只保留最新一条。未知通道在 ``onboard``
        阶段告警并返回 ``None``。
        """
        if channel not in CHANNELS:
            if _reject("latest: 未知通道 %r" % (channel,)):
                return None
        return self._latest[channel]

    def on_state(self, state: str) -> None:
        """事件层回调：收到 ``/state`` 后更新内部状态。

        状态**变化**时打印一行 ``[mux] state -> <新状态>``（设计 3.7 第 5 条日志）。
        非法类型在 ``onboard`` 阶段被忽略（保持原状态，避免误切换通道）。
        """
        if not isinstance(state, str):
            if _reject("on_state: state 必须为 str，实际为 %r" % (state,)):
                return
        if state == self._state:
            return
        self._state = state
        _LOG.info("[mux] state -> %s", state)

    def on_command(self, channel: str, command: Any) -> Optional[Any]:
        """事件层回调：收到某路指令后转发或丢弃。

        - 先更新该通道最新指令缓存（无论是否放行，均只留最新一条）；
        - 若 ``channel`` 即当前放行通道，**原样返回** ``command``（转发）；
          否则返回 ``None``（丢弃）。

        未知通道在 ``onboard`` 阶段告警并丢弃。
        """
        if channel not in CHANNELS:
            if _reject("on_command: 未知通道 %r" % (channel,)):
                return None
        self._latest[channel] = command
        if channel == self.channel:
            return command
        return None
