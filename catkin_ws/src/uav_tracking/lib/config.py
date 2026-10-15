"""运行阶段小配置（全项目共用，代码书写原则第 1 条）。

代码分两个运行阶段，由环境变量 ``UAV_STAGE`` 选择：

==========  ==============  ==================================================
取值         阶段            非法输入处理
==========  ==============  ==================================================
``dev``      开发（默认）    直接抛 ``ValueError``，问题尽早暴露、停止运行
``onboard``  上机            发出 ``RuntimeWarning``，由各模块给默认值兜底
==========  ==============  ==================================================

用法：各模块统一 ``from config import STAGE, ONBOARD``，并在自身 ``_reject``
中按 ``ONBOARD`` 决定「抛错」还是「告警 + 默认值」。

.. note::
   阶段在 **import 时** 读取一次；测试中可 ``monkeypatch.setattr(模块, "ONBOARD", ...)``
   模拟，无需改环境变量。若运行中修改 ``UAV_STAGE``，因本模块已被缓存，需
   ``importlib.reload(config)`` 后再 ``reload`` 使用方模块方可生效。
"""
from __future__ import annotations

import os

__all__ = [
    "STAGE",
    "ONBOARD",
    "TRACKER_RATE_HZ",
    "TRACKER_ASSOCIATION_THRESHOLD",
    "TRACKER_MAX_MISSED_FRAMES",
    "TRACKER_PROCESS_NOISE",
    "TRACKER_MEASUREMENT_NOISE",
    "TRACKER_INITIAL_COVARIANCE",
    "NAVIGATOR_RATE_HZ",
    "NAVIGATOR_HOLD_TIMEOUT",
    "PID_RATE_HZ",
    "PID_HOLD_TIMEOUT",
    "PID_IMAGE_WIDTH",
    "PID_IMAGE_HEIGHT",
    "PID_YAW_KP",
    "PID_YAW_KI",
    "PID_YAW_KD",
    "PID_YAW_MAX_OUT",
    "PID_VERTICAL_KP",
    "PID_VERTICAL_KI",
    "PID_VERTICAL_KD",
    "PID_VERTICAL_MAX_OUT",
    "PID_DEPTH_KP",
    "PID_DEPTH_KI",
    "PID_DEPTH_KD",
    "PID_DEPTH_MAX_OUT",
    "PID_INTEGRAL_LIMIT",
    "PID_MIN_AREA",
    "PID_DESIRED_AREA",
    "PID_FORWARD_SPEED_LIMIT",
    "PID_IMPACT_AREA_THRESHOLD",
    "PID_TERMINAL_FORWARD_SPEED",
]

#: 运行阶段名（小写）：``"dev"`` 或 ``"onboard"``。
STAGE = os.environ.get("UAV_STAGE", "dev").strip().lower()

#: 是否处于上机阶段（``True`` 时非法输入告警并兜底，不抛异常）。
ONBOARD = STAGE == "onboard"

# 模块 2 参数集中配置。tracker 为事件驱动算法；该频率仅用于首帧默认 dt。
TRACKER_RATE_HZ = 30.0
TRACKER_ASSOCIATION_THRESHOLD = 2.0
TRACKER_MAX_MISSED_FRAMES = 5
TRACKER_PROCESS_NOISE = 1.0
TRACKER_MEASUREMENT_NOISE = 0.5
TRACKER_INITIAL_COVARIANCE = 1.0

# 模块 3：导航节点固定 50 Hz 发布；保持窗口由时钟层统一判断。
NAVIGATOR_RATE_HZ = 50.0
NAVIGATOR_HOLD_TIMEOUT = 0.5

# 模块 6：视觉 PID。节点固定 50 Hz 发布，速度指令通过中间话题交给 mux。
PID_RATE_HZ = 50.0
PID_HOLD_TIMEOUT = 0.2
PID_IMAGE_WIDTH = 640.0
PID_IMAGE_HEIGHT = 480.0

# 三通道增益和限幅。具体飞行调参在部署配置中覆盖，不在节点内硬编码。
PID_YAW_KP = 0.01
PID_YAW_KI = 0.0
PID_YAW_KD = 0.0
PID_YAW_MAX_OUT = 1.0
PID_VERTICAL_KP = 0.01
PID_VERTICAL_KI = 0.0
PID_VERTICAL_KD = 0.0
PID_VERTICAL_MAX_OUT = 1.0
PID_DEPTH_KP = 1.0
PID_DEPTH_KI = 0.0
PID_DEPTH_KD = 0.0
PID_DEPTH_MAX_OUT = 2.0
PID_INTEGRAL_LIMIT = 10.0
PID_MIN_AREA = 1.0e-4
PID_DESIRED_AREA = 0.10
PID_FORWARD_SPEED_LIMIT = 2.0
PID_IMPACT_AREA_THRESHOLD = 0.35
PID_TERMINAL_FORWARD_SPEED = 0.0
