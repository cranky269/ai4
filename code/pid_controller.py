"""模块 6：纯视觉 PID 控制逻辑。

本模块不导入 rospy 或 ROS 消息。它只处理 ``VisualTrack``、数值 PID 和
无 ROS 的 ``PositionCommand``；``PositionTarget`` 构造及 ROS 定时发布在节点层。

ROS 侧速度语义遵循项目约定：x 前方、y 左方、z 上方。MAVROS
``setpoint_raw`` 插件负责后续 FLU/FRD 转换，算法层不重复转换。
"""
from __future__ import annotations

import math
import warnings
from typing import List, Optional

from config import (
    ONBOARD,
    PID_DEPTH_KD,
    PID_DEPTH_KI,
    PID_DEPTH_KP,
    PID_DEPTH_MAX_OUT,
    PID_DESIRED_AREA,
    PID_FORWARD_SPEED_LIMIT,
    PID_HOLD_TIMEOUT,
    PID_IMAGE_HEIGHT,
    PID_IMAGE_WIDTH,
    PID_IMPACT_AREA_THRESHOLD,
    PID_INTEGRAL_LIMIT,
    PID_MIN_AREA,
    PID_RATE_HZ,
    PID_TERMINAL_FORWARD_SPEED,
    PID_VERTICAL_KD,
    PID_VERTICAL_KI,
    PID_VERTICAL_KP,
    PID_VERTICAL_MAX_OUT,
    PID_YAW_KD,
    PID_YAW_KI,
    PID_YAW_KP,
    PID_YAW_MAX_OUT,
)
from datatypes import PositionCommand, VisualTrack

__all__ = [
    "PositionCommand",
    "PIDController",
    "select_track",
    "command_from_track",
    "stop_setpoint",
    "on_timer",
]


BODY_OFFSET_NED_FRAME = 9
VELOCITY_TYPE_MASK = 1479


def _reject(message: str) -> bool:
    if ONBOARD:
        warnings.warn("[pid] %s；上机阶段按安全默认值处理" % message,
                      RuntimeWarning, stacklevel=2)
        return True
    raise ValueError("[pid] " + message)


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


class PIDController:
    """带积分限幅和输出限幅的离散 PID。"""

    def __init__(self, kp, ki, kd, max_out, integral_limit=PID_INTEGRAL_LIMIT):
        values = (kp, ki, kd, max_out, integral_limit)
        if not all(_finite(value) for value in values) or float(max_out) < 0.0 or float(integral_limit) < 0.0:
            if _reject("PID 参数必须为有限值，且 max_out/integral_limit >= 0"):
                kp, ki, kd, max_out, integral_limit = 0.0, 0.0, 0.0, 0.0, 0.0
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.max_out = float(max_out)
        self.integral_limit = float(integral_limit)
        self.integral = 0.0
        self.previous_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        if not (_finite(setpoint) and _finite(measurement)):
            if _reject("setpoint 和 measurement 必须为有限数值"):
                return 0.0
        if not _finite(dt) or float(dt) <= 0.0:
            # dt 无效时返回当前比例项，不推进积分/微分，避免 NaN 和积分爆炸。
            error = float(setpoint) - float(measurement)
            return max(-self.max_out, min(self.max_out, self.kp * error))

        error = float(setpoint) - float(measurement)
        dt = float(dt)
        candidate_integral = self.integral + error * dt
        candidate_integral = max(-self.integral_limit, min(self.integral_limit, candidate_integral))
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        unclamped = self.kp * error + self.ki * candidate_integral + self.kd * derivative
        output = max(-self.max_out, min(self.max_out, unclamped))

        # 简单抗积分饱和：输出饱和且误差继续推动饱和方向时不积累。
        drives_high = unclamped > self.max_out and error > 0.0
        drives_low = unclamped < -self.max_out and error < 0.0
        if not (drives_high or drives_low):
            self.integral = candidate_integral
        self.previous_error = error
        return output


def _valid_track(track: object) -> bool:
    return isinstance(track, VisualTrack) and isinstance(track.id, int) and _finite(track.area_ratio) and float(track.area_ratio) >= 0.0


def select_track(tracks: List[VisualTrack]) -> Optional[VisualTrack]:
    """选择面积最大的目标；面积相同取 id 最小。"""
    if not isinstance(tracks, list):
        if _reject("tracks 必须为 list"):
            return None
    valid = []
    for index, track in enumerate(tracks):
        if not _valid_track(track):
            if _reject("tracks[%d] 不是有效 VisualTrack" % index):
                continue
        valid.append(track)
    return min(valid, key=lambda track: (-float(track.area_ratio), int(track.id))) if valid else None


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def command_from_track(
    track: VisualTrack,
    yaw_pid: PIDController,
    vertical_pid: PIDController,
    depth_pid: PIDController,
    dt: float,
    image_width: float = PID_IMAGE_WIDTH,
    image_height: float = PID_IMAGE_HEIGHT,
    desired_area: float = PID_DESIRED_AREA,
    min_area: float = PID_MIN_AREA,
    forward_speed_limit: float = PID_FORWARD_SPEED_LIMIT,
    impact_area_threshold: float = PID_IMPACT_AREA_THRESHOLD,
    terminal_forward_speed: float = PID_TERMINAL_FORWARD_SPEED,
) -> PositionCommand:
    """从一个视觉航迹计算 ROS 侧速度命令。"""
    if not _valid_track(track):
        if _reject("track 不是有效 VisualTrack"):
            return stop_setpoint()
    numeric = (image_width, image_height, desired_area, min_area, forward_speed_limit, impact_area_threshold, terminal_forward_speed)
    if not all(_finite(value) for value in numeric) or float(image_width) <= 0.0 or float(image_height) <= 0.0 or float(desired_area) <= 0.0 or float(min_area) <= 0.0 or float(forward_speed_limit) < 0.0:
        if _reject("视觉/PID 配置必须为有限且有效数值"):
            return stop_setpoint()

    x, y, w, h = (float(value) for value in track.bbox)
    center_x = x + w / 2.0
    center_y = y + h / 2.0
    image_center_x = float(image_width) / 2.0
    image_center_y = float(image_height) / 2.0

    # 正误差表示目标在图像右侧/下方；compute(0, error) 产生负 yaw/z。
    yaw_rate = yaw_pid.compute(0.0, center_x - image_center_x, dt)
    velocity_z = vertical_pid.compute(0.0, center_y - image_center_y, dt)

    safe_area = max(float(track.area_ratio), float(min_area))
    # 面积小于期望值时，期望值 - 实际值为正，明确产生前进速度。
    depth_output = depth_pid.compute(float(desired_area), safe_area, dt)
    velocity_x = _clamp(depth_output, -float(depth_pid.max_out), float(forward_speed_limit))
    area_ratio = float(track.area_ratio)
    threshold = float(impact_area_threshold)
    # VisualTrack.msg 使用 float32；数学值 0.35 经 ROS 序列化后通常为
    # 0.3499999940395355。边界值仍应进入末端分支，但 0.3499 等真实低于
    # 阈值的输入不能被提前吸收，因此只使用明确的 float32 量级绝对容差。
    if area_ratio > threshold or math.isclose(
        area_ratio, threshold, rel_tol=0.0, abs_tol=1e-7
    ):
        velocity_x = _clamp(float(terminal_forward_speed), -float(depth_pid.max_out), float(forward_speed_limit))

    return PositionCommand(
        coordinate_frame=BODY_OFFSET_NED_FRAME,
        type_mask=VELOCITY_TYPE_MASK,
        velocity=(velocity_x, 0.0, velocity_z),
        yaw_rate=yaw_rate,
    )


def stop_setpoint() -> PositionCommand:
    """无有效目标时的正式公共零速度指令。"""
    return PositionCommand(
        coordinate_frame=BODY_OFFSET_NED_FRAME,
        type_mask=VELOCITY_TYPE_MASK,
        velocity=(0.0, 0.0, 0.0),
        yaw_rate=0.0,
    )


def _default_controllers():
    return (
        PIDController(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_YAW_MAX_OUT),
        PIDController(PID_VERTICAL_KP, PID_VERTICAL_KI, PID_VERTICAL_KD, PID_VERTICAL_MAX_OUT),
        PIDController(PID_DEPTH_KP, PID_DEPTH_KI, PID_DEPTH_KD, PID_DEPTH_MAX_OUT),
    )


def on_timer(
    tracks: List[VisualTrack],
    valid_age: float,
    last_cmd: Optional[PositionCommand],
    hold_timeout: float = PID_HOLD_TIMEOUT,
    dt: float = 1.0 / PID_RATE_HZ,
    controllers=None,
    **kwargs,
) -> PositionCommand:
    """时钟层三选一：新鲜控制量、短时保持、长时零速。

    valid_age 是距上次非空输入的时间；非空缓存不能绕过断流超时。
    """
    if not _finite(valid_age) and float(valid_age) != float("inf"):
        if _reject("valid_age 必须为非负有限数值或 inf"):
            valid_age = float("inf")
    if _finite(valid_age) and float(valid_age) < 0.0:
        if _reject("valid_age 必须 >= 0"):
            valid_age = float("inf")
    if not _finite(hold_timeout) or float(hold_timeout) < 0.0:
        if _reject("hold_timeout 必须 >= 0"):
            hold_timeout = PID_HOLD_TIMEOUT
    track = select_track(tracks)
    if track is not None and float(valid_age) <= float(hold_timeout):
        if controllers is None:
            controllers = _default_controllers()
        yaw_pid, vertical_pid, depth_pid = controllers
        return command_from_track(track, yaw_pid, vertical_pid, depth_pid, dt, **kwargs)
    if float(valid_age) <= float(hold_timeout) and last_cmd is not None:
        return last_cmd
    return stop_setpoint()
