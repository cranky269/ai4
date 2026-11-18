"""模块 6 纯 PID 算法测试，不依赖 ROS master。"""
import struct

import pytest

import pid_controller as pid
from datatypes import PositionCommand, VisualTrack


def _track(track_id=1, area=0.05, bbox=(320.0, 240.0, 20.0, 20.0)):
    return VisualTrack(track_id, bbox, area)


def _controllers():
    return (
        pid.PIDController(0.01, 0.0, 0.0, 2.0),
        pid.PIDController(0.01, 0.0, 0.0, 2.0),
        pid.PIDController(10.0, 0.0, 0.0, 5.0),
    )


def test_pid_compute_proportional_and_output_limit():
    controller = pid.PIDController(2.0, 0.0, 0.0, 1.0)
    assert controller.compute(1.0, 0.0, 0.1) == pytest.approx(1.0)


def test_pid_dt_nonpositive_does_not_integrate_or_raise():
    controller = pid.PIDController(1.0, 1.0, 1.0, 10.0)
    assert controller.compute(1.0, 0.0, 0.0) == pytest.approx(1.0)
    assert controller.integral == pytest.approx(0.0)
    assert controller.compute(1.0, 0.0, -1.0) == pytest.approx(1.0)


def test_select_track_area_then_id():
    selected = pid.select_track([_track(9, 0.2), _track(4, 0.8), _track(2, 0.8)])
    assert selected.id == 2


def test_far_target_has_positive_forward_speed():
    command = pid.command_from_track(_track(area=0.01), *_controllers(), dt=0.1, desired_area=0.1, min_area=0.001, impact_area_threshold=0.9)
    assert isinstance(command, PositionCommand)
    assert command.coordinate_frame == 9
    assert command.type_mask == 1479
    assert command.velocity_x > 0.0
    assert command.velocity_x <= 2.0


def test_desired_area_is_near_zero_speed():
    command = pid.command_from_track(_track(area=0.1), *_controllers(), dt=0.1, desired_area=0.1, min_area=0.001, impact_area_threshold=0.9)
    assert command.velocity_x == pytest.approx(0.0)


def test_near_target_is_not_above_forward_approach_limit():
    command = pid.command_from_track(_track(area=0.3), *_controllers(), dt=0.1, desired_area=0.1, min_area=0.001, forward_speed_limit=0.7, impact_area_threshold=0.9)
    assert command.velocity_x <= 0.7
    assert command.velocity_x <= 0.0


def test_min_area_protects_tiny_area():
    command = pid.command_from_track(_track(area=0.0), *_controllers(), dt=0.1, desired_area=0.1, min_area=0.05, impact_area_threshold=0.9)
    assert command.velocity_x == pytest.approx(0.5)


def test_impact_threshold_uses_terminal_speed():
    command = pid.command_from_track(_track(area=0.5), *_controllers(), dt=0.1, desired_area=0.1, impact_area_threshold=0.4, terminal_forward_speed=0.0)
    assert command.velocity_x == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("area", "terminal"),
    [
        (0.35, True),
        (struct.unpack("f", struct.pack("f", 0.35))[0], True),
        (0.3499, False),
        (0.3501, True),
    ],
    ids=["python_exact", "ros_float32", "below", "above"],
)
def test_impact_threshold_handles_float32_boundary_without_early_trigger(area, terminal):
    command = pid.command_from_track(
        _track(area=area),
        *_controllers(),
        dt=0.1,
        desired_area=0.1,
        impact_area_threshold=0.35,
        terminal_forward_speed=0.0,
    )
    if terminal:
        assert command.velocity_x == pytest.approx(0.0)
    else:
        assert command.velocity_x < 0.0


def test_image_right_and_below_have_negative_yaw_and_z():
    command = pid.command_from_track(_track(area=0.1, bbox=(400.0, 300.0, 20.0, 20.0)), *_controllers(), dt=0.1, desired_area=0.1, impact_area_threshold=0.9)
    assert command.yaw_rate < 0.0
    assert command.velocity_z < 0.0


def test_on_timer_short_loss_holds_and_long_loss_stops():
    last = PositionCommand(9, 1479, velocity=(1.0, 0.0, -0.1), yaw_rate=0.2)
    assert pid.on_timer([], 0.1, last, hold_timeout=0.2, dt=0.1, controllers=_controllers()) == last
    assert pid.on_timer([], 0.3, last, hold_timeout=0.2, dt=0.1, controllers=_controllers()) == pid.stop_setpoint()


@pytest.mark.parametrize("valid_age", [0.21, float("inf")])
def test_nonempty_stale_tracks_stop(valid_age):
    tracks = [_track(area=0.01, bbox=(400, 300, 20, 20))]
    last = PositionCommand(9, 1479, velocity=(1.0, 0.0, -0.1), yaw_rate=-0.2)
    assert pid.on_timer(tracks, valid_age, last, hold_timeout=0.2,
                        dt=0.1, controllers=_controllers()) == pid.stop_setpoint()
