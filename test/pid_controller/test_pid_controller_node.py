"""模块 6 ROS 消息适配测试，不启动 ROS master。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

rospy = pytest.importorskip("rospy")
pytest.importorskip("mavros_msgs.msg")
pytest.importorskip("uav_tracking.msg")

from mavros_msgs.msg import PositionTarget
from uav_tracking.msg import VisualTrack as VisualTrackMsg, VisualTrackArray
from datatypes import PositionCommand

_NODE_PATH = Path(__file__).resolve().parents[2] / "catkin_ws/src/uav_tracking/scripts/pid_controller_node.py"
_SPEC = importlib.util.spec_from_file_location("pid_controller_node_under_test", str(_NODE_PATH))
node = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(node)


def test_tracks_from_msg():
    msg = VisualTrackArray()
    item = VisualTrackMsg()
    item.id = 7
    item.bbox = [1.0, 2.0, 3.0, 4.0]
    item.area_ratio = 0.25
    msg.tracks.append(item)
    tracks = node.tracks_from_msg(msg)
    assert tracks[0].id == 7
    assert tracks[0].bbox == (1.0, 2.0, 3.0, 4.0)
    assert tracks[0].area_ratio == pytest.approx(0.25)


def test_velocity_target_uses_body_offset_frame_and_mask():
    command = PositionCommand(9, 1479, velocity=(1.2, 0.0, -0.3), yaw_rate=-0.4)
    msg = node.command_to_position_target(command)
    assert msg.coordinate_frame == PositionTarget.FRAME_BODY_OFFSET_NED
    assert msg.type_mask == node.VELOCITY_TYPE_MASK
    assert msg.type_mask == 1479
    assert msg.velocity.x == pytest.approx(1.2)
    assert msg.velocity.y == pytest.approx(0.0)
    assert msg.velocity.z == pytest.approx(-0.3)
    assert msg.yaw_rate == pytest.approx(-0.4)
    assert not msg.type_mask & PositionTarget.IGNORE_VX
    assert not msg.type_mask & PositionTarget.IGNORE_VY
    assert not msg.type_mask & PositionTarget.IGNORE_VZ
    assert not msg.type_mask & PositionTarget.IGNORE_YAW_RATE


def test_interface_and_rate_are_intermediate_only():
    assert node.TOPIC_TRACKS == "/visual/tracks"
    assert node.TOPIC_OUTPUT == "/ctrl/visual"
    assert node.TOPIC_OUTPUT != "/mavros/setpoint_raw/local"
    from config import PID_RATE_HZ
    assert PID_RATE_HZ == 50.0


def test_lateral_velocity_is_always_enabled_and_zero():
    msg = node.command_to_position_target(
        PositionCommand(9, 1479, velocity=(1.0, 2.0, 0.3), yaw_rate=0.4)
    )
    assert msg.velocity.y == 0.0
    assert not msg.type_mask & PositionTarget.IGNORE_VY


def test_ros_float32_terminal_area_enters_terminal_branch(monkeypatch):
    """ROS float32 的数学值 0.35 仍必须触发末端零前进速度。"""
    clock = [10.0]
    published = []
    monkeypatch.setattr(rospy.Time, "now", lambda: rospy.Time.from_sec(clock[0]))
    monkeypatch.setattr(rospy, "get_param", lambda name, default: default)
    monkeypatch.setattr(rospy, "Publisher", lambda *a, **kw: SimpleNamespace(publish=published.append))
    monkeypatch.setattr(rospy, "Subscriber", lambda *a, **kw: None)
    monkeypatch.setattr(rospy, "Timer", lambda *a, **kw: None)
    instance = node.PIDControllerNode()

    msg = VisualTrackArray()
    item = VisualTrackMsg()
    item.id = 1
    item.bbox = [76.0, 40.0, 168.0, 160.0]
    item.area_ratio = 0.35
    assert float(item.area_ratio) == pytest.approx(0.3499999940395355)
    msg.tracks.append(item)

    instance.on_tracks(msg)
    instance.on_timer(None)
    target = published[-1]
    assert target.velocity.x == pytest.approx(0.0)
    assert target.coordinate_frame == PositionTarget.FRAME_BODY_OFFSET_NED
    assert target.type_mask == 1479


@pytest.mark.parametrize("send_empty", [False, True], ids=["upstream_silent", "empty_array"])
def test_valid_target_then_loss_stops(monkeypatch, send_empty):
    # Mock transport/time only: run the real callbacks and pure PID logic.
    clock = [10.0]
    published = []
    monkeypatch.setattr(rospy.Time, "now", lambda: rospy.Time.from_sec(clock[0]))
    monkeypatch.setattr(rospy, "get_param", lambda name, default: default)
    monkeypatch.setattr(rospy, "Publisher", lambda *a, **kw: SimpleNamespace(publish=published.append))
    monkeypatch.setattr(rospy, "Subscriber", lambda *a, **kw: None)
    monkeypatch.setattr(rospy, "Timer", lambda *a, **kw: None)
    instance = node.PIDControllerNode()
    msg = VisualTrackArray()
    item = VisualTrackMsg()
    item.id, item.bbox, item.area_ratio = 1, [400.0, 300.0, 20.0, 20.0], 0.01
    msg.tracks.append(item)
    instance.on_tracks(msg)
    instance.on_timer(None)
    target = published[-1]
    assert target.velocity.x > 0.0
    assert target.velocity.z < 0.0
    assert target.yaw_rate < 0.0

    clock[0] = 10.0 + instance.hold_timeout / 2.0
    if send_empty:
        instance.on_tracks(VisualTrackArray())
    instance.on_timer(None)
    assert instance._last_valid_time.to_sec() == 10.0
    assert published[-1] == target

    for elapsed in (instance.hold_timeout + 0.01, instance.hold_timeout + 1.0):
        clock[0] = 10.0 + elapsed
        if send_empty:
            instance.on_tracks(VisualTrackArray())
        instance.on_timer(None)
        stopped = published[-1]
        assert (stopped.velocity.x, stopped.velocity.y, stopped.velocity.z, stopped.yaw_rate) == (0.0, 0.0, 0.0, 0.0)
        assert stopped.coordinate_frame == PositionTarget.FRAME_BODY_OFFSET_NED
        assert stopped.type_mask == 1479
        assert instance._last_valid_time.to_sec() == 10.0
    assert bool(instance._tracks) is not send_empty
