"""Regression checks for the validated latest-frame and expiry behavior."""
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rospy')
pytest.importorskip('uav_tracking.msg')
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'catkin_ws/src/uav_tracking/scripts'))
import detector_node as module
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


@pytest.fixture
def node(monkeypatch):
    obj = object.__new__(module.DetectorNode)
    obj._condition = threading.Condition()
    obj._stopping = False
    obj._pending = None
    obj._last_clock = None
    obj._last_stamp = None
    obj._last_unstamped_seq = None
    obj._epoch = 0
    obj._sim_time = True
    obj._timing_enabled = False
    obj.bridge = CvBridge()
    obj.outputs = []
    obj.debug_outputs = []
    obj.errors = []
    obj.pub = SimpleNamespace(publish=obj.outputs.append)
    obj.pub_debug = SimpleNamespace(publish=obj.debug_outputs.append)
    obj.detector = SimpleNamespace(detect=lambda image: [])
    obj.clock = 100.10
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time.from_sec(obj.clock)))
    monkeypatch.setattr(module.rospy, 'logwarn_throttle', lambda *args: None)
    monkeypatch.setattr(module.rospy, 'logerr_throttle', lambda *args: obj.errors.append(args))
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(module, 'image_msg_to_array', lambda *args: np.zeros((8, 8, 3), np.uint8))
    return obj


def image(stamp):
    msg = Image()
    msg.header.stamp = module.rospy.Time.from_sec(stamp)
    msg.header.frame_id = 'camera_optical'
    return msg


def process(node):
    node._process_image(image(100), time.monotonic(), .1, module.rospy.Time.from_sec(100.05), 0)


def test_pending_frame_is_replaced_and_duplicates_do_not_overwrite(node):
    node.on_image(image(100))
    newest = image(100.05)
    node.on_image(newest)
    node.on_image(image(100))
    assert node._pending[0] is newest


def test_expired_input_is_rejected(node):
    node.on_image(image(98))
    assert node._pending is None


def test_empty_success_publishes_empty_and_preserves_debug_header(node):
    process(node)
    assert len(node.outputs) == 1 and not node.outputs[0].targets
    assert node.debug_outputs[0].header.stamp == module.rospy.Time.from_sec(100)
    assert node.debug_outputs[0].header.frame_id == 'camera_optical'


def test_inference_error_is_logged_without_false_empty(node):
    def fail(_image):
        raise RuntimeError('test inference error')
    node.detector.detect = fail
    process(node)
    assert node.errors and not node.outputs and not node.debug_outputs


@pytest.mark.parametrize('end_clock', [102., 99., 100.05])
def test_expiry_rewind_and_pause_suppress_old_results(node, end_clock):
    def finish(_image):
        node.clock = end_clock
        return []
    node.detector.detect = finish
    if end_clock == 100.05:
        node.clock = end_clock
    process(node)
    assert not node.outputs and not node.debug_outputs


def test_idle_worker_stops_and_unregisters_subscription(node):
    done = []
    node._subscriber = SimpleNamespace(unregister=lambda: done.append(True))
    node._worker = threading.Thread(target=node._run_worker, daemon=True)
    node._worker.start()
    node._stop_worker()
    assert done == [True] and not node._worker.is_alive()
