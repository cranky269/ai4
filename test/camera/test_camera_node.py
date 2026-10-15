"""模块 4 相机驱动（``/camera_node``）单元测试。

覆盖 ``scripts/camera_node.py`` 中**不依赖 ROS master** 的部分：

- **契约常量**：``IMAGE_TOPIC`` / ``SUPPORTED_SOURCES`` / 默认值 / 队列长度；
- **``FrameSource``**：``image`` 源的打开与读取、返回值是**独立副本**、
  缺失文件抛 ``RuntimeError``、``release`` 幂等、``loop`` 归一化为 ``bool``；
- **``CameraNode`` 三个回调**（用最小替身当 ``self``，不构造真节点）：
  - ``on_image``（gazebo 中转）：仅在 ``frame_id`` 为空时回填；
  - ``on_timer``（主动源）：**无帧时不发布**（不产生空消息/不给下游断拍假象）；
  - ``_publish``：``encoding=bgr8``（与 ``/detector`` 的约定）+ ``frame_id`` + 时间戳。

运行前提：已 ``source /opt/ros/noetic/setup.bash``（需要 ``rospy`` 与 ``cv_bridge``）；
否则本文件整体跳过，不影响纯逻辑层测试。
"""
import os
import sys

import numpy as np
import pytest

# 将 catkin 包 scripts/ 加入 import 路径，使 camera_node 可被 import。
_SCRIPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "catkin_ws", "src", "uav_tracking", "scripts",
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# 依赖 ROS（rospy + cv_bridge）；缺失则本文件整体跳过
pytest.importorskip("rospy")
pytest.importorskip("cv_bridge")

import rospy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import camera_node as cn  # noqa: E402


# ---------------------------------------------------------------------------
# 替身：不构造真节点（``CameraNode.__init__`` 需要 ROS master）
# ---------------------------------------------------------------------------
class _DummyPub:
    """最小 Publisher 替身：只记录被 publish 的消息。"""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _NodeStub:
    """给 ``CameraNode`` 的方法当 ``self`` 用的最小替身。

    自带 ``_publish`` 记录器：``on_timer`` 内部走 ``self._publish(...)``，
    替身必须自己提供，否则无法验证"有没有发布"。
    """

    def __init__(self, frame_id="camera_optical_frame"):
        self.frame_id = frame_id
        self.pub = _DummyPub()
        self.bridge = CvBridge()
        self.source = "video"
        self.src = None
        self.published = []          # on_timer 是否发布了帧

    def _publish(self, frame):
        self.published.append(frame)


class _SrcStub:
    """``FrameSource`` 替身：``read()`` 恒返回指定帧。"""

    def __init__(self, frame):
        self._frame = frame

    def read(self):
        return self._frame


def _write_png(path, width=32, height=24, value=200):
    """写一张纯色 PNG，返回其 ``(h, w, 3)`` 数组。"""
    import cv2

    img = np.full((height, width, 3), value, np.uint8)
    assert cv2.imwrite(str(path), img)
    return img


# ---------------------------------------------------------------------------
# 契约常量
# ---------------------------------------------------------------------------
def test_image_topic_is_the_contract():
    """发布话题必须是 /detector 订阅的那个名字，不可改名。"""
    assert cn.IMAGE_TOPIC == "/camera/image_raw"


def test_supported_sources_are_the_four_documented_ones():
    assert set(cn.SUPPORTED_SOURCES) == {"usb", "video", "image", "gazebo"}


def test_defaults_follow_code_style_rule_6():
    """未指定参数做成模块级常量（代码书写原则第 6 条）。"""
    assert cn.DEFAULT_SOURCE in cn.SUPPORTED_SOURCES
    assert (cn.DEFAULT_WIDTH, cn.DEFAULT_HEIGHT) == (640, 480)
    assert cn.DEFAULT_FPS > 0
    assert cn.DEFAULT_DEVICE_ID == 0


def test_queue_size_is_one_to_keep_only_latest_frame():
    """只保最新帧：防止仿真帧率高于下游处理速度时延迟累积。"""
    assert cn.QUEUE_SIZE == 1


# ---------------------------------------------------------------------------
# FrameSource
# ---------------------------------------------------------------------------
def test_framesource_loop_is_normalized_to_bool():
    assert cn.FrameSource("video", path="x", loop=0).loop is False
    assert cn.FrameSource("video", path="x", loop=1).loop is True
    assert cn.FrameSource("video", path="x", loop=None).loop is False


def test_framesource_image_open_and_read_shape(tmp_path):
    p = tmp_path / "t.png"
    _write_png(p, width=32, height=24, value=77)

    src = cn.FrameSource("image", path=str(p), loop=True).open()
    frame = src.read()

    assert frame is not None
    assert frame.shape == (24, 32, 3)


def test_framesource_image_read_returns_independent_copy(tmp_path):
    """``read()`` 必须返回副本，否则下游改图会污染内部缓存。"""
    p = tmp_path / "t.png"
    _write_png(p, width=16, height=16, value=50)

    src = cn.FrameSource("image", path=str(p)).open()
    first = src.read()
    first[:] = 0                       # 破坏返回值
    second = src.read()

    assert int(second.max()) == 50     # 内部图像未被破坏


def test_framesource_image_read_before_open_returns_none():
    assert cn.FrameSource("image", path="whatever.png").read() is None


def test_framesource_open_missing_image_raises(tmp_path):
    with pytest.raises(RuntimeError):
        cn.FrameSource("image", path=str(tmp_path / "nope.png")).open()


def test_framesource_open_missing_video_raises(tmp_path):
    with pytest.raises(RuntimeError):
        cn.FrameSource("video", path=str(tmp_path / "nope.avi")).open()


def test_framesource_release_is_idempotent(tmp_path):
    p = tmp_path / "t.png"
    _write_png(p)

    src = cn.FrameSource("image", path=str(p)).open()
    src.release()
    src.release()                      # 第二次不应抛异常
    assert src.cap is None


# ---------------------------------------------------------------------------
# CameraNode.on_image（gazebo 中转）
# ---------------------------------------------------------------------------
def test_on_image_fills_missing_frame_id():
    stub = _NodeStub(frame_id="camera_optical_frame")
    msg = Image()
    msg.header.frame_id = ""

    cn.CameraNode.on_image(stub, msg)

    assert len(stub.pub.sent) == 1
    assert stub.pub.sent[0].header.frame_id == "camera_optical_frame"


def test_on_image_keeps_existing_frame_id():
    """已有 frame_id 时不得覆盖（仿真插件的坐标系更有意义）。"""
    stub = _NodeStub(frame_id="camera_optical_frame")
    msg = Image()
    msg.header.frame_id = "gz_camera_link"

    cn.CameraNode.on_image(stub, msg)

    assert stub.pub.sent[0].header.frame_id == "gz_camera_link"


def test_on_image_republishes_every_message():
    stub = _NodeStub()
    for _ in range(3):
        cn.CameraNode.on_image(stub, Image())
    assert len(stub.pub.sent) == 3


# ---------------------------------------------------------------------------
# CameraNode.on_timer（主动源时钟层）
# ---------------------------------------------------------------------------
def test_on_timer_does_not_publish_when_no_frame(monkeypatch):
    """读不到帧时**不得发布**——否则下游会把空图当真图处理。"""
    stub = _NodeStub()
    stub.src = _SrcStub(None)
    # logwarn_throttle 内部要取 rospy 时间，未 init_node 时会抛 ROSInitException
    monkeypatch.setattr(cn.rospy, "logwarn_throttle", lambda *a, **k: None)

    cn.CameraNode.on_timer(stub, None)

    assert stub.published == []


def test_on_timer_publishes_available_frame():
    stub = _NodeStub()
    frame = np.zeros((8, 8, 3), np.uint8)
    stub.src = _SrcStub(frame)

    cn.CameraNode.on_timer(stub, None)

    assert len(stub.published) == 1
    assert stub.published[0] is frame


# ---------------------------------------------------------------------------
# CameraNode._publish（图像打包）
# ---------------------------------------------------------------------------
def test_publish_uses_bgr8_and_sets_header(monkeypatch):
    """``encoding`` 必须是 bgr8（与 /detector 的 ``image_msg_to_array`` 约定一致）。"""
    stub = _NodeStub(frame_id="cam_frame")
    frame = np.full((10, 20, 3), 128, np.uint8)
    stamp = rospy.Time(123, 456)

    monkeypatch.setattr(cn.rospy, "Time",
                        type("_T", (), {"now": staticmethod(lambda: stamp)}))

    cn.CameraNode._publish(stub, frame)

    assert len(stub.pub.sent) == 1
    out = stub.pub.sent[0]
    assert out.encoding == "bgr8"
    assert (out.width, out.height) == (20, 10)
    assert out.header.frame_id == "cam_frame"
    assert (out.header.stamp.secs, out.header.stamp.nsecs) == (123, 456)


def test_publish_does_not_crash_on_bad_frame(monkeypatch):
    """打包失败时记日志返回，**不得抛异常打断节点**。"""
    stub = _NodeStub()
    monkeypatch.setattr(cn.rospy, "logerr_throttle", lambda *a, **k: None)

    cn.CameraNode._publish(stub, None)     # 非法输入：只记日志，不抛异常

    assert stub.pub.sent == []
