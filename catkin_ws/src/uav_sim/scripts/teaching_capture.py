#!/usr/bin/python3
"""两机教学演示的桌面录制、原始图像导出及只读文件核对。"""
import ctypes
import json
import re
import signal
import subprocess
import sys
import time
from pathlib import Path


def gst():
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)
    return Gst


def screenshot(path):
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.primaryScreen().grabWindow(0).save(str(path))


def arrange(out):
    """沿用第三轮的固定旁观视角和窗口位置，不改机载相机。"""
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rect = app.primaryScreen().geometry()
    if rect.width() < 2200 or rect.height() < 1180:
        raise RuntimeError('当前桌面不足2200×1180，不能完整保持已验证的并排布局；请调整桌面分辨率后重试')
    pose = ('camera {name: "user_camera" pose {position {x: 2 y: -6 z: 3} '
            'orientation {x: -0.140480431 y: 0.140480431 z: 0.693011723 w: 0.693011723}}}')
    view = subprocess.run(['gz', 'topic', '-p', '/gazebo/base_scene/gui', '-m', pose],
                          capture_output=True, text=True, timeout=12)
    if view.returncode:
        raise RuntimeError('固定旁观视角设置失败：' + view.stderr)
    x = ctypes.CDLL('libX11.so.6')
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x.XOpenDisplay.restype = ctypes.c_void_p
    display = x.XOpenDisplay(None)
    if not display:
        raise RuntimeError('无法访问X桌面')
    x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x.XDefaultRootWindow.restype = ctypes.c_ulong
    root = x.XDefaultRootWindow(display)
    x.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x.XInternAtom.restype = ctypes.c_ulong

    class Data(ctypes.Union):
        _fields_ = [('b', ctypes.c_char * 20), ('s', ctypes.c_short * 10), ('l', ctypes.c_long * 5)]

    class Client(ctypes.Structure):
        _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong), ('send', ctypes.c_int),
                    ('display', ctypes.c_void_p), ('window', ctypes.c_ulong),
                    ('message_type', ctypes.c_ulong), ('format', ctypes.c_int), ('data', Data)]

    class Event(ctypes.Union):
        _fields_ = [('client', Client), ('pad', ctypes.c_long * 24)]

    x.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.POINTER(Event)]
    x.XMoveResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
    x.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x.XFlush.argtypes = [ctypes.c_void_p]
    x.XCloseDisplay.argtypes = [ctypes.c_void_p]
    try:
        end = time.monotonic() + 25
        while time.monotonic() < end:
            tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True, timeout=5)
            a = re.search(r'(0x[0-9a-f]+) "Gazebo":', tree)
            b = re.search(r'(0x[0-9a-f]+) "AI4 onboard camera - live":', tree)
            if a and b:
                break
            time.sleep(.2)
        else:
            raise RuntimeError('Gazebo或实时相机窗口未就绪')
        wins = [int(a[1], 16), int(b[1], 16)]
        for window in wins:
            event = Event()
            event.client.type = 33
            event.client.display = display
            event.client.window = window
            event.client.message_type = x.XInternAtom(display, b'_NET_WM_STATE', 0)
            event.client.format = 32
            event.client.data.l[:] = [0, x.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_VERT', 0),
                                     x.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_HORZ', 0), 1, 0]
            x.XSendEvent(display, root, 0, (1 << 20) | (1 << 19), ctypes.byref(event))
        x.XFlush(display)
        time.sleep(1)
        x.XMoveResizeWindow(display, wins[0], 80, 100, 1290, 1000)
        x.XMoveResizeWindow(display, wins[1], 1480, 280, 640, 480)
        for window in wins:
            x.XRaiseWindow(display, window)
        x.XFlush(display)
        time.sleep(2)
        screenshot(out / 'before.png')
        (out / 'window_geometry.txt').write_text(subprocess.check_output(['xwininfo', '-root', '-tree'], text=True, timeout=5))
        layout = {'screen': [rect.width(), rect.height()], 'capture': [60, 60, 2240, 1120],
                  'video': [1120, 560], 'camera_source': [640, 480], 'windows': wins,
                  'spectator_pose_message': pose}
        # 固定第三轮区域；不随窗口漂移裁剪相机内容。
        if rect.width() < 2300:
            raise RuntimeError('保持第三轮录屏区域需要桌面宽度至少2300像素')
        (out / 'layout.json').write_text(json.dumps(layout, ensure_ascii=False, indent=2))
        return layout
    finally:
        x.XCloseDisplay(display)


def record_screen(out):
    Gst = gst()
    layout = json.loads((out / 'layout.json').read_text())
    width, height = layout['screen']
    stopping = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
    pipeline = ('ximagesrc use-damage=false show-pointer=false ! video/x-raw,framerate=6/1 ! '
                'queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! '
                'videocrop left=60 right=%d top=60 bottom=%d ! videoscale ! '
                'video/x-raw,width=1120,height=560,pixel-aspect-ratio=1/1 ! videoconvert ! '
                'video/x-raw,format=I420 ! vp8enc name=encoder deadline=1 cpu-used=8 threads=2 '
                'target-bitrate=1800000 ! webmmux ! filesink location="%s"') % (
                    width - 2300, height - 1180, out / 'desktop.webm')
    (out / 'desktop_pipeline.txt').write_text(pipeline)
    pipe = Gst.parse_launch(pipeline)
    bus = pipe.get_bus()
    frames = []

    def encoded(pad, info):
        frames.append([time.monotonic(), info.get_buffer().pts / Gst.SECOND])
        if len(frames) == 3:
            (out / 'screen_ready.json').write_text(json.dumps({'monotonic': time.monotonic()}))
        return Gst.PadProbeReturn.OK

    pipe.get_by_name('encoder').get_static_pad('src').add_probe(Gst.PadProbeType.BUFFER, encoded)
    error = None
    eos = False
    try:
        if pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('桌面录屏管道启动失败')
        start = time.monotonic()
        while not stopping[0] and time.monotonic() - start < 300:
            msg = bus.timed_pop_filtered(100 * Gst.MSECOND, Gst.MessageType.ERROR)
            if msg:
                raise RuntimeError(str(msg.parse_error()))
        pipe.send_event(Gst.Event.new_eos())
        msg = bus.timed_pop_filtered(20 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        eos = bool(msg and msg.type == Gst.MessageType.EOS)
        if not eos:
            raise RuntimeError('桌面录像未正常收到EOS')
    except Exception as exc:
        error = str(exc)
    finally:
        pipe.set_state(Gst.State.NULL)
        (out / 'screen_result.json').write_text(json.dumps({'eos': eos, 'error': error, 'encoded_frames': frames}, indent=2))
    return 1 if error else 0


def export_camera(out):
    """从本轮原始RGB图像导出，PTS为采集仿真时间；不合成或识别内容。"""
    import rosbag
    Gst = gst()
    target = out / 'camera.webm'
    if target.exists():
        raise RuntimeError('相机视频已存在，拒绝覆盖：' + str(target))
    with rosbag.Bag(str(out / 'camera.bag')) as bag:
        first = next(bag.read_messages(topics=['/uav/front_camera/image_raw']), None)
    if first is None:
        raise RuntimeError('bag中无原始相机图像')
    image = first.message
    if image.encoding != 'rgb8':
        raise RuntimeError('本入口仅导出已验证的rgb8，不猜测其他编码：' + image.encoding)
    pipeline = ('appsrc name=source format=time is-live=false block=true max-bytes=3000000 '
                'caps="video/x-raw,format=RGB,width=%d,height=%d,framerate=0/1" ! '
                'videoconvert ! video/x-raw,format=I420 ! vp8enc deadline=1 cpu-used=8 threads=2 '
                'target-bitrate=1800000 ! webmmux ! filesink location="%s"') % (image.width, image.height, target)
    pipe = Gst.parse_launch(pipeline)
    source = pipe.get_by_name('source')
    bus = pipe.get_bus()
    start_stamp = None
    last_stamp = None
    count = 0
    try:
        if pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('相机视频导出管道启动失败')
        with rosbag.Bag(str(out / 'camera.bag')) as bag:
            for _, msg, _ in bag.read_messages(topics=['/uav/front_camera/image_raw']):
                stamp = msg.header.stamp.to_nsec()
                if last_stamp is not None and stamp <= last_stamp:
                    raise RuntimeError('图像采集时间未严格递增，停止导出，保留原始bag')
                if msg.width != image.width or msg.height != image.height or msg.encoding != 'rgb8':
                    raise RuntimeError('图像格式在记录中改变，保留bag并拒绝混合导出')
                if start_stamp is None:
                    start_stamp = stamp
                # 去掉可能的行填充；不改变任何RGB像素。
                raw = bytes(msg.data) if msg.step == msg.width * 3 else b''.join(
                    bytes(msg.data[y * msg.step:y * msg.step + msg.width * 3]) for y in range(msg.height))
                buf = Gst.Buffer.new_allocate(None, len(raw), None)
                buf.fill(0, raw)
                buf.pts = stamp - start_stamp
                if source.emit('push-buffer', buf) != Gst.FlowReturn.OK:
                    raise RuntimeError('相机视频编码器拒绝数据')
                count += 1
                last_stamp = stamp
        source.emit('end-of-stream')
        msg = bus.timed_pop_filtered(30 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        if not msg or msg.type != Gst.MessageType.EOS:
            raise RuntimeError('相机视频导出未正常完成：' + str(msg.parse_error() if msg else 'timeout'))
    finally:
        pipe.set_state(Gst.State.NULL)
    result = {'frames': count, 'source': 'camera.bag /uav/front_camera/image_raw rgb8',
              'time_base': 'header.stamp仿真采集时间（非墙钟播放时长）',
              'source_sim_span_s': (last_stamp - start_stamp) / 1e9}
    (out / 'camera_export.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    return result


def check_files(out):
    import rosbag
    Gst = gst()
    import gi
    gi.require_version('GstPbutils', '1.0')
    from gi.repository import GstPbutils
    result = {'videos': {}, 'topics': {}, 'static_frames': [], 'static_latching': False}
    with rosbag.Bag(str(out / 'camera.bag')) as bag:
        for topic, info in bag.get_type_and_topic_info().topics.items():
            result['topics'][topic] = {'type': info.msg_type, 'count': info.message_count}
        frames = set()
        for _, msg, _ in bag.read_messages(topics=['/tf_static']):
            frames.update(t.child_frame_id for t in msg.transforms)
        result['static_frames'] = sorted(frames)
        # 每条静态连接均保持锁存，供rosbag play的后加入订阅者读取。
        connections = list(bag._get_connections(topics=['/tf_static']))
        result['static_latching'] = bool(connections) and all(str(c.header.get('latching', '')) == '1' for c in connections)
    for name in ('desktop.webm', 'camera.webm'):
        path = out / name
        if not path.is_file():
            result['videos'][name] = {'error': '未生成'}
            continue
        info = GstPbutils.Discoverer.new(15 * Gst.SECOND).discover_uri(path.as_uri())
        pipe = Gst.parse_launch('filesrc location="%s" ! decodebin ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink sync=false max-buffers=3' % path)
        sink = pipe.get_by_name('sink')
        frames = 0
        previous = None
        max_gap = 0.
        error = None
        width = height = 0
        try:
            pipe.set_state(Gst.State.PLAYING)
            while True:
                sample = sink.emit('try-pull-sample', 10 * Gst.SECOND)
                if sample is None:
                    msg = pipe.get_bus().pop_filtered(Gst.MessageType.ERROR)
                    if msg:
                        error = str(msg.parse_error())
                    break
                stamp = sample.get_buffer().pts / Gst.SECOND
                caps = sample.get_caps().get_structure(0)
                width, height = caps.get_value('width'), caps.get_value('height')
                if previous is not None:
                    if stamp <= previous:
                        error = '视频时间戳未严格递增'
                    max_gap = max(max_gap, stamp - previous)
                previous = stamp
                frames += 1
        finally:
            pipe.set_state(Gst.State.NULL)
        result['videos'][name] = {'decoded_frames': frames, 'size': [width, height],
                                 'duration_s': info.get_duration() / Gst.SECOND, 'max_frame_gap_s': max_gap, 'error': error}
    required = ['/uav/front_camera/image_raw', '/uav/front_camera/camera_info', '/tf', '/tf_static', '/clock']
    result['readable'] = (all(result['topics'].get(t, {}).get('count', 0) > 0 for t in required)
                          and {'uav_camera_link', 'uav_camera_optical_frame'} <= set(result['static_frames'])
                          and result['static_latching']
                          and all(v.get('decoded_frames', 0) > 0 and not v.get('error') for v in result['videos'].values()))
    (out / 'file_check.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    return result


if __name__ == '__main__':
    action, output = sys.argv[1], Path(sys.argv[2]).resolve()
    if action == 'screen':
        sys.exit(record_screen(output))
    if action == 'export':
        print(json.dumps(export_camera(output), ensure_ascii=False))
    elif action == 'check':
        checked = check_files(output)
        print(json.dumps(checked, ensure_ascii=False, indent=2))
        sys.exit(0 if checked['readable'] else 1)
