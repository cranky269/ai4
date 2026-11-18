#!/usr/bin/env python3
"""Display the live M0 -> real M4 -> real M5 -> real M6 isolated chain.

The helper is subscriber-only.  It never publishes commands and expects M6
to be remapped to /test/ctrl_visual before it is started.
"""
import argparse
import math
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospy
from PIL import Image as PILImage, ImageDraw, ImageFont
from mavros_msgs.msg import PositionTarget
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from uav_tracking.msg import Detections, VisualTrackArray


WIDTH, HEIGHT = 1280, 720
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_CACHE = {}


def add_texts(frame, labels):
    image = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(image)
    for value, xy, size, color in labels:
        if size not in FONT_CACHE:
            FONT_CACHE[size] = ImageFont.truetype(FONT_PATH, size)
        draw.text(xy, str(value), font=FONT_CACHE[size],
                  fill=(color[2], color[1], color[0]))
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def recent_rate(rows, now, window=5.0):
    values = [value for value in rows if now - value <= window]
    if len(values) < 2 or values[-1] <= values[0]:
        return 0.0
    return (len(values) - 1) / (values[-1] - values[0])


class LivePanel:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.started = time.monotonic()
        self.ros_now = None
        self.image = None
        self.image_size = (0, 0)
        self.detections = []
        self.tracks = []
        self.relay_tracks = []
        self.command = None
        self.last_track_message_ros = None
        self.last_valid_ros = None
        self.detection_times = deque(maxlen=500)
        self.track_times = deque(maxlen=500)
        self.command_times = deque(maxlen=1000)
        self.curves = deque(maxlen=2500)
        rospy.Subscriber("/clock", Clock, self.on_clock, queue_size=100)
        rospy.Subscriber("/camera/image_raw", Image, self.on_image, queue_size=1)
        rospy.Subscriber("/detections", Detections, self.on_detections, queue_size=20)
        rospy.Subscriber("/visual/tracks", VisualTrackArray, self.on_source_tracks, queue_size=20)
        rospy.Subscriber("/test/visual_tracks", VisualTrackArray, self.on_relay_tracks, queue_size=20)
        rospy.Subscriber("/test/ctrl_visual", PositionTarget, self.on_command, queue_size=200)

    def wall(self):
        return time.monotonic() - self.started

    def on_clock(self, msg):
        with self.lock:
            self.ros_now = msg.clock.to_sec()

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size < msg.height * msg.step:
            return
        image = data[:msg.height * msg.step].reshape(msg.height, msg.step)
        image = image[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.image = image.copy()
            self.image_size = (int(msg.width), int(msg.height))

    def on_detections(self, msg):
        rows = []
        for item in msg.targets:
            rows.append({
                "class_id": int(item.class_id),
                "confidence": float(item.confidence),
                "bbox": tuple(float(value) for value in item.bbox),
            })
        with self.lock:
            self.detections = rows
            self.detection_times.append(self.wall())

    @staticmethod
    def unpack_tracks(msg):
        rows = []
        for item in msg.tracks:
            rows.append({
                "id": int(item.id),
                "area_ratio": float(item.area_ratio),
                "bbox": tuple(float(value) for value in item.bbox),
            })
        return rows

    def on_source_tracks(self, msg):
        rows = self.unpack_tracks(msg)
        with self.lock:
            self.tracks = rows
            self.track_times.append(self.wall())

    def on_relay_tracks(self, msg):
        rows = self.unpack_tracks(msg)
        now_ros = rospy.Time.now().to_sec()
        with self.lock:
            self.relay_tracks = rows
            self.last_track_message_ros = now_ros
            if rows:
                self.last_valid_ros = now_ros

    def on_command(self, msg):
        now = self.wall()
        row = {
            "vx": float(msg.velocity.x),
            "vz": float(msg.velocity.z),
            "yaw_rate": float(msg.yaw_rate),
            "frame": int(msg.coordinate_frame),
            "mask": int(msg.type_mask),
        }
        with self.lock:
            self.command = row
            self.command_times.append(now)
            self.curves.append((now, row["vx"], row["vz"], row["yaw_rate"]))

    def state(self, ros_now, tracks, last_message, last_valid):
        if self.command is None:
            return "NO OUTPUT", None
        if ros_now is None or last_valid is None:
            return "STOP", None
        age = max(0.0, ros_now - last_valid)
        message_age = math.inf if last_message is None else max(0.0, ros_now - last_message)
        if tracks and message_age <= 0.15:
            return "TRACK", age
        if age <= 0.2:
            return "HOLD", age
        return "STOP", age

    @staticmethod
    def draw_arrow(frame, start, value, axis, color, label):
        magnitude = min(90, max(12, int(abs(value) * 65)))
        sign = 1 if value >= 0 else -1
        if axis == "x":
            end = (start[0] + sign * magnitude, start[1])
        else:
            end = (start[0], start[1] - sign * magnitude)
        cv2.arrowedLine(frame, start, end, color, 4, tipLength=.22)
        cv2.putText(frame, label, (start[0] - 60, start[1] + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)

    def render(self):
        with self.lock:
            image = None if self.image is None else self.image.copy()
            image_size = self.image_size
            detections = [dict(row) for row in self.detections]
            tracks = [dict(row) for row in self.tracks]
            relay_tracks = [dict(row) for row in self.relay_tracks]
            command = None if self.command is None else dict(self.command)
            ros_now = self.ros_now
            last_message = self.last_track_message_ros
            last_valid = self.last_valid_ros
            curves = list(self.curves)
            detection_times = list(self.detection_times)
            track_times = list(self.track_times)
            command_times = list(self.command_times)
        now = self.wall()
        state, valid_age = self.state(ros_now, relay_tracks, last_message, last_valid)
        state_colors = {
            "TRACK": (60, 220, 80), "HOLD": (0, 215, 255),
            "STOP": (80, 80, 255), "NO OUTPUT": (130, 130, 130),
        }
        state_color = state_colors[state]
        frame = np.full((HEIGHT, WIDTH, 3), (20, 25, 34), np.uint8)

        view_box = (20, 80, 660, 560)
        cv2.rectangle(frame, view_box[:2], view_box[2:], (75, 85, 100), 1)
        if image is not None:
            view = cv2.resize(image, (640, 480), interpolation=cv2.INTER_LINEAR)
            scale_x = 640.0 / max(1, image_size[0])
            scale_y = 480.0 / max(1, image_size[1])
            for item in detections:
                x, y, w, h = item["bbox"]
                p0 = (int(x * scale_x), int(y * scale_y))
                p1 = (int((x + w) * scale_x), int((y + h) * scale_y))
                cv2.rectangle(view, p0, p1, (40, 70, 255), 2)
                cv2.putText(view, "drone %.2f" % item["confidence"],
                            (p0[0], max(18, p0[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                            .55, (40, 70, 255), 2, cv2.LINE_AA)
            for item in tracks:
                x, y, w, h = item["bbox"]
                p0 = (int(x * scale_x), int(y * scale_y))
                p1 = (int((x + w) * scale_x), int((y + h) * scale_y))
                cv2.rectangle(view, p0, p1, (70, 255, 110), 2)
                cv2.putText(view, "ID%d area %.4f" % (item["id"], item["area_ratio"]),
                            (p0[0], min(470, p1[1] + 20)), cv2.FONT_HERSHEY_SIMPLEX,
                            .5, (70, 255, 110), 2, cv2.LINE_AA)
            cv2.drawMarker(view, (320, 240), (255, 190, 60), cv2.MARKER_CROSS, 24, 2)
            frame[80:560, 20:660] = view
        else:
            cv2.putText(frame, "WAITING FOR /camera/image_raw", (145, 320),
                        cv2.FONT_HERSHEY_SIMPLEX, .8, (80, 80, 255), 2)

        cv2.rectangle(frame, (685, 80), (1260, 340), (75, 85, 100), 1)
        if command is not None:
            self.draw_arrow(frame, (810, 205), command["vx"], "x", (80, 230, 255), "forward/back")
            self.draw_arrow(frame, (995, 205), command["vz"], "z", (100, 255, 120), "up/down")
            yaw_end = (1180 + int(max(-75, min(75, -command["yaw_rate"] * 90))), 205)
            cv2.arrowedLine(frame, (1180, 205), yaw_end, (255, 150, 90), 4, tipLength=.22)
            cv2.putText(frame, "yaw left/right", (1110, 233), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (255, 150, 90), 1, cv2.LINE_AA)

        graph = (685, 380, 1260, 680)
        cv2.rectangle(frame, graph[:2], graph[2:], (75, 85, 100), 1)
        recent = [row for row in curves if now - row[0] <= 15.0]
        if len(recent) > 1:
            values = [value for row in recent for value in row[1:]]
            bound = max(.2, max(abs(value) for value in values) * 1.15)
            zero_y = int((graph[1] + graph[3]) / 2)
            cv2.line(frame, (graph[0], zero_y), (graph[2], zero_y), (80, 90, 105), 1)
            for column, color in ((1, (80, 230, 255)), (2, (100, 255, 120)), (3, (255, 150, 90))):
                points = []
                span = max(.001, recent[-1][0] - recent[0][0])
                for row in recent:
                    x = int(graph[0] + (row[0] - recent[0][0]) / span * (graph[2] - graph[0]))
                    y = int(zero_y - row[column] / bound * (graph[3] - graph[1]) / 2)
                    points.append((x, y))
                cv2.polylines(frame, [np.asarray(points, np.int32)], False, color, 2)

        track_text = ", ".join("ID%d area=%.4f" % (row["id"], row["area_ratio"])
                               for row in tracks) or "--"
        command_text = "--" if command is None else "vx=%+.3f  vz=%+.3f  yaw_rate=%+.3f" % (
            command["vx"], command["vz"], command["yaw_rate"])
        labels = [
            ("LIVE GAZEBO", (20, 10), 30, (60, 80, 255)),
            ("实时 M0→真实 M4→真实 M5→真实 M6", (210, 14), 27, (80, 230, 255)),
            ("隔离输出 /test/ctrl_visual；非飞控闭环", (830, 18), 20, (100, 180, 255)),
            ("camera + M4 detections + M5 tracks", (30, 86), 18, (235, 235, 235)),
            (state, (700, 92), 30, state_color),
            ("valid_age %s ROS s" % ("--" if valid_age is None else "%.3f" % valid_age),
             (700, 140), 19, state_color),
            ("M4 %.2f Hz wall | M5 %.2f Hz wall | M6 %.2f Hz wall" % (
                recent_rate(detection_times, now), recent_rate(track_times, now),
                recent_rate(command_times, now)), (700, 172), 17, (225, 225, 225)),
            (track_text, (700, 275), 18, (100, 255, 140)),
            (command_text, (700, 307), 18, (225, 225, 225)),
            ("frame=%s  mask=%s" % (
                "--" if command is None else command["frame"],
                "--" if command is None else command["mask"]), (1060, 92), 19, (235, 235, 235)),
            ("velocity.x / velocity.z / yaw_rate (last 15 wall s)",
             (700, 390), 17, (220, 220, 220)),
            ("develop " + self.args.develop_sha[:12], (25, 678), 17, (190, 190, 190)),
            ("RUN_DIR " + Path(self.args.run_dir).name, (270, 678), 17, (190, 190, 190)),
        ]
        return add_texts(frame, labels)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--develop-sha", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--window", default="LIVE GAZEBO - real M0 to M4 to M5 to isolated M6")
    args = parser.parse_args()
    rospy.init_node("m5_m6_live_panel", anonymous=True, disable_signals=True)
    panel = LivePanel(args)
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window, WIDTH, HEIGHT)
    try:
        while not rospy.is_shutdown():
            cv2.imshow(args.window, panel.render())
            key = cv2.waitKey(10) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyWindow(args.window)


if __name__ == "__main__":
    main()
