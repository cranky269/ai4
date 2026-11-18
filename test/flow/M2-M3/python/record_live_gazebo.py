#!/usr/bin/env python3
"""Write a video directly from active M2/M3 ROS topics and controlled pose."""
import argparse
import json
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospy
from PIL import Image as PILImage, ImageDraw, ImageFont
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from uav_tracking.msg import TrackArray


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
WIDTH, HEIGHT, FPS = 1280, 720, 25.0
FONT_CACHE = {}


def add_texts(frame, labels):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = PILImage.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    for value, xy, size, color in labels:
        if size not in FONT_CACHE:
            FONT_CACHE[size] = ImageFont.truetype(FONT_PATH, size)
        draw.text(xy, str(value), font=FONT_CACHE[size],
                  fill=(color[2], color[1], color[0]))
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def rate(values, use_ros=True, window=2.0):
    if len(values) < 2:
        return 0.0
    index = 1 if use_ros else 0
    end = values[-1][index]
    rows = [v for v in values if end - v[index] <= window]
    return (len(rows) - 1) / (rows[-1][index] - rows[0][index]) if len(rows) > 1 else 0.0


class Recorder:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.wall_start = time.monotonic()
        self.ros_start = self.ros_now = None
        self.camera = None
        self.source_track = self.relay_track = None
        self.pose = None
        self.command = None
        self.last_source_ros = None
        self.last_relay_ros = None
        self.outputs = deque(maxlen=5000)
        self.states_seen = set()
        self.observed_state = "NO OUTPUT"
        self.observed_age = None
        self.hold_events = 0
        self.rendered_states = set()
        self.frame_values, self.mask_values = set(), set()
        rospy.Subscriber("/clock", Clock, self.on_clock, queue_size=100)
        rospy.Subscriber("/camera/image_raw", Image, self.on_camera, queue_size=1)
        rospy.Subscriber("/radar/tracks", TrackArray, self.on_source, queue_size=20)
        rospy.Subscriber("/test/radar_tracks", TrackArray, self.on_relay, queue_size=20)
        rospy.Subscriber("/test/local_pose", PoseStamped, self.on_pose, queue_size=10)
        rospy.Subscriber("/test/ctrl_radar", PositionTarget, self.on_command, queue_size=200)

    def wall(self):
        return time.monotonic() - self.wall_start

    @staticmethod
    def select(msg):
        if not msg.tracks:
            return None
        item = sorted(msg.tracks, key=lambda x: (-int(x.age), int(x.id)))[0]
        return {"id": int(item.id), "age": int(item.age),
                "position": tuple(float(v) for v in item.position)}

    def on_clock(self, msg):
        with self.lock:
            value = msg.clock.to_sec()
            if self.ros_start is None:
                self.ros_start = value
            self.ros_now = value

    def on_camera(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        row = np.frombuffer(msg.data, np.uint8)
        if row.size < msg.height * msg.step:
            return
        image = row[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * 3]
        image = image.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.camera = image.copy()

    def on_source(self, msg):
        with self.lock:
            self.source_track = self.select(msg)
            if msg.tracks:
                self.last_source_ros = rospy.Time.now().to_sec()

    def on_relay(self, msg):
        with self.lock:
            self.relay_track = self.select(msg)
            if msg.tracks:
                self.last_relay_ros = rospy.Time.now().to_sec()

    def on_pose(self, msg):
        with self.lock:
            self.pose = (float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z))

    def on_command(self, msg):
        wall, ros = self.wall(), rospy.Time.now().to_sec()
        row = {"wall": wall, "ros": ros, "position": (float(msg.position.x), float(msg.position.y), float(msg.position.z)),
               "frame": int(msg.coordinate_frame), "mask": int(msg.type_mask)}
        with self.lock:
            self.command = row
            self.outputs.append((wall, ros, row["position"]))
            self.frame_values.add(row["frame"])
            self.mask_values.add(row["mask"])
            state, age = self.state(ros)
            if state == "HOLD" and self.observed_state != "HOLD":
                self.hold_events += 1
            self.observed_state, self.observed_age = state, age
            self.states_seen.add(state)

    def state(self, ros_now):
        if self.command is None or self.pose is None:
            return "NO OUTPUT", None
        age = None if self.last_relay_ros is None else max(0.0, ros_now - self.last_relay_ros)
        if age is None:
            return "HOVER", age
        if age > .5:
            return "HOVER", age
        # During injected silence the real source continues while the isolated
        # relay stops.  This distinguishes HOLD from a normal inter-message gap.
        relay_is_silent = (self.last_source_ros is not None
                           and self.last_relay_ros is not None
                           and self.last_source_ros - self.last_relay_ros > .05)
        if relay_is_silent:
            return "HOLD", age
        return "TRACK", age

    @staticmethod
    def plan(xyz):
        x, y, _ = xyz
        return int(640 + (x - 10.0) * 18), int(365 - (y - 20.0) * 18)

    def render(self):
        with self.lock:
            camera = None if self.camera is None else self.camera.copy()
            source = None if self.source_track is None else dict(self.source_track)
            relay = None if self.relay_track is None else dict(self.relay_track)
            pose = self.pose
            command = None if self.command is None else dict(self.command)
            outputs = list(self.outputs)
            ros_now, ros_start = self.ros_now, self.ros_start
            frames, masks = sorted(self.frame_values), sorted(self.mask_values)
        now = self.wall()
        with self.lock:
            state, age = self.observed_state, self.observed_age
            self.rendered_states.add(state)
        colors = {"TRACK": (60, 220, 80), "HOLD": (0, 215, 255),
                  "HOVER": (255, 150, 60), "NO OUTPUT": (100, 100, 120)}
        color = colors[state]
        frame = np.full((HEIGHT, WIDTH, 3), (20, 25, 34), np.uint8)
        cv2.rectangle(frame, (20, 80), (435, 390), (70, 80, 95), 1)
        if camera is not None:
            frame[80:390, 20:435] = cv2.resize(camera, (415, 310))
        cv2.rectangle(frame, (455, 80), (940, 500), (70, 80, 95), 1)
        cv2.line(frame, (485, 455), (900, 455), (100, 110, 125), 1)
        cv2.line(frame, (695, 110), (695, 480), (100, 110, 125), 1)
        if pose is not None:
            cv2.circle(frame, self.plan(pose), 10, (255, 150, 60), -1)
        if source is not None:
            cv2.circle(frame, self.plan(source["position"]), 9, (70, 230, 90), -1)
        if command is not None:
            cv2.drawMarker(frame, self.plan(command["position"]), color, cv2.MARKER_DIAMOND, 26, 3)
        # Position command curves are revealed as messages arrive live.
        graph = (40, 535, 930, 685)
        cv2.rectangle(frame, graph[:2], graph[2:], (95, 105, 120), 1)
        if len(outputs) > 1:
            recent = [v for v in outputs if now - v[0] <= 12.0]
            if len(recent) > 1:
                all_v = [x for row in recent for x in row[2]]
                lo, hi = min(all_v) - .5, max(all_v) + .5
                for index, curve_color in enumerate(((70, 220, 255), (100, 255, 120), (255, 170, 80))):
                    pts = []
                    for row in recent:
                        x = int(graph[0] + (row[0] - recent[0][0]) / max(.001, recent[-1][0] - recent[0][0]) * (graph[2] - graph[0]))
                        y = int(graph[3] - (row[2][index] - lo) / (hi - lo) * (graph[3] - graph[1]))
                        pts.append((x, y))
                    cv2.polylines(frame, [np.asarray(pts, np.int32)], False, curve_color, 2)
        ros_elapsed = None if ros_now is None or ros_start is None else ros_now - ros_start
        rtf = 0.0 if ros_elapsed is None or now <= 0 else ros_elapsed / now
        track_str = "--" if source is None else "id=%d age=%d (%.2f, %.2f, %.2f)" % (
            source["id"], source["age"], *source["position"])
        cmd_str = "--" if command is None else "(%.2f, %.2f, %.2f)" % command["position"]
        pose_str = "--" if pose is None else "(%.2f, %.2f, %.2f)" % pose
        labels = [
            ("LIVE GAZEBO", (20, 10), 30, (60, 80, 255)),
            ("实时 Gazebo 雷达航迹 + 受控 local pose：真实 M2 → 真实 M3", (205, 14), 25, (80, 230, 255)),
            ("非 MAVROS 位姿，非飞控闭环", (900, 48), 18, (100, 180, 255)),
            ("M0 live camera", (30, 88), 18, (230, 230, 230)),
            ("world / local ENU alignment remains a design assumption", (470, 90), 16, (210, 210, 210)),
            (state, (975, 95), 32, color),
            ("wall %.2f s" % now, (970, 155), 19, (235, 235, 235)),
            ("ROS sim %s s" % ("--" if ros_now is None else "%.2f" % ros_now), (970, 188), 19, (235, 235, 235)),
            ("RTF %.3f" % rtf, (970, 221), 19, (235, 235, 235)),
            ("valid_age %s ROS s" % ("--" if age is None else "%.3f" % age), (970, 270), 19, color),
            ("output %.2f Hz ROS" % rate(outputs, True), (970, 305), 19, (235, 235, 235)),
            ("frame %s" % (frames or "--"), (970, 350), 21, (235, 235, 235)),
            ("mask %s" % (masks or "--"), (970, 385), 21, (235, 235, 235)),
            ("Track.position: " + track_str, (465, 440), 17, (100, 255, 140)),
            ("controlled pose: " + pose_str, (25, 420), 17, (255, 180, 100)),
            ("Navigator command: " + cmd_str, (465, 475), 17, color),
            ("x/y/z command timeline (last 12 wall s)", (45, 545), 17, (220, 220, 220)),
            ("develop " + self.args.develop_sha[:12], (965, 670), 17, (190, 190, 190)),
        ]
        return add_texts(frame, labels)

    def run(self):
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        captured = []
        capture_started = time.monotonic()
        while not rospy.is_shutdown() and self.wall() < self.args.duration:
            frame = self.render()
            captured.append((min(self.wall(), self.args.duration), frame))
            time.sleep(.01)
        capture_duration = time.monotonic() - capture_started
        if not captured:
            raise RuntimeError("No live frames captured")
        encoded = cv2.VideoWriter(str(self.args.output), cv2.VideoWriter_fourcc(*"avc1"),
                                  FPS, (WIDTH, HEIGHT))
        if not encoded.isOpened():
            raise RuntimeError("OpenCV could not open H.264 writer")
        count = int(self.args.duration * FPS)
        source_index = 0
        for index in range(count):
            output_time = index / FPS
            while (source_index + 1 < len(captured)
                   and captured[source_index + 1][0] <= output_time):
                source_index += 1
            encoded.write(captured[source_index][1])
        encoded.release()
        metadata = {
            "live": True, "source": "live M0/M1/M2 tracks + controlled local pose + real M3",
            "controlled_pose": True, "mavros_pose": False, "flight_loop": False,
            "video": str(self.args.output), "frames": count, "fps": FPS,
            "resolution": [WIDTH, HEIGHT], "capture_wall_duration_s": capture_duration,
            "video_duration_s": count / FPS, "codec": "H.264/avc1",
            "live_rendered_frames": len(captured),
            "develop_sha": self.args.develop_sha, "run_dir": self.args.run_dir,
            "ros_master_uri": rospy.get_master().getUri(), "states_seen": sorted(self.states_seen),
            "hold_events_observed_at_50hz_output": self.hold_events,
            "states_rendered_from_live_topics": sorted(self.rendered_states),
            "coordinate_frames": sorted(self.frame_values), "type_masks": sorted(self.mask_values),
            "flight_controller_connected": False,
        }
        self.args.metadata.write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--develop-sha", required=True)
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    rospy.init_node("m2_m3_live_video_recorder", anonymous=True, disable_signals=True)
    Recorder(args).run()


if __name__ == "__main__":
    main()
