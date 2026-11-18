#!/usr/bin/env python3
"""Write a video directly from the active M0 -> M1 -> M2 ROS topics."""
import argparse
import json
import math
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from PIL import Image as PILImage, ImageDraw, ImageFont
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
from uav_tracking.msg import CentroidArray, TrackArray


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
WIDTH, HEIGHT, FPS = 1280, 720, 25.0


def add_texts(frame, labels):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = PILImage.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    fonts = {}
    for value, xy, size, color in labels:
        if size not in fonts:
            fonts[size] = ImageFont.truetype(FONT_PATH, size)
        draw.text(xy, str(value), font=fonts[size],
                  fill=(color[2], color[1], color[0]))
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def rate(values, now, window=5.0):
    rows = [v for v in values if now - v <= window]
    return (len(rows) - 1) / (rows[-1] - rows[0]) if len(rows) > 1 else 0.0


class Recorder:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.wall_start = time.monotonic()
        self.ros_start = None
        self.ros_now = None
        self.camera = None
        self.points = []
        self.centroids = []
        self.relay_empty = False
        self.empty_count = 0
        self.tracks = []
        self.truth = None
        self.track_tails = {}
        self.truth_tail = deque(maxlen=250)
        self.centroid_times = deque(maxlen=200)
        self.track_times = deque(maxlen=200)
        self.radar_times = deque(maxlen=200)
        self.ids_before_fault = set()
        self.ids_after_fault = set()
        self.had_tracks = False
        self.deleted = False
        self.recovered = False
        self.frame_ids = set()
        rospy.Subscriber("/clock", Clock, self.on_clock, queue_size=100)
        rospy.Subscriber("/camera/image_raw", Image, self.on_camera, queue_size=1)
        rospy.Subscriber("/radar/points", PointCloud2, self.on_points, queue_size=1)
        rospy.Subscriber("/radar/centroid", CentroidArray, self.on_centroids, queue_size=20)
        rospy.Subscriber("/test/radar_centroid", CentroidArray, self.on_relay, queue_size=20)
        rospy.Subscriber("/radar/tracks", TrackArray, self.on_tracks, queue_size=20)
        rospy.Subscriber("/target/ground_truth/pose", PoseStamped, self.on_truth, queue_size=20)

    def wall(self):
        return time.monotonic() - self.wall_start

    def on_clock(self, msg):
        with self.lock:
            value = msg.clock.to_sec()
            if self.ros_start is None:
                self.ros_start = value
            self.ros_now = value

    def on_camera(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        row = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if row.size < expected:
            return
        image = row[:expected].reshape(msg.height, msg.step)[:, :msg.width * 3]
        image = image.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.camera = image.copy()

    def on_points(self, msg):
        values = []
        try:
            for index, point in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
                if index % 3 == 0:
                    xyz = tuple(float(v) for v in point)
                    if all(math.isfinite(v) for v in xyz):
                        values.append(xyz)
                if len(values) >= 900:
                    break
        except Exception:
            return
        with self.lock:
            self.points = values
            self.frame_ids.add(msg.header.frame_id)
            self.radar_times.append(self.wall())

    def on_centroids(self, msg):
        values = []
        for item in msg.centroids:
            values.append((float(item.point.x), float(item.point.y), float(item.point.z)))
            self.frame_ids.add(item.header.frame_id)
        with self.lock:
            self.centroids = values
            self.centroid_times.append(self.wall())

    def on_relay(self, msg):
        with self.lock:
            self.relay_empty = not bool(msg.centroids)
            if self.relay_empty:
                self.empty_count += 1

    def on_tracks(self, msg):
        now = self.wall()
        values = []
        for item in msg.tracks:
            xyz = tuple(float(v) for v in item.position)
            row = {"id": int(item.id), "age": int(item.age), "position": xyz}
            values.append(row)
            self.track_tails.setdefault(row["id"], deque(maxlen=180)).append((now, xyz))
        with self.lock:
            self.tracks = values
            self.track_times.append(now)
            if values and self.empty_count == 0:
                self.ids_before_fault.update(v["id"] for v in values)
            if self.empty_count >= self.args.empty_count:
                if not values and self.had_tracks:
                    self.deleted = True
                if values and self.deleted:
                    self.recovered = True
                    self.ids_after_fault.update(v["id"] for v in values)
            self.had_tracks |= bool(values)

    def on_truth(self, msg):
        xyz = (float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z))
        with self.lock:
            self.truth = xyz
            self.truth_tail.append((self.wall(), xyz))

    def project(self, xyz, origin=(730, 515), scale=17.0):
        x, y, z = xyz
        return (int(origin[0] + scale * (x - .55 * y)),
                int(origin[1] - scale * (z + .35 * y)))

    def render(self):
        with self.lock:
            camera = None if self.camera is None else self.camera.copy()
            points = list(self.points)
            centroids = list(self.centroids)
            tracks = [dict(v) for v in self.tracks]
            truth = self.truth
            tails = {k: list(v) for k, v in self.track_tails.items()}
            truth_tail = list(self.truth_tail)
            ros_now, ros_start = self.ros_now, self.ros_start
            empty_count, deleted, recovered = self.empty_count, self.deleted, self.recovered
            frames = sorted(self.frame_ids)
            now = self.wall()
            radar_hz = rate(self.radar_times, now)
            centroid_hz = rate(self.centroid_times, now)
            track_hz = rate(self.track_times, now)
        frame = np.full((HEIGHT, WIDTH, 3), (20, 25, 34), np.uint8)
        cv2.rectangle(frame, (20, 75), (500, 435), (70, 80, 95), 1)
        if camera is not None:
            view = cv2.resize(camera, (480, 360), interpolation=cv2.INTER_LINEAR)
            frame[75:435, 20:500] = view
        else:
            cv2.putText(frame, "WAITING FOR CAMERA", (110, 255), cv2.FONT_HERSHEY_SIMPLEX,
                        .8, (80, 80, 255), 2, cv2.LINE_AA)
        cv2.rectangle(frame, (520, 75), (965, 635), (75, 85, 100), 1)
        # Isometric world projection: points, truth, centroids, tracks, and tails.
        for xyz in points:
            p = self.project(xyz)
            if 525 <= p[0] <= 960 and 80 <= p[1] <= 630:
                cv2.circle(frame, p, 1, (120, 120, 135), -1)
        axes = [((0, 0, 0), (5, 0, 0), (80, 100, 255), "X"),
                ((0, 0, 0), (0, 5, 0), (80, 255, 120), "Y"),
                ((0, 0, 0), (0, 0, 5), (255, 170, 80), "Z")]
        for start, end, color, label in axes:
            a, b = self.project(start), self.project(end)
            cv2.arrowedLine(frame, a, b, color, 2, tipLength=.08)
            cv2.putText(frame, label, b, cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
        if len(truth_tail) > 1:
            pts = np.asarray([self.project(row[1]) for row in truth_tail], np.int32)
            cv2.polylines(frame, [pts], False, (0, 165, 255), 2)
        if truth is not None:
            cv2.circle(frame, self.project(truth), 8, (0, 165, 255), -1)
        for xyz in centroids:
            cv2.drawMarker(frame, self.project(xyz), (50, 60, 255), cv2.MARKER_CROSS, 16, 2)
        palette = [(80, 255, 80), (255, 220, 80), (255, 100, 220)]
        for index, (ident, rows) in enumerate(sorted(tails.items())):
            if len(rows) > 1:
                pts = np.asarray([self.project(row[1]) for row in rows], np.int32)
                cv2.polylines(frame, [pts], False, palette[index % len(palette)], 2)
        for index, item in enumerate(tracks):
            p = self.project(item["position"])
            cv2.circle(frame, p, 7, palette[index % len(palette)], -1)
            cv2.putText(frame, "ID%d age%d" % (item["id"], item["age"]),
                        (p[0] + 8, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, .48,
                        palette[index % len(palette)], 1, cv2.LINE_AA)
        error = None
        if truth is not None and tracks:
            error = min(float(np.linalg.norm(np.asarray(t["position"]) - truth)) for t in tracks)
        if recovered:
            phase, color = "RECOVERY / NEW ID", (80, 255, 140)
        elif deleted:
            phase, color = "TRACK DELETED AFTER EMPTY INPUT", (80, 80, 255)
        elif empty_count:
            phase, color = "EMPTY INPUT %d/%d" % (min(empty_count, self.args.empty_count), self.args.empty_count), (0, 210, 255)
        else:
            phase, color = "TRACKING", (80, 255, 140)
        ros_elapsed = None if ros_now is None or ros_start is None else ros_now - ros_start
        rtf = 0.0 if ros_elapsed is None or now <= 0 else ros_elapsed / now
        labels = [
            ("LIVE GAZEBO", (20, 12), 32, (60, 80, 255)),
            ("实时 M0 → 真实 M1 → 真实 M2（非实体传感器）", (220, 15), 27, (80, 230, 255)),
            ("M0 camera", (30, 82), 18, (230, 230, 230)),
            ("world point cloud / centroid / filtered track", (535, 83), 18, (230, 230, 230)),
            (phase, (980, 90), 24, color),
            ("wall %.2f s" % now, (985, 145), 20, (235, 235, 235)),
            ("ROS sim %s s" % ("--" if ros_now is None else "%.2f" % ros_now), (985, 178), 20, (235, 235, 235)),
            ("RTF %.3f" % rtf, (985, 211), 20, (235, 235, 235)),
            ("radar %.2f Hz wall" % radar_hz, (985, 260), 19, (210, 210, 210)),
            ("M1 %.2f Hz wall" % centroid_hz, (985, 292), 19, (80, 100, 255)),
            ("M2 %.2f Hz wall" % track_hz, (985, 324), 19, (80, 255, 120)),
            ("frame_id: %s" % (",".join(frames) or "--"), (985, 370), 18, (220, 220, 220)),
            ("ID/age: %s" % (", ".join("%d/%d" % (v["id"], v["age"]) for v in tracks) or "--"), (985, 405), 18, (220, 220, 220)),
            ("instant error: %s m" % ("--" if error is None else "%.3f" % error), (985, 440), 18, (220, 220, 220)),
            ("empty relay: %d" % empty_count, (985, 475), 18, (220, 220, 220)),
            ("orange truth | red centroid | green track", (535, 650), 18, (220, 220, 220)),
            ("develop " + self.args.develop_sha[:12], (20, 682), 18, (190, 190, 190)),
        ]
        return add_texts(frame, labels)

    def run(self):
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        capture_path = self.args.output.with_suffix(".live_capture.avi")
        writer = cv2.VideoWriter(str(capture_path), cv2.VideoWriter_fourcc(*"MJPG"),
                                 FPS, (WIDTH, HEIGHT))
        if not writer.isOpened():
            raise RuntimeError("OpenCV could not open MJPEG live-capture writer")
        count = 0
        last_frame = None
        capture_started = time.monotonic()
        try:
            while not rospy.is_shutdown() and self.wall() < self.args.duration:
                frame = self.render()
                last_frame = frame
                target = max(count + 1, int(self.wall() * FPS))
                while count < target:
                    writer.write(frame)
                    count += 1
                time.sleep(.01)
            target = int(self.args.duration * FPS)
            while last_frame is not None and count < target:
                writer.write(last_frame)
                count += 1
        finally:
            writer.release()
        capture_duration = time.monotonic() - capture_started
        source = cv2.VideoCapture(str(capture_path))
        encoded = cv2.VideoWriter(str(self.args.output), cv2.VideoWriter_fourcc(*"avc1"),
                                  FPS, (WIDTH, HEIGHT))
        if not source.isOpened() or not encoded.isOpened():
            source.release(); encoded.release()
            raise RuntimeError("OpenCV could not open H.264 transcode pipeline")
        encoded_count = 0
        while True:
            ok, frame = source.read()
            if not ok:
                break
            encoded.write(frame); encoded_count += 1
        source.release(); encoded.release()
        if encoded_count != count:
            raise RuntimeError("H.264 transcode frame loss: %d != %d" % (encoded_count, count))
        capture_path.unlink()
        metadata = {
            "live": True, "source": "active ROS topics from live M0 Gazebo, real M1 and real M2",
            "video": str(self.args.output), "frames": count, "fps": FPS,
            "resolution": [WIDTH, HEIGHT], "capture_wall_duration_s": capture_duration,
            "video_duration_s": count / FPS, "codec": "H.264/avc1",
            "develop_sha": self.args.develop_sha, "run_dir": self.args.run_dir,
            "ros_master_uri": rospy.get_master().getUri(), "frame_ids": sorted(self.frame_ids),
            "empty_messages_seen": self.empty_count, "track_deleted": self.deleted,
            "recovered": self.recovered, "ids_before_fault": sorted(self.ids_before_fault),
            "ids_after_fault": sorted(self.ids_after_fault), "flight_controller_connected": False,
        }
        self.args.metadata.write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--empty-count", type=int, default=6)
    parser.add_argument("--develop-sha", required=True)
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    rospy.init_node("m1_m2_live_video_recorder", anonymous=True, disable_signals=True)
    Recorder(args).run()


if __name__ == "__main__":
    main()
