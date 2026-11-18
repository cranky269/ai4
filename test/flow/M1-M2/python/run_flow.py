#!/usr/bin/env python3
"""Record and visualize the real M1 -> M2 ROS flow."""
import argparse
import json
import math
import os
import time
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rospy
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - registers 3D projection
from geometry_msgs.msg import PoseStamped
from uav_tracking.msg import CentroidArray, TrackArray

DEVELOP_SHA = os.environ.get("AI4_DEVELOP_SHA", "431c049408f323580a351d6e3532daba8c6742f5")


class Audit:
    def __init__(self, duration, fault_start, empty_count):
        self.duration = duration
        self.fault_start = fault_start
        self.empty_count = empty_count
        self.start = time.monotonic()
        self.source_times, self.centroid_times, self.track_times = [], [], []
        self.centroid_history, self.truth_history = [], []
        self.relay_events, self.centroid_frames, self.track_frames = [], [], []
        self.track_history = defaultdict(list)
        self.latest_truth = None
        self.errors = []
        self.rejections = []
        self.frames = []
        self.age_regressions = 0
        self.last_age = {}
        self.id_sequences = []
        self.empty_streak = 0
        self.max_empty_streak = 0
        self.deleted_on_sixth = False
        self.recovered_after_empty = False
        self.seen_empty = False
        self.was_empty = False
        self.injected_empty_count = 0
        self.relay_pub = rospy.Publisher("/test/radar_centroid", CentroidArray, queue_size=20)
        rospy.Subscriber("/target/ground_truth/pose", PoseStamped, self.on_truth, queue_size=20)
        rospy.Subscriber("/radar/centroid", CentroidArray, self.on_source, queue_size=50)
        rospy.Subscriber("/test/radar_centroid", CentroidArray, self.on_centroids, queue_size=50)
        rospy.Subscriber("/radar/tracks", TrackArray, self.on_tracks, queue_size=50)

    def on_source(self, msg):
        self.source_times.append(time.monotonic())
        for item in msg.centroids:
            if item.header.frame_id != "world":
                self.reject("source centroid frame_id=%r, expected world" % item.header.frame_id)
                return
        now = self.elapsed()
        injected = (now >= self.fault_start
                    and self.injected_empty_count < self.empty_count)
        self.relay_events.append({
            "time": now, "injected_empty": injected,
            "source_centroid_count": len(msg.centroids),
        })
        if injected:
            self.injected_empty_count += 1
            self.relay_pub.publish(CentroidArray())
        else:
            self.relay_pub.publish(msg)

    def elapsed(self):
        return time.monotonic() - self.start

    def reject(self, reason):
        if reason not in self.rejections:
            self.rejections.append(reason)
        rospy.logerr("DATA REJECTED: %s", reason)

    def on_truth(self, msg):
        if msg.header.frame_id != "world":
            self.reject("truth frame_id=%r, expected world" % msg.header.frame_id)
            return
        p = msg.pose.position
        xyz = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(v) for v in xyz):
            self.reject("truth contains non-finite values")
            return
        self.latest_truth = xyz
        self.truth_history.append((self.elapsed(),) + xyz)

    def on_centroids(self, msg):
        now = self.elapsed()
        points = []
        if msg.centroids:
            frames = {p.header.frame_id for p in msg.centroids}
            stamps = {p.header.stamp.to_nsec() for p in msg.centroids}
            if frames != {"world"}:
                self.reject("centroid frame set=%r, expected {'world'}" % sorted(frames))
                return
            if len(stamps) != 1:
                self.reject("centroid array mixes acquisition timestamps")
                return
            self.frames.extend(frames)
            for item in msg.centroids:
                xyz = (float(item.point.x), float(item.point.y), float(item.point.z))
                if not all(math.isfinite(v) for v in xyz):
                    self.reject("centroid contains non-finite values")
                    return
                points.append(xyz)
                self.centroid_history.append((now,) + xyz)
            if self.was_empty:
                self.recovered_after_empty = True
            self.empty_streak = 0
            self.was_empty = False
        else:
            self.empty_streak += 1
            self.max_empty_streak = max(self.max_empty_streak, self.empty_streak)
            self.seen_empty = True
            self.was_empty = True
        self.centroid_times.append(time.monotonic())
        self.frames.append("empty" if not points else "world")
        self.centroid_frames.append({
            "time": now, "points": [list(point) for point in points],
            "empty": not points, "empty_streak": self.empty_streak,
        })

    def on_tracks(self, msg):
        now = self.elapsed()
        self.track_times.append(time.monotonic())
        ids = []
        frame_tracks = []
        for item in msg.tracks:
            xyz = tuple(float(v) for v in item.position)
            vel = tuple(float(v) for v in item.velocity)
            if not all(math.isfinite(v) for v in xyz + vel):
                self.reject("Track contains non-finite position/velocity")
                return
            ident, age = int(item.id), int(item.age)
            if ident in self.last_age and age < self.last_age[ident]:
                self.age_regressions += 1
            self.last_age[ident] = age
            ids.append(ident)
            self.track_history[ident].append((now,) + xyz + vel + (age,))
            frame_tracks.append({
                "id": ident, "age": age,
                "position": list(xyz), "velocity": list(vel),
            })
        self.id_sequences.append(ids)
        if (self.injected_empty_count >= self.empty_count
                and not msg.tracks):
            self.deleted_on_sixth = True
        if self.latest_truth is not None and msg.tracks:
            truth = np.asarray(self.latest_truth)
            self.errors.append(min(float(np.linalg.norm(np.asarray(t.position) - truth)) for t in msg.tracks))
        frame_error = None
        if self.latest_truth is not None and frame_tracks:
            frame_error = min(float(np.linalg.norm(
                np.asarray(track["position"]) - np.asarray(self.latest_truth)))
                for track in frame_tracks)
        self.track_frames.append({
            "time": now, "tracks": frame_tracks,
            "nearest_truth_error_m": frame_error,
        })

    @staticmethod
    def rate(times):
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0

    def finish(self, output):
        output.mkdir(parents=True, exist_ok=True)
        stable_ids = [ids[0] for ids in self.id_sequences if ids]
        switches = sum(a != b for a, b in zip(stable_ids, stable_ids[1:]))
        summary = {
            "develop_sha": DEVELOP_SHA,
            "duration_wall_s": self.elapsed(),
            "source_centroid_messages": len(self.source_times),
            "centroid_messages": len(self.centroid_times),
            "track_messages": len(self.track_times),
            "centroid_rate_hz": self.rate(self.centroid_times),
            "track_event_rate_hz": self.rate(self.track_times),
            "tracker_event_driven": abs(len(self.centroid_times) - len(self.track_times)) <= 2,
            "frames_seen": sorted(set(self.frames)),
            "data_rejected": self.rejections,
            "ids_seen": sorted(self.track_history),
            "primary_id_switches": switches,
            "age_regressions": self.age_regressions,
            "empty_frames_seen": self.seen_empty,
            "injected_empty_frames": self.injected_empty_count,
            "max_consecutive_empty_frames": self.max_empty_streak,
            "deleted_on_or_after_sixth_empty": self.deleted_on_sixth,
            "recovered_after_empty": self.recovered_after_empty,
            "position_error_mean_m": float(np.mean(self.errors)) if self.errors else None,
            "position_error_max_m": float(np.max(self.errors)) if self.errors else None,
        }
        summary["pass"] = bool(
            not self.rejections and self.centroid_times and self.track_times
            and "world" in self.frames and self.track_history and self.age_regressions == 0
            and summary["tracker_event_driven"] and self.errors
            and self.injected_empty_count == self.empty_count
            and self.deleted_on_sixth and self.recovered_after_empty
        )
        (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        timeline = {
            "develop_sha": DEVELOP_SHA,
            "data_source": "live Gazebo M0 + real M1 cluster + real M2 tracker",
            "duration_wall_s": summary["duration_wall_s"],
            "fault_start_s": self.fault_start,
            "configured_empty_frames": self.empty_count,
            "relay_events": self.relay_events,
            "centroid_frames": self.centroid_frames,
            "track_frames": self.track_frames,
            "truth_samples": [
                {"time": row[0], "position": list(row[1:4])}
                for row in self.truth_history
            ],
            "summary": summary,
        }
        (output / "timeline.json").write_text(
            json.dumps(timeline, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        self.plot(output / "m1_m2_real.png", summary)
        print(json.dumps(summary, ensure_ascii=False))
        return 0 if summary["pass"] else 2

    def plot(self, path, summary):
        fig = plt.figure(figsize=(13, 8))
        ax = fig.add_subplot(121, projection="3d")
        if self.centroid_history:
            a = np.asarray(self.centroid_history)
            ax.scatter(a[:, 1], a[:, 2], a[:, 3], s=8, alpha=.25, c="tab:red", label="M1 centroid")
        if self.truth_history:
            a = np.asarray(self.truth_history)
            ax.plot(a[:, 1], a[:, 2], a[:, 3], c="orange", lw=2, label="truth")
        for ident, rows in sorted(self.track_history.items()):
            a = np.asarray(rows)
            ax.plot(a[:, 1], a[:, 2], a[:, 3], lw=1.5, label="track id=%d" % ident)
            ax.scatter(a[-1, 1], a[-1, 2], a[-1, 3], s=35)
        ax.set(xlabel="world X (m)", ylabel="world Y (m)", zlabel="world Z (m)")
        ax.set_title("M1 centroids, M2 filtered tracks and truth")
        ax.legend(loc="best", fontsize=8)
        info = fig.add_subplot(122); info.axis("off")
        title = "DATA REJECTED" if self.rejections else ("PASS" if summary["pass"] else "INCOMPLETE")
        lines = [title, "", "input/output: %d / %d" % (summary["centroid_messages"], summary["track_messages"]),
                 "event rates: %.2f / %.2f Hz" % (summary["centroid_rate_hz"], summary["track_event_rate_hz"]),
                 "IDs: %s; switches=%d" % (summary["ids_seen"], summary["primary_id_switches"]),
                 "age regressions: %d" % summary["age_regressions"],
                 "error mean/max: %s / %s m" % (summary["position_error_mean_m"], summary["position_error_max_m"]),
                 "max empty run: %d; delete6=%s; recovery=%s" % (summary["max_consecutive_empty_frames"], summary["deleted_on_or_after_sixth_empty"], summary["recovered_after_empty"])]
        lines.extend(self.rejections)
        info.text(.02, .95, "\n".join(lines), va="top", family="monospace", fontsize=11,
                  color="red" if self.rejections else "black")
        fig.tight_layout(); fig.savefig(str(path), dpi=150); plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=45.0)
    p.add_argument("--fault-start", type=float, default=15.0)
    p.add_argument("--empty-count", type=int, default=6)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    rospy.init_node("m1_m2_receiver_audit", anonymous=True)
    audit = Audit(args.duration, args.fault_start, args.empty_count)
    deadline = time.monotonic() + args.duration
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(.05)
    raise SystemExit(audit.finish(args.output))


if __name__ == "__main__":
    main()
