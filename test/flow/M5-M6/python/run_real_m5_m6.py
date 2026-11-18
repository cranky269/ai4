#!/usr/bin/env python3
"""受控 Detections -> 真实 M5 -> 真实 M6 的隔离小联调。

本脚本只创建独立 ROS master，并只启动 visual_tracker_node.py 与
pid_controller_node.py。输出固定重映射到 /test/ctrl_visual；不启动或连接
control_mux、state_machine、MAVROS、PX4、OFFBOARD。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosgraph
import rosnode
import rospy
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import Image
from uav_tracking.msg import DetectedTarget, Detections, VisualTrackArray


DEVELOP_SHA = "431c049408f323580a351d6e3532daba8c6742f5"
WIDTH = 320
HEIGHT = 240
IMAGE_AREA = WIDTH * HEIGHT
EXPECTED_FRAME = 9
EXPECTED_MASK = 1479


PHASES = [
    ("WARMUP", 0.60, None),
    ("CENTER", 1.40, (112.0, 80.0, 96.0, 80.0)),
    ("CLEAR_CENTER", 0.35, "empty"),
    ("RIGHT_DOWN_FAR", 1.40, (260.0, 180.0, 40.0, 30.0)),
    ("CLEAR_FAR", 0.35, "empty"),
    ("TERMINAL_035", 1.40, (76.0, 40.0, 168.0, 160.0)),
    ("CLEAR_TERMINAL", 0.35, "empty"),
    ("BELOW_03499", 1.20, (76.024, 40.0, 167.952, 160.0)),
    ("CLEAR_BELOW", 0.35, "empty"),
    ("ABOVE_03501", 1.20, (75.976, 40.0, 168.048, 160.0)),
    ("CLEAR_ABOVE", 0.35, "empty"),
    ("PRE_TIMEOUT_FAR", 1.00, (260.0, 180.0, 40.0, 30.0)),
    ("EMPTY", 0.80, "empty"),
    ("SILENCE", 0.80, None),
    ("RECOVERY", 1.20, (260.0, 180.0, 40.0, 30.0)),
]


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def json_line(value) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


def rate_hz(stamps) -> float:
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def all_zero(command, tolerance=1e-6) -> bool:
    return all(abs(float(command[key])) <= tolerance
               for key in ("vx", "vy", "vz", "yaw_rate"))


class FlowAudit:
    def __init__(self, run_dir: Path, image_dir: Path, expected_nodes: set[str]):
        self.run_dir = run_dir
        self.image_dir = image_dir
        self.expected_nodes = expected_nodes
        self.started = time.monotonic()
        self.phase = "SETUP"
        self.phase_started = self.started
        self.input_events = []
        self.track_events = []
        self.command_events = []
        self.rejections = []
        self.last_nonempty_track_time = None
        self.last_detection_bbox = {}
        self.publisher = rospy.Publisher(
            "/test/detections", Detections, queue_size=10)
        self.image_publisher = rospy.Publisher(
            "/test/camera/image_raw", Image, queue_size=2)
        self.track_subscriber = rospy.Subscriber(
            "/test/visual_tracks", VisualTrackArray, self.on_tracks, queue_size=100)
        self.command_subscriber = rospy.Subscriber(
            "/test/ctrl_visual", PositionTarget, self.on_command, queue_size=300)

    def elapsed(self) -> float:
        return time.monotonic() - self.started

    def phase_elapsed(self) -> float:
        return time.monotonic() - self.phase_started

    def reject(self, reason: str) -> None:
        if reason not in self.rejections:
            self.rejections.append(reason)
        rospy.logerr("DATA REJECTED: %s", reason)

    def on_tracks(self, msg: VisualTrackArray) -> None:
        now = self.elapsed()
        tracks = []
        for item in msg.tracks:
            bbox = [float(value) for value in item.bbox]
            area = float(item.area_ratio)
            if len(bbox) != 4 or not all(math.isfinite(v) for v in bbox + [area]):
                self.reject("VisualTrack contains an invalid bbox or area_ratio")
                return
            if bbox[2] < 0.0 or bbox[3] < 0.0 or area < 0.0:
                self.reject("VisualTrack contains a negative size or area_ratio")
                return
            tracks.append({"id": int(item.id), "bbox": bbox, "area_ratio": area})
        if tracks:
            self.last_nonempty_track_time = now
        self.track_events.append({
            "time": now, "phase": self.phase, "tracks": tracks,
        })

    def on_command(self, msg: PositionTarget) -> None:
        now = self.elapsed()
        values = {
            "vx": float(msg.velocity.x), "vy": float(msg.velocity.y),
            "vz": float(msg.velocity.z), "yaw_rate": float(msg.yaw_rate),
        }
        if not all(math.isfinite(value) for value in values.values()):
            self.reject("PositionTarget contains a non-finite value")
            return
        if self.phase == "EMPTY":
            age = math.inf if self.last_nonempty_track_time is None else now - self.last_nonempty_track_time
            state = "HOLD" if age <= 0.2 else "STOP"
        elif self.phase == "SILENCE":
            state = "STOP"
        elif self.phase == "RECOVERY":
            state = "RECOVERY"
        elif self.phase.startswith("CLEAR") or self.phase in ("SETUP", "WARMUP"):
            state = "EMPTY"
        else:
            state = "TRACK"
        self.command_events.append({
            "time": now, "phase": self.phase, "state": state,
            "frame": int(msg.coordinate_frame), "mask": int(msg.type_mask),
            **values,
        })

    def set_phase(self, name: str) -> None:
        self.phase = name
        self.phase_started = time.monotonic()

    def make_image(self, bbox) -> Image:
        array = np.full((HEIGHT, WIDTH, 3), 32, dtype=np.uint8)
        array[:, :, 1] = 45
        if isinstance(bbox, tuple):
            x, y, w, h = bbox
            x0, y0 = max(0, int(x)), max(0, int(y))
            x1, y1 = min(WIDTH - 1, int(x + w)), min(HEIGHT - 1, int(y + h))
            array[y0:min(y0 + 2, HEIGHT), x0:x1 + 1] = (0, 0, 255)
            array[max(0, y1 - 1):y1 + 1, x0:x1 + 1] = (0, 0, 255)
            array[y0:y1 + 1, x0:min(x0 + 2, WIDTH)] = (0, 0, 255)
            array[y0:y1 + 1, max(0, x1 - 1):x1 + 1] = (0, 0, 255)
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "controlled_camera"
        msg.height = HEIGHT
        msg.width = WIDTH
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = WIDTH * 3
        msg.data = array.tobytes()
        return msg

    def publish_detection(self, bbox) -> None:
        msg = Detections()
        targets = []
        if isinstance(bbox, tuple):
            target = DetectedTarget()
            target.class_id = 0
            target.confidence = 0.95
            target.bbox = list(bbox)
            msg.targets.append(target)
            targets.append({
                "class_id": 0, "confidence": float(target.confidence),
                "bbox": [float(v) for v in target.bbox],
                "mathematical_area_ratio": float(target.bbox[2] * target.bbox[3] / IMAGE_AREA),
            })
            self.last_detection_bbox[self.phase] = targets[0]["bbox"]
        self.publisher.publish(msg)
        self.input_events.append({
            "time": self.elapsed(), "phase": self.phase, "targets": targets,
        })

    def check_output_subscribers(self) -> None:
        _publishers, subscribers, _services = rosgraph.Master(rospy.get_name()).getSystemState()
        nodes = set(dict(subscribers).get("/test/ctrl_visual", []))
        allowed = {rospy.get_name()}
        unexpected = sorted(nodes - allowed)
        if unexpected:
            self.reject("unexpected /test/ctrl_visual subscribers: %r" % unexpected)
        if any("mux" in name.lower() or "mavros" in name.lower() for name in nodes):
            self.reject("control_mux or MAVROS subscribed to isolated output: %r" % sorted(nodes))

    def wait_for_connections(self, timeout=8.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if (self.publisher.get_num_connections() >= 1
                    and self.image_publisher.get_num_connections() >= 1
                    and self.track_subscriber.get_num_connections() >= 1
                    and self.command_subscriber.get_num_connections() >= 1):
                return
            time.sleep(0.05)
        raise RuntimeError("timed out waiting for M5/M6 topic connections")

    def run_phases(self) -> None:
        self.wait_for_connections()
        self.check_output_subscribers()
        for name, duration, payload in PHASES:
            self.set_phase(name)
            deadline = time.monotonic() + duration
            next_publish = 0.0
            next_image = 0.0
            while time.monotonic() < deadline and not rospy.is_shutdown():
                phase_time = self.phase_elapsed()
                if phase_time >= next_image:
                    self.image_publisher.publish(
                        self.make_image(payload if isinstance(payload, tuple) else None))
                    next_image += 0.10
                if payload is not None and phase_time >= next_publish:
                    self.publish_detection(payload)
                    next_publish += 0.05
                if int(self.elapsed() * 10) % 10 == 0:
                    self.check_output_subscribers()
                time.sleep(0.005)
        time.sleep(0.15)
        self.check_output_subscribers()

    def freeze_callbacks(self) -> None:
        """在保存过话题图后冻结样本，避免绘图时回调继续扩展列表。"""
        self.track_subscriber.unregister()
        self.command_subscriber.unregister()
        time.sleep(0.05)

    def stable_tracks(self, phase: str):
        phase_rows = [row for row in self.track_events
                      if row["phase"] == phase and row["tracks"]]
        return phase_rows[len(phase_rows) // 3:] if phase_rows else []

    def stable_commands(self, phase: str):
        phase_rows = [row for row in self.command_events if row["phase"] == phase]
        return phase_rows[len(phase_rows) // 3:] if phase_rows else []

    def topic_graph(self):
        master = rosgraph.Master(rospy.get_name())
        publishers, subscribers, services = master.getSystemState()
        types = dict(master.getPublishedTopics(""))
        return {
            "nodes": sorted(rosnode.get_node_names()),
            "publishers": {topic: sorted(nodes) for topic, nodes in publishers},
            "subscribers": {topic: sorted(nodes) for topic, nodes in subscribers},
            "services": {name: sorted(nodes) for name, nodes in services},
            "types": types,
        }

    def write_raw_logs(self, graph) -> None:
        def write_jsonl(name, rows):
            text = "".join(json_line(row) + "\n" for row in rows)
            (self.run_dir / name).write_text(text, encoding="utf-8")

        write_jsonl("sample_detections.log", self.input_events)
        write_jsonl("sample_visual_tracks.log", self.track_events)
        write_jsonl("sample_ctrl_visual.log", self.command_events)
        (self.run_dir / "node_list.log").write_text(
            "\n".join(graph["nodes"]) + "\n", encoding="utf-8")
        (self.run_dir / "topic_graph.json").write_text(
            json.dumps(graph, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8")
        relevant = [
            "/test/detections", "/test/camera/image_raw",
            "/test/visual_tracks", "/test/ctrl_visual",
            "/ctrl/visual", "/mavros/setpoint_raw/local",
        ]
        topic_lines = []
        for topic in relevant:
            topic_lines.extend([
                "TOPIC " + topic,
                "  type: " + graph["types"].get(topic, "<not published>"),
                "  publishers: " + repr(graph["publishers"].get(topic, [])),
                "  subscribers: " + repr(graph["subscribers"].get(topic, [])),
            ])
        (self.run_dir / "topic_info.log").write_text(
            "\n".join(topic_lines) + "\n", encoding="utf-8")
        (self.run_dir / "message_types.log").write_text(
            "\n".join("%s %s" % (topic, graph["types"].get(topic, "<not published>"))
                      for topic in relevant) + "\n", encoding="utf-8")

        rows = []
        for event in self.input_events:
            rows.append({"kind": "detections", **event})
        for event in self.track_events:
            rows.append({"kind": "visual_tracks", **event})
        for event in self.command_events:
            rows.append({"kind": "ctrl_visual", **event})
        rows.sort(key=lambda row: row["time"])
        fieldnames = [
            "time", "kind", "phase", "state", "frame", "mask",
            "vx", "vy", "vz", "yaw_rate", "payload",
        ]
        with (self.run_dir / "timeline.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow({
                    "time": "%.6f" % row["time"], "kind": row["kind"],
                    "phase": row["phase"], "state": row.get("state", ""),
                    "frame": row.get("frame", ""), "mask": row.get("mask", ""),
                    "vx": row.get("vx", ""), "vy": row.get("vy", ""),
                    "vz": row.get("vz", ""), "yaw_rate": row.get("yaw_rate", ""),
                    "payload": json_line(row.get("targets", row.get("tracks", []))),
                })

    def evaluate(self, graph):
        phase_tracks = {name: self.stable_tracks(name) for name, _duration, _payload in PHASES}
        phase_commands = {name: self.stable_commands(name) for name, _duration, _payload in PHASES}

        def track_values(phase):
            return [track for row in phase_tracks[phase] for track in row["tracks"]]

        def commands(phase):
            return phase_commands[phase]

        boundary_areas = [t["area_ratio"] for t in track_values("TERMINAL_035")]
        below_areas = [t["area_ratio"] for t in track_values("BELOW_03499")]
        above_areas = [t["area_ratio"] for t in track_values("ABOVE_03501")]
        empty_tracks = [row for row in self.track_events if row["phase"] == "EMPTY"]
        silence_tracks = [row for row in self.track_events if row["phase"] == "SILENCE"]
        empty_commands = [row for row in self.command_events if row["phase"] == "EMPTY"]
        silence_commands = [row for row in self.command_events if row["phase"] == "SILENCE"]
        hold_commands = [row for row in empty_commands if row["state"] == "HOLD"]
        stop_commands = [row for row in empty_commands if row["state"] == "STOP"]
        last_valid_before_empty = max(
            (row["time"] for row in self.track_events
             if row["phase"] == "PRE_TIMEOUT_FAR" and row["tracks"]),
            default=None,
        )
        first_stop_time = min(
            (row["time"] for row in stop_commands), default=None)
        observed_stop_latency = (
            first_stop_time - last_valid_before_empty
            if first_stop_time is not None and last_valid_before_empty is not None
            else None
        )
        all_commands = self.command_events
        input_times = [row["time"] for row in self.input_events]
        track_times = [row["time"] for row in self.track_events]
        command_times = [row["time"] for row in all_commands]
        detection_phases = [name for name, _duration, payload in PHASES
                            if isinstance(payload, tuple)]
        input_phase_rates = {
            phase: rate_hz([row["time"] for row in self.input_events if row["phase"] == phase])
            for phase in detection_phases
        }
        m5_phase_rates = {
            phase: rate_hz([row["time"] for row in self.track_events if row["phase"] == phase])
            for phase in detection_phases
        }
        ids_by_phase = {
            phase: sorted({track["id"] for track in track_values(phase)})
            for phase in (
                "CENTER", "RIGHT_DOWN_FAR", "TERMINAL_035",
                "BELOW_03499", "ABOVE_03501", "PRE_TIMEOUT_FAR", "RECOVERY",
            )
        }
        checks = {
            "real_m5_node_present": any("m5_flow_visual_tracker" in node for node in graph["nodes"]),
            "real_m6_node_present": any("m6_flow_pid_controller" in node for node in graph["nodes"]),
            "detections_type": graph["types"].get("/test/detections") == "uav_tracking/Detections",
            "visual_tracks_type": graph["types"].get("/test/visual_tracks") == "uav_tracking/VisualTrackArray",
            "ctrl_visual_type": graph["types"].get("/test/ctrl_visual") == "mavros_msgs/PositionTarget",
            "no_real_control_publisher": not graph["publishers"].get("/mavros/setpoint_raw/local"),
            "no_unremapped_ctrl_visual_publisher": not graph["publishers"].get("/ctrl/visual"),
            "center_track_created": bool(track_values("CENTER")),
            "right_down_far_track_created": bool(track_values("RIGHT_DOWN_FAR")),
            "bbox_xywh_semantics": bool(track_values("RIGHT_DOWN_FAR")) and all(
                len(track["bbox"]) == 4 and track["bbox"][2] > 0 and track["bbox"][3] > 0
                for track in track_values("RIGHT_DOWN_FAR")),
            "area_ratio_computed_by_m5": all(
                abs(track["area_ratio"] - track["bbox"][2] * track["bbox"][3] / IMAGE_AREA) < 2e-5
                for phase in ("CENTER", "RIGHT_DOWN_FAR", "TERMINAL_035", "BELOW_03499", "ABOVE_03501")
                for track in track_values(phase)) and bool(boundary_areas),
            "id_stable_within_each_phase": all(len(ids) == 1 for ids in ids_by_phase.values()),
            "empty_detections_produce_empty_tracks": bool(empty_tracks) and all(not row["tracks"] for row in empty_tracks),
            "detection_silence_produces_no_m5_events": not silence_tracks,
            "frame_9": bool(all_commands) and {row["frame"] for row in all_commands} == {EXPECTED_FRAME},
            "mask_1479": bool(all_commands) and {row["mask"] for row in all_commands} == {EXPECTED_MASK},
            "vy_zero": bool(all_commands) and all(abs(row["vy"]) <= 1e-12 for row in all_commands),
            "m6_rate_near_50hz": 45.0 <= rate_hz(command_times) <= 55.0,
            "right_yaw_negative": any(row["yaw_rate"] < -1e-6 for row in commands("RIGHT_DOWN_FAR")),
            "down_vz_negative": any(row["vz"] < -1e-6 for row in commands("RIGHT_DOWN_FAR")),
            "far_vx_positive": any(row["vx"] > 1e-6 for row in commands("RIGHT_DOWN_FAR")),
            "float32_035_observed": bool(boundary_areas) and any(
                abs(area - 0.3499999940395355) <= 1e-9 for area in boundary_areas),
            "terminal_035_zero": bool(commands("TERMINAL_035")) and all(
                abs(row["vx"]) <= 1e-6 for row in commands("TERMINAL_035")),
            "below_03499_not_terminal": bool(below_areas) and max(below_areas) < 0.35 - 1e-6
                and any(row["vx"] < -1e-6 for row in commands("BELOW_03499")),
            "above_03501_terminal": bool(above_areas) and min(above_areas) > 0.35
                and all(abs(row["vx"]) <= 1e-6 for row in commands("ABOVE_03501")),
            "empty_has_hold": bool(hold_commands),
            "empty_stops_after_0_2": bool(stop_commands) and all(all_zero(row) for row in stop_commands),
            "stop_latency_matches_0_2_seconds": observed_stop_latency is not None
                and 0.18 <= observed_stop_latency <= 0.24,
            "silence_stays_stopped": bool(silence_commands) and all(all_zero(row) for row in silence_commands),
            "recovery_tracks": bool(track_values("RECOVERY")),
            "recovery_restores_control": any(
                row["vx"] > 1e-6 and row["vz"] < -1e-6 and row["yaw_rate"] < -1e-6
                for row in commands("RECOVERY")),
            "isolated_output_has_only_auditor_subscriber": not self.rejections,
        }
        frequencies = {
            "controlled_detections_overall_hz": rate_hz(input_times),
            "real_m5_visual_tracks_overall_hz": rate_hz(track_times),
            "controlled_detections_active_phase_mean_hz": float(np.mean(list(input_phase_rates.values()))),
            "real_m5_visual_tracks_active_phase_mean_hz": float(np.mean(list(m5_phase_rates.values()))),
            "real_m6_ctrl_visual_hz": rate_hz(command_times),
            "controlled_image_nominal_hz": 10.0,
            "controlled_detections_by_phase_hz": input_phase_rates,
            "real_m5_visual_tracks_by_phase_hz": m5_phase_rates,
        }
        summary = {
            "develop_sha": DEVELOP_SHA,
            "result": "PASS" if all(checks.values()) else "FAIL",
            "real_m5_used": True,
            "real_m6_used": True,
            "real_detector_m4_used": False,
            "detections_source": "controlled",
            "image_source": "controlled compliant sensor_msgs/Image",
            "image_width": WIDTH,
            "image_height": HEIGHT,
            "m5_m6_interface_result": "PASS" if all(checks[key] for key in (
                "real_m5_node_present", "real_m6_node_present", "detections_type",
                "visual_tracks_type", "ctrl_visual_type", "area_ratio_computed_by_m5",
            )) else "FAIL",
            "m6_float32_boundary_result": "PASS" if all(checks[key] for key in (
                "float32_035_observed", "terminal_035_zero",
                "below_03499_not_terminal", "above_03501_terminal",
            )) else "FAIL",
            "live_m4_chain_result": "NOT_REPRODUCED: M4 dependencies and accepted model assets are not deployed on this machine",
            "checks": checks,
            "frequencies": frequencies,
            "counts": {
                "detections": len(self.input_events),
                "visual_track_arrays": len(self.track_events),
                "ctrl_visual": len(self.command_events),
                "empty_visual_track_arrays": sum(not row["tracks"] for row in self.track_events),
            },
            "ids_by_phase": ids_by_phase,
            "area_ratio_samples": {
                "terminal_035": boundary_areas[-5:],
                "below_03499": below_areas[-5:],
                "above_03501": above_areas[-5:],
            },
            "timeout_observation": {
                "last_nonempty_visual_track_time": last_valid_before_empty,
                "first_zero_stop_time": first_stop_time,
                "observed_seconds": observed_stop_latency,
                "configured_seconds": 0.2,
            },
            "coordinate_frames": sorted({row["frame"] for row in all_commands}),
            "type_masks": sorted({row["mask"] for row in all_commands}),
            "data_rejected": self.rejections,
            "coordinate_note": "M6 supplies ROS body FLU values; it performs no manual FLU->FRD or ENU->NED conversion",
            "conclusion": (
                "M5→M6 小联调 PASS：使用受控 Detections 驱动真实 M5，真实 M5 输出 "
                "/test/visual_tracks，真实 M6 正确响应。"
                if all(checks.values()) else
                "M5→M6 小联调 FAIL：详见 checks 和原始日志。"
            ),
        }
        (self.run_dir / "frequency.log").write_text(
            json.dumps(frequencies, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        (self.run_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return summary

    def representative(self, phase: str):
        tracks = self.stable_tracks(phase)
        commands = self.stable_commands(phase)
        track = tracks[-1]["tracks"][0] if tracks and tracks[-1]["tracks"] else None
        command = commands[-1] if commands else None
        return track, command

    def plot_phase(self, phase: str, filename: str, title: str) -> None:
        track, command = self.representative(phase)
        detection = self.last_detection_bbox.get(phase)
        fig, (view, values) = plt.subplots(1, 2, figsize=(11, 4.8))
        view.set_xlim(0, WIDTH)
        view.set_ylim(HEIGHT, 0)
        view.set_aspect("equal")
        view.set_facecolor("#202a32")
        view.scatter([WIDTH / 2], [HEIGHT / 2], marker="+", s=130,
                     color="cyan", label="image center")
        if detection:
            x, y, w, h = detection
            view.add_patch(plt.Rectangle((x, y), w, h, fill=False, linestyle="--",
                                         linewidth=2, edgecolor="orange", label="input Detection"))
        if track:
            x, y, w, h = track["bbox"]
            view.add_patch(plt.Rectangle((x, y), w, h, fill=False, linewidth=2,
                                         edgecolor="lime", label="real M5 VisualTrack"))
            view.scatter([x + w / 2], [y + h / 2], color="red")
            view.text(5, 18, "id=%d  area_ratio=%.9f" % (track["id"], track["area_ratio"]),
                      color="white", bbox={"facecolor": "black", "alpha": 0.65})
        view.legend(loc="lower left", fontsize=8)
        view.set_title(title)
        labels = ["velocity.x", "velocity.y", "velocity.z", "yaw_rate"]
        vals = [command[key] if command else 0.0 for key in ("vx", "vy", "vz", "yaw_rate")]
        colors = ["#2ca02c" if value >= 0 else "#d62728" for value in vals]
        values.bar(labels, vals, color=colors)
        values.axhline(0.0, color="black", linewidth=0.8)
        values.grid(True, axis="y", alpha=0.3)
        values.set_title("real M6 output\nframe=%s mask=%s" % (
            command["frame"] if command else "?", command["mask"] if command else "?"))
        for index, value in enumerate(vals):
            values.text(index, value, "%+.4f" % value,
                        ha="center", va="bottom" if value >= 0 else "top")
        fig.tight_layout()
        fig.savefig(str(self.image_dir / filename), dpi=160)
        plt.close(fig)

    def plot_stop(self) -> None:
        rows = [row for row in self.command_events if row["phase"] in ("PRE_TIMEOUT_FAR", "EMPTY", "SILENCE")]
        fig, ax = plt.subplots(figsize=(11, 5))
        if rows:
            t0 = min(row["time"] for row in rows if row["phase"] == "EMPTY")
            last_valid = max(
                (row["time"] for row in self.track_events
                 if row["phase"] == "PRE_TIMEOUT_FAR" and row["tracks"]),
                default=t0,
            )
            timeout_from_empty = last_valid - t0 + 0.2
            for key, label in (("vx", "velocity.x"), ("vz", "velocity.z"), ("yaw_rate", "yaw_rate")):
                ax.plot([row["time"] - t0 for row in rows], [row[key] for row in rows], label=label)
            ax.axvline(0.0, color="orange", linestyle="--", label="empty Detections begins")
            ax.axvline(timeout_from_empty, color="red", linestyle="--",
                       label="0.2 s since last valid track")
            ax.axvline(0.8, color="purple", linestyle="--", label="complete silence begins")
        ax.set_xlabel("time from empty input (s)")
        ax.set_ylabel("command")
        ax.set_title("real M5 empty output / silence -> M6 HOLD then STOP")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(str(self.image_dir / "04_empty_silence_stop.png"), dpi=160)
        plt.close(fig)

    def plot_timeline(self, summary) -> None:
        fig, (top, bottom) = plt.subplots(2, 1, figsize=(15, 9), sharex=True)
        command_rows = list(self.command_events)
        if command_rows:
            times = [row["time"] for row in command_rows]
            top.plot(times, [row["vx"] for row in command_rows], label="velocity.x")
            top.plot(times, [row["vz"] for row in command_rows], label="velocity.z")
            top.plot(times, [row["yaw_rate"] for row in command_rows], label="yaw_rate")
        area_times, areas = [], []
        for row in self.track_events:
            for track in row["tracks"]:
                area_times.append(row["time"])
                areas.append(track["area_ratio"])
        if area_times:
            bottom.scatter(area_times, areas, s=10, label="real M5 area_ratio")
        bottom.axhline(0.35, color="red", linestyle="--", label="terminal threshold 0.35")
        colors = plt.cm.tab20(np.linspace(0, 1, len(PHASES)))
        for color, (name, duration, _payload) in zip(colors, PHASES):
            phase_rows = [row for row in command_rows if row["phase"] == name]
            if not phase_rows:
                continue
            relative = phase_rows[0]["time"]
            phase_end = phase_rows[-1]["time"]
            for axis in (top, bottom):
                axis.axvspan(relative, phase_end, color=color, alpha=0.08)
            bottom.text((relative + phase_end) / 2, 0.39, name, rotation=90,
                        ha="center", va="bottom", fontsize=7)
        top.set_ylabel("M6 command")
        bottom.set_ylabel("area_ratio")
        bottom.set_xlabel("wall elapsed time (s)")
        top.grid(True)
        bottom.grid(True)
        top.legend(loc="upper right")
        bottom.legend(loc="upper right")
        fig.suptitle("controlled Detections -> real M5 -> real M6 | %s | M6 %.2f Hz" % (
            summary["result"], summary["frequencies"]["real_m6_ctrl_visual_hz"]))
        fig.tight_layout()
        fig.savefig(str(self.image_dir / "05_full_timeline.png"), dpi=160)
        plt.close(fig)

    def make_plots(self, summary) -> None:
        self.plot_phase("CENTER", "01_real_m5_normal_tracking.png", "normal centered tracking")
        self.plot_phase("RIGHT_DOWN_FAR", "02_right_down_far_direction.png", "right / down / far target")
        self.plot_phase("TERMINAL_035", "03_float32_terminal_boundary.png", "M5-computed 0.35 boundary")
        self.plot_stop()
        self.plot_timeline(summary)


def wait_for_master(timeout=8.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if rosgraph.is_master_online():
            return
        time.sleep(0.05)
    raise RuntimeError("isolated ROS master did not become ready")


def start_process(command, log_path: Path, env):
    stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command, stdout=stream, stderr=subprocess.STDOUT, env=env,
        start_new_session=True,
    )
    return process, stream


def stop_process(name, process, stream, cleanup_lines) -> None:
    if process is None:
        return
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=4)
    cleanup_lines.append("%s pid=%d exit=%s stopped=true" % (
        name, process.pid, process.returncode))
    stream.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--image-dir", required=True, type=Path)
    args = parser.parse_args()
    if args.run_dir.exists() or args.image_dir.exists():
        raise SystemExit("refusing to overwrite an existing run/image directory")
    args.run_dir.mkdir(parents=True)
    args.image_dir.mkdir(parents=True)

    port = free_port()
    ros_home = tempfile.mkdtemp(prefix="ai4_m5_m6_ros_home_")
    env = dict(os.environ)
    env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
    env["ROS_HOSTNAME"] = "127.0.0.1"
    env["ROS_HOME"] = ros_home
    os.environ.update({key: env[key] for key in ("ROS_MASTER_URI", "ROS_HOSTNAME", "ROS_HOME")})
    unique = str(os.getpid())
    m5_name = "/m5_flow_visual_tracker_" + unique
    m6_name = "/m6_flow_pid_controller_" + unique
    processes = []
    cleanup_lines = [
        "ROS_MASTER_URI=" + env["ROS_MASTER_URI"],
        "ROS_HOME=" + ros_home,
    ]
    exit_code = 2
    audit = None
    try:
        core = start_process(["roscore", "-p", str(port)], args.run_dir / "roscore.log", env)
        processes.append(("roscore", *core))
        wait_for_master()
        m5 = start_process([
            "rosrun", "uav_tracking", "visual_tracker_node.py",
            "__name:=" + m5_name.lstrip("/"),
            "/detections:=/test/detections",
            "/camera/image_raw:=/test/camera/image_raw",
            "/visual/tracks:=/test/visual_tracks",
            "_width:=320", "_height:=240", "_use_image:=true", "_track_buffer:=3",
        ], args.run_dir / "visual_tracker_node.log", env)
        processes.append(("visual_tracker", *m5))
        m6 = start_process([
            "rosrun", "uav_tracking", "pid_controller_node.py",
            "__name:=" + m6_name.lstrip("/"),
            "/visual/tracks:=/test/visual_tracks",
            "/ctrl/visual:=/test/ctrl_visual",
            "_image_width:=320", "_image_height:=240",
            "_rate_hz:=50.0", "_hold_timeout:=0.2",
            "_desired_area:=0.10", "_min_area:=0.0001",
            "_forward_speed_limit:=2.0", "_impact_area_threshold:=0.35",
            "_terminal_forward_speed:=0.0",
        ], args.run_dir / "pid_controller_node.log", env)
        processes.append(("pid_controller", *m6))

        (args.run_dir / "session.json").write_text(json.dumps({
            "develop_sha": DEVELOP_SHA,
            "ros_master_uri": env["ROS_MASTER_URI"],
            "ros_home": ros_home,
            "processes": [
                {"name": name, "pid": process.pid}
                for name, process, _stream in processes
            ],
            "nodes": {"m5": m5_name, "m6": m6_name},
            "safety": {
                "control_mux_started": False,
                "state_machine_started": False,
                "mavros_started": False,
                "px4_started": False,
                "offboard_started": False,
                "real_control_topic_published": False,
            },
        }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

        rospy.init_node("m5_m6_real_flow_audit_" + unique, disable_signals=True)
        audit = FlowAudit(args.run_dir, args.image_dir, {m5_name, m6_name})
        audit.run_phases()
        graph = audit.topic_graph()
        audit.freeze_callbacks()
        audit.write_raw_logs(graph)
        summary = audit.evaluate(graph)
        audit.make_plots(summary)
        exit_code = 0 if summary["result"] == "PASS" else 2
        print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    except Exception as exc:
        (args.run_dir / "runner_error.log").write_text(
            "%s: %s\n" % (type(exc).__name__, exc), encoding="utf-8")
        if audit is not None:
            audit.reject("runner exception: %s" % exc)
        raise
    finally:
        if rospy.core.is_initialized():
            rospy.signal_shutdown("M5-M6 isolated flow finished")
        for name, process, stream in reversed(processes):
            stop_process(name, process, stream, cleanup_lines)
        alive = ["%s:%d" % (name, process.pid) for name, process, _stream in processes
                 if process.poll() is None]
        cleanup_lines.append("all_task_processes_stopped=" + str(not alive).lower())
        cleanup_lines.append("alive=" + repr(alive))
        (args.run_dir / "cleanup_confirmation.log").write_text(
            "\n".join(cleanup_lines) + "\n", encoding="utf-8")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
