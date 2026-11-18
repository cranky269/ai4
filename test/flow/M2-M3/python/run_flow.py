#!/usr/bin/env python3
"""Relay real M2 tracks to an isolated M3 and visualize TRACK/HOLD/HOVER."""
import argparse
import json
import math
import os
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from uav_tracking.msg import TrackArray

DEVELOP_SHA = os.environ.get("AI4_DEVELOP_SHA", "431c049408f323580a351d6e3532daba8c6742f5")


class Flow:
    def __init__(self, args):
        self.a = args
        self.t0 = time.monotonic()
        self.source_count = self.forward_count = self.output_count = 0
        self.last_nonempty = None
        self.last_nonempty_ros = None
        self.source_nonempty = False
        self.latest_selected = None
        self.last_forwarded_selected = None
        self.source_records, self.forwarded_records, self.output_records = [], [], []
        self.frame_values, self.mask_values = set(), set()
        self.output_wall, self.output_ros = [], []
        self.last_state, self.state_since_ros = None, None
        self.unknown_subscribers = []
        self.output_before_pose = 0
        self.pose_started = False
        self.recovery_seen = False
        self.hover_seen = False
        self.hold_seen = False
        self.track_seen = False
        self.rejections = []
        self.pose_pub = rospy.Publisher("/test/local_pose", PoseStamped, queue_size=1)
        self.track_pub = rospy.Publisher("/test/radar_tracks", TrackArray, queue_size=10)
        rospy.Subscriber("/radar/tracks", TrackArray, self.on_source, queue_size=50)
        rospy.Subscriber("/test/ctrl_radar", PositionTarget, self.on_output, queue_size=200)

    def elapsed(self): return time.monotonic() - self.t0
    def gate(self): return not (self.a.silence_start <= self.elapsed() < self.a.silence_end)

    def reject(self, text):
        if text not in self.rejections: self.rejections.append(text)
        rospy.logerr("DATA REJECTED: %s", text)

    def on_source(self, msg):
        self.source_count += 1
        now = self.elapsed()
        self.source_nonempty = bool(msg.tracks)
        selected = None
        if msg.tracks:
            selected = sorted(msg.tracks, key=lambda x: (-int(x.age), int(x.id)))[0]
            xyz = tuple(float(v) for v in selected.position)
            if not all(math.isfinite(v) for v in xyz):
                self.reject("Track.position contains non-finite values"); return
            self.latest_selected = (int(selected.id), int(selected.age), xyz)
        self.source_records.append((now, self.latest_selected if msg.tracks else None))
        if self.gate():
            self.track_pub.publish(msg)
            self.forward_count += 1
            self.forwarded_records.append((now, self.latest_selected if msg.tracks else None))
            if msg.tracks:
                self.last_nonempty = now
                self.last_nonempty_ros = rospy.Time.now().to_sec()
                self.last_forwarded_selected = self.latest_selected
            if now >= self.a.silence_end and msg.tracks: self.recovery_seen = True

    def state_now(self, now):
        if not self.pose_started: return "NOPOSE", float("inf")
        ros_now = rospy.Time.now().to_sec()
        age = (float("inf") if self.last_nonempty_ros is None
               else max(0.0, ros_now - self.last_nonempty_ros))
        if not self.gate() or (self.source_records and self.source_records[-1][1] is None):
            return ("HOLD" if age <= .5 else "HOVER"), age
        return ("TRACK" if self.latest_selected is not None else ("HOLD" if age <= .5 else "HOVER")), age

    def on_output(self, msg):
        now = self.elapsed()
        if not self.pose_started: self.output_before_pose += 1
        state, age = self.state_now(now)
        ros_now = rospy.Time.now().to_sec()
        if state != self.last_state:
            self.last_state, self.state_since_ros = state, ros_now
        settled = (self.state_since_ros is not None
                   and ros_now - self.state_since_ros >= .12)
        xyz = (float(msg.position.x), float(msg.position.y), float(msg.position.z))
        self.output_count += 1
        self.output_wall.append(time.monotonic()); self.output_ros.append(ros_now)
        self.frame_values.add(int(msg.coordinate_frame)); self.mask_values.add(int(msg.type_mask))
        target = self.last_forwarded_selected[2] if self.last_forwarded_selected else None
        self.output_records.append((now, state, age, xyz, target, settled))
        self.track_seen |= state == "TRACK"; self.hold_seen |= state == "HOLD"; self.hover_seen |= state == "HOVER"

    def publish_pose(self):
        msg = PoseStamped(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.a.pose
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg); self.pose_started = True

    def safety(self):
        pubs, subs, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        sub_nodes = dict(subs).get("/test/ctrl_radar", [])
        allowed = {rospy.get_name()}
        # The live video recorder is a read-only audit subscriber owned by this
        # flow.  It must not be confused with a control consumer.
        allowed.update(node for node in sub_nodes if "m2_m3_live_video_recorder" in node)
        self.unknown_subscribers = sorted(set(sub_nodes) - allowed)
        dangerous = [n for n in sub_nodes if "mux" in n.lower() or "mavros" in n.lower()]
        if dangerous or self.unknown_subscribers:
            self.reject("unexpected /test/ctrl_radar subscribers: %r" % sub_nodes)
            return False
        return True

    @staticmethod
    def rate(rows):
        return (len(rows)-1)/(rows[-1]-rows[0]) if len(rows)>1 and rows[-1]>rows[0] else 0.0

    def run(self):
        deadline = self.t0 + self.a.duration
        rate = rospy.Rate(20)
        safety_checked = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = self.elapsed()
            if now >= self.a.pose_delay: self.publish_pose()
            if now > self.a.pose_delay + .5 and not safety_checked:
                safety_checked = True
                if not self.safety(): break
            rate.sleep()
        return self.finish()

    def finish(self):
        out = self.a.output; out.mkdir(parents=True, exist_ok=True)
        hz = self.rate(self.output_ros)
        wall_hz = self.rate(self.output_wall)
        track_errors, hover_errors = [], []
        for out_t, state, _, xyz, target, settled in self.output_records:
            if state == "TRACK" and target is not None and settled:
                # ROS callbacks and the 50 Hz timer are asynchronous.  Match an
                # output against actual prior relay samples instead of whatever
                # source callback happened to run most recently.
                candidates = [row[1][2] for row in self.forwarded_records
                              if row[1] is not None and 0.0 <= out_t - row[0] <= 2.0]
                if candidates:
                    track_errors.append(min(float(np.linalg.norm(np.asarray(xyz) - candidate))
                                            for candidate in candidates))
            if state == "HOVER" and settled:
                hover_errors.append(float(np.linalg.norm(np.asarray(xyz)-np.asarray(self.a.pose))))
        summary = dict(
            develop_sha=DEVELOP_SHA,
            source_track_messages=self.source_count, forwarded_messages=self.forward_count,
            output_messages=self.output_count, output_rate_hz=hz, output_wall_rate_hz=wall_hz,
            coordinate_frames=sorted(self.frame_values), type_masks=sorted(self.mask_values),
            output_before_first_pose=self.output_before_pose,
            states=dict(TRACK=self.track_seen,HOLD=self.hold_seen,HOVER=self.hover_seen),
            silence=dict(start_s=self.a.silence_start,end_s=self.a.silence_end,timeout_s=.5),
            recovery_seen=self.recovery_seen, unknown_output_subscribers=self.unknown_subscribers,
            track_output_error_max_m=max(track_errors) if track_errors else None,
            hover_error_max_m=max(hover_errors) if hover_errors else None,
            world_local_alignment="design assumption; not validated by this flow",
            data_rejected=self.rejections)
        summary["pass"] = bool(not self.rejections and self.source_count and self.output_count and
            45 <= hz <= 55 and self.frame_values == {1} and self.mask_values == {3576} and
            self.output_before_pose == 0 and self.track_seen and self.hold_seen and self.hover_seen and
            self.recovery_seen and track_errors and max(track_errors) < 1e-6 and hover_errors and max(hover_errors) < 1e-6)
        (out/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
        timeline = {
            "develop_sha": DEVELOP_SHA,
            "data_source": "live real M2 tracks + controlled local pose + real M3 navigator",
            "duration_wall_s": self.elapsed(),
            "pose": list(self.a.pose),
            "pose_delay_s": self.a.pose_delay,
            "silence_start_s": self.a.silence_start,
            "silence_end_s": self.a.silence_end,
            "hold_timeout_ros_s": 0.5,
            "source_records": [
                {
                    "time": row[0],
                    "selected": None if row[1] is None else {
                        "id": row[1][0], "age": row[1][1],
                        "position": list(row[1][2]),
                    },
                }
                for row in self.source_records
            ],
            "forwarded_records": [
                {
                    "time": row[0],
                    "selected": None if row[1] is None else {
                        "id": row[1][0], "age": row[1][1],
                        "position": list(row[1][2]),
                    },
                }
                for row in self.forwarded_records
            ],
            "output_records": [
                {
                    "time": row[0], "state": row[1],
                    "valid_age": row[2] if math.isfinite(row[2]) else None,
                    "position": list(row[3]),
                    "track_position": None if row[4] is None else list(row[4]),
                    "settled": bool(row[5]),
                    "coordinate_frame": 1, "type_mask": 3576,
                }
                for row in self.output_records
            ],
            "summary": summary,
        }
        (out/"timeline.json").write_text(
            json.dumps(timeline,indent=2,ensure_ascii=False)+"\n",
            encoding="utf-8")
        self.plot(out/"timeline.png", None)
        self.plot(out/"tracking.png", "TRACK")
        self.plot(out/"timeout_hover.png", "HOVER")
        print(json.dumps(summary,ensure_ascii=False)); return 0 if summary["pass"] else 2

    def plot(self, path, focus):
        rows=list(self.output_records)
        if focus == "TRACK": rows=[r for r in rows if r[1]=="TRACK"]
        elif focus == "HOVER": rows=[r for r in rows if r[0]>=self.a.silence_start]
        fig, axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
        colors={"TRACK":"#2ca02c","HOLD":"#ffbf00","HOVER":"#1f77b4","NOPOSE":"gray"}
        if rows:
            t=np.array([r[0] for r in rows]); xyz=np.array([r[3] for r in rows]); states=[r[1] for r in rows]
            for i,label in enumerate("xyz"): axes[0].plot(t,xyz[:,i],label="output "+label)
            axes[0].axhline(self.a.pose[0],ls=":",c="k",label="pose x")
            ages=np.array([min(r[2],2.0) for r in rows]); axes[1].plot(t,ages,label="valid_age")
            axes[1].axhline(.5,ls="--",c="red",label="0.5s timeout")
            axes[2].scatter(t,[{"TRACK":2,"HOLD":1,"HOVER":0,"NOPOSE":-1}[s] for s in states],c=[colors[s] for s in states],s=8)
            axes[2].set_yticks([0,1,2])
            axes[2].set_yticklabels(["HOVER","HOLD","TRACK"])
            for ax in axes:
                ax.axvline(self.a.silence_start,c="red",ls="--"); ax.axvline(self.a.silence_start+.5,c="purple",ls="--"); ax.axvline(self.a.silence_end,c="green",ls="--"); ax.grid(True); ax.legend(loc="best")
        axes[0].set_ylabel("position (m)"); axes[1].set_ylabel("valid_age (s)"); axes[2].set_xlabel("wall time (s)")
        fig.suptitle("M2→M3 %s | frame=%s mask=%s | %.2f Hz"%(focus or "timeline",sorted(self.frame_values),sorted(self.mask_values),self.rate(self.output_wall)))
        fig.tight_layout(); fig.savefig(str(path),dpi=150); plt.close(fig)


def main():
    p=argparse.ArgumentParser(); p.add_argument("--output",type=Path,required=True); p.add_argument("--duration",type=float,default=12)
    p.add_argument("--pose-delay",type=float,default=1.5); p.add_argument("--silence-start",type=float,default=5); p.add_argument("--silence-end",type=float,default=6.2)
    p.add_argument("--pose",type=float,nargs=3,default=(10.,20.,5.)); a=p.parse_args()
    rospy.init_node("m2_m3_receiver_audit",anonymous=True); raise SystemExit(Flow(a).run())
if __name__ == "__main__": main()
