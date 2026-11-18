#!/usr/bin/env python3
"""Relay real M5 tracks to isolated M6 with one visible silence interval.

Only /test/visual_tracks is published.  The source messages are forwarded
unchanged outside the requested interval.  This helper never publishes a
PositionTarget or any MAVROS topic.
"""
import argparse

import rosgraph
import rospy
from uav_tracking.msg import VisualTrackArray


class Relay:
    def __init__(self, args):
        self.args = args
        self.started_ros = None
        self.last_ros = None
        self.silence_announced = False
        self.recovery_announced = False
        self.publisher = rospy.Publisher("/test/visual_tracks", VisualTrackArray, queue_size=20)
        rospy.Subscriber("/visual/tracks", VisualTrackArray, self.on_tracks, queue_size=20)
        rospy.Timer(rospy.Duration(0.5), self.check_output_boundary)

    def elapsed_ros(self):
        now = rospy.Time.now().to_sec()
        if self.last_ros is not None and now < self.last_ros:
            self.started_ros = now
            self.silence_announced = False
            self.recovery_announced = False
            rospy.logwarn("ROS time reset; live relay timeline restarted")
        self.last_ros = now
        if self.started_ros is None and now > 0.0:
            self.started_ros = now
        return 0.0 if self.started_ros is None else max(0.0, now - self.started_ros)

    def on_tracks(self, msg):
        elapsed = self.elapsed_ros()
        silence_end = self.args.silence_start + self.args.silence_duration
        if self.args.silence_start <= elapsed < silence_end:
            if not self.silence_announced:
                rospy.logwarn("live demo silence started at %.3f ROS s", elapsed)
                self.silence_announced = True
            return
        if elapsed >= silence_end and self.silence_announced and not self.recovery_announced:
            rospy.loginfo("live demo relay recovered at %.3f ROS s", elapsed)
            self.recovery_announced = True
        self.publisher.publish(msg)

    @staticmethod
    def check_output_boundary(_event):
        try:
            _publishers, subscribers, _services = rosgraph.Master(
                rospy.get_name()).getSystemState()
            nodes = set(dict(subscribers).get("/test/ctrl_visual", []))
            unexpected = sorted(
                node for node in nodes if "m5_m6_live_panel" not in node)
            dangerous = sorted(
                node for node in nodes
                if "mux" in node.lower() or "mavros" in node.lower())
            if unexpected or dangerous:
                rospy.logfatal(
                    "unsafe /test/ctrl_visual subscribers: %r", sorted(nodes))
                rospy.signal_shutdown("isolated output boundary violated")
        except Exception as exc:
            rospy.logerr_throttle(2.0, "output-boundary check failed: %s", exc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--silence-start", type=float, default=3.0,
                        help="ROS seconds after the first M5 message")
    parser.add_argument("--silence-duration", type=float, default=0.8,
                        help="ROS seconds; must exceed the M6 0.2 s timeout")
    args = parser.parse_args()
    if args.silence_start < 0.0 or args.silence_duration <= 0.2:
        parser.error("silence must start at >=0 and last longer than 0.2 ROS seconds")
    rospy.init_node("m5_m6_live_relay", disable_signals=False)
    Relay(args)
    rospy.spin()


if __name__ == "__main__":
    main()
