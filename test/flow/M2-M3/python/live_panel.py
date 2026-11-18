#!/usr/bin/env python3
"""Show the existing M2->M3 live renderer in a persistent desktop window.

This display-only helper consumes the isolated Navigator test topics.  The
local pose shown by the renderer is controlled test input, not MAVROS pose.
"""
import argparse
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import cv2
import rospy


def load_renderer():
    source = Path(__file__).with_name("record_live_gazebo.py")
    spec = importlib.util.spec_from_file_location("m2_m3_live_renderer", str(source))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--develop-sha", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--window", default="LIVE GAZEBO tracks + controlled pose - M2 to M3")
    args = parser.parse_args()

    renderer = load_renderer()
    # run_flow.py explicitly permits read-only nodes containing this recorder
    # name while still rejecting control_mux/MAVROS and unknown consumers.
    rospy.init_node("m2_m3_live_video_recorder_panel", anonymous=True, disable_signals=True)
    view = renderer.Recorder(SimpleNamespace(
        develop_sha=args.develop_sha,
        run_dir=args.run_dir,
    ))

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window, renderer.WIDTH, renderer.HEIGHT)
    try:
        while not rospy.is_shutdown():
            cv2.imshow(args.window, view.render())
            key = cv2.waitKey(10) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyWindow(args.window)


if __name__ == "__main__":
    main()
