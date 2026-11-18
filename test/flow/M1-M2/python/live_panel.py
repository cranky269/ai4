#!/usr/bin/env python3
"""Show the existing M1->M2 live renderer in a persistent desktop window.

This is a display-only helper.  It subscribes to the active ROS topics and
does not publish, record, start nodes, or alter the fault-injection flow.
"""
import argparse
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import cv2
import rospy


def load_renderer():
    source = Path(__file__).with_name("record_live_gazebo.py")
    spec = importlib.util.spec_from_file_location("m1_m2_live_renderer", str(source))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--develop-sha", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--empty-count", type=int, default=6)
    parser.add_argument("--window", default="LIVE GAZEBO - M0 to real M1 to real M2")
    args = parser.parse_args()

    renderer = load_renderer()
    rospy.init_node("m1_m2_live_panel", anonymous=True, disable_signals=True)
    view = renderer.Recorder(SimpleNamespace(
        develop_sha=args.develop_sha,
        run_dir=args.run_dir,
        empty_count=args.empty_count,
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
