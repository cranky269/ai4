#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 ``/camera_node`` E2E 用的测试素材。

产出两个文件（默认落在本脚本同目录的 ``logs/tmp/``）：

- ``e2e_test.avi`` —— 15 fps / 320x240 / 90 帧，含**移动的红色方块**与
  逐帧递增的帧号文字（便于人工确认画面真的在变）；
- ``e2e_test.png`` —— 最后一帧的静态图，供 ``source:=image`` 使用。

设计说明
--------
与 ``fake_camera.py`` / ``fake_detections.py`` 不同，本脚本**不发布 ROS 消息**，
只往磁盘写素材，因此在 ``roscore`` 启动**之前**就能跑，也**不需要 rospy**。
故参数用 ``argv`` 而非 ``rosparam``。

用法::

    python3 make_media.py [输出目录] [帧率]
"""
import os
import sys

VIDEO_NAME = "e2e_test.avi"
IMAGE_NAME = "e2e_test.png"

WIDTH = 320
HEIGHT = 240
FRAMES = 90
FPS = 15.0


def main(argv):
    import cv2
    import numpy as np

    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = argv[1] if len(argv) > 1 else os.path.join(here, "logs", "tmp")
    fps = float(argv[2]) if len(argv) > 2 else FPS
    os.makedirs(out_dir, exist_ok=True)

    video_path = os.path.join(out_dir, VIDEO_NAME)
    image_path = os.path.join(out_dir, IMAGE_NAME)

    writer = cv2.VideoWriter(
        video_path, cv2.VideoWriter_fourcc(*"MJPG"), fps, (WIDTH, HEIGHT)
    )
    if not writer.isOpened():
        print("FAIL: 无法创建视频写入器（编码器不可用）")
        return 1

    last = None
    for i in range(FRAMES):
        frame = np.full((HEIGHT, WIDTH, 3), 30, np.uint8)
        x = int(20 + (WIDTH - 90) * i / FRAMES)
        cv2.rectangle(frame, (x, 90), (x + 50, 140), (0, 0, 255), -1)
        cv2.putText(frame, "F%03d" % i, (8, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        writer.write(frame)
        last = frame
    writer.release()

    if not cv2.imwrite(image_path, last):
        print("FAIL: 无法写出图片 %s" % image_path)
        return 1

    print("素材目录: %s" % out_dir)
    print("  视频: %s  (%d 帧 @ %.1f fps, %dx%d)" % (
        VIDEO_NAME, FRAMES, fps, WIDTH, HEIGHT))
    print("  图片: %s" % IMAGE_NAME)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
