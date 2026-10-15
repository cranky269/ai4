#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""合成「会动的」测试视频，喂给 camera_node 的 video 源。

为什么需要它
------------
`camera_node _source:=image` 是把**同一张图**反复发 —— 只能证明"链路通"。
虚拟机里既没有摄像头（usb 源用不了），也没接 Gazebo 相机插件，
所以用真图合成运动序列。

★ 只做「真照片 + 取景框运动」，不做任何合成贴片 —— 为什么 ★
------------------------------------------------------------------
跟踪器（以及下游 PID）**只看图像像素**。目标在图像里怎么移动，
它无法也不必区分"是目标在动"还是"相机在动" —— 两者在图像坐标上完全等价。

而**本任务里相机一定在动**（无人机飞向目标），所以"取景框在动、真实照片不变形"
**恰好就是任务里真实发生的情形**，这才是对的测试方向。

> 反面做法（**已弃用**）：把照片里一块区域裁下来当"目标贴片"贴在合成背景上滑来滑去。
> 那样看着"目标在动"，但引入了**现实中不存在的假特征**（硬边矩形、重采样模糊），
> 反而会让检出断续、把测试搞歪 —— 属于自欺欺人。

三种运动（都是同一张真照片的取景框运动）：

| 模式 | 画面里看到什么 | 主要检验 |
|---|---|---|
| `approach` | 镜头往前推（目标**变大**，中心基本不挪） | 尺度变化（→ PID 的接近速度，看 `area_ratio`） |
| `sweep`    | 镜头横向扫（目标**在画面里横移**，大小不变） | 方向变化（→ PID 的 yaw_rate，看 bbox 中心） |
| `both`     | 缩放 + 横移（**默认**） | 两维一起变 |

用法：
    python3 make_moving_clip.py                        # both，输出 assets/bus_moving.mp4
    python3 make_moving_clip.py 出.mp4 150 10 sweep    # 指定输出/帧数/fps/模式

> **注意**：这个视频只是**给人看的演示**，不承担验证职责。
> M4 的**验收证据**是单测 + E2E：
>   `test_id_stable_across_moving_frames`（移动中 ID 不变）
>   E2E 阶段A（假检测源每帧位移 4/3 像素 + 断言「框确实在动 + ID 不变」）
"""
import os
import sys

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ASSETS, "bus_moving.mp4")
NFRAMES = int(sys.argv[2]) if len(sys.argv) > 2 else 150
FPS = int(sys.argv[3]) if len(sys.argv) > 3 else 10
MODE = sys.argv[4] if len(sys.argv) > 4 else "both"

SRC = os.path.join(ASSETS, "bus.jpg")

# 缩放与横向漂移的范围（都对准公交+行人那一块）
Z0, Z1 = 1.00, 1.70
CX0, CX1 = 300, 560      # 横向：从偏左扫到偏右 → 目标在画面里横移
CY0, CY1 = 520, 560


def ease(t):
    return t * t * (3 - 2 * t)


def frame_crop(img, t, do_zoom, do_pan):
    """镜头运动：对原图做缩放 + 中心漂移，再拉回原尺寸。"""
    h, w = img.shape[:2]
    z = (Z0 + (Z1 - Z0) * ease(t)) if do_zoom else 1.0
    cx = (CX0 + (CX1 - CX0) * ease(t)) if do_pan else 420
    cy = (CY0 + (CY1 - CY0) * ease(t)) if do_pan else 520

    cw, ch = int(w / z), int(h / z)
    x0 = int(min(max(cx - cw / 2, 0), w - cw))
    y0 = int(min(max(cy - ch / 2, 0), h - ch))
    crop = img[y0:y0 + ch, x0:x0 + cw]
    return cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)


def main():
    img = cv2.imread(SRC)
    if img is None:
        raise SystemExit("读不到源图: %s（先跑 get_assets.sh）" % SRC)
    h, w = img.shape[:2]
    print("源图: %s  %dx%d   模式: %s" % (SRC, w, h, MODE))

    modes = {
        "approach": lambda t: frame_crop(img, t, True, False),
        "sweep":    lambda t: frame_crop(img, t, False, True),
        "both":     lambda t: frame_crop(img, t, True, True),
    }
    if MODE not in modes:
        raise SystemExit("未知模式 %r，可选 %s" % (MODE, list(modes)))
    gen = modes[MODE]

    for fourcc_name, ext in (("mp4v", ".mp4"), ("XVID", ".avi")):
        path = OUT if OUT.lower().endswith(ext) else os.path.splitext(OUT)[0] + ext
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc_name), FPS, (w, h))
        if not writer.isOpened():
            print("  编码器 %s 不可用，换下一个" % fourcc_name)
            continue

        print("  写 %d 帧 @ %d fps -> %s（%s）" % (NFRAMES, FPS, path, fourcc_name))
        for i in range(NFRAMES):
            t = i / float(max(NFRAMES - 1, 1))
            frame = gen(t)
            cv2.putText(frame, "%s  frame %03d/%d" % (MODE, i + 1, NFRAMES),
                        (14, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            writer.write(frame)
        writer.release()

        cap = cv2.VideoCapture(path)
        got = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        print("  完成: %s  %.0f KB  帧数 %d" % (path, os.path.getsize(path) / 1024.0, got))
        if got > 0:
            return 0

    print("!! 所有编码器都失败")
    return 1


if __name__ == "__main__":
    sys.exit(main())
