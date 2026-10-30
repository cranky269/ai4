#!/usr/bin/env python3
"""三维雷达点云实时 3D 显示（matplotlib，可鼠标旋转/缩放）。

world 场景轴：z 向上，单位米。status=1 黄色、3/4 边界红色，
绿色立方体线框为五块标定靶当前位置，蓝点为雷达原点。
"""
import math
import os

os.environ.setdefault("DISPLAY", ":0")
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from gazebo_msgs.srv import GetModelState

BOARDS = ["radar_board_%d" % i for i in range(5)]
MODE_NAMES = ["static", "sway", "figure8", "circle_v", "fast"]
# 板尺寸 0.2(X) x 2.4(Y) x 0.18(Z)，world 系


def board_corners_radar(pose):
    """Gazebo 位姿 -> world 系下的 8 个角点。"""
    ox, oy, oz = pose.position.x, pose.position.y, pose.position.z
    q = pose.orientation
    yaw = 2 * math.atan2(q.z, q.w)
    dx, dy, dz = 0.1, 1.2, 0.09  # 半尺寸
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                x, y = sx * dx, sy * dy
                # 绕 z 转 yaw
                rx = x * math.cos(yaw) - y * math.sin(yaw)
                ry = x * math.sin(yaw) + y * math.cos(yaw)
                corners.append((ox + rx, oy + ry, oz + sz * dz))
    return corners


EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7),
         (0, 4), (1, 5), (2, 6), (3, 7)]


def main():
    rospy.init_node("radar_3d_view", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=20)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    plt.ion()
    fig = plt.figure(figsize=(11, 7.5))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.set_zlabel("z up (m)")
    ax.set_xlim(0, 10)
    ax.set_ylim(-6, 6)
    ax.set_zlim(-1, 2.5)
    ax.view_init(elev=22, azim=-110)
    title = ax.set_title("waiting...")
    scat = ax.scatter([], [], [], s=4, c="yellow", depthshade=False)
    origin = ax.scatter([0], [-2], [1], s=60, c="cyan", marker="^")
    board_lines = [ax.plot([], [], [], c="lime", lw=1.2)[0] for _ in BOARDS]
    mode_texts = [ax.text(0, 0, 0, "", fontsize=7, color="lime") for _ in BOARDS]
    ax.text(0, -2, .65, "radar", color="cyan", fontsize=8)
    fig.canvas.draw_idle()
    plt.show(block=False)

    print("waiting for /radar/points ...")
    msg = rospy.wait_for_message("/radar/points", PointCloud2, timeout=30)
    print("showing 3D cloud. drag to rotate.")
    frames = 0
    while not rospy.is_shutdown():
        assert msg.header.frame_id == 'world', 'Legacy local cloud rejected'
        xs, ys, zs, cs = [], [], [], []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z", "status"),
                                          skip_nans=False):
            x, y, z, s = float(p[0]), float(p[1]), float(p[2]), int(p[3])
            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                continue
            xs.append(x); ys.append(y); zs.append(z)
            cs.append("red" if s in (3, 4) else "yellow")
        scat._offsets3d = (xs, ys, zs)
        scat.set_color(cs if cs else ["yellow"])

        for line, name, txt, mode in zip(board_lines, BOARDS, mode_texts, MODE_NAMES):
            try:
                r = get_state(name, "world")
            except rospy.ServiceException:
                continue
            corners = board_corners_radar(r.pose)
            pts = corners
            exs = [pts[a][0] for a, b in EDGES] + [pts[b][0] for a, b in EDGES]
            eys = [pts[a][1] for a, b in EDGES] + [pts[b][1] for a, b in EDGES]
            ezs = [pts[a][2] for a, b in EDGES] + [pts[b][2] for a, b in EDGES]
            line.set_data(exs, eys)
            line.set_3d_properties(ezs)
            txt.set_position((pts[0][0], pts[0][1]))
            txt.set_3d_properties(pts[0][2] + 0.25, "z")
            txt.set_text(mode)

        frames += 1
        title.set_text("3D radar cloud  frame=%d  valid=%d" % (frames, len(xs)))
        fig.canvas.draw_idle()
        plt.pause(0.001)
        try:
            msg = rospy.wait_for_message("/radar/points", PointCloud2, timeout=2)
        except rospy.ROSException:
            pass


if __name__ == "__main__":
    main()
