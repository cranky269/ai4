#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M4 实时直播服务 —— 把 M4 的运行画面/状态/日志通过 HTTP 播出去。

在**虚拟机里**运行：

    python3 live_stream.py            # 默认 0.0.0.0:8090
    python3 live_stream.py 8091       # 换端口

然后从宿主机浏览器打开：

    http://<虚拟机IP>:8090/

页面上有三块，全部**实时刷新**：
    * 左：带检测框的画面（/detector/debug_image 的实时流，MJPEG）
    * 右上：状态（当前状态 / 目标数 / 航迹 ID / area_ratio / 帧率 / 消息计数）
    * 右下：四个节点的日志实时尾巴

为什么不用现成工具：`rqt_image_view` / `image_view` 这个镜像里没装，
而这个脚本只用 stdlib + cv2 + rospy，零额外依赖。
"""
import json
import os
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from uav_tracking.msg import Detections, VisualTrackArray

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8090
IMG_TOPIC = "/detector/debug_image"
LOGS = {
    "camera": "/tmp/m4_live/camera.log",
    "detector": "/tmp/m4_live/detector.log",
    "tracker": "/tmp/m4_live/tracker.log",
    "state": "/tmp/m4_live/state.log",
}

LOCK = threading.Lock()
LATEST = {"jpeg": None, "n": 0}
INFO = {
    "state": "?", "state_src": "topic",
    "det_n": 0, "det_conf": 0.0, "det_msgs": 0,
    "ids": [], "areas": [], "trk_msgs": 0,
    "start": time.time(), "img_shape": None, "t": "",
}
BRIDGE = CvBridge()
ASPECT = 1080.0 / 810.0


def state_from_log():
    """``/state`` 只在变化时发布，晚订阅收不到 —— 从日志兜底。"""
    try:
        with open(LOGS["state"]) as fh:
            lines = fh.readlines()
    except OSError:
        return None
    for line in reversed(lines):
        if "->" in line:
            return line.split("->")[-1].split("(")[0].strip()
        if "state=" in line:
            return line.split("state=")[-1].strip()
    return None


# --------------------------------------------------------------------------- #
# ROS 回调
# --------------------------------------------------------------------------- #
def on_image(msg):
    try:
        img = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as exc:
        rospy.logwarn_throttle(5.0, "stream: 图像转换失败 %s", exc)
        return
    with LOCK:
        INFO["img_shape"] = list(img.shape)
    canvas = draw_frame(img)
    ok, buf = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if ok:
        with LOCK:
            LATEST["jpeg"] = buf.tobytes()
            LATEST["n"] += 1


def on_state(msg):
    with LOCK:
        INFO["state"] = msg.data
        INFO["state_src"] = "topic"


def on_det(msg):
    with LOCK:
        INFO["det_msgs"] += 1
        INFO["det_n"] = len(msg.targets)
        INFO["det_conf"] = max([t.confidence for t in msg.targets] or [0.0])


def on_trk(msg):
    with LOCK:
        INFO["trk_msgs"] += 1
        INFO["ids"] = [t.id for t in msg.tracks]
        INFO["areas"] = [getattr(t, "area_ratio", 0.0) for t in msg.tracks]


def draw_frame(img):
    """在画面上叠状态面板（cv2.putText 不支持中文，故用英文）。"""
    with LOCK:
        st = INFO["state"]
        if st == "?":
            fb = state_from_log()
            if fb:
                st, INFO["state_src"] = fb, "log"
        color = (0, 255, 0) if "VISUAL_LOCK" in st else (0, 200, 255)
        el = max(time.time() - INFO["start"], 0.001)
        lines = [
            ("STATE  : %s (%s)" % (st, INFO["state_src"]), color),
            ("DET    : %d target(s), max conf %.2f" % (INFO["det_n"], INFO["det_conf"]),
             (255, 255, 255)),
            ("TRACKS : %d  ids=%s" % (len(INFO["ids"]), INFO["ids"][:6]), (255, 255, 255)),
            ("AREA   : " + " ".join("%.3f" % a for a in INFO["areas"][:4]), (200, 200, 200)),
            ("RATE   : %.1f Hz  det_msgs=%d" % (INFO["det_msgs"] / el, INFO["det_msgs"]),
             (200, 200, 200)),
            (time.strftime("TIME   : %Y-%m-%d %H:%M:%S"), (180, 180, 180)),
        ]
    canvas = img.copy()
    h, w = 26, 0
    for text, _ in lines:
        (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
        w = max(w, tw)
    x, y = 14, 30
    overlay = canvas.copy()
    cv2.rectangle(overlay, (x - 12, y - 22), (x + w + 24, y + h * len(lines) + 6), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.65, canvas, 0.35, 0, canvas)
    for i, (text, c) in enumerate(lines):
        cv2.putText(canvas, text, (x, y + i * h), cv2.FONT_HERSHEY_SIMPLEX, 0.62, c, 2)
    return canvas


# --------------------------------------------------------------------------- #
# HTTP
# --------------------------------------------------------------------------- #
PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>M4 实时直播</title><style>
*{box-sizing:border-box}body{margin:0;background:#14161a;color:#dfe3e8;
 font:14px/1.5 system-ui,"Microsoft YaHei",sans-serif}
header{padding:10px 16px;background:#1c2027;border-bottom:1px solid #2b313a;
 display:flex;align-items:center;gap:14px}
h1{font-size:15px;margin:0;font-weight:600}
.badge{padding:2px 10px;border-radius:10px;background:#2b313a;font-size:12px}
.ok{background:#12452a;color:#7ee2a8}.warn{background:#4a3a12;color:#f0c674}
main{display:grid;grid-template-columns:minmax(0,auto) 1fr;gap:12px;padding:12px;
 align-items:start}
#shot{max-height:calc(100vh - 90px);border:1px solid #2b313a;border-radius:6px;display:block}
aside{display:flex;flex-direction:column;gap:12px;min-width:0}
.card{background:#1c2027;border:1px solid #2b313a;border-radius:6px;padding:10px 12px}
.card h2{margin:0 0 8px;font-size:12px;color:#8b95a3;font-weight:600;
 letter-spacing:.06em;text-transform:uppercase}
table{border-collapse:collapse;width:100%}td{padding:3px 0;vertical-align:top}
td.k{color:#8b95a3;width:96px;white-space:nowrap}
pre{margin:0;max-height:34vh;overflow:auto;background:#0f1115;border-radius:4px;
 padding:8px;font:12px/1.45 Consolas,monospace;white-space:pre-wrap;color:#b6c0cc}
.mono{font-family:Consolas,monospace}
.big{font-size:20px;font-weight:700}
button.badge,a.badge{border:0;font:inherit;color:inherit;text-decoration:none;cursor:pointer}
#hint{margin-left:auto;color:#8b95a3}
</style></head><body>
<header>
  <h1>M4 实时直播</h1>
  <span id="state" class="badge">连接中…</span>
  <span class="badge" id="rate">–</span>
  <span class="badge" id="frames">–</span>
  <span class="badge" id="clock">–</span>
  <span id="hint">画面: 正常</span>
  <a class="badge" href="/snapshot" target="_blank"
     title="新标签打开当前这一帧的原始分辨率 JPEG（不经任何缩放）">原图</a>
</header>
<main>
  <img id="shot" src="/stream.mjpg" alt="live">
  <aside>
    <div class="card"><h2>检测 / 跟踪</h2>
      <table>
        <tr><td class="k">目标数</td><td id="det">–</td></tr>
        <tr><td class="k">最高置信度</td><td id="conf">–</td></tr>
        <tr><td class="k">航迹 ID</td><td id="ids" class="mono">–</td></tr>
        <tr><td class="k">area_ratio</td><td id="areas" class="mono">–</td></tr>
        <tr><td class="k">画面尺寸</td><td id="shape" class="mono">–</td></tr>
      </table>
    </div>
    <div class="card"><h2>节点日志（实时）</h2><pre id="logs">…</pre></div>
  </aside>
</main>
<script>
async function tick(){
  try{
    const r = await fetch('/status', {cache:'no-store'});
    const d = await r.json();
    const s = document.getElementById('state');
    s.textContent = d.state;
    s.className = 'badge ' + (d.state.includes('VISUAL_LOCK') ? 'ok' : 'warn');
    document.getElementById('rate').textContent = d.rate.toFixed(1) + ' Hz';
    document.getElementById('frames').textContent = 'frames ' + d.frames;
    document.getElementById('clock').textContent = d.t;
    document.getElementById('det').textContent = d.det_n + ' 个';
    document.getElementById('conf').textContent = d.det_conf.toFixed(2);
    document.getElementById('ids').textContent = JSON.stringify(d.ids);
    document.getElementById('areas').textContent = d.areas.map(a=>a.toFixed(3)).join('  ');
    document.getElementById('shape').textContent = JSON.stringify(d.img_shape);
    document.getElementById('logs').textContent = d.logs;
    watchFrames(d.frames);
  }catch(e){
    document.getElementById('state').textContent = '服务断开';
  }
}
setInterval(tick, 1000); tick();

/* ★ 曾经做过「适应窗口 / 原始 1:1」切换按钮，已删除 —— 实测无价值：
   画面的像素信息就那么多（JPEG 质量 80），整体放大只是把马赛克放大。
   真要看单帧细节，用顶栏的「原图」（`/snapshot`，原始分辨率、不经任何缩放）就够了。 */
const shot = document.getElementById('shot');
const hintEl = document.getElementById('hint');

/* ── 画面兜底：自动降级成 /snapshot 轮询 ──────────────────────────────
   为什么需要：MJPEG（multipart/x-mixed-replace）不是所有内核都渲染 ——
   已实测虚拟机自带的旧 Firefox 就是不渲染（服务端字节流是规范的）。
   两个触发条件，任一命中就降级：
     ① 页面加载后 3 秒仍没有第一帧（naturalWidth === 0）
     ② /status 里的 frames 连续 5 秒不变 → 画面停住了（浏览器不自动重连）
   /snapshot 是单张 JPEG，任何浏览器都能显示。 */
let pollTimer = null;
function startPolling(why){
  if (pollTimer) return;
  pollTimer = setInterval(() => { shot.src = '/snapshot?t=' + Date.now(); }, 250);
  hintEl.textContent = '画面: 轮询兜底（' + why + '，4 fps）';
  console.warn('[M4] MJPEG 不可用，已降级为 /snapshot 轮询：' + why);
}
let lastFrames = -1, stallCount = 0;
function watchFrames(n){
  if (n === lastFrames) stallCount++;
  else stallCount = 0;
  /* ★ frames 回退 = 服务端重启过（LATEST["n"] 从 0 重新计数）。
     服务重启会把所有已打开的 MJPEG 连接掐断，而**浏览器不会自动重连** ——
     现象就是"状态/日志还在动，画面却空了"。（本页自己就是这么被搞空过。）
     对策：重新给 <img> 赋一次带时间戳的 src，强制建立新连接。 */
  if (n < lastFrames - 1 && !pollTimer) {
    shot.src = '/stream.mjpg?t=' + Date.now();
    hintEl.textContent = '画面: 已重连（服务端重启过）';
  }
  lastFrames = n;
  if (stallCount >= 5) startPolling('画面已停顿');
}
/* 前 12 秒内反复确认：只要一直没拿到第一帧就降级。
   为什么要查多次：页面可能正好开在「服务重启的空档」里 —— 那时 <img> 连一次
   失败后浏览器不会自己重试，单查一次（3 秒）会漏掉这种情形。 */
for (const ms of [3000, 6000, 9000, 12000]) {
  setTimeout(() => {
    if (!pollTimer && !shot.naturalWidth) startPolling('MJPEG 未渲染出第一帧');
  }, ms);
}
shot.addEventListener('error', () => startPolling('画面流失效'));
</script></body></html>
"""


def tail(path, n=14):
    try:
        with open(path, "rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            fh.seek(max(size - 4096, 0))
            data = fh.read().decode("utf-8", "replace")
        lines = [l.strip() for l in data.splitlines() if l.strip()]
        return "\n".join(lines[-n:])
    except OSError:
        return "(暂无日志)"


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):        # 静音，避免刷屏
        pass

    def do_GET(self):
        if self.path.startswith("/stream.mjpg"):
            self.stream()
        elif self.path.startswith("/status"):
            self.status()
        elif self.path.startswith("/snapshot"):
            self.snapshot()
        else:
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            # ★ 必须禁缓存：否则浏览器会用旧的 HTML（旧 JS），
            #   现象是"改了页面但看不到变化"、"兜底逻辑不生效"（实测踩到过）。
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            self.wfile.write(body)

    def snapshot(self):
        with LOCK:
            jpg = LATEST["jpeg"]
        if not jpg:
            self.send_error(503, "no frame yet")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(jpg)

    def status(self):
        with LOCK:
            el = max(time.time() - INFO["start"], 0.001)
            data = {
                "state": INFO["state"] if INFO["state"] != "?" else (state_from_log() or "?"),
                "det_n": INFO["det_n"], "det_conf": INFO["det_conf"],
                "ids": INFO["ids"], "areas": INFO["areas"],
                "rate": INFO["det_msgs"] / el, "frames": LATEST["n"],
                "img_shape": INFO["img_shape"], "t": time.strftime("%H:%M:%S"),
                "logs": "\n".join("=== %s ===\n%s" % (k, tail(v)) for k, v in LOGS.items()),
            }
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def stream(self):
        """MJPEG：multipart/x-mixed-replace，浏览器 <img> 直接就能播。"""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while True:
                with LOCK:
                    jpg = LATEST["jpeg"]
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpg))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    rospy.init_node("m4_live_stream", disable_signals=True, anonymous=True)
    rospy.Subscriber(IMG_TOPIC, Image, on_image, queue_size=1)
    rospy.Subscriber("/state", String, on_state, queue_size=1)
    rospy.Subscriber("/detections", Detections, on_det, queue_size=1)
    rospy.Subscriber("/visual/tracks", VisualTrackArray, on_trk, queue_size=1)

    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print("M4 stream serving on http://0.0.0.0:%d/  (topic=%s)" % (PORT, IMG_TOPIC))
    sys.stdout.flush()

    try:
        rospy.spin()
    except Exception:
        traceback.print_exc()


if __name__ == "__main__":
    main()
