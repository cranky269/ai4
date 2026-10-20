#!/usr/bin/python3
"""直接录制真实并排桌面区域；不与ROS采样器共享Python进程。"""
import json,time,sys
from pathlib import Path
import gi
gi.require_version('Gst','1.0')
from gi.repository import Gst
Gst.init(None);out=Path(sys.argv[1])
# Root2358x1360 -> two complete windows region2160x1080 -> uniform2/3 recording scale.
p='ximagesrc use-damage=false show-pointer=false ! video/x-raw,framerate=8/1 ! videocrop left=60 right=138 top=60 bottom=220 ! videoscale ! video/x-raw,width=1440,height=720,pixel-aspect-ratio=1/1 ! queue max-size-buffers=3 max-size-time=0 max-size-bytes=0 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! vp8enc name=encoder deadline=1 cpu-used=8 threads=2 target-bitrate=2000000 ! webmmux ! filesink location="'+str(out/'无人机移动与实时相机.webm')+'"'
(out/'录屏管道.txt').write_text(p+'\n真实桌面区域(60,60,2160,1080)等比缩放2/3；不移动图像内容，不补帧，不用文件重放。\n')
pipeline=Gst.parse_launch(p);bus=pipeline.get_bus();start=time.monotonic()
def first_encoded(pad,info):
 if not (out/'recorder_ready.json').exists():
  (out/'recorder_ready.json').write_text(json.dumps({'started_monotonic':start,'ready_monotonic':time.monotonic(),'first_encoded_pts_s':info.get_buffer().pts/Gst.SECOND}))
 return Gst.PadProbeReturn.OK
pipeline.get_by_name('encoder').get_static_pad('src').add_probe(Gst.PadProbeType.BUFFER,first_encoded)
assert pipeline.set_state(Gst.State.PLAYING)!=Gst.StateChangeReturn.FAILURE
error=None
while time.monotonic()-start<29.5:
 m=bus.timed_pop_filtered(100*Gst.MSECOND,Gst.MessageType.ERROR)
 if m:error=str(m.parse_error());break
stop=time.monotonic();pipeline.send_event(Gst.Event.new_eos())
m=bus.timed_pop_filtered(15*Gst.SECOND,Gst.MessageType.EOS|Gst.MessageType.ERROR)
if m and m.type==Gst.MessageType.ERROR:error=str(m.parse_error())
pipeline.set_state(Gst.State.NULL)
r={'record_wall_s':stop-start,'eos':bool(m and m.type==Gst.MessageType.EOS),'error':error,'capture_rectangle':[60,60,2160,1080],'video_size':[1440,720],'requested_fps':8}
(out/'录屏执行结果.json').write_text(json.dumps(r,ensure_ascii=False,indent=2));print(r,flush=True)
assert r['eos'] and error is None
