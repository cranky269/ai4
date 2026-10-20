#!/usr/bin/python3
from pathlib import Path
import sys,json,hashlib
import numpy as np
from PIL import Image
import gi
gi.require_version('Gst','1.0');gi.require_version('GstPbutils','1.0')
from gi.repository import Gst,GstPbutils
Gst.init(None);out=Path(sys.argv[1]).resolve();video=out/'无人机移动与实时相机.webm'
info=GstPbutils.Discoverer.new(15*Gst.SECOND).discover_uri(video.as_uri())
layout=json.loads((out/'窗口布局.json').read_text());x,y,w,h=layout['camera']['client'];x=int(round((x-60)*2/3));y=int(round((y-60)*2/3));w=int(round(w*2/3));h=int(round(h*2/3))
p=Gst.parse_launch('filesrc location="'+str(video)+'" ! decodebin ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink sync=false max-buffers=2');sink=p.get_by_name('sink');p.set_state(Gst.State.PLAYING)
frames=[];selected={};targets=[1,6,9,15,21,28];folder=out/'录像解码核验';folder.mkdir(exist_ok=True);error=None
try:
 while True:
  sample=sink.emit('try-pull-sample',10*Gst.SECOND)
  if sample is None:
   m=p.get_bus().pop_filtered(Gst.MessageType.ERROR)
   if m:error=str(m.parse_error())
   break
  b=sample.get_buffer();c=sample.get_caps().get_structure(0);ww,hh=c.get_value('width'),c.get_value('height');ok,m=b.map(Gst.MapFlags.READ);assert ok
  a=np.frombuffer(m.data,np.uint8).reshape(hh,len(m.data)//hh)[:,:ww*3].reshape(hh,ww,3)
  view=a[y:y+h,x:x+w];red=(view[:,:,0]>200)&(view[:,:,1]<60)&(view[:,:,2]<60);blue=(view[:,:,2]>200)&(view[:,:,0]<60)&(view[:,:,1]<60);yy,xx=np.where(red)
  bbox=None if not len(xx) else [int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())]
  pts=b.pts/Gst.SECOND;frames.append({'pts_s':pts,'red_pixels':int(red.sum()),'blue_pixels':int(blue.sum()),'red_bbox':bbox})
  for t in targets:
   if t not in selected or abs(pts-t)<selected[t][0]:selected[t]=(abs(pts-t),a.copy())
  b.unmap(m)
finally:p.set_state(Gst.State.NULL)
for t,(_,a) in selected.items():Image.fromarray(a).save(str(folder/('实录_%02d秒.png'%t)))
(out/'录像逐帧核验.json').write_text(json.dumps(frames,ensure_ascii=False,indent=2))
pts=np.array([f['pts_s'] for f in frames]);bb=np.array([f['red_bbox'] for f in frames if f['red_bbox']]);centers=(bb[:,0]+bb[:,2])/2;widths=bb[:,2]-bb[:,0]+1
r={'duration_s':info.get_duration()/Gst.SECOND,'decoded_frames':len(frames),'video_size':[ww,hh],'first_pts_s':float(pts[0]),'last_pts_s':float(pts[-1]),'actual_video_fps':float((len(frames)-1)/(pts[-1]-pts[0])),'max_frame_interval_s':float(np.diff(pts).max()),'pts_increasing':bool(np.all(np.diff(pts)>0)),'camera_red_blue_visible_every_frame':all(f['red_pixels']>500 and f['blue_pixels']>300 for f in frames),'red_center_span_video_px':float(np.ptp(centers)),'red_width_ratio_video':float(widths.max()/widths.min()),'decode_error':error,'source':'实时桌面录屏，仅本步骤解码核验'}
(out/'录像核验结果.json').write_text(json.dumps(r,ensure_ascii=False,indent=2));print(json.dumps(r,ensure_ascii=False,indent=2),flush=True)
assert error is None and 25<r['duration_s']<31 and r['camera_red_blue_visible_every_frame'] and r['pts_increasing']
assert r['red_center_span_video_px']>60 and r['red_width_ratio_video']>1.25
