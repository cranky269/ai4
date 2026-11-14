"""Manual item 2: read-only asynchronous visualization of original ROS tracks."""
import sys,time,json,threading,math
from pathlib import Path
import cv2,rospy
from PyQt5 import QtWidgets,QtCore,QtGui
from sensor_msgs.msg import Image
from uav_tracking.msg import VisualTrackArray
from cv_bridge import CvBridge
out=Path(sys.argv[1]);bridge=CvBridge();lock=threading.Lock();data={};counts={'raw':0,'debug':0,'tracks':0};seen=set();last=[0.]
rospy.init_node('manual_m4_track_view',disable_signals=True)
def cb(m,key):
 with lock:
  counts[key]+=1
  if key=='tracks':
   data[key]=[dict(id=t.id,bbox=list(t.bbox),area_ratio=t.area_ratio) for t in m.tracks];last[0]=time.monotonic();seen.update(t.id for t in m.tracks)
   with (out/'tracks.jsonl').open('a') as f:f.write(json.dumps(dict(wall=time.time(),tracks=data[key]))+'\n')
  else:data[key]=bridge.imgmsg_to_cv2(m,'rgb8')
subs=[rospy.Subscriber(t,c,cb,callback_args=k,queue_size=1,buff_size=2**22) for t,c,k in [('/camera/image_raw',Image,'raw'),('/detector/debug_image',Image,'debug'),('/visual/tracks',VisualTrackArray,'tracks')]]
app=QtWidgets.QApplication(sys.argv);win=QtWidgets.QWidget();win.resize(1000,680);win.move(30,40);layout=QtWidgets.QVBoxLayout(win)
head=QtWidgets.QLabel();head.setWordWrap(True);layout.addWidget(head);panels=QtWidgets.QHBoxLayout();labels={}
for key,title in [('debug','实时 /detector/debug_image：原始检测结果'),('tracks','实时原图 + 最近航迹框/ID（异步叠加，非同帧配对）')]:
 box=QtWidgets.QVBoxLayout();box.addWidget(QtWidgets.QLabel(title));v=QtWidgets.QLabel('等待数据');v.setMinimumSize(440,310);v.setAlignment(QtCore.Qt.AlignCenter);v.setStyleSheet('background:#202020;color:white');box.addWidget(v);labels[key]=v;panels.addLayout(box)
layout.addLayout(panels);status=QtWidgets.QLabel();layout.addWidget(status);text=QtWidgets.QPlainTextEdit();text.setReadOnly(True);text.setMaximumHeight(145);layout.addWidget(text)
layout.addWidget(QtWidgets.QLabel('来源：当前真实 Gazebo，非历史回放。原版跟踪器；ID重建与漏检如实展示，不承诺稳定身份。'))
def show(label,im):
 q=QtGui.QImage(im.data,im.shape[1],im.shape[0],im.strides[0],QtGui.QImage.Format_RGB888).copy();label.setPixmap(QtGui.QPixmap.fromImage(q).scaled(label.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.FastTransformation))
def refresh():
 try:stage=json.loads((out/'manual-control.json').read_text())
 except (FileNotFoundError,ValueError):stage={'phase':'ready'}
 phase=stage['phase'];titles={'ready':'第2项已准备：发“开始”后，在10秒倒计时内启动OBS','countdown':'请启动 OBS，倒计时结束后开始跟踪演示','normal_visible':'1/3 目标运动：观察跟踪框与ID（可能换号）','target_away':'2/3 目标离场：观察 tracks: []，航迹框消失','target_restored':'3/3 目标重入：观察重新建轨，可能分配新ID','finished':'演示结束，可以停止 OBS 录制；请确认第二项结果','error':'演示中断，请停止录制，等待检查'}
 title=titles.get(phase,phase)
 if phase in ['countdown','normal_visible','target_away','target_restored']:title+='  %d秒'%max(0,math.ceil(stage.get('deadline',time.time())-time.time()))
 head.setText(title);head.setStyleSheet('font-size:21px;font-weight:bold;color:'+('#087522' if phase=='finished' else '#202020'));win.setWindowTitle('M4 人工验收 02 — 视觉跟踪 / '+title)
 with lock:
  tracks=data.get('tracks',[]);age=time.monotonic()-last[0] if last[0] else None
  if 'debug' in data:show(labels['debug'],data['debug'])
  if 'raw' in data:
   im=data['raw'].copy()
   if age is not None and age<1.5:
    for t in tracks:
     x,y,w,h=t['bbox'];cv2.rectangle(im,(int(x),int(y)),(int(x+w),int(y+h)),(0,255,255),1);cv2.putText(im,'ID %d'%t['id'],(max(0,int(x)),max(10,int(y)-3)),cv2.FONT_HERSHEY_SIMPLEX,.38,(0,255,255),1)
   show(labels['tracks'],im)
  status.setText('相机 %d 帧 | 检测图 %d 帧 | 航迹消息 %d 条 | 当前ID %s | 航迹接收时龄 %s秒'%(counts['raw'],counts['debug'],counts['tracks'],[t['id'] for t in tracks], '--' if age is None else '%.2f'%age))
  text.setPlainText('/visual/tracks\n'+json.dumps({'tracks':tracks},ensure_ascii=False))
  (out/'viewer-status.json').write_text(json.dumps(dict(counts=counts,seen_ids=sorted(seen),current_tracks=tracks,age_wall=age,source='live Gazebo, original tracker',overlay='latest received tracks on latest raw image, asynchronous, display hidden if reception stale >1.5s; ROS output unchanged')))
 if (out/'stop.request').exists():app.quit()
timer=QtCore.QTimer();timer.timeout.connect(refresh);timer.start(250);win.show();sys.exit(app.exec_())
