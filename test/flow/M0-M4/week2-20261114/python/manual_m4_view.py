"""Read-only manual acceptance display; no target movement or recording controls."""
import sys,os,time,json,threading,math
from pathlib import Path
from PyQt5 import QtWidgets,QtCore,QtGui
import rospy
from sensor_msgs.msg import Image
from uav_tracking.msg import Detections
from cv_bridge import CvBridge
out=Path(sys.argv[1]);bridge=CvBridge();data={};lock=threading.Lock();counts={'raw':0,'debug':0,'det':0};start=time.monotonic()
rospy.init_node('manual_m4_view',disable_signals=True)
def cb(msg,key):
 with lock:
  counts[key]+=1
  if key=='det':
   data[key]=[dict(class_id=t.class_id,confidence=round(t.confidence,4),bbox=list(t.bbox)) for t in msg.targets]
   with (out/'detections.jsonl').open('a') as f:f.write(json.dumps(dict(wall=time.time(),targets=data[key]))+'\n')
  else:
   im=bridge.imgmsg_to_cv2(msg,'rgb8');data[key]=(QtGui.QImage(im.data,im.shape[1],im.shape[0],im.strides[0],QtGui.QImage.Format_RGB888).copy(),msg.header.stamp.to_sec(),msg.width,msg.height)
subs=[rospy.Subscriber(t,c,cb,callback_args=k,queue_size=1,buff_size=2**22) for t,c,k in [('/camera/image_raw',Image,'raw'),('/detector/debug_image',Image,'debug'),('/detections',Detections,'det')]]
app=QtWidgets.QApplication(sys.argv);win=QtWidgets.QWidget();win.setWindowTitle('M4 人工验收 01 — 实时 Gazebo / 等待用户开始');win.resize(1000,640);win.move(30,40)
layout=QtWidgets.QVBoxLayout(win);header=QtWidgets.QLabel('第1项：真实 Gazebo → M4 | 等待用户“开始” | OBS由用户操作');header.setStyleSheet('font-size:20px;font-weight:bold');layout.addWidget(header)
panels=QtWidgets.QHBoxLayout();labels={}
for key,title in [('raw','/camera/image_raw — 原始相机'),('debug','/detector/debug_image — 模型检测结果')]:
 box=QtWidgets.QVBoxLayout();box.addWidget(QtWidgets.QLabel(title));label=QtWidgets.QLabel('等待图像');label.setMinimumSize(440,310);label.setAlignment(QtCore.Qt.AlignCenter);label.setStyleSheet('background:#202020;color:white');box.addWidget(label);labels[key]=label;panels.addLayout(box)
layout.addLayout(panels);status=QtWidgets.QLabel();layout.addWidget(status);text=QtWidgets.QPlainTextEdit();text.setReadOnly(True);text.setMaximumHeight(145);layout.addWidget(text)
foot=QtWidgets.QLabel('真实实时图像；未降低阈值。偶发漏检如实展示。目标离场操作须等用户发出“开始”。');layout.addWidget(foot)
def refresh():
 try:
  stage=json.loads((out/'manual-control.json').read_text())
 except (FileNotFoundError,ValueError):stage={'phase':'ready'}
 names={'ready':'已准备好：请在聊天中发“开始”，然后在10秒倒计时内启动OBS',
        'countdown':'请启动 OBS 录制 — 演示将在倒计时结束后开始',
        'normal_visible':'正在演示 1/3：正常目标与检测框',
        'target_away':'正在演示 2/3：目标已移出画面，请观察 targets: []',
        'target_restored':'正在演示 3/3：目标恢复，请观察重新检测',
        'finished':'演示结束，可以停止 OBS 录制；请回聊天确认结果',
        'error':'演示中断：请停止录制，等待检查日志'}
 state=stage.get('phase','ready');remaining=max(0,math.ceil(stage.get('deadline',time.time())-time.time()))
 title=names.get(state,state)+(('  %d 秒'%remaining) if state in ['countdown','normal_visible','target_away','target_restored'] else '')
 header.setText(title);header.setWordWrap(True)
 header.setStyleSheet('font-size:21px;font-weight:bold;color:'+('#087522' if state=='finished' else '#b06000' if state=='countdown' else '#202020'))
 win.setWindowTitle('M4 人工验收 01 — '+title)
 foot.setText('只有出现上方“可以停止 OBS 录制”后才结束录像；不代表人工验收已通过。')
 with lock:
  elapsed=time.monotonic()-start
  for key in ['raw','debug']:
   if key in data:
    im,stamp,w,h=data[key];labels[key].setPixmap(QtGui.QPixmap.fromImage(im).scaled(labels[key].size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.FastTransformation))
  age=rospy.Time.now().to_sec()-data['debug'][1] if 'debug' in data else None
  status.setText('模型：YOLO11s-drone ONNX / CPU / ORT 1线程 / conf=0.50 / imgsz=640\n接收：相机 %d 帧 | 检测 %d 条 | 调试图 %d 帧 | 调试图仿真时龄 %s 秒'%(counts['raw'],counts['det'],counts['debug'],'--' if age is None else '%.3f'%age))
  text.setPlainText('/detections\n'+json.dumps({'targets':data.get('det',[])},ensure_ascii=False))
  if counts['raw'] and counts['debug'] and counts['det']:
   (out/'viewer-status.json').write_text(json.dumps(dict(counts=counts,age_sim=age,master=os.environ.get('ROS_MASTER_URI'),raw_dimensions=list(data['raw'][2:]))))
timer=QtCore.QTimer();timer.timeout.connect(refresh);timer.start(250);win.show();sys.exit(app.exec_())
