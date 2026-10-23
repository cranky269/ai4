#!/usr/bin/env python3
"""Real live ROS Image + embedded RViz; lossless-file frame browser with timestamp pairing."""
import sys,os,json,csv,time,bisect,argparse
from pathlib import Path
import numpy as np
from PyQt5 import QtWidgets,QtCore,QtGui
import rospy
from sensor_msgs.msg import Image,PointCloud2,PointField
from rosgraph_msgs.msg import Clock
from rviz import bindings as rviz
class Window(QtWidgets.QMainWindow):
 camera_signal=QtCore.pyqtSignal(object)
 radar_signal=QtCore.pyqtSignal(object)
 def __init__(self,run,offline=False,self_test=False):
  super().__init__();self.root=Path(run);self.offline=offline;self.self_test=self_test
  self.setWindowTitle('AI4 Camera + 3D Radar | '+('Recorded frames' if offline else 'LIVE'))
  self.resize(1500,820);self.counts={'camera':0,'radar':0};self.latest={};self.pixmap=None;self.index=0;self.playing=False
  body=QtWidgets.QWidget();self.setCentralWidget(body);layout=QtWidgets.QVBoxLayout(body)
  title=QtWidgets.QLabel('Camera: uav_camera_optical_frame | Radar: radar_link (X forward, Y left, Z up), metres | Mouse: left rotate, wheel zoom, middle pan')
  title.setWordWrap(True);layout.addWidget(title)
  split=QtWidgets.QSplitter();layout.addWidget(split,1)
  self.image=QtWidgets.QLabel('Waiting for actual camera image');self.image.setAlignment(QtCore.Qt.AlignCenter);self.image.setMinimumSize(320,240);self.image.setStyleSheet('background:#15181c;color:white');split.addWidget(self.image)
  self.render=rviz.VisualizationFrame();self.render.setSplashPath('');self.render.initialize();self.render.setMenuBar(None);self.render.setStatusBar(None);self.render.setHideButtonVisibility(False);[dock.hide() for dock in self.render.findChildren(QtWidgets.QDockWidget)];split.addWidget(self.render);split.setSizes([640,740])
  self.manager=self.render.getManager();self.manager.setFixedFrame('radar_link')
  grid=self.manager.createDisplay('rviz/Grid','Metre grid',True);grid.subProp('Cell Size').setValue(1.0)
  axes=self.manager.createDisplay('rviz/Axes','Radar axes [m]',True);axes.subProp('Length').setValue(.5);axes.subProp('Radius').setValue(.015)
  self.cloud=self.manager.createDisplay('rviz/PointCloud2','Measured Ray XYZ',True)
  self.cloud.subProp('Position Transformer').setValue('XYZ')
  self.cloud.subProp('Topic').setValue('/viewer/points' if offline else '/radar/points')
  self.cloud.subProp('Queue Size').setValue(50);self.cloud.subProp('Style').setValue('Flat Squares');self.cloud.subProp('Size (m)').setValue(.035);self.cloud.subProp('Size (Pixels)').setValue(4)
  self.cloud.subProp('Color Transformer').setValue('AxisColor');self.cloud.subProp('Axis').setValue('Z');self.cloud.subProp('Decay Time').setValue(0.0)
  self.manager.getViewManager().setCurrentViewControllerType('rviz/Orbit')
  view=self.manager.getViewManager().getCurrent();view.subProp('Distance').setValue(7.0);view.subProp('Focal Point').subProp('X').setValue(2.0);view.subProp('Pitch').setValue(.5);view.subProp('Yaw').setValue(.9)
  self.status=QtWidgets.QLabel();layout.addWidget(self.status)
  controls=QtWidgets.QHBoxLayout();layout.addLayout(controls)
  self.mode=QtWidgets.QComboBox();self.mode.addItems(['Camera frames','Radar frames']);controls.addWidget(self.mode)
  self.prev=QtWidgets.QPushButton('Previous [Left]');self.next=QtWidgets.QPushButton('Next [Right]');self.play=QtWidgets.QPushButton('Play / Pause [Space]')
  for w in (self.prev,self.next,self.play):controls.addWidget(w)
  self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);controls.addWidget(self.slider,1)
  colors=QtWidgets.QComboBox();colors.addItems(['Height Z [m]','Range [m]']);controls.addWidget(colors);colors.currentIndexChanged.connect(self.color)
  capture=QtWidgets.QPushButton('Save view');controls.addWidget(capture);capture.clicked.connect(self.capture)
  self.camera_signal.connect(self.show_camera);self.radar_signal.connect(self.note_radar)
  self.timer=QtCore.QTimer();self.timer.timeout.connect(self.advance)
  self.prev.clicked.connect(lambda:self.step(-1));self.next.clicked.connect(lambda:self.step(1));self.play.clicked.connect(self.toggle);self.slider.valueChanged.connect(self.seek);self.mode.currentIndexChanged.connect(self.change_mode)
  self.evidence=QtCore.QTimer();self.evidence.timeout.connect(self.capture);self.evidence.start(10000);self.shots=0
  if offline:
   self.evidence.stop()
   self.camera_rows=list(csv.DictReader((self.root/'camera_frames.csv').open()));self.radar_rows=list(csv.DictReader((self.root/'frames.csv').open()))
   self.times=[[int(r['stamp_sec'])+int(r['stamp_nsec'])*1e-9 for r in rows] for rows in (self.camera_rows,self.radar_rows)]
   self.publisher=rospy.Publisher('/viewer/points',PointCloud2,queue_size=10,latch=True);self.clock=rospy.Publisher('/clock',Clock,queue_size=10)
   self.mode.setCurrentIndex(0);self.change_mode();QtCore.QTimer.singleShot(1200,self.display_pair)
   if self_test:QtCore.QTimer.singleShot(2500,self.test_controls)
  else:
   for w in (self.mode,self.prev,self.next,self.play,self.slider):w.setEnabled(False)
   self.subs=[rospy.Subscriber('/camera/image_raw',Image,self.camera_signal.emit,queue_size=100,buff_size=64*1024*1024),rospy.Subscriber('/radar/points',PointCloud2,self.radar_signal.emit,queue_size=100)]
   rospy.set_param('~ready',True)
 def color(self,index):
  if index==0:self.cloud.subProp('Color Transformer').setValue('AxisColor');self.cloud.subProp('Axis').setValue('Z')
  else:self.cloud.subProp('Color Transformer').setValue('Intensity');self.cloud.subProp('Channel Name').setValue('range')
 def show_camera(self,m):
  self.counts['camera']+=1;self.latest['camera']=[m.header.seq,m.header.stamp.to_sec()]
  if m.encoding not in ('rgb8','bgr8','mono8'):raise ValueError(m.encoding)
  fmt=QtGui.QImage.Format_Grayscale8 if m.encoding=='mono8' else QtGui.QImage.Format_RGB888
  image=QtGui.QImage(bytes(m.data),m.width,m.height,m.step,fmt).copy()
  if m.encoding=='bgr8':image=image.rgbSwapped()
  self.pixmap=QtGui.QPixmap.fromImage(image);self.fit();self.update_status()
 def fit(self):
  if self.pixmap:self.image.setPixmap(self.pixmap.scaled(self.image.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))
 def resizeEvent(self,e):super().resizeEvent(e);self.fit()
 def note_radar(self,m):self.counts['radar']+=1;self.latest['radar']=[m.header.seq,m.header.stamp.to_sec()];self.update_status()
 def update_status(self):
  c=self.latest.get('camera');r=self.latest.get('radar')
  self.status.setText('Camera frame %s  |  Radar frame %s  |  delta t (camera - radar) = %s s  |  %s'%(str(c),str(r),'%.6f'%(c[1]-r[1]) if c and r else '?','LIVE' if not self.offline else ('PAUSED' if not self.playing else 'PLAYING')))
 def change_mode(self):
  if not self.offline:return
  self.playing=False;self.timer.stop();self.index=0;self.slider.setMaximum(len(self.times[self.mode.currentIndex()])-1);self.slider.setValue(0);self.display_pair()
 def nearest(self,arr,t):
  i=bisect.bisect_left(arr,t);return min([max(0,i-1),min(len(arr)-1,i)],key=lambda k:abs(arr[k]-t))
 def display_pair(self):
  if not self.offline:return
  mode=self.mode.currentIndex();t=self.times[mode][self.index];ci=self.index if mode==0 else self.nearest(self.times[0],t);ri=self.index if mode==1 else self.nearest(self.times[1],t)
  row=self.camera_rows[ci];self.pixmap=QtGui.QPixmap(str(self.root/row['file']));self.fit()
  self.latest={'camera':[ci,self.times[0][ci]],'radar':[ri,self.times[1][ri]]}
  row=self.radar_rows[ri]
  with np.load(self.root/row['file']) as a:
   dtype=np.dtype({'names':['x','y','z','range','ring','column','status'],'formats':['<f4','<f4','<f4','<f4','<u2','<u2','u1'],'offsets':[0,4,8,12,16,18,20],'itemsize':24})
   raw=np.zeros(a['x'].shape,dtype=dtype)
   for k in dtype.names:raw[k]=a[k]
   m=PointCloud2();m.header.frame_id=str(a['frame_id']);m.header.stamp=rospy.Time(int(a['stamp_sec']),int(a['stamp_nsec']));m.height,m.width=raw.shape;m.point_step=24;m.row_step=24*m.width;m.data=raw.tobytes();m.is_dense=False
   m.fields=[PointField(n,off,typ,1) for n,off,typ in zip(dtype.names,[0,4,8,12,16,18,20],[7,7,7,7,4,4,2])]
  # Offline ROS master uses wall time: seeking backward must not clear RViz clouds.
  self.publisher.publish(m);self.update_status()
 def seek(self,i):self.index=i;self.display_pair()
 def step(self,d):
  if not self.offline:return
  self.slider.setValue(max(0,min(self.slider.maximum(),self.index+d)))
 def toggle(self):
  self.playing=not self.playing
  if self.playing:self.schedule()
  else:self.timer.stop()
  self.update_status()
 def schedule(self):
  arr=self.times[self.mode.currentIndex()]
  if self.index>=len(arr)-1:self.playing=False;self.timer.stop();return
  self.timer.start(max(1,int((arr[self.index+1]-arr[self.index])*1000)))
 def advance(self):self.step(1);self.schedule()
 def keyPressEvent(self,e):
  if e.key()==QtCore.Qt.Key_Left:self.step(-1)
  elif e.key()==QtCore.Qt.Key_Right:self.step(1)
  elif e.key()==QtCore.Qt.Key_Space:self.toggle()
  else:super().keyPressEvent(e)
 def capture(self):
  folder=self.root/'viewer_evidence';folder.mkdir(exist_ok=True)
  self.shots+=1
  if self.shots>=6:self.evidence.stop()
  config=rviz.Config();self.manager.save(config);writer=rviz.YamlConfigWriter();writer.writeFile(config,str(folder/'viewer.rviz'))
  name=('offline' if self.offline else 'live')+'_'+time.strftime('%H%M%S')
  self.raise_();self.activateWindow();QtWidgets.QApplication.processEvents()
  pos=self.mapToGlobal(QtCore.QPoint(0,0))
  QtWidgets.QApplication.primaryScreen().grabWindow(0,pos.x(),pos.y(),self.width(),self.height()).save(str(folder/(name+'.png')))
  (folder/(name+'.json')).write_text(json.dumps(dict(mode='offline' if self.offline else 'live',displayed=self.counts,latest=self.latest,index=self.index),indent=2))
 def test_controls(self):
  self.slider.setValue(min(30,max(0,self.slider.maximum()-2)));initial=self.index;self.next.click();forward=self.index;self.prev.click();back=self.index
  assert forward==initial+1 and back==initial
  self.toggle()
  def finish():
   advanced=self.index>initial;self.playing=False;self.timer.stop();self.mode.setCurrentIndex(1);self.step(2);assert self.index==2
   self.color(1)
   view=self.manager.getViewManager().getCurrent()
   before=float(view.subProp('Yaw').getValue());view.subProp('Yaw').setValue(before+.4)
   view.subProp('Distance').setValue(5.5);view.subProp('Focal Point').subProp('Y').setValue(.2)
   QtCore.QTimer.singleShot(500,self.capture)
   (self.root/'viewer_evidence/offline_controls.json').write_text(json.dumps(dict(previous_next=True,play_advanced=advanced,paused=not self.playing,radar_step=True,rotation=float(view.subProp('Yaw').getValue())!=before,zoom_distance=float(view.subProp('Distance').getValue()),pan_y=float(view.subProp('Focal Point').subProp('Y').getValue()),latest=self.latest),indent=2))
   assert advanced
  QtCore.QTimer.singleShot(1500,finish)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run');p.add_argument('--offline',action='store_true');p.add_argument('--self-test',action='store_true');args,_=p.parse_known_args()
 rospy.init_node('sensor_viewer',disable_signals=True)
 run=args.run or rospy.get_param('~run_dir')
 app=QtWidgets.QApplication(sys.argv);window=Window(run,args.offline,args.self_test);window.show()
 timer=QtCore.QTimer();timer.timeout.connect(lambda:app.quit() if rospy.is_shutdown() else None);timer.start(200)
 import signal
 signal.signal(signal.SIGINT,lambda *_:app.quit());signal.signal(signal.SIGTERM,lambda *_:app.quit())
 code=app.exec_()
 # RViz's SIP/Ogre bindings on Noetic can crash during Python finalization.
 # This viewer owns no bag/PNG writer. Stop the UI and Python ROS clients,
 # flush its small evidence output, then release native objects with the
 # process rather than invoking unsafe SIP destructors. Preserve Qt's exit code.
 timer.stop();window.timer.stop();window.evidence.stop()
 for sub in getattr(window,'subs',[]):sub.unregister()
 window.manager.stopUpdate()
 rospy.signal_shutdown('viewer closed')
 sys.stdout.flush();sys.stderr.flush()
 os._exit(code)
