"""Manual items 3/4, isolated original receiver classes, no flight connections."""
import os,sys,time,json,socket,subprocess,signal,tempfile,threading,importlib.util
from pathlib import Path
item=int(sys.argv[1]);root=Path('/home/demo/ai4-m4-acceptance-c421ec7');out=Path(tempfile.mkdtemp(prefix='m4-manual-item%d-'%item,dir='/home/demo'));print('SESSION='+str(out),flush=True)
with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_LOG_DIR=str(out/'ros'),DISPLAY=':0',PYTHONDONTWRITEBYTECODE='1')
children=[];events=[];ui={'phase':'ready','message':'已准备好：聊天发“开始”，在10秒倒计时内启动OBS','deadline':0};node=None
def launch(cmd,n):
 with (out/(n+'.log')).open('w') as f:p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 children.append(p)
def mark(name,text,seconds=0):
 ui.update(phase=name,message=text,deadline=time.time()+seconds);events.append(dict(kind='stage',wall=time.time(),**ui));tmp=out/'phase.tmp';tmp.write_text(json.dumps(ui));tmp.replace(out/'phase.json')
def load(name,file):
 spec=importlib.util.spec_from_file_location(name,str(root/'catkin_ws/src/uav_tracking/scripts'/file));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
try:
 launch(['roscore','-p',str(port)],'master');time.sleep(3)
 import rospy
 from PyQt5 import QtWidgets,QtCore
 from uav_tracking.msg import Detections,DetectedTarget,VisualTrackArray,VisualTrack,TrackArray,Track
 from std_msgs.msg import String
 from mavros_msgs.msg import PositionTarget
 rospy.init_node('state_machine' if item==3 else 'pid_controller',disable_signals=True)
 if item==3:module=load('original_m8','state_machine_node.py');node=module.StateMachineNode()
 else:
  rospy.set_param('~image_width',320);rospy.set_param('~image_height',240)
  module=load('original_m6','pid_controller_node.py');node=module.PIDControllerNode()
 latest={'state':'RADAR_GUIDE','x':0,'z':0,'yaw':0};pubs={}
 def state(m):latest['state']=m.data;events.append(dict(kind='state',wall=time.time(),value=m.data))
 def cmd(m):latest.update(x=m.velocity.x,z=m.velocity.z,yaw=m.yaw_rate);events.append(dict(kind='command',wall=time.time(),x=m.velocity.x,z=m.velocity.z,yaw=m.yaw_rate))
 subs=[rospy.Subscriber('/state',String,state,queue_size=100),rospy.Subscriber('/ctrl/visual',PositionTarget,cmd,queue_size=1000)]
 dp=rospy.Publisher('/detections',Detections,queue_size=10);vp=rospy.Publisher('/visual/tracks',VisualTrackArray,queue_size=10,latch=True);rp=rospy.Publisher('/radar/tracks',TrackArray,queue_size=10,latch=True)
 radar=TrackArray();t=Track();t.id=1;t.position=[3.,0.,1.];t.velocity=[0.,0.,0.];t.covariance=[0.]*36;radar.tracks=[t]
 visual=VisualTrackArray();v=VisualTrack();v.id=1;v.bbox=[80.,60.,80.,60.];v.area_ratio=.0625;visual.tracks=[v]
 good=Detections();d=DetectedTarget();d.class_id=0;d.confidence=.9;d.bbox=v.bbox;good.targets=[d]
 def wait(sec):
  end=time.monotonic()+sec
  while time.monotonic()<end:
   if (out/'stop.request').exists():return False
   time.sleep(.05)
  return True
 def sequence():
  try:
   while not (out/'start.request').exists():
    if not wait(.1):return
   launch(['rosbag','record','--lz4','-O',str(out/'live.bag'),'/detections','/visual/tracks','/radar/tracks','/state','/ctrl/visual'],'record')
   mark('countdown','请启动OBS：10秒后开始受控节点演示',10);wait(10)
   if item==3:
    rp.publish(radar);vp.publish(visual);wait(.5);dp.publish(Detections());mark('radar','RADAR_GUIDE：雷达有效，准备展示N=3锁定',4);wait(4)
    for i in range(3):
     mark('N','N：第 %d / 3 条合格检测（conf=0.90，area=0.0625）'%(i+1),2);dp.publish(good);wait(2)
    mark('locked','应观察 VISUAL_LOCK：N=3已发送',5);wait(5)
    vp.publish(VisualTrackArray());wait(.3)
    for i in range(3):mark('M','M：第 %d / 3 条空检测，观察丢失回退'%(i+1),2);dp.publish(Detections());wait(2)
    mark('radar','应观察 RADAR_GUIDE：M=3丢失回退',4);wait(4)
    rp.publish(TrackArray());wait(.3)
    for i in range(5):mark('K','K：第 %d / 5 条雷达丢失检测节拍'%(i+1),2);dp.publish(Detections());wait(2)
    mark('search','应观察 SEARCH/RTH：K=5已发送',5);wait(5)
    rp.publish(radar);wait(.3);dp.publish(Detections());mark('restore','雷达恢复：应观察退出搜索，回RADAR_GUIDE',5);wait(5)
   else:
    for name,cx,cy in [('居中',160,120),('左',120,120),('右',200,120),('上',160,90),('下',160,150)]:
     mark('position','目标'+name+'：观察误差和ROS端 yaw_rate / vz',5)
     for _ in range(50):
      vt=VisualTrackArray();z=VisualTrack();z.id=1;z.bbox=[cx-10,cy-10,20,20];z.area_ratio=400/76800;vt.tracks=[z];vp.publish(vt);ui['center']=[cx,cy];wait(.1)
    mark('lost','空航迹：超过0.2秒后应归零',5);vp.publish(VisualTrackArray());wait(5)
    mark('restore','恢复右侧目标：应自动恢复输出',5)
    for _ in range(50):
     vt=VisualTrackArray();z=VisualTrack();z.id=1;z.bbox=[190,110,20,20];z.area_ratio=400/76800;vt.tracks=[z];vp.publish(vt);ui['center']=[200,120];wait(.1)
    mark('silence','航迹断流：超过0.2秒后应归零',5);wait(5)
   # Close this recording only; keep the original receiver and window visible.
   if children and len(children)>1:
    p=children[-1]
    if p.poll() is None:os.killpg(p.pid,signal.SIGINT);p.wait(timeout=15)
   (out/'events.json').write_text(json.dumps(events));mark('finished','演示结束，可以停止OBS录制；等待你确认本项结果')
  except Exception as ex:mark('error','演示异常，请停止录制：'+str(ex))
 app=QtWidgets.QApplication(sys.argv);win=QtWidgets.QWidget();win.resize(1050,670);win.move(30,40);layout=QtWidgets.QVBoxLayout(win)
 banner=QtWidgets.QLabel('第%d项 · 隔离受控节点级测试 · 人工消息 · 非自然锁定/非飞行验收'%item);banner.setStyleSheet('font-size:20px;color:#a05000');layout.addWidget(banner)
 stage=QtWidgets.QLabel();stage.setWordWrap(True);layout.addWidget(stage);button=QtWidgets.QPushButton('开始10秒倒计时（确认OBS已准备）');button.clicked.connect(lambda:((out/'start.request').touch(),button.setEnabled(False)));layout.addWidget(button);big=QtWidgets.QLabel();big.setStyleSheet('font-size:48px;font-weight:bold;color:#167c3a');layout.addWidget(big)
 detail=QtWidgets.QLabel();detail.setWordWrap(True);detail.setStyleSheet('font-size:22px');layout.addWidget(detail)
 log=QtWidgets.QPlainTextEdit();log.setReadOnly(True);layout.addWidget(log)
 layout.addWidget(QtWidgets.QLabel('原c421ec7接收代码；不改正式阈值。OBS由用户操作，结束不自动判通过。'))
 def refresh():
  if (out/'stop.request').exists():app.quit();return
  remaining=max(0,int(ui['deadline']-time.time()+.99));stage.setText(ui['message']+(('  %d秒'%remaining) if remaining else ''));stage.setStyleSheet('font-size:25px;font-weight:bold;color:'+('#087522' if ui['phase']=='finished' else '#202020'))
  if item==3:
   big.setText(node.state);detail.setText('实际内部计数：'+json.dumps(node.counters)+'\n正式阈值：N=3 / M=3 / K=5，置信度>0.8，面积>0.02；只由检测消息推进')
  else:
   big.setText('yaw_rate %.3f    vz %.3f    vx %.3f'%(latest['yaw'],latest['z'],latest['x']));center=ui.get('center',[160,120]);detail.setText('临时320×240；目标中心%s，像素误差%s\nROS FLU：右偏航为负yaw，下降为负z；MAVROS负责后续转换。'%(center,[center[0]-160,center[1]-120]))
  log.setPlainText('\n'.join(json.dumps(x,ensure_ascii=False) for x in events[-10:]));win.setWindowTitle('M4 人工验收 0%d — '%item+ui['message']);(out/'viewer-status.json').write_text(json.dumps(dict(item=item,phase=ui['phase'],state=node.state if item==3 else latest)))
 timer=QtCore.QTimer();timer.timeout.connect(refresh);timer.start(150);mark('ready',ui['message']);(out/'armed.ready').write_text('ready');threading.Thread(target=sequence,daemon=True).start();win.show();app.exec_()
finally:
 for p in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
 (out/'events.json').write_text(json.dumps(events))
