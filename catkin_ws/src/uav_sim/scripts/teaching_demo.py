#!/usr/bin/python3
"""可重复运行的既有两机预设演示；只管理本入口创建的实例。"""
import argparse
import fcntl
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
RUNS = ROOT / 'reports/teaching-demo-runs'
ACTIVE = RUNS / 'active.json'


def write_json(path, data):
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    temp.replace(path)


def identity(pid):
    try:
        return Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def active():
    try:
        data = json.loads(ACTIVE.read_text())
        return data if identity(data['pid']) == data['birth'] else None
    except (OSError, ValueError, KeyError):
        return None


def manage(action):
    data = active()
    if not data:
        print('本入口当前没有运行中的演示。')
        return 0
    print('本次数据绝对保存路径：' + data['output'], flush=True)
    if action == 'status':
        print(json.dumps(data, ensure_ascii=False, indent=2))
        return 0
    os.kill(data['pid'], signal.SIGINT)
    end = time.monotonic() + 100
    while identity(data['pid']) == data['birth'] and time.monotonic() < end:
        time.sleep(.2)
    if identity(data['pid']) == data['birth']:
        print('停止仍在收尾；请查看该目录runner.log，不会停止其他实例。', file=sys.stderr)
        return 1
    result = Path(data['output']) / 'result.json'
    if result.exists():
        print(result.read_text())
    print('本次演示已结束，数据已保留。')
    return 0


class Stopped(Exception):
    pass


class Demo:
    def __init__(self, out):
        self.out = out
        self.stop = threading.Event()
        self.scene = None
        self.screen = None
        self.bag = None
        self.subs = []
        self.lock = threading.RLock()
        self.accept = False
        self.static = {}
        self.camera_info = None
        self.latest_state = None
        self.samples = {'images': [], 'carrier': [], 'target': [], 'state': []}
        self.rospy = None
        self.result = {'started': datetime.now().astimezone().isoformat(), 'output': str(out),
                       'source_root': str(ROOT), 'status': 'starting', 'formats': {}, 'errors': []}
        self.log = (out / 'runner.log').open('a', buffering=1)

    def say(self, message):
        print(message, flush=True)
        self.log.write(datetime.now().astimezone().isoformat() + ' ' + message + '\n')

    def pause(self, seconds):
        if self.stop.wait(seconds):
            raise Stopped()
        if self.scene is not None and self.scene.poll() is not None:
            raise RuntimeError('本次roslaunch提前退出，请查看launch.log')

    def service(self, name):
        process = subprocess.run(['rosservice', 'call', name, '{}'], capture_output=True,
                                 text=True, timeout=12)
        import yaml
        if process.returncode:
            raise RuntimeError(name + '调用失败：' + process.stderr)
        value = yaml.safe_load(process.stdout)
        if not value or not value.get('success'):
            raise RuntimeError(name + '返回失败：' + process.stdout)
        return value

    def static_snapshot(self):
        # 将多个静态发布者的锁存消息合并为同一完整锁存快照。
        if not self.bag or not self.static:
            return
        from tf2_msgs.msg import TFMessage
        snapshot = TFMessage(transforms=list(self.static.values()))
        header = {'topic': '/tf_static', 'type': TFMessage._type,
                  'md5sum': TFMessage._md5sum, 'message_definition': TFMessage._full_text,
                  'callerid': '/teaching_demo_static_snapshot', 'latching': '1'}
        self.bag.write('/tf_static', snapshot, t=self.rospy.Time.now(), connection_header=header)

    def receive(self, topic, key, msg):
        with self.lock:
            # 静态TF/CameraInfo从订阅开始就缓存，不受录屏预热或运动开关影响。
            if key == 'static':
                for transform in msg.transforms:
                    self.static[transform.child_frame_id] = transform
                if self.accept:
                    self.static_snapshot()
                return
            if key == 'info':
                self.camera_info = msg
            if not self.accept or not self.bag:
                return
            self.bag.write(topic, msg, t=self.rospy.Time.now())
            wall = time.monotonic()
            if key == 'images':
                self.samples[key].append([wall, msg.header.stamp.to_sec(), msg.width, msg.height,
                                          msg.encoding, msg.header.frame_id])
            elif key in ('carrier', 'target'):
                p = msg.pose
                self.samples[key].append([wall, msg.header.stamp.to_sec(), p.position.x,
                                          p.position.y, p.position.z, p.orientation.z, p.orientation.w])
            elif key == 'state':
                self.latest_state = json.loads(msg.data)
                self.samples[key].append([wall, self.latest_state])

    def run(self):
        import rospy
        import rosbag
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import String
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        self.rospy = rospy
        if not os.environ.get('DISPLAY'):
            raise RuntimeError('DISPLAY未设置；请从实际桌面终端启动')
        if subprocess.run(['xdpyinfo'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
            raise RuntimeError('无法访问X桌面；核对DISPLAY/XAUTHORITY')
        import teaching_capture
        Gst = teaching_capture.gst()
        for plugin in ('ximagesrc', 'vp8enc', 'webmmux', 'videoconvert', 'appsink'):
            if not Gst.ElementFactory.find(plugin):
                raise RuntimeError('缺少GStreamer组件：' + plugin)
        self.result['source_commit'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        command = ['bash', str(ROOT / 'catkin_ws/src/uav_sim/scripts/start_static_scene.sh'),
                   'gui:=true', 'dynamic_target:=true', 'markers:=false', 'camera_view:=true', 'lidar:=false']
        self.result['launch_command'] = command
        self.scene = subprocess.Popen(command, stdout=(self.out / 'launch.log').open('w'),
                                      stderr=subprocess.STDOUT, start_new_session=True)
        self.result['launch_pid'] = self.scene.pid
        self.result['launch_birth'] = identity(self.scene.pid)
        write_json(self.out / 'result.json', self.result)
        connection = None
        deadline = time.monotonic() + 65
        while time.monotonic() < deadline:
            match = re.search(r'静态场景日志：(.*?)；', (self.out / 'launch.log').read_text())
            if match:
                connection = Path(match[1]) / 'connection.bash'
                break
            self.pause(.2)
        if connection is None:
            raise RuntimeError('未得到本轮连接文件')
        self.result['connection'] = str(connection)
        for key in ('ROS_MASTER_URI', 'GAZEBO_MASTER_URI'):
            os.environ[key] = re.search(r'export ' + key + r'=(\S+)', connection.read_text())[1]
        os.environ.update(ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1', GAZEBO_IP='127.0.0.1',
                          ROS_HOME=str(self.out / 'ros_home'))
        rospy.init_node('teaching_demo_recorder', anonymous=True, disable_signals=True)
        # 在订阅前打开bag，初始锁存消息会立即写入；从不等到run才开记录。
        self.bag = rosbag.Bag(str(self.out / 'camera.bag'), 'w', compression='lz4')
        self.accept = True
        topics = [('/uav/front_camera/image_raw', 'images', Image),
                  ('/uav/front_camera/camera_info', 'info', CameraInfo),
                  ('/camera_motion_demo/actual_pose', 'carrier', PoseStamped),
                  ('/target/ground_truth/pose', 'target', PoseStamped),
                  ('/target/expected_pose', 'expected', PoseStamped),
                  ('/camera_motion_demo/state', 'state', String), ('/clock', 'clock', Clock),
                  ('/tf', 'tf', TFMessage), ('/tf_static', 'static', TFMessage)]
        for topic, key, cls in topics:
            self.subs.append(rospy.Subscriber(topic, cls, lambda msg, t=topic, k=key: self.receive(t, k, msg),
                                             queue_size=100, buff_size=8 * 1024 * 1024))
        deadline = time.monotonic() + 70
        while time.monotonic() < deadline:
            with self.lock:
                ready = (len(self.samples['images']) >= 3 and self.samples['carrier']
                         and self.samples['target'] and self.camera_info is not None
                         and {'uav_camera_link', 'uav_camera_optical_frame'} <= set(self.static))
            if ready:
                break
            self.pause(.2)
        else:
            raise RuntimeError('图像/实际位姿/CameraInfo/两段静态TF未全部就绪，拒绝伪成功')
        self.say('图像、相机参数和两段锁存静态TF均已收到；准备并排显示。')
        teaching_capture.arrange(self.out)
        self.screen = subprocess.Popen(['/usr/bin/python3', str(Path(__file__).with_name('teaching_capture.py')),
                                        'screen', str(self.out)], stdout=(self.out / 'screen.log').open('w'),
                                       stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 25
        while not (self.out / 'screen_ready.json').exists():
            if time.monotonic() > deadline or self.screen.poll() is not None:
                raise RuntimeError('桌面录屏未就绪，参见screen.log')
            self.pause(.1)
        with self.lock:
            self.static_snapshot()
            self.bag.write('/uav/front_camera/camera_info', self.camera_info, t=rospy.Time.now())
        self.result['status_before'] = self.service('/camera_motion_demo/status')
        self.result['run_service'] = self.service('/camera_motion_demo/run')
        start = time.monotonic()
        self.result['trigger_monotonic'] = start
        self.result['status'] = 'recording'
        write_json(self.out / 'result.json', self.result)
        self.say('正在运行两机预设并记录；约32秒后自动正常停止。Ctrl-C或另一个终端执行 stop 可提前结束。')
        self.say('本次数据绝对保存路径：' + str(self.out))
        while time.monotonic() - start < 120:
            with self.lock:
                target = [p for p in self.samples['target'] if p[0] >= start]
                span = target[-1][1] - target[0][1] if len(target) > 1 else 0.
                complete = self.latest_state and self.latest_state.get('mode') == 'idle' and self.latest_state.get('elapsed_wall_s', 0) >= 26
            if time.monotonic() - start >= 32 and span >= 14 and complete:
                self.result['full_preset_and_period'] = True
                self.result['target_sim_span_after_trigger_s'] = span
                self.result['preset_wall_s'] = time.monotonic() - start
                break
            if self.screen.poll() is not None:
                raise RuntimeError('桌面录屏提前退出')
            self.pause(.05)
        else:
            raise RuntimeError('120秒内未覆盖完整预设和目标周期，保留部分记录')
        self.result['status_after'] = self.service('/camera_motion_demo/status')
        teaching_capture.screenshot(self.out / 'after.png')

    def finish(self):
        self.say('正在收尾：停止预设、关闭记录、停止本次场景并检查文件……')
        if self.rospy is not None and self.scene and self.scene.poll() is None:
            try:
                self.result['stop_service'] = self.service('/camera_motion_demo/stop')
            except Exception as exc:
                self.result['errors'].append('停止预设：' + str(exc))
        if self.screen and self.screen.poll() is None:
            os.kill(self.screen.pid, signal.SIGINT)
            try:
                self.screen.wait(timeout=28)
            except subprocess.TimeoutExpired:
                self.screen.terminate()
                self.screen.wait(timeout=5)
                self.result['errors'].append('录屏EOS收尾超时')
        screen_result = self.out / 'screen_result.json'
        if screen_result.exists():
            self.result['desktop_recording'] = json.loads(screen_result.read_text())
        with self.lock:
            if self.bag:
                self.static_snapshot()
                self.accept = False
                self.bag.close()
                self.bag = None
            self.accept = False
        for subscription in self.subs:
            subscription.unregister()
        # image_view由该roslaunch创建，先结束自己的查看节点，再结束自己的launch。
        if self.scene and self.scene.poll() is None:
            try:
                subprocess.run(['rosnode', 'kill', '/onboard_camera_view'], stdout=self.log, stderr=self.log, timeout=8)
            except subprocess.TimeoutExpired:
                self.result['errors'].append('本次图像节点单独停止超时，继续由本次launch定向收尾')
            os.killpg(self.scene.pid, signal.SIGINT)
            try:
                self.scene.wait(timeout=35)
            except subprocess.TimeoutExpired:
                os.killpg(self.scene.pid, signal.SIGTERM)
                try:
                    self.scene.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    self.result['errors'].append('本次roslaunch停止超时，请核实本轮PID，未操作其他实例')
        if self.scene:
            self.result['launch_exit'] = self.scene.poll()
            self.result['launch_pid_exists'] = identity(self.scene.pid) == self.result.get('launch_birth') and self.result.get('launch_birth') is not None
        ports = {}
        for key in ('ROS_MASTER_URI', 'GAZEBO_MASTER_URI'):
            if self.result.get('connection') and os.environ.get(key):
                port = int(os.environ[key].rstrip('/').rsplit(':', 1)[1])
                sock = socket.socket()
                sock.settimeout(1)
                ports[key] = {'port': port, 'connect_return': sock.connect_ex(('127.0.0.1', port))}
                sock.close()
        self.result['ports_after_stop'] = ports
        if self.rospy is not None:
            self.rospy.signal_shutdown('本入口正常收尾')
        write_json(self.out / 'samples.json', self.samples)
        # 导出发生在场景停止以后，避免改变第三轮演示时的计算负载和显示效果。
        helper = Path(__file__).with_name('teaching_capture.py')
        if (self.out / 'camera.bag').exists() and self.samples['images']:
            try:
                subprocess.run(['/usr/bin/python3', str(helper), 'export', str(self.out)],
                               stdout=self.log, stderr=self.log, timeout=90, check=True, start_new_session=True)
                self.result['formats']['camera_video'] = 'camera.webm'
            except Exception as exc:
                self.result['errors'].append('相机视频导出：' + str(exc))
            try:
                subprocess.run(['/usr/bin/python3', str(helper), 'check', str(self.out)],
                               stdout=self.log, stderr=self.log, timeout=90, check=True, start_new_session=True)
                self.result['files_readable'] = True
            except Exception as exc:
                self.result['errors'].append('文件核对：' + str(exc))
                self.result['files_readable'] = False
        self.result['formats'].update({'raw_ros_bag': 'camera.bag' if (self.out / 'camera.bag').exists() else None,
                                       'desktop_video': 'desktop.webm' if (self.out / 'desktop.webm').exists() else None,
                                       'mp4': None})
        self.result['finished'] = datetime.now().astimezone().isoformat()
        self.result['status'] = 'failed' if self.result['errors'] else ('stopped_early' if self.stop.is_set() and not self.result.get('full_preset_and_period') else 'complete')
        write_json(self.out / 'result.json', self.result)
        self.say('结束状态：' + self.result['status'] + '；本次数据：' + str(self.out))
        self.log.close()


def main():
    parser = argparse.ArgumentParser(description='已有两机预设演示与记录；start自动运行一轮，stop只停止本入口')
    parser.add_argument('action', nargs='?', choices=['start', 'stop', 'status'], default='start')
    args = parser.parse_args()
    RUNS.mkdir(parents=True, exist_ok=True)
    if args.action != 'start':
        return manage(args.action)
    with (RUNS / 'instance.lock').open('w') as guard:
        try:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('已有本入口实例；请运行 status 查看本次路径，或 stop 正常停止。', file=sys.stderr)
            return 1
        if subprocess.run(['pgrep', '-x', 'gzserver'], stdout=subprocess.DEVNULL).returncode == 0:
            print('已有gzserver，请确认归属后复用或由其启动者停止；未创建新实例。', file=sys.stderr)
            return 1
        stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        out = RUNS / (stamp + '-' + str(os.getpid()))
        out.mkdir(exist_ok=False)
        print('本次数据绝对保存路径：' + str(out), flush=True)
        data = {'pid': os.getpid(), 'birth': identity(os.getpid()), 'output': str(out)}
        write_json(ACTIVE, data)
        demo = Demo(out)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: demo.stop.set())
        try:
            demo.run()
        except Stopped:
            demo.say('收到正常停止请求，保留本次已记录的数据。')
        except Exception as exc:
            demo.result['errors'].append(str(exc))
            demo.say('运行失败：' + str(exc))
        finally:
            try:
                demo.finish()
            except Exception as exc:
                demo.result['errors'].append('收尾异常：' + str(exc))
                demo.result['status'] = 'failed'
                write_json(out / 'result.json', demo.result)
                print('收尾异常，检查本轮日志与PID：' + str(exc), file=sys.stderr)
            if active() and active()['pid'] == os.getpid():
                ACTIVE.unlink()
        return 1 if demo.result['errors'] else 0


if __name__ == '__main__':
    sys.exit(main())
