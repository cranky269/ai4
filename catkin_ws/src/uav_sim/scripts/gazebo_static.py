#!/usr/bin/python3
"""已有静态场景的桌面入口；不启动运动或控制功能。"""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
RUNS = ROOT / 'reports/unified-static-runs'
STATE = RUNS / 'active.json'
children = []


def identity(pid):
    try:
        return Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')', 1)[1].split()[19]
    except (FileNotFoundError, ProcessLookupError):
        return None


def active():
    try:
        data = json.loads(STATE.read_text())
        return data if identity(data['pid']) == data['birth'] else None
    except (FileNotFoundError, ValueError, KeyError):
        return None


def cleanup(*_):
    # 每个子进程均创建独立会话；只向本入口创建的会话发送信号。
    for sig, seconds in [(signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)]:
        for proc in children:
            if proc.poll() is None:
                os.killpg(proc.pid, sig)
        until = time.monotonic() + seconds
        while any(p.poll() is None for p in children) and time.monotonic() < until:
            time.sleep(.1)
    if active() and active()['pid'] == os.getpid():
        STATE.unlink(missing_ok=True)


def stop_signal(*_):
    raise KeyboardInterrupt


def main():
    action = sys.argv[1] if len(sys.argv) == 2 else 'start' if len(sys.argv) == 1 else ''
    if action not in ('start', 'stop', 'status'):
        sys.exit('用法：gazebo_static.py [start|stop|status]')
    RUNS.mkdir(parents=True, exist_ok=True)
    if action != 'start':
        data = active()
        if not data:
            print('统一静态场景未运行。')
            return
        print(json.dumps(data, ensure_ascii=False, indent=2))
        if action == 'stop':
            os.kill(data['pid'], signal.SIGTERM)
            for _ in range(250):
                if not active():
                    print('本入口启动的进程已停止。')
                    return
                time.sleep(.1)
            sys.exit('停止超时，请查看上述日志；未对其他进程发送信号。')
        return
    lock = (RUNS / 'instance.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit('已有统一静态场景实例，请使用 status 或 stop。日志：' + str(RUNS))
    logdir = RUNS / (time.strftime('%Y%m%d_%H%M%S') + '-' + str(os.getpid()))
    logdir.mkdir()
    print('日志：' + str(logdir), flush=True)
    with (logdir / 'launcher.log').open('w', buffering=1) as log:
        def fail(message):
            log.write('错误：' + message + '\n')
            raise RuntimeError(message)
        required = ['/opt/ros/noetic/setup.bash', str(ROOT / 'catkin_ws/devel/setup.bash'),
                    '/opt/ros/noetic/lib/image_view/image_view',
                    '/opt/ros/noetic/lib/libgazebo_ros_camera.so',
                    '/usr/share/gazebo-11/models/sun/model.sdf',
                    '/usr/share/gazebo-11/models/ground_plane/model.sdf']
        required += [str(ROOT / p) for p in [
            'catkin_ws/src/uav_sim/scripts/start_static_scene.sh',
            'catkin_ws/src/uav_sim/launch/static_uav.launch',
            'catkin_ws/src/uav_sim/worlds/static_uav.world',
            'catkin_ws/src/uav_description/models/ai4_static_iris/model.sdf']]
        try:
            for path in required:
                if not Path(path).is_file():
                    fail('缺少资源：' + path + '；请人工准备，不会自动安装。')
            for element in ET.parse(required[-1]).iter('uri'):
                uri_text = element.text or ''
                if uri_text.startswith('model://'):
                    resource = ROOT / 'catkin_ws/src/uav_description/models' / uri_text[8:]
                elif uri_text.startswith('file://'):
                    resource = Path('/usr/share/gazebo-11') / uri_text[7:]
                else:
                    fail('无法识别的资源路径：' + uri_text)
                if not resource.exists():
                    fail('模型资源缺失：' + str(resource))
            if not os.environ.get('DISPLAY'):
                fail('DISPLAY 未设置，请从实际桌面终端启动。')
            check = subprocess.run(['xdpyinfo'], stdout=subprocess.DEVNULL, stderr=log)
            if check.returncode:
                fail('无法访问 X 图形桌面，请核对 DISPLAY / XAUTHORITY。')
            signal.signal(signal.SIGTERM, stop_signal)
            signal.signal(signal.SIGINT, stop_signal)
            STATE.write_text(json.dumps({'pid': os.getpid(), 'birth': identity(os.getpid()),
                                         'log': str(logdir)}, ensure_ascii=False))
            launchlog = (logdir / 'scene.log').open('w')
            proc = subprocess.Popen(['bash', required[6], 'gui:=true', 'dynamic_target:=false',
                                     'markers:=true', 'lidar:=false'], stdout=launchlog,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            children.append(proc)
            # 原入口分配独立 master；从其实际日志读取，避免误连其他实例。
            until = time.monotonic() + 90
            uri = None
            while time.monotonic() < until and proc.poll() is None:
                for line in (logdir / 'scene.log').read_text().splitlines():
                    if line.startswith('ROS_MASTER_URI='):
                        uri = line.split('=', 1)[1]
                if uri:
                    break
                time.sleep(.2)
            if not uri:
                fail('场景启动失败，未获得 ROS master；详见 scene.log。')
            env = os.environ.copy()
            env['ROS_MASTER_URI'] = uri
            env['ROS_IP'] = env['ROS_HOSTNAME'] = '127.0.0.1'
            command = 'source /opt/ros/noetic/setup.bash && until rostopic echo -n 1 /uav/front_camera/camera_info; do sleep 1; done'
            ready = subprocess.Popen(['bash', '-c', command], env=env, stdout=log,
                                     stderr=log, start_new_session=True)
            children.append(ready)
            try:
                result = ready.wait(timeout=90)
            except subprocess.TimeoutExpired:
                fail('90秒内未收到相机信息；详见 scene.log 和 launcher.log。')
            if result or proc.poll() is not None:
                fail('场景或相机未就绪；详见 scene.log。')
            viewer = subprocess.Popen(['bash', '-c', 'source /opt/ros/noetic/setup.bash && exec rosrun image_view image_view image:=/uav/front_camera/image_raw __name:=static_camera_view'],
                                      env=env, stdout=log, stderr=log, start_new_session=True)
            children.append(viewer)
            print('场景和相机已就绪。停止：Ctrl-C 或 ' + str(Path(__file__)) + ' stop', flush=True)
            while proc.poll() is None and viewer.poll() is None:
                time.sleep(.5)
            fail('Gazebo 或相机窗口已退出；详见日志。')
        except KeyboardInterrupt:
            print('正在停止本入口启动的进程……', flush=True)
        except Exception as error:
            log.write(str(error) + '\n')
            print('启动/运行失败：{}；日志：{}'.format(error, logdir), file=sys.stderr)
            return 1
        finally:
            cleanup()
    return 0


if __name__ == '__main__':
    sys.exit(main())
