"""串行运行一个模块的真实 ROS E2E，仅清理本次创建的进程组。"""
import importlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime
from xmlrpc.client import ServerProxy


ROOT = Path(__file__).resolve().parents[2]
PYTHON = str(ROOT / '.venv/bin/python')


def main(module):
    e2e = ROOT / 'test' / module / 'e2e_test'
    logs = e2e / 'logs' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    logs.mkdir(parents=True)
    env = dict(os.environ, ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
               ROS_HOME=str(logs / 'ros_home'), ROS_LOG_DIR=str(logs / 'ros'),
               PYTHONUNBUFFERED='1')
    env.pop('ROS_NAMESPACE', None)
    processes = []
    stopped = set()
    checks = []
    result = dict(module=module, status='BLOCKED', exit_code=2,
                  logs=str(logs), interpreter=sys.executable, checks=checks)
    socket.setdefaulttimeout(2)

    def start(name, command):
        stream = (logs / (name + '.log')).open('w')
        try:
            proc = subprocess.Popen(command, env=env, stdout=stream,
                                    stderr=subprocess.STDOUT, start_new_session=True)
        finally:
            stream.close()
        processes.append((name, proc))
        print('启动', name, 'PID', proc.pid, flush=True)
        return proc

    def stop(proc):
        if proc.pid in stopped:
            return
        # start_new_session使PID也是本次独立进程组ID；不会触及已有用户进程。
        for sig, delay in [(signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)]:
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                stopped.add(proc.pid)
                break
            until = time.monotonic() + delay
            while time.monotonic() < until:
                proc.poll()
                try:
                    os.killpg(proc.pid, 0)
                except ProcessLookupError:
                    stopped.add(proc.pid)
                    return
                time.sleep(0.1)
        proc.wait(timeout=2)
        stopped.add(proc.pid)

    def wait_for(predicate, message, seconds=12):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                if predicate():
                    return
            except (OSError, RuntimeError):
                pass
            for name, proc in processes:
                if proc.pid not in stopped and proc.poll() is not None and not name.startswith('check'):
                    raise RuntimeError(name + '提前退出，见日志')
            time.sleep(0.1)
        raise TimeoutError(message)

    def check(name, script, args):
        proc = start(name, [PYTHON, str(e2e / script)] + args)
        rc = proc.wait(timeout=30)
        content = (logs / (name + '.log')).read_text()
        checks.append(dict(name=name, exit_code=rc, output=content))
        print(content, flush=True)
        if rc != 0 or 'PASS:' not in content:
            raise RuntimeError(name + '检查器未通过，退出码' + str(rc))

    def interrupted(signum, frame):
        raise TimeoutError('总超时或收到终止信号: ' + str(signum))

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGALRM):
        signal.signal(sig, interrupted)
    signal.alarm(90)
    try:
        for name in ['rospy', 'sensor_msgs.msg', 'geometry_msgs.msg', 'std_msgs.msg',
                     'mavros_msgs.msg', 'uav_tracking.msg', 'numpy', 'scipy', 'sklearn']:
            imported = importlib.import_module(name)
            print('依赖', name, getattr(imported, '__file__', ''), flush=True)
        assert Path(sys.executable) == Path(PYTHON), '必须使用项目venv解释器'
        node = module + '_node.py'
        located = subprocess.check_output(['rosrun', '--prefix', 'echo', 'uav_tracking', node],
                                          env=env, text=True, timeout=10).strip()
        expected = ROOT / 'catkin_ws/src/uav_tracking/scripts' / node
        assert Path(located).resolve() == expected.resolve(), 'rosrun查找路径不符: ' + located
        assert os.access(located, os.X_OK), '节点不可执行'
        print('rosrun路径', located, '节点解释器', PYTHON, flush=True)
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        env['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(port)
        result['ROS_MASTER_URI'] = env['ROS_MASTER_URI']
        print('独立master', env['ROS_MASTER_URI'], flush=True)
        start('roscore', ['roscore', '-p', str(port)])
        master = ServerProxy(env['ROS_MASTER_URI'])

        def master_ready():
            return master.getPid('/e2e_runner')[0] == 1

        wait_for(master_ready, 'master启动超时')
        assert master.getSystemState('/e2e_runner')[0] == 1
        # 本次新master只允许自身rosout；不会复用已有ROS图。
        state = master.getSystemState('/e2e_runner')[2]
        nodes = {n for section in state for _, ns in section for n in ns}
        assert nodes <= {'/rosout'}, '新master出现非本次节点: ' + repr(nodes)
        assert master.setParam('/e2e_runner', '/use_sim_time', False)[0] == 1
        assert master.getParam('/e2e_runner', '/use_sim_time')[2] is False
        result['use_sim_time'] = False
        print('use_sim_time=false; ROS_IP/ROS_HOSTNAME=127.0.0.1', flush=True)
        result['status'] = 'FAIL'
        result['exit_code'] = 1
        args = {'cluster': ['_z_thresh:=0.3', '_eps:=0.5', '_min_samples:=5'],
                'state_machine': ['_conf_thresh:=0.8', '_area_thresh:=0.02',
                                  '_n_visual_lock:=3', '_m_visual_lost:=3', '_k_radar_lost:=5'],
                'control_mux': []}[module]
        start(module, ['rosrun', '--prefix', PYTHON, 'uav_tracking', node] + args)

        def registered(node_name, kind):
            state = master.getSystemState('/e2e_runner')[2]
            return any(node_name in ns for _, ns in state[kind])

        wait_for(lambda: registered('/' + module, 0), '业务节点发布接口注册超时')
        if module == 'cluster':
            for mode, target in [('target', 'true'), ('empty', 'false')]:
                fake = start('fake_radar_' + mode, [PYTHON, str(e2e / 'fake_radar.py'),
                             '_with_target:=' + target, '_x:=8.0', '_y:=3.0', '_z:=2.0', '_rate:=10.0'])
                wait_for(lambda: registered('/fake_radar', 0), '雷达数据源注册超时')
                time.sleep(0.5)
                check('check_' + mode, 'check_centroid.py',
                      ['_mode:=' + mode, '_duration:=6.0', '_min_msgs:=20'])
                stop(fake)
                time.sleep(0.5)
        elif module == 'state_machine':
            checker = start('check_state', [PYTHON, str(e2e / 'check_state.py'), '_duration:=16.0'])
            wait_for(lambda: registered('/check_state', 1), '状态检查器订阅注册超时')
            time.sleep(1.5)
            start('fake_sensors', [PYTHON, str(e2e / 'fake_sensors.py'),
                                  '_rate:=20.0', '_phase_seconds:=2.5'])
            rc = checker.wait(timeout=30)
            content = (logs / 'check_state.log').read_text()
            checks.append(dict(name='check_state', exit_code=rc, output=content))
            print(content, flush=True)
            if rc != 0 or 'PASS:' not in content:
                raise RuntimeError('状态检查器未通过，退出码' + str(rc))
        else:
            start('fake_mux_input', [PYTHON, str(e2e / 'fake_mux_input.py'),
                                    '_rate:=50.0', '_phase_seconds:=4.0'])
            wait_for(lambda: registered('/fake_mux_input', 0), '控制数据源注册超时')
            time.sleep(0.5)
            check('check_mux_output', 'check_mux_output.py', ['_duration:=12.0', '_min_msgs:=300'])
        # 检查器通过时，业务节点和master必须仍存活。
        assert all(proc.poll() is None for name, proc in processes
                   if not name.startswith('check') and not name.startswith('fake_radar')), '业务进程提前退出'
        result.update(status='PASS', exit_code=0)
    except Exception as exc:
        result['reason'] = str(exc)
        print(result['status'] + ': ' + str(exc), flush=True)
    finally:
        signal.alarm(0)
        # 清理期间不让第二次终止信号中断finally。
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cleanup = []
        for name, proc in reversed(processes):
            stop(proc)
            try:
                os.killpg(proc.pid, 0)
                remaining = True
            except ProcessLookupError:
                remaining = False
            cleanup.append(dict(name=name, pid=proc.pid, returncode=proc.poll(),
                                remaining_process_group=remaining))
        result['cleanup'] = cleanup
        if any(p['remaining_process_group'] for p in cleanup):
            result.update(status='FAIL', exit_code=1, reason='本次进程组清理不完整')
        (logs / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print('E2E RESULT:', result['status'], '退出码', result['exit_code'], '日志', logs, flush=True)
        print('进程清理:', cleanup, flush=True)
    return result['exit_code']


if __name__ == '__main__':
    if len(sys.argv) != 2 or sys.argv[1] not in ('cluster', 'state_machine', 'control_mux'):
        raise SystemExit('用法: run_isolated.py cluster|state_machine|control_mux')
    raise SystemExit(main(sys.argv[1]))
