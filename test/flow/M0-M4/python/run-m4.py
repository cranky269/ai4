"""Start the unchanged detector and tracker against an existing ROS master."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / 'catkin_ws/src/uav_tracking'
MODEL_SHA = '320ec090ff91a333116d120479f2ad2a9f97a1893e5826e99cf2ad7fd2ca1bd7'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, type=Path)
    parser.add_argument('--ort-deps', type=Path, default=ROOT / 'lib/m4_onnx')
    parser.add_argument('--check', action='store_true', help='Validate imports and model without starting ROS nodes')
    args = parser.parse_args()
    model = args.model.resolve(strict=True)
    if hashlib.sha256(model.read_bytes()).hexdigest() != MODEL_SHA:
        parser.error('Model SHA-256 differs from the validated YOLO11s ONNX artifact')
    deps = args.ort_deps.resolve(strict=True)
    sys.path[:0] = [str(ROOT / 'lib/m4'), str(deps)]
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    import onnx
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model), options, providers=['CPUExecutionProvider'])
    if ort.__version__ != '1.19.2':
        parser.error('Expected the validated ONNX Runtime version 1.19.2')
    metadata = session.get_modelmeta().custom_metadata_map
    print(json.dumps(dict(model=str(model), sha256=MODEL_SHA, ort=ort.__version__,
                          provider=session.get_providers(), metadata=metadata), ensure_ascii=False), flush=True)
    del session
    if args.check:
        return

    # Source the same connection.bash as M0 in the parent shell.
    import rosgraph
    import rosnode
    master = rosgraph.Master('/m4_launcher_check')
    publishers = dict(master.getSystemState()[0])
    if not publishers.get('/camera/image_raw'):
        parser.error('The current ROS master has no /camera/image_raw publisher')
    if {'/detector', '/visual_tracker'} & set(rosnode.get_node_names()):
        parser.error('An M4 detector or tracker is already running; stop its owner before starting another')
    subprocess.run(['rosparam', 'load', str(PACKAGE / 'config/detector.yaml'), '/detector'], check=True)
    subprocess.run(['rosparam', 'load', str(PACKAGE / 'config/visual_tracker.yaml'), '/visual_tracker'], check=True)
    output = Path(tempfile.mkdtemp(prefix='m4-run.'))
    print('M4_LOG_DIR=' + str(output), flush=True)
    env = dict(os.environ)
    env.update(PYTHONPATH=':'.join([str(ROOT / 'lib/m4'), str(deps), env.get('PYTHONPATH', '')]),
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1', M4_TIMING_LOG='1',
               YOLO_AUTOINSTALL='false')
    commands = [
        ([sys.executable, '-B', '-u', str(PACKAGE / 'scripts/visual_tracker_node.py')], None, 'tracker'),
        ([sys.executable, '-B', '-u', str(Path(__file__).with_name('run-detector-ort1.py')),
          str(PACKAGE / 'scripts/detector_node.py'), '_model_path:=' + str(model),
          '_imgsz:=640', '_conf_thres:=0.50'], env, 'detector'),
    ]
    children = []
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        for command, environment, name in commands:
            with (output / (name + '.log')).open('w') as log:
                children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                  env=environment, start_new_session=True))
        print('Detector and original tracker started. Ctrl+C stops both.', flush=True)
        while True:
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError('M4 node exited: ' + str(child.args))
            time.sleep(.25)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)


if __name__ == '__main__':
    main()
