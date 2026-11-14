"""Opt-in experiment: cap ONNX Runtime threads before starting the unchanged M4 node."""
import runpy
import sys
from pathlib import Path

import onnxruntime

original = onnxruntime.InferenceSession

def limited(path, sess_options=None, **kwargs):
    options = sess_options or onnxruntime.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    return original(path, options, **kwargs)

if __name__ == "__main__":
    node = Path(sys.argv.pop(1)).resolve(strict=True)
    onnxruntime.InferenceSession = limited
    try:
        runpy.run_path(str(node), run_name="__main__")
    finally:
        onnxruntime.InferenceSession = original
