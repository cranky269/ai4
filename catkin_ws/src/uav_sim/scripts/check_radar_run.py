#!/usr/bin/env python3
"""Reopen every saved radar message and NPZ, validate, and preview measured data."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import rosbag


def check(root):
    root = Path(root)
    rows = list(csv.DictReader((root/'frames.csv').open()))
    stamps = []
    counts = []
    preview = None
    from record_radar import save_cloud
    import tempfile
    with tempfile.TemporaryDirectory() as temp, rosbag.Bag(str(root/'raw.bag')) as bag:
        i = 0
        for topic,msg,t in bag.read_messages(topics=['/radar/points']):
            assert i < len(rows), 'bag has extra frames'
            row = rows[i]
            assert int(row['frame']) == i
            assert int(row['stamp_sec']) == msg.header.stamp.secs
            assert int(row['stamp_nsec']) == msg.header.stamp.nsecs
            assert row['frame_id'] == msg.header.frame_id
            assert t == msg.header.stamp
            actual = root/row['file']
            scratch = Path(temp)/'frame.npz'
            valid = save_cloud(scratch,msg)
            with np.load(str(actual)) as saved, np.load(str(scratch)) as expected:
                for name in expected.files:
                    if expected[name].dtype.kind == 'f':
                        np.testing.assert_equal(saved[name],expected[name],err_msg=name)
                    else:
                        assert np.array_equal(saved[name],expected[name]), name
                assert int(row['valid_points']) == valid
                assert np.all(np.isnan(saved['x'][~saved['valid']]))
                if preview is None or valid > preview[0]:
                    preview = (valid,{k:saved[k].copy() for k in ('x','y','z','ring','valid')})
            stamps.append(msg.header.stamp.to_nsec()); counts.append(valid)
            i += 1
    assert i == len(rows) and i > 1, 'missing or insufficient frames'
    dt = np.diff(np.array(stamps,dtype=np.int64))/1e9
    assert np.all(dt > 0), 'nonmonotonic scan stamps'
    result = dict(frames=i,sim_span_s=(stamps[-1]-stamps[0])/1e9,
                  measured_sim_hz=1/float(dt.mean()),valid_points_total=sum(counts),
                  valid_points_min=min(counts),valid_points_max=max(counts),
                  bag_npz_exact_match=True)
    if preview and preview[0]:
        import matplotlib
        matplotlib.use('Agg')
        from mpl_toolkits.mplot3d import Axes3D  # register on system Matplotlib
        import matplotlib.pyplot as plt
        a = preview[1]; v = a['valid']
        fig = plt.figure(figsize=(8,6)); ax = fig.add_subplot(111,projection='3d')
        ax.scatter(a['x'][v],a['y'][v],a['z'][v],c=a['ring'][v],s=4)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
        ax.set_title('Measured Ray cloud in radar_link')
        fig.savefig(str(root/'pointcloud_preview.png')); plt.close(fig)
    (root/'verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('run_dir')
    check(parser.parse_args().run_dir)
