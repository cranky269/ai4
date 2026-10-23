#!/usr/bin/env python3
"""Find the newest CLOSED and independently verified run, never merely newest folder."""
import argparse
import csv
import json
from pathlib import Path
import sys


def complete(root):
    try:
        stats = json.loads((root/'statistics.json').read_text())
        checked = json.loads((root/'sensor_verification.json').read_text())
        shutdown = json.loads((root/'shutdown.json').read_text())
        if stats['status'] != 'closed' or int((root/'launcher_exit_code.txt').read_text()) != 0:
            return False
        if shutdown.get('pause_returncode') != 0:
            return False
        if any(stats[k] for k in ('queue_drops','sequence_gaps','timestamp_gaps','write_errors')):
            return False
        if not all(checked.get(k) for k in ('camera_complete','radar_complete','cluster_complete')):
            return False
        bag = root/'raw.bag'
        if not bag.is_file() or bag.stat().st_size == 0:
            return False
        if (root/'sensor_verification.json').stat().st_mtime < bag.stat().st_mtime:
            return False  # Raw data was modified after the claimed verification.
        for index, count in [('frames.csv',stats['radar_frames']),
                             ('camera_frames.csv',stats['camera_png_frames'])]:
            rows = list(csv.DictReader((root/index).open()))
            if not count or len(rows) != count or not all((root/r['file']).is_file() for r in rows):
                return False
        return True
    except (OSError, ValueError, KeyError):
        return False


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data_root',type=Path)
    args = parser.parse_args()
    directories = sorted((p for p in args.data_root.glob('*') if p.is_dir()),
                         key=lambda p:p.name, reverse=True)
    for directory in directories:
        if complete(directory):
            print(directory.resolve())
            break
    else:
        print('没有已正常关闭且通过完整源到盘校验的记录；请先运行check_sensor_run.py。',file=sys.stderr)
        sys.exit(1)
