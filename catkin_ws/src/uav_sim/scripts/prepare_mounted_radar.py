#!/usr/bin/env python3
"""Generate a per-run world; preserve camera/model, insert the native Ray in its body."""
import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def prepare(root, output, config, markers=False):
    root, output = Path(root), Path(output)
    cfg = yaml.safe_load(Path(config).read_text())
    # Reuse the established validation and defaults, but validate before spawning.
    for axis in ('horizontal', 'vertical'):
        n = cfg[axis+'_samples']
        if not isinstance(n, int) or not 2 <= n <= 65535:
            raise ValueError('samples must be integers >=2')
        lo, hi = cfg[axis+'_min_angle'], cfg[axis+'_max_angle']
        import math
        limit = math.pi / 2 if axis == 'vertical' else math.pi
        if not -limit < lo < hi < limit:
            raise ValueError('invalid nonzero FOV')
    import math
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in cfg.values()):
        raise ValueError('all config values must be finite numbers')
    if not (0 < cfg['range_min'] < cfg['range_max'] and cfg['update_rate'] > 0
            and cfg['range_resolution'] > 0 and cfg['noise_stddev'] >= 0):
        raise ValueError('invalid range/rate/noise')
    if cfg['horizontal_samples'] * cfg['vertical_samples'] > 1000000:
        raise ValueError('too many rays')
    desc = root/'catkin_ws/src/uav_description/models'
    model = ET.parse(str(desc/'ai4_static_iris/model.sdf')).getroot().find('model')
    model.set('name', 'static_uav')
    # Keep original physical camera; throttle ROS publication at the sensor's 15 sim Hz.
    # updateRate=0 otherwise republishes identical stamps during paused rendering.
    model.find(".//plugin[@name='front_camera_ros']/updateRate").text = '15'
    ET.SubElement(model, 'pose').text = '0 0 0.12 0 0 0'
    sensor = copy.deepcopy(ET.parse(str(desc/'ai4_radar_3d/model.sdf')).find('.//sensor'))
    ET.SubElement(sensor, 'pose').text = '0.30 0 0.12 0 0 0'
    paths = {'horizontal_samples': 'ray/scan/horizontal/samples',
             'vertical_samples': 'ray/scan/vertical/samples',
             'range_min': 'ray/range/min', 'range_max': 'ray/range/max',
             'range_resolution': 'ray/range/resolution', 'update_rate': 'update_rate',
             'noise_mean': 'ray/noise/mean', 'noise_stddev': 'ray/noise/stddev'}
    for axis in ('horizontal','vertical'):
        for end in ('min','max'):
            paths[axis+'_'+end+'_angle'] = 'ray/scan/'+axis+'/'+end+'_angle'
    for key, path in paths.items():
        sensor.find(path).text = str(cfg[key])
    # Attached to base_link: SetModelState moves this sensor rigidly with the camera.
    model.find("link[@name='base_link']").append(sensor)
    world_name = 'static_uav.world' if markers else 'static_uav_no_markers.world'
    world = ET.parse(str(root/'catkin_ws/src/uav_sim/worlds'/world_name))
    w = world.getroot().find('world')
    for include in list(w.findall('include')):
        if include.findtext('name') == 'static_uav':
            w.remove(include)
    w.append(model)
    output.mkdir(parents=True, exist_ok=True)
    world.write(str(output/'mounted.world'), encoding='utf-8', xml_declaration=True)
    (output/'radar.yaml').write_text(yaml.safe_dump(cfg))
    import datetime, json
    (output/'run.json').write_text(json.dumps(dict(
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        status='prepared_only', root=str(root), config=cfg, markers=markers,
        note='此文件仅表示准备完成；实际帧数与关闭状态见statistics.json。'),ensure_ascii=False,indent=2))
    return cfg


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('root'); p.add_argument('output'); p.add_argument('config')
    p.add_argument('--markers', action='store_true')
    a = p.parse_args()
    prepare(a.root, a.output, a.config, a.markers)
