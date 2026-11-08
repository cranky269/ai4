#!/usr/bin/env python3
"""Check actual scan XYZ against independent Gazebo physical-step model poses.
Uses actual saved ranges and SDF mounting/angles, never the published UAV pose.
"""
import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from tf.transformations import euler_matrix, quaternion_matrix

p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);a=p.parse_args();r=a.run_dir
world=ET.parse(r/'mounted.world');body=world.find(".//model[@name='static_uav']/link[@name='base_link']");sensor=body.find("sensor[@name='ray_3d']")
def fixed(element):
 v=[float(x) for x in element.findtext('pose','0 0 0 0 0 0').split()];m=euler_matrix(*v[3:]);m[:3,3]=v[:3];return m
mount=fixed(body)@fixed(sensor)
h=sensor.find('ray/scan/horizontal');v=sensor.find('ray/scan/vertical')
az=np.linspace(float(h.findtext('min_angle')),float(h.findtext('max_angle')),int(h.findtext('samples')))
el=np.linspace(float(v.findtext('min_angle')),float(v.findtext('max_angle')),int(v.findtext('samples')))
az,el=np.meshgrid(az,el);directions=np.stack([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)],axis=-1)
truth={};epoch=0;last=-1
for row in csv.DictReader((r/'model_pose.csv').open()):
 t=int(row['sec'])*10**9+int(row['nsec'])
 if t<last:epoch+=1
 last=t;m=quaternion_matrix([float(row[k]) for k in ('qx','qy','qz','qw')]);m[:3,3]=[float(row[k]) for k in 'xyz'];truth[(epoch,t)]=m
rows=json.loads((r/'actual_scans.json').read_text());errors=[];omitted=[];double=[];missing=[];epochs=set();points=0;empty_scans=0;phase_counts={}
for row in rows:
 key=(row['epoch'],row['sec']*10**9+row['nsec'])
 if key not in truth:missing.append(key);continue
 with np.load(r/row['file']) as f:
  dtype=np.dtype({'names':['x','y','z','range'],'formats':['<f4']*4,'offsets':[0,4,8,12],'itemsize':24});cloud=np.frombuffer(f['data'].tobytes(),dtype=dtype).reshape(int(f['height']),int(f['width']))
  measured=np.stack([cloud[k] for k in 'xyz'],axis=-1);valid=np.isfinite(measured).all(axis=-1)&np.isfinite(cloud['range'])
  if not valid.any():empty_scans+=1;continue
  measured=measured[valid];local=directions[valid]*cloud['range'][valid,None]
  T=truth[key]@mount;expected=local@T[:3,:3].T+T[:3,3]
  phase=','.join('%.3f'%x for x in truth[key][:3,3]);phase_counts[phase]=phase_counts.get(phase,0)+1
  errors.append(float(np.max(np.linalg.norm(measured-expected,axis=-1))));omitted.append(float(np.mean(np.linalg.norm(measured-local,axis=-1))));double.append(float(np.mean(np.linalg.norm(measured-(expected@T[:3,:3].T+T[:3,3]),axis=-1))));points+=int(valid.sum());epochs.add(row['epoch'])
result=dict(scans=len(errors),phase_scans_with_returns=phase_counts,empty_scans=empty_scans,actual_points=points,missing_truth=missing,epochs=sorted(epochs),mount_matrix=mount.tolist(),max_xyz_error_m=max(errors),omitted_transform_mean_error_min_m=min(omitted),double_transform_mean_error_min_m=min(double))
result['passed']=len(errors)>20 and not missing and max(errors)<1e-5 and len(epochs)>1 and min(omitted)>.1 and min(double)>.1
(r/'actual_xyz_comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
