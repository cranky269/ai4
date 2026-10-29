#!/usr/bin/env python3
"""Compare paired ROS poses with independent same-step Gazebo model poses."""
import argparse,csv,json,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('run_dir');a=p.parse_args();run=Path(a.run_dir)
truth={};epoch=0;last=-1
for row in csv.DictReader((run/'model_pose.csv').open()):
 t=int(row['sec'])*10**9+int(row['nsec'])
 if t<last:epoch+=1
 last=t;truth[(epoch,t)]=[float(row[k]) for k in ('x','y','z','qx','qy','qz','qw')]
data=json.loads((run/'pose_pairs.json').read_text());results=[];missing=[]
for row in data['pairs']:
 key=(row['epoch'],row['sec']*10**9+row['nsec'])
 if key not in truth:missing.append(key);continue
 gt=truth[key];q=row['quaternion_xyzw'];pos=math.sqrt(sum((x-y)**2 for x,y in zip(gt[:3],row['xyz'])))
 # q and -q represent the same rotation; use chord error, stable near zero.
 chord=min(math.sqrt(sum((x-s*y)**2 for x,y in zip(q,gt[3:]))) for s in (1,-1))
 angle=4*math.asin(min(1,chord/2))
 results.append(dict(epoch=key[0],sec=row['sec'],nsec=row['nsec'],position_error_m=pos,orientation_error_rad=angle))
summary=dict(pairs=len(data['pairs']),matched=len(results),missing=missing,max_position_error_m=max(r['position_error_m'] for r in results),max_orientation_error_rad=max(r['orientation_error_rad'] for r in results),time_resets=data['time_resets'],example=data['pairs'][len(data['pairs'])//2],errors=data['errors'])
phases=json.loads((run/'phases.json').read_text())
phase_counts=[]
for phase in phases:
 count=0
 for row in data['pairs']:
  if max(abs(x-y) for x,y in zip(row['xyz'],phase['xyz']))<1e-6 and min(
      max(abs(x-sign*y) for x,y in zip(row['quaternion_xyzw'],phase['quaternion_xyzw'])) for sign in (1,-1))<1e-6:
   count+=1
 phase_counts.append(count)
summary['phase_pair_counts']=phase_counts
summary['unmatched_clouds']=data['unmatched_clouds']
summary['unmatched_poses']=data['unmatched_poses']
(run/'comparison.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
assert not missing and not data['errors'] and len(results)>20 and data['time_resets']
assert summary['max_position_error_m']<1e-6 and summary['max_orientation_error_rad']<1e-6

assert min(phase_counts)>=3
