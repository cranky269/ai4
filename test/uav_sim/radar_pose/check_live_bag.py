#!/usr/bin/env python3
"""Read a CLOSED two-topic bag and pair original sampling stamps per reset epoch."""
import argparse
import json
import math
from pathlib import Path
import rosbag

p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
cloud_topic='/radar/points';pose_topic='/radar/uav_pose_world'
counts={cloud_topic:0,pose_topic:0};epochs={t:0 for t in counts};last={};seen={t:{} for t in counts};errors=[]
with rosbag.Bag(str(a.bag),'r') as bag:
 for topic,msg,_ in bag.read_messages(topics=list(counts)):
  counts[topic]+=1;t=msg.header.stamp.to_nsec()
  if topic in last and t<last[topic]:epochs[topic]+=1
  last[topic]=t;key=(epochs[topic],t)
  if key in seen[topic]:errors.append('duplicate stamp within epoch: '+topic)
  if topic==cloud_topic:
   if msg.header.frame_id!='world':errors.append('cloud frame changed')
   seen[topic][key]=dict(width=msg.width,height=msg.height,frame=msg.header.frame_id)
  else:
   v=msg.pose.position;q=msg.pose.orientation;xyz=[v.x,v.y,v.z];quat=[q.x,q.y,q.z,q.w]
   if msg.header.frame_id!='world' or not all(math.isfinite(x) for x in xyz+quat) or abs(sum(x*x for x in quat)-1)>1e-6:errors.append('invalid world pose')
   seen[topic][key]=dict(xyz_m=xyz,quaternion_xyzw=quat,frame=msg.header.frame_id)
keys=sorted(set(seen[cloud_topic])&set(seen[pose_topic]));interior_missing=[]
if keys:
 lo,hi=keys[0],keys[-1]
 for topic in counts:
  interior_missing.extend([dict(topic=topic,epoch=k[0],stamp_ns=k[1]) for k in seen[topic] if lo<=k<=hi and k not in keys])
example_key=keys[len(keys)//2] if keys else None
result=dict(bag=str(a.bag.resolve()),counts=counts,paired_frames=len(keys),epochs=epochs,
            unpaired_counts={t:len(seen[t])-len(keys) for t in counts},interior_missing=interior_missing,errors=errors,
            example=dict(epoch=example_key[0],stamp_ns=example_key[1],cloud=seen[cloud_topic][example_key],pose=seen[pose_topic][example_key]) if keys else None)
result['passed']=min(counts.values())>=5 and len(keys)>=5 and not errors and not interior_missing
if a.output:a.output.write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
