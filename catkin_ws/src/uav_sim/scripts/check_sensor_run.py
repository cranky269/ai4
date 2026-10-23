#!/usr/bin/env python3
"""Verify independent native source stamps, ROS publisher/receiver, bag, and every PNG/NPZ."""
import csv,json,sys,collections
from pathlib import Path
import rosbag,numpy as np
from PIL import Image
from check_radar_run import check
root=Path(sys.argv[1]);radar=check(root)
def rows(name):return list(csv.DictReader((root/name).open()))
def stamps(data):return [(int(r['stamp_sec']),int(r['stamp_nsec'])) for r in data]
camera_rows=rows('camera_frames.csv');radar_rows=rows('frames.csv');publisher=rows('camera_publisher.csv');native=rows('source_sensor_stamps.csv');cluster=rows('cluster_received.csv') if (root/'cluster_received.csv').exists() else []
native_camera=stamps([r for r in native if r['sensor']=='camera'])
# Native renderer events and configured ROS publications are distinct: the camera
# plugin publishes at updateRate (sim time), not every forced render update.
import xml.etree.ElementTree as ET
rate=float(ET.parse(str(root/'mounted.world')).find(".//plugin[@name='front_camera_ros']/updateRate").text)
assert rate>0,'cannot derive ROS publisher count for unthrottled camera'
last=0;eligible=[]
for sec,nsec in native_camera:
    ns=sec*10**9+nsec
    if ns-last >= 10**9/rate:
        eligible.append((sec,nsec));last=ns
expected_camera=collections.Counter(eligible)
expected_radar=collections.Counter(stamps([r for r in native if r['sensor']=='radar']))
saved_camera=collections.Counter(stamps(camera_rows));saved_radar=collections.Counter(stamps(radar_rows))
bag_counts=collections.Counter();camera_i=0;canonical=[];source_seqs=[];centroids=0
with rosbag.Bag(str(root/'raw.bag')) as bag:
 for topic,m,t in bag.read_messages():
  bag_counts[topic]+=1
  if topic=='/camera/image_raw':canonical.append((m.header.stamp.secs,m.header.stamp.nsecs))
  if topic=='/radar/centroid':centroids+=1
  if topic=='/uav/front_camera/image_raw':
   row=camera_rows[camera_i];assert (int(row['stamp_sec']),int(row['stamp_nsec']))==(m.header.stamp.secs,m.header.stamp.nsecs)
   assert row['frame_id']==m.header.frame_id and int(row['source_seq'])==m.header.seq
   assert t==m.header.stamp and len(m.data)==m.height*m.step
   assert m.encoding in ('rgb8','bgr8','mono8')
   channels=1 if m.encoding=='mono8' else 3
   original=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*channels].reshape((m.height,m.width) if channels==1 else (m.height,m.width,channels))
   if m.encoding=='bgr8':original=original[:,:,::-1]
   np.testing.assert_array_equal(np.asarray(Image.open(root/row['file'])),original)
   source_seqs.append(m.header.seq);camera_i+=1
result=dict(radar=radar,native_camera_distinct_render_frames=len(native_camera),camera_expected_ros_published_frames=len(eligible),camera_render_events_filtered_by_config=len(native_camera)-len(eligible),camera_ros_rate_config_hz=rate,native_radar_frames=sum(expected_radar.values()),camera_publisher_frames=len(publisher),camera_saved_png=len(camera_rows),camera_bag_original=bag_counts['/uav/front_camera/image_raw'],camera_bag_documented_topic=bag_counts['/camera/image_raw'],cluster_received_frames=len(cluster),cluster_output_frames=centroids,bag_counts=dict(bag_counts),camera_published_missing=sum((expected_camera-saved_camera).values()),camera_published_extra=sum((saved_camera-expected_camera).values()),radar_native_missing=sum((expected_radar-saved_radar).values()),radar_native_extra=sum((saved_radar-expected_radar).values()),camera_source_seq_first=source_seqs[0] if source_seqs else None,camera_source_seq_last=source_seqs[-1] if source_seqs else None,camera_source_sequence_gaps=sum(max(0,b-a-1) for a,b in zip(source_seqs,source_seqs[1:])),all_png_pixels_exact=True)
result['camera_complete']=expected_camera==saved_camera==collections.Counter(stamps(publisher))==collections.Counter(canonical) and bool(camera_i) and source_seqs[0]==0 and source_seqs[-1]+1==camera_i
result['radar_complete']=expected_radar==saved_radar and bool(len(radar_rows))
result['cluster_input_frames']=sorted(set(r['frame_id'] for r in cluster))
result['cluster_min_finite_points']=min((int(r['finite_points']) for r in cluster),default=0)
result['cluster_measured_sim_hz']=radar['measured_sim_hz'] if cluster else None
result['cluster_complete']=collections.Counter(stamps(cluster))==saved_radar and centroids==len(radar_rows)
(root/'sensor_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,indent=2))
assert result['camera_complete'],'native camera / publisher / disk stamp mismatch'
assert result['radar_complete'],'native radar / disk mismatch'
assert result['cluster_complete'],'cluster did not process every radar frame'
assert result['cluster_input_frames']==['radar_link']
by_stamp={k:int(r['valid_points']) for k,r in zip(stamps(radar_rows),radar_rows)}
assert all(int(r['finite_points'])==by_stamp[k] for k,r in zip(stamps(cluster),cluster))
assert not result['camera_source_sequence_gaps']
