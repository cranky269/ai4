#!/usr/bin/env python3
"""Offline production recorder roundtrip; no simulated/live-data claims."""
from pathlib import Path
import sys, tempfile, unittest, ast
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
root=Path(__file__).resolve().parents[3]
scripts=root/'catkin_ws/src/uav_sim/scripts'
sys.path.insert(0,str(scripts))
from record_radar import save_cloud, Recorder
from unittest.mock import patch
from geometry_msgs.msg import PoseStamped
import rospy, rosbag, csv
from check_radar_run import check_pose_pairs
# Load the actual pure replay guard without initializing Qt/OGRE or a ROS master.
module=ast.parse((scripts/'sensor_viewer.py').read_text())
guard=next(n for n in module.body if isinstance(n,ast.FunctionDef) and n.name=='require_world_recording')
ns={};exec(compile(ast.Module(body=[guard],type_ignores=[]),'<production replay guard>','exec'),ns)
class RecordingTests(unittest.TestCase):
 def test_legacy_and_unknown_are_explicit(self):
  for rows in ([],[{'frame_id':'radar_link'}],[{}],[{'frame_id':'map'}]):
   with self.assertRaisesRegex(ValueError,'Historical/local or unknown'):ns['require_world_recording'](rows)
  ns['require_world_recording']([{'frame_id':'world'}])
 def test_epoch_pairing_not_nearest(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)
   (p/'frames.csv').write_text('epoch,stamp_sec,stamp_nsec,frame_id\n0,10,1,world\n1,10,1,world\n')
   header='epoch,stamp_sec,stamp_nsec,frame_id,x,y,z,qx,qy,qz,qw\n'
   (p/'radar_poses.csv').write_text(header+'0,10,1,world,1,2,3,0,0,0,1\n1,10,1,world,4,5,6,0,0,0,1\n')
   r=check_pose_pairs(p);self.assertTrue(r['complete']);self.assertEqual(r['paired_frames'],2)
   (p/'radar_poses.csv').write_text(header+'0,10,2,world,1,2,3,0,0,0,1\n')
   r=check_pose_pairs(p);self.assertFalse(r['complete']);self.assertEqual(r['paired_frames'],0)
 def test_real_writer_bag_npz_and_sampled_pose(self):
  # Actual writer and rosbag/NPZ IO; only ROS registration is replaced.
  with tempfile.TemporaryDirectory() as d:
   root=Path(d)
   (root/'radar.yaml').write_text('update_rate: 5.0\n')
   (root/'radar_pose.yaml').write_text('body_to_sensor: [0.3, 0, 0.12, 0, 0, 0]\nscan_parent: static_uav::base_link\n')
   with patch('rospy.get_param',return_value=d),patch('rospy.set_param'),patch('rospy.Subscriber'):
    recorder=Recorder()
   try:
    for i in range(6):
     stamp=rospy.Time(1,(i%3)*200000000)
     c=PointCloud2();c.header.seq=i;c.header.stamp=stamp;c.header.frame_id='world'
     c.width=2;c.height=2;c.point_step=24;c.row_step=48;c.data=bytes(96)
     c.fields=[PointField(n,o,t,1) for n,o,t in zip(['x','y','z','range','ring','column','status'],[0,4,8,12,16,18,20],[7,7,7,7,4,4,2])]
     pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='world';pose.pose.orientation.w=1
     pose.pose.position.x=i
     recorder.q.put(('/radar/uav_pose_world',pose,stamp))
     recorder.q.put(('/radar/points',c,stamp))
   finally:recorder.close()
   self.assertEqual(recorder.stats['write_errors'],0)
   pairs=check_pose_pairs(root);self.assertTrue(pairs['complete']);self.assertEqual(pairs['paired_frames'],6)
   with (root/'frames.csv').open() as f:rows=list(csv.DictReader(f))
   self.assertEqual([int(r['epoch']) for r in rows],[0,0,0,1,1,1])
   with np.load(root/rows[3]['file']) as a:self.assertEqual(int(a['simulation_epoch']),1)
   with rosbag.Bag(str(root/'raw.bag')) as bag:
    info=bag.get_type_and_topic_info().topics
    self.assertEqual(info['/radar/points'].message_count,6)
    self.assertEqual(info['/radar/uav_pose_world'].message_count,6)

 def test_world_roundtrip_both_endians(self):
  for big in (False,True):
   dtype=np.dtype({'names':['x','y','z','range','ring','column','status'],
    'formats':[('>' if big else '<')+v for v in ['f4']*4+['u2']*2+['u1']],
    'offsets':[0,4,8,12,16,18,20],'itemsize':24})
   a=np.zeros((2,2),dtype=dtype)
   a['x']=10;a['y']=20;a['z']=30;a['range']=2;a['status']=1
   for k in ('x','y','z'):a[k][1,1]=np.nan
   a['range'][1,1]=np.inf;a['status'][1,1]=0
   m=PointCloud2();m.header.frame_id='world';m.header.stamp.secs=12;m.header.stamp.nsecs=345
   m.height=2;m.width=2;m.point_step=24;m.row_step=48;m.is_bigendian=big;m.data=a.tobytes()
   m.fields=[PointField(n,o,t,1) for n,o,t in zip(dtype.names,[0,4,8,12,16,18,20],[7,7,7,7,4,4,2])]
   with tempfile.TemporaryDirectory() as d:
    f=Path(d)/'cloud.npz';self.assertEqual(save_cloud(f,m,epoch=2),3)
    with np.load(f) as loaded:
     self.assertEqual(int(loaded['simulation_epoch']),2)
     self.assertEqual(str(loaded['frame_id']),'world');self.assertEqual(int(loaded['stamp_nsec']),345)
     for k in dtype.names:np.testing.assert_equal(loaded[k],a[k])
     self.assertEqual(str(loaded['range_reference']),'sensor');self.assertEqual(int(loaded['coordinate_schema']),2)
if __name__=='__main__':unittest.main()
