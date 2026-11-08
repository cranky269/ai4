#!/usr/bin/env python3
"""Offline compatibility regression, not live simulation evidence."""
import importlib.util
from pathlib import Path
import unittest
from geometry_msgs.msg import PointStamped, PoseStamped
from uav_tracking.msg import CentroidArray

spec=importlib.util.spec_from_file_location('m0_world_view',Path(__file__).with_name('world_centroid_view.py'))
view=importlib.util.module_from_spec(spec);spec.loader.exec_module(view)

class WorldViewTests(unittest.TestCase):
 def point(self, stamp=7, frame='world'):
  p=PointStamped();p.header.frame_id=frame;p.header.stamp.secs=stamp
  p.point.x=4.25;p.point.y=-1.5;p.point.z=1.42
  return p
 def test_world_values_are_unchanged(self):
  p=self.point();self.assertEqual(view.world_xyz(p),(4.25,-1.5,1.42))
  pose=PoseStamped();pose.header=p.header;pose.pose.position.x=4.25
  pose.pose.position.y=-1.5;pose.pose.position.z=1.42
  pose.pose.orientation.z=1;pose.pose.orientation.w=0
  self.assertEqual(view.world_xyz(pose),(4.25,-1.5,1.42))
 def test_old_or_unknown_frame_is_rejected(self):
  for frame in ('radar_link','map',''):
   with self.assertRaisesRegex(ValueError,'Expected world'):view.world_xyz(self.point(frame=frame))
 def test_same_sample_centroids(self):
  m=CentroidArray();m.centroids=[self.point(),self.point()]
  stamp,xyz=view.centroid_values(m)
  self.assertEqual(stamp,7000000000);self.assertEqual(xyz,[(4.25,-1.5,1.42)]*2)
  self.assertEqual(view.centroid_values(CentroidArray()),(None,[]))
 def test_mixed_stamp_and_nonfinite_rejected(self):
  m=CentroidArray();m.centroids=[self.point(),self.point(8)]
  with self.assertRaisesRegex(ValueError,'mixed acquisition'):view.centroid_values(m)
  p=self.point();p.point.z=float('nan')
  with self.assertRaisesRegex(ValueError,'Nonfinite'):view.world_xyz(p)
if __name__=='__main__':unittest.main()
