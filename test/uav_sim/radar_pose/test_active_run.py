#!/usr/bin/env python3
"""Selection must fail closed for stale masters, dead publishers and ambiguity."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

root=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('active',root/'catkin_ws/src/uav_sim/scripts/find_active_sensor_run.py')
active=importlib.util.module_from_spec(spec);spec.loader.exec_module(active)

class SelectionTests(unittest.TestCase):
 def test_launcher_exports_with_multiple_assignments(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)/'connection.bash';p.write_text('export ROS_MASTER_URI=http://127.0.0.1:12345\nexport GAZEBO_MASTER_URI=http://127.0.0.1:12346\nexport ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 GAZEBO_IP=127.0.0.1\n')
   self.assertEqual(active.connection_env(p)['ROS_HOSTNAME'],'127.0.0.1')
 def test_wrong_master_identity_rejected(self):
  class WrongMaster:
   def getPid(self,*_):return [1,'',1]
   def getParam(self,*_):return [1,'','/another/run']
  with patch.object(active,'connection_env',return_value={'ROS_MASTER_URI':'http://127.0.0.1:12345'}),patch.object(active,'proxy',return_value=WrongMaster()):
   with self.assertRaisesRegex(ValueError,'another run'):active.inspect_run(Path('/this/run'))
 def test_registered_but_dead_publisher_rejected(self):
  class DeadPublisher:
   def getPid(self,*_):return [1,'',1]
   def getParam(self,*_):return [1,'','/this/run']
   def getPublishedTopics(self,*_):return [1,'',list(active.TOPICS.items())]
   def getSystemState(self,*_):return [1,'',[[[t,['/dead']] for t in active.TOPICS],[],[]]]
   def lookupNode(self,*_):raise OSError('publisher is dead')
  with patch.object(active,'connection_env',return_value={'ROS_MASTER_URI':'http://127.0.0.1:12345'}),patch.object(active,'proxy',return_value=DeadPublisher()):
   with self.assertRaisesRegex(ValueError,'not alive'):active.inspect_run(Path('/this/run'))
 def run_main(self,items,args):
  out,err=io.StringIO(),io.StringIO()
  with patch.object(active,'discover',return_value=(items,[])),patch('sys.argv',['find_active_sensor_run.py',*args]),contextlib.redirect_stdout(out),contextlib.redirect_stderr(err):code=active.main()
  return code,out.getvalue(),err.getvalue()
 def test_no_instance_has_start_instruction(self):
  code,out,err=self.run_main([],[]);self.assertEqual(code,1);self.assertIn('请先启动',err);self.assertEqual(out,'')
 def test_multiple_instances_requires_choice(self):
  items=[(Path('/runs/one'),{'ROS_MASTER_URI':'http://127.0.0.1:1'}),(Path('/runs/two'),{'ROS_MASTER_URI':'http://127.0.0.1:2'})]
  code,out,err=self.run_main(items,[]);self.assertEqual(code,2);self.assertEqual(out,'');self.assertIn('pose --run one',err);self.assertIn('record --run two',err)
  code,out,err=self.run_main(items,['--run','two']);self.assertEqual(code,0);self.assertEqual(out.strip(),'/runs/two')

if __name__=='__main__':unittest.main()
