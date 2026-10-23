#!/usr/bin/env python3
"""仿真验收适配器：复用上游 ClusterNode，不修改其他模块源码。"""
import importlib.util
from pathlib import Path

import rospkg
import rospy

source = Path(rospkg.RosPack().get_path('uav_tracking')) / 'scripts/cluster_node.py'
spec = importlib.util.spec_from_file_location('upstream_cluster_node', str(source))
upstream = importlib.util.module_from_spec(spec)
spec.loader.exec_module(upstream)


class AuditedCluster(upstream.ClusterNode):
    def __init__(self):
        self.audit = open(rospy.get_param('~audit_file'), 'w', buffering=1)
        self.audit.write('seq,stamp_sec,stamp_nsec,frame_id,finite_points,clusters\n')
        super().__init__()
        rospy.on_shutdown(self.audit.close)
        rospy.set_param('~ready', True)

    def on_points(self, msg):
        super().on_points(msg)
        # Audit receipt only after the original callback has successfully published.
        points = upstream.cloud_to_points(msg)
        self.audit.write('%d,%d,%d,%s,%d,\n' % (
            msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,
            msg.header.frame_id, len(points)))


if __name__ == '__main__':
    rospy.init_node('cluster')
    AuditedCluster()
    rospy.spin()
