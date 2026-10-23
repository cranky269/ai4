#!/usr/bin/env python3
"""加载真实多层Ray和可选静态几何验收靶，不生成或修改任何测距数据。"""
import math
import xml.etree.ElementTree as ET
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def main():
    rospy.init_node('spawn_radar_3d')
    tree = ET.parse(rospy.get_param('~model_file'))
    root = tree.getroot()
    values = {}
    for key, path in {
        'horizontal_samples': './/horizontal/samples',
        'horizontal_min_angle': './/horizontal/min_angle',
        'horizontal_max_angle': './/horizontal/max_angle',
        'vertical_samples': './/vertical/samples',
        'vertical_min_angle': './/vertical/min_angle',
        'vertical_max_angle': './/vertical/max_angle',
        'range_min': './/range/min', 'range_max': './/range/max',
        'range_resolution': './/range/resolution',
        'update_rate': './/update_rate', 'noise_stddev': './/noise/stddev',
        'noise_mean': './/noise/mean',
    }.items():
        value = float(rospy.get_param('~' + key))
        if not math.isfinite(value):
            raise ValueError(key + ' must be finite')
        if key.endswith('samples'):
            if int(value) != value or value < 2 or value > 65535:
                raise ValueError(key + ' must be integer >=2 and <=65535')
            value = int(value)
        values[key] = value
        root.find(path).text = str(value)
    for axis in ['horizontal', 'vertical']:
        lo, hi = values[axis + '_min_angle'], values[axis + '_max_angle']
        if not lo < hi or lo < -math.pi or hi > math.pi:
            raise ValueError(axis + ' requires nonzero ordered FOV in [-pi, pi]')
    if abs(values['vertical_min_angle']) >= math.pi / 2 or abs(values['vertical_max_angle']) >= math.pi / 2:
        raise ValueError('vertical angles must be within (-pi/2, pi/2)')
    if not 0 < values['range_min'] < values['range_max'] or values['noise_stddev'] < 0:
        raise ValueError('require 0 < range_min < range_max and noise_stddev >= 0 m')
    if values['update_rate'] <= 0 or values['range_resolution'] <= 0:
        raise ValueError('rate/resolution must be positive')
    if values['horizontal_samples'] * values['vertical_samples'] > 1000000:
        raise ValueError('too many rays (limit 1000000)')
    xyz = [float(rospy.get_param('~' + axis)) for axis in ['x', 'y', 'z']]
    if not all(math.isfinite(v) for v in xyz):
        raise ValueError('installation xyz must be finite')
    rospy.set_param('~effective_sdf', ET.tostring(root, encoding='unicode'))
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=90)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    def load(name, xml, position):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1
        result = spawn(name, xml, '/radar', pose, 'world')
        if not result.success:
            raise RuntimeError(name + ': ' + result.status_message)

    # Fixture fronts at local x=5m, five different physical heights; no truth publisher.
    if rospy.get_param('~calibration', False):
        for index, degrees in enumerate([-10, -5, 0, 5, 10]):
            z = xyz[2] + 5 * math.tan(math.radians(degrees))
            xml = '''<sdf version="1.6"><model name="board"><static>true</static>
              <link name="board"><collision name="surface"><geometry><box><size>0.2 2.4 0.18</size></box></geometry></collision>
              <visual name="surface"><geometry><box><size>0.2 2.4 0.18</size></box></geometry>
              <material><ambient>0.2 0.8 0.3 1</ambient><diffuse>0.2 0.8 0.3 1</diffuse></material></visual>
              </link></model></sdf>'''
            load('radar_board_' + str(index), xml, [xyz[0] + 5.1, xyz[1], z])
    load('radar_3d', ET.tostring(root, encoding='unicode'), xyz)
    rospy.loginfo('Real 3D Ray loaded: %s; xyz=%s; zero-mean native noise only', values, xyz)
    rospy.spin()


if __name__ == '__main__':
    main()
