"""Static model/interface checks; run live dynamics checks on Jazzy/Harmonic."""
import math
from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET

import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src/k9_robot/src/description/k9.urdf.xacro'
DESCRIPTION = ROOT / 'src/k9_description'
CONTROLLERS = DESCRIPTION / 'config/controllers.yaml'
BRIDGE = ROOT / 'src/k9_robot_bringup/config/gazebo_bridge.yaml'


def expand(share=DESCRIPTION, sim='true'):
    return ET.fromstring(xacro.process_file(str(SOURCE), mappings={
        'description_share': str(share),
        'controllers_file': str(share / 'config/controllers.yaml'),
        'sim': sim,
    }).toxml())


class ModelContract(unittest.TestCase):
    def test_tree_and_sensor_frames_are_connected(self):
        model = expand()
        links = {link.attrib['name'] for link in model.findall('link')}
        parents = {}
        for joint in model.findall('joint'):
            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']
            self.assertIn(parent, links)
            self.assertIn(child, links)
            self.assertNotIn(child, parents)
            parents[child] = parent
        self.assertEqual(links - parents.keys(), {'base_footprint'})
        for link in links:
            visited = set()
            while link in parents:
                self.assertNotIn(link, visited)
                visited.add(link)
                link = parents[link]
            self.assertEqual(link, 'base_footprint')
        for sensor in model.findall('.//sensor'):
            self.assertIn(sensor.findtext('gz_frame_id'), links,
                          f'{sensor.attrib["name"]} has no resolvable ROS frame')

    def test_ear_beams_follow_tilted_servo_axes(self):
        model = expand()
        for side in ('l', 'r'):
            servo = model.find(f'joint[@name="{side}_ear_joint"]')
            roll, pitch, yaw = map(float, servo.find('origin').get('rpy').split())
            self.assertAlmostEqual(pitch, math.radians(7))
            self.assertEqual((roll, yaw), (0, 0))
            self.assertEqual(servo.find('axis').get('xyz'), '0 0 1')
            mount = model.find(f'joint[@name="{side}_ear_sensor_joint"]')
            self.assertEqual(mount.find('parent').get('link'), f'{side}_ear_link')
            self.assertEqual(mount.get('type'), 'fixed')
            mount_xyz = list(map(float, mount.find('origin').get('xyz').split()))
            mount_rpy = list(map(float, mount.find('origin').get('rpy').split()))
            self.assertAlmostEqual(mount_rpy[1], math.radians(30))
            self.assertEqual((mount_rpy[0], mount_rpy[2]), (0, 0))
            sensor = model.find(f'gazebo[@reference="{side}_ear_link"]/sensor')
            self.assertEqual(list(map(float, sensor.findtext('pose').split())),
                             mount_xyz + mount_rpy)
            for tag in ('frame_id', 'gz_frame_id'):
                self.assertEqual(sensor.findtext(tag), f'{side}_ear_sensor_link')
            # R_y(axis tilt) R_z(servo angle) R_y(mount pitch) applied to +X.
            beta = mount_rpy[1]
            def beam(q):
                return (math.cos(pitch)*math.cos(q)*math.cos(beta)
                        - math.sin(pitch)*math.sin(beta),
                        math.sin(q)*math.cos(beta),
                        -math.sin(pitch)*math.cos(q)*math.cos(beta)
                        - math.cos(pitch)*math.sin(beta))
            self.assertAlmostEqual(math.asin(-beam(0)[2]), math.radians(37))
            for limit in ('lower', 'upper'):
                q = float(servo.find('limit').get(limit))
                direction = beam(q)
                self.assertAlmostEqual(sum(v*v for v in direction), 1)
                self.assertLess(math.asin(-direction[2]), math.radians(37))
                self.assertGreater(direction[1]*q, 0)

    def test_controller_interfaces_and_drive_geometry(self):
        model = expand()
        config = yaml.safe_load(CONTROLLERS.read_text())
        drive = config['diff_drive_controller']['ros__parameters']
        controls = {j.attrib['name']: j for j in model.findall('./ros2_control/joint')}
        self.assertEqual(len(controls), 4)
        wheel_y = []
        for joint_name in drive['left_wheel_names'] + drive['right_wheel_names']:
            self.assertEqual(controls[joint_name].find('command_interface').attrib['name'], 'velocity')
            self.assertIn('position', {i.attrib['name'] for i in controls[joint_name].findall('state_interface')})
            joint = model.find(f'joint[@name="{joint_name}"]')
            wheel_y.append(float(joint.find('origin').attrib['xyz'].split()[1]))
            link = model.find(f'link[@name="{joint.find("child").attrib["link"]}"]')
            self.assertAlmostEqual(float(link.find('collision/geometry/cylinder').attrib['radius']), drive['wheel_radius'])
        self.assertAlmostEqual(wheel_y[0] - wheel_y[1], drive['wheel_separation'])
        for name in config['ears_position_controller']['ros__parameters']['joints']:
            self.assertEqual(controls[name].find('command_interface').attrib['name'], 'position')
        self.assertGreater(drive['cmd_vel_timeout'], 0)
        self.assertFalse(drive['open_loop'])

    def test_single_drive_and_tf_authority(self):
        model = expand()
        plugins = model.findall('./gazebo/plugin')
        self.assertEqual([p.attrib['name'] for p in plugins],
                         ['gz_ros2_control::GazeboSimROS2ControlPlugin'])
        bridge = yaml.safe_load(BRIDGE.read_text())
        topics = [entry['ros_topic_name'] for entry in bridge]
        self.assertEqual(len(topics), len(set(topics)))
        self.assertFalse({'/tf', '/odom', '/joint_states', '/cmd_vel_nav'} & set(topics))
        self.assertTrue(all(entry['direction'] == 'GZ_TO_ROS' for entry in bridge))
        sensors = {s.findtext('topic') for s in model.findall('.//sensor')}
        for topic in sensors - {'/oak'}:
            self.assertIn(topic, {entry['gz_topic_name'] for entry in bridge})
        for suffix in ['image', 'depth_image', 'camera_info', 'points']:
            self.assertIn('/oak/' + suffix, topics)

    def test_relocated_install_with_spaces(self):
        with tempfile.TemporaryDirectory(prefix='k9 install ') as tmp:
            share = Path(tmp) / 'share/k9_description'
            shutil.copytree(DESCRIPTION, share)
            model = expand(share)
            mesh_path = model.find('.//mesh').attrib['filename']
            self.assertTrue(mesh_path.startswith('file://' + str(share)))
            self.assertTrue(Path(mesh_path.removeprefix('file://')).is_file())
            controller_path = model.findtext('./gazebo/plugin/parameters')
            self.assertTrue(Path(controller_path).is_file())
            self.assertNotIn('/home/hopkira', ET.tostring(model, encoding='unicode'))

    def test_display_does_not_start_control(self):
        model = expand(sim='false')
        self.assertIsNone(model.find('ros2_control'))
        self.assertEqual(model.findall('./gazebo/plugin'), [])

    def test_xml_resources_parse(self):
        for pattern in ['src/**/package.xml', 'src/**/*.launch.xml', 'worlds/*.sdf']:
            for path in ROOT.glob(pattern):
                with self.subTest(path=path):
                    ET.parse(path)


if __name__ == '__main__':
    unittest.main()
