"""Physical geometry and SDF export contracts, independent of a ROS runtime."""
import math
import shutil
import struct
import sys
import unittest
import xml.etree.ElementTree as ET

from test_model_contract import ROOT, expand
sys.path.insert(0, str(ROOT / 'src/k9_robot_bringup'))
from k9_robot_bringup.model import add_spherical_caster, generate_sdf


def diagonal(model, name):
    tensor = model.find(f"link[@name='{name}']/inertial/inertia")
    return [float(tensor.get(key)) for key in ('ixx', 'iyy', 'izz')]


class PhysicsContract(unittest.TestCase):
    def test_component_inertias_match_solid_geometry(self):
        model = expand()
        for name, mass, dims in [('battery_link', 5, (.25, .20, .25)),
                                 ('motors_link', 3, (.20, .20, .10))]:
            x, y, z = dims
            expected = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
            for actual, target in zip(diagonal(model, name), expected):
                self.assertAlmostEqual(actual, target)
        for side in ('l', 'r'):
            values = diagonal(model, f'drivewhl_{side}_link')
            self.assertAlmostEqual(values[0], values[2])
            self.assertAlmostEqual(values[1], .5*.0694**2)
            self.assertAlmostEqual(values[0], (3*.0694**2+.025**2)/12)
        for value in diagonal(model, 'ball_caster_link'):
            self.assertAlmostEqual(value, .4*.5*.04**2)
        total = sum(float(m.get('value')) for m in model.findall('link/inertial/mass'))
        self.assertAlmostEqual(total, 29.87)

    def test_two_boxes_enclose_sampled_shell_without_ground_contact(self):
        model = expand()
        boxes = []
        for collision in model.findall("link[@name='base_link']/collision"):
            origin = collision.find('origin')
            centre = [float(v) for v in origin.get('xyz').split()]
            centre[2] += .0694
            pitch = float(origin.get('rpy', '0 0 0').split()[1])
            size = [float(v) for v in collision.find('geometry/box').get('size').split()]
            lowest = centre[2] - (abs(math.sin(pitch))*size[0]+abs(math.cos(pitch))*size[2])/2
            self.assertGreater(lowest, 0)
            boxes.append((centre, pitch, size))
        self.assertEqual(len(boxes), 2)

        def contained(point):
            for centre, pitch, size in boxes:
                x, y, z = [a-b for a, b in zip(point, centre)]
                x, z = math.cos(pitch)*x-math.sin(pitch)*z, math.sin(pitch)*x+math.cos(pitch)*z
                if all(abs(v) <= extent/2+1e-6 for v, extent in zip((x, y, z), size)):
                    return True
            return False

        data = (ROOT / 'src/k9_description/urdf/k9.stl').read_bytes()
        for i in range(struct.unpack_from('<I', data, 80)[0]):
            values = struct.unpack_from('<12f', data, 84+50*i)
            vertices = [(-.5-.01*values[j], 1.905-.01*values[j+1], .01*values[j+2])
                        for j in (3, 6, 9)]
            samples = vertices + [tuple(sum(v[k] for v in vertices)/3 for k in range(3))]
            samples += [tuple((vertices[j][k]+vertices[(j+1)%3][k])/2 for k in range(3))
                        for j in range(3)]
            for point in samples:
                self.assertTrue(contained(point), f'Shell point {point} outside collision envelopes')
        # Empty space alongside the narrow head must no longer be collidable.
        self.assertFalse(contained((.4, .20, .65)))
        self.assertTrue(contained((-.4, 0, .06)))

    def test_caster_retains_nominal_ground_contact_and_conversion_marker(self):
        model = expand()
        self.assertEqual(model.findtext("gazebo[@reference='ball_caster_joint']/preserveFixedJoint"), 'true')
        joint = model.find("joint[@name='ball_caster_joint']")
        z = float(joint.find('origin').get('xyz').split()[2])+.0694
        radius = float(model.find("link[@name='ball_caster_link']/collision/geometry/sphere").get('radius'))
        self.assertAlmostEqual(z-radius, 0)

    def test_native_ball_conversion_preserves_physical_properties(self):
        # Representative sdformat output; validate that modifying the joint
        # cannot move/reweight the ball or silently leave it welded to the base.
        fixture = '''<sdf version="1.10"><model name="k9_robot">
          <link name="base_footprint"/>
          <link name="ball_caster_link"><pose relative_to="ball_caster_joint">0 0 0 0 0 0</pose>
            <inertial><mass>0.5</mass><inertia><ixx>0.00032</ixx><iyy>0.00032</iyy><izz>0.00032</izz></inertia></inertial>
            <collision name="ball"><geometry><sphere><radius>0.04</radius></sphere></geometry>
              <surface><friction><ode><mu>0.2</mu><mu2>0.2</mu2></ode></friction></surface>
            </collision></link>
          <joint name="ball_caster_joint" type="fixed"><parent>base_footprint</parent>
            <child>ball_caster_link</child><pose relative_to="base_footprint">-0.3 0 0.04 0 0 0</pose>
          </joint></model></sdf>'''
        result = ET.fromstring(add_spherical_caster(fixture))
        expected = ET.fromstring(fixture)
        expected.find('model/joint').set('type', 'ball')
        ET.indent(expected, space='  ')
        self.assertEqual(ET.tostring(result), ET.tostring(expected))
        self.assertEqual(add_spherical_caster(add_spherical_caster(fixture)), add_spherical_caster(fixture))
        with self.assertRaisesRegex(ValueError, 'merged'):
            add_spherical_caster('<sdf><model name="k9_robot"/></sdf>')

    @unittest.skipUnless(shutil.which('gz'), 'Gazebo converter is not installed on this host')
    def test_real_sdf_export(self):
        urdf = expand()
        sdf = ET.fromstring(generate_sdf(ET.tostring(urdf, encoding='unicode')))
        self.assertEqual(sdf.find("model/joint[@name='ball_caster_joint']").get('type'), 'ball')
        total = sum(float(m.text) for m in sdf.findall('model/link/inertial/mass'))
        self.assertAlmostEqual(total, 29.87)
        self.assertEqual(len(sdf.findall("model/link[@name='base_footprint']/collision")), 4)


if __name__ == '__main__':
    unittest.main()
