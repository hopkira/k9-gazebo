"""Convert K9 URDF to SDF and enable its native passive spherical caster."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def add_spherical_caster(sdf_text):
    """Keep the URDF's preserved caster link, but release all three rotations.

    URDF has no ball-joint type. Its fixed caster frame is only a nominal ROS
    frame; the sphere has no sensor or orientation-dependent collision shape.
    Fail loudly if conversion lost the joint/link instead of simulating a skid.
    """
    root = ET.fromstring(sdf_text)
    model = root.find("model[@name='k9_robot']")
    if model is None:
        raise ValueError('SDF conversion did not produce the k9_robot model')
    joint = model.find("joint[@name='ball_caster_joint']")
    sphere = model.find("link[@name='ball_caster_link']")
    if joint is None or sphere is None:
        raise ValueError('Caster was merged during conversion; preserveFixedJoint is required')
    if joint.get('type') not in ('fixed', 'ball'):
        raise ValueError('Unexpected caster joint type: ' + str(joint.get('type')))
    if joint.findtext('child') != 'ball_caster_link':
        raise ValueError('Caster joint does not connect to ball_caster_link')
    if sphere.find('collision/geometry/sphere') is None:
        raise ValueError('Caster collision must be a sphere')
    joint.set('type', 'ball')
    # A native spherical joint does not use scalar joint axes or limits.
    for tag in ('axis', 'axis2'):
        for axis in joint.findall(tag):
            joint.remove(axis)
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')


def generate_sdf(urdf_text):
    """Use the host's Gazebo converter, then make the caster genuinely passive."""
    with tempfile.TemporaryDirectory(prefix='k9-sdf-') as tmp:
        urdf = Path(tmp) / 'k9.urdf'
        urdf.write_text(urdf_text)
        result = subprocess.run(
            ['gz', 'sdf', '-p', str(urdf)], capture_output=True, text=True,
            check=False, timeout=60,
        )
    if result.returncode != 0:
        raise RuntimeError('Gazebo URDF conversion failed: ' + result.stderr.strip())
    return add_spherical_caster(result.stdout)
