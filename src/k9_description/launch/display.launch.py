"""Display the shared model without simulation control or shell path quoting."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory('k9_description'))
    model = xacro.process_file(str(share / 'model/k9.urdf.xacro'), mappings={
        'description_share': str(share),
        'controllers_file': str(share / 'config/controllers.yaml'),
        'sim': 'false',
    }).toxml()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': model}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(share / 'rviz/urdf_config.rviz')]),
    ])
