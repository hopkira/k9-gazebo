"""Launch the current K9 model with Gazebo Harmonic and ROS 2 Jazzy control."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from k9_robot_bringup.model import generate_sdf


def launch_setup(context):
    description = Path(get_package_share_directory('k9_description'))
    bringup = Path(get_package_share_directory('k9_robot_bringup'))
    controllers = str(description / 'config/controllers.yaml')
    robot_description = xacro.process_file(
        str(description / 'model/k9.urdf.xacro'),
        mappings={'description_share': str(description), 'controllers_file': controllers},
    ).toxml()
    simulation_sdf = generate_sdf(robot_description)
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {"" if gui else "-s "}"{world}"'}.items(),
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'k9_robot', '-string', simulation_sdf, '-z', '0.03'],
        parameters=[{'use_sim_time': True}],
    )

    def spawner(names, ros_args=None):
        args = [*names, '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '120', '--switch-timeout', '30',
                '--param-file', controllers]
        if ros_args:
            args += ['--controller-ros-args', ros_args]
        return Node(package='controller_manager', executable='spawner',
                    arguments=args, output='screen')

    joints = spawner(['joint_state_broadcaster'])
    drive = spawner(['diff_drive_controller'],
                    '--ros-args -r /diff_drive_controller/cmd_vel:=/cmd_vel_nav '
                    '-r /diff_drive_controller/odom:=/odom')
    ears = spawner(['ears_position_controller'])

    def after_success(next_actions):
        def callback(event, _context):
            if event.returncode != 0:
                return [LogInfo(msg='ERROR: K9 spawn/controller startup failed; stopping simulation.'),
                        EmitEvent(event=Shutdown(reason='K9 startup failed'))]
            return next_actions
        return callback

    # Register handlers before starting processes; do not start controllers on failed spawn.
    handlers = [
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_success([joints]))),
        RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=after_success([drive]))),
        RegisterEventHandler(OnProcessExit(target_action=drive, on_exit=after_success([ears]))),
        RegisterEventHandler(OnProcessExit(target_action=ears, on_exit=after_success([]))),
    ]
    resource_paths = os.pathsep.join(filter(None, [
        str(description.parent), str(bringup.parent),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),
        *handlers,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
             output='screen'),
        gazebo,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(bringup / 'config/gazebo_bridge.yaml'),
                          'use_sim_time': True}], output='screen'),
        spawn,
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(description / 'rviz/urdf_config.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    bringup = Path(get_package_share_directory('k9_robot_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(bringup / 'worlds/empty_world.sdf')),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
