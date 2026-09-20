# K9 Gazebo simulation

Targets **ROS 2 Jazzy / Gazebo Harmonic** on Ubuntu 24.04.

## Model and launch

The current source is `src/k9_robot/src/description/k9.urdf.xacro`, installed as
`share/k9_description/model/k9.urdf.xacro`. Both simulation launch entry points
use it. `src/k9_description/urdf/k9_urdf.xacro` and `original_urdf.urdf` are legacy
models and are not used by the launch files.

From this repository in a ROS workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
# Keep simulation separate from K9 hardware (domain 9).
export ROS_DOMAIN_ID=19 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=k9_model_validation
unset CYCLONEDDS_URI
ros2 launch k9_robot_bringup k9_robot_gazebo.launch.py
```

The existing `k9_robot_gazebo.launch.xml` and
`ros2 launch k9_description k9_gazebo.launch.xml` forward to the same launch.
The latter compatibility entry point requires `k9_robot_bringup` to be installed.
For model inspection only: `ros2 launch k9_description display.launch.xml`.

Options:

```bash
ros2 launch k9_robot_bringup k9_robot_gazebo.launch.py gui:=false rviz:=false
ros2 launch k9_robot_bringup k9_robot_gazebo.launch.py \
  world:="$(ros2 pkg prefix --share k9_robot_bringup)/worlds/kitchen.sdf"
```

The default empty world is self-contained. Meshes, worlds and controller YAML
resolve from installed package shares; the launch preserves and extends
`GZ_SIM_RESOURCE_PATH`. The kitchen mesh is installed with its world. Its pre-existing material file
references floor textures absent from this repository, so those surfaces may
render without their intended textures.
All simulation ROS nodes use simulation time.

The launch converts the current Xacro to SDF before spawning, including a native
passive ball joint for the rolling caster. See [physics notes](docs/PHYSICS.md)
for collision envelopes, corrected inertias, and the motor-model recommendation.

## Drive and sensor interfaces

A single `gz_ros2_control` manager owns the two wheel velocity interfaces and
two ear position interfaces. Startup spawns the model, then activates
`joint_state_broadcaster`, `diff_drive_controller`, and
`ears_position_controller`. A failed startup step shuts down the launch.
There is no Gazebo DiffDrive plugin or second odometry publisher.

`src/k9_description/config/controllers.yaml` defines the drive geometry
(0.2022 m track, 0.0694 m wheel radius), wheel-feedback odometry, velocity and
acceleration limits, and a 0.5 s command timeout. Recheck these dimensions if the
Xacro geometry changes.

| ROS topic | Type | Owner / use |
| --- | --- | --- |
| `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | Input to diff-drive controller |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel-feedback odometry from controller |
| `/tf` | `tf2_msgs/msg/TFMessage` | Controller: `odom → base_footprint`; robot_state_publisher: moving robot joints |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Fixed robot joints from robot_state_publisher |
| `/joint_states` | `sensor_msgs/msg/JointState` | ROS joint-state broadcaster |
| `/ears_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Left, right ear positions in radians |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo clock bridge |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo `/ld06/scan` bridge |
| `/k9/imu` | `sensor_msgs/msg/Imu` | IMU bridge |
| `/l_ear/scan`, `/r_ear/scan` | `sensor_msgs/msg/LaserScan` | Ear range-sensor bridges |
| `/oak/image`, `/oak/depth_image` | `sensor_msgs/msg/Image` | RGB-D image bridges |
| `/oak/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB-D intrinsics bridge |
| `/oak/points` | `sensor_msgs/msg/PointCloud2` | RGB-D point-cloud bridge |

Jazzy's `diff_drive_controller` requires **TwistStamped** with a current timestamp
in simulation time. An unstamped `Twist` publisher is not compatible. When Nav2
is added, configure stamped output throughout its velocity pipeline and remap
its final command output to `/cmd_vel_nav`. Existing teleoperation that publishes
`Twist` will need an explicit stamped adapter or publisher update.
The Gazebo bridge carries only clock and sensors; it does not bridge commands,
odometry, joint states, or TF.

## Verification on the simulation host

After launching, check that all three controllers are `active`:

```bash
ros2 control list_controllers
ros2 topic hz /clock
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_laser
```

In another terminal, send a slow command (simulation only):

```bash
ros2 topic pub --rate 10 /cmd_vel_nav geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}' \
  --ros-args -p use_sim_time:=true
```

Check forward movement, changing odometry and rotating wheel joints; stop the
publisher and verify the command timeout stops the robot. Repeat with angular
velocity, and check all sensor topics and their TF frames. Nav2 itself is not
launched by this repository yet.

To export a standalone model on a configured ROS/Gazebo host, run
`bash src/k9_robot/src/description/gen.sh [output-directory]` (default:
`/tmp/k9-generated-model`). Generated URDF/SDF files contain machine-specific
resolved paths and are no longer checked in. The older friction patch script is
retained for historical exports; normal generation uses friction in the Xacro. Both export and launch use the same
SDF converter; spawning raw URDF would retain the fixed nominal caster frame.

Static regression checks (requires Python `xacro` and `PyYAML`):

```bash
python3 -m unittest discover -s tests -v
```

These checks do not substitute for a live Jazzy/Harmonic run.


## Review and validation snapshot (2026-09-20)

The simulation and sensor-geometry changes have been reviewed together. Generated
URDF/SDF exports are intentionally removed from version control; regenerate them
from the installed Xacro using the command above. The source Xacro matches the
snapshot deployed for the real robot description; final OAK mounting calibration
is still outstanding (see the physics notes).

In an isolated Ubuntu 24.04/Jazzy workspace on the Pi, all 12 model tests passed,
including the real Gazebo SDF conversion test. Both `k9_description` and
`k9_robot_bringup` built successfully. Full controller activation, sensor rendering
and physical settling have not been validated: the host lacks the complete
simulation control stack. No changes were applied to the running hardware launch.

Review also corrected an unsupported Jazzy `LogError` action and replaced fragile
XML command quoting with a Python display launch. Simulation and display launch
argument introspection now pass. The installed exporter runs successfully and
`gz sdf -k` reports `Valid`, with extension warnings for sensor frame tags.
The `gz_ros2_control` dependency belongs to simulation bring-up, so the shared
description package no longer requires that simulation controller plugin.


A live Jetson Orin NX run is now verified: GPU sensor rendering, controller
activation, drive/stop behaviour, ear actuation and sensor TF. See
[Jetson simulation validation](docs/SIMULATION_VALIDATION.md) for measurements,
reproduction commands and remaining limits.
