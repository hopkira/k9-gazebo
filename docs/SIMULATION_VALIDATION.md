# Jetson simulation validation — 2026-09-20

Host: k9-ros2-jetson.local, Ubuntu 24.04 / ROS 2 Jazzy. Separate workspace:
`~/k9_sim_ws`, repository `src/k9-gazebo`, model baseline commit `44a0a9f`.
Hardware ROS nodes remain in domain 9. All tests used domain 19, localhost
ROS discovery and Gazebo transport partition `k9_model_validation`.

## Installation and rendering

Installed `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control` and
`ros-jazzy-joint-state-publisher-gui`; ros2 controllers were already installed.
Gazebo control plugin version: 1.2.20; ros-gz version: 1.0.24.
Desktop glxinfo: direct rendering, NVIDIA Tegra Orin, OpenGL 4.6 NVIDIA 595.78.
Gazebo Ogre2 log independently confirmed NVIDIA Tegra Orin / NVIDIA Corporation,
OpenGL 4.5. The test ran server-only with access to the desktop X display;
this was not a validation of display-free EGL rendering.

EGL probes for other devices produced warnings before successful NVIDIA rendering.
The first incomplete launch (missing controller package) left a Gazebo server
behind. It was stopped before the clean run; check for leftover servers after a
failed startup rather than starting another instance in the same partition.

## Results

- Both packages built; all 12 model tests passed, including SDF conversion.
- joint_state_broadcaster, diff_drive_controller and ears_position_controller active.
- Stationary odometry remained effectively zero.
- A 0.1 m/s forward command for 4 simulation seconds produced about 0.380 m
  wheel odometry. After command publication stopped, the watchdog and ramp stopped
  the robot at about 0.445 m. Gazebo world pose independently showed x=0.44468 m
  and essentially zero ground-frame height/tilt. The ball caster orientation changed.
- Reverse at -0.1 m/s for 4 seconds moved odometry back to x=0.0635 m; an explicit
  stop settled at x=0.0448 m.
- Turn at 0.15 rad/s for 4 seconds produced 0.548 rad yaw; cessation of commands
  stopped at 0.672 rad. Final reported linear/angular velocities were effectively zero.
- All sensor topics delivered data: LD06, left/right ear scans, IMU, OAK points.
  The simulated OAK cloud is 640x400 (not the real camera's 320x200 configuration).
- TF resolved from base_footprint to every sensor message frame: base_laser,
  l_ear_sensor_link, r_ear_sensor_link, imu_link and camera_depth_frame.
  odom -> base_footprint also resolved.
- Ear commands +0.2/-0.2 rad reached their targets; resetting both to zero returned
  measured positions to within 3e-12 rad of zero.
- A settled Gazebo stats sample reported real-time factor 0.99983 and 1 ms step.
  Startup and sensor-heavy phases can run slower; this is not a sustained benchmark.

These are model smoke tests, not a calibrated motor simulation or a navigation
acceptance test. Obstacle detection accuracy, contact behaviour against furniture,
kitchen world appearance, prolonged stability and realistic friction remain to be
validated. The model's OAK mount is still awaiting physical calibration.

## Reproduce

In a Jetson desktop terminal (so DISPLAY and X authentication are available):

```bash
source /opt/ros/jazzy/setup.bash
source ~/k9_sim_ws/install/local_setup.bash
export ROS_DOMAIN_ID=19 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp GZ_PARTITION=k9_model_validation
unset CYCLONEDDS_URI
ros2 launch k9_robot_bringup k9_robot_gazebo.launch.py
```

Use the same environment in a second terminal. `gui:=false rviz:=false` runs the
server-only configuration used for the tests. No simulation launch is added to
normal robot startup. The bounded diagnostic script below publishes simulated
motion commands and refuses to run outside domain 19:

```bash
python3 ~/k9_sim_ws/src/k9-gazebo/tools/check_sim_motion.py
```

The script reports measurements; it does not encode pass/fail thresholds for
physical calibration. Test logs reside at `~/k9_sim_ws/simulation-test.log` and
`~/.gz/rendering/ogre2.log`. The test simulation was stopped after validation.
