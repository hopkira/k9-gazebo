#!/usr/bin/env bash
set -euo pipefail

# Source the built workspace first. The same converter is used by the launch:
# expand Xacro, convert with gz sdf, and replace the preserved caster joint with
# a native SDF ball joint. Do not spawn raw URDF: it retains a fixed caster frame.
exec ros2 run k9_robot_bringup export_model "${1:-/tmp/k9-generated-model}"
