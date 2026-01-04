#!/usr/bin/env bash
set -euo pipefail

# Run from the directory where the xacro and patch script live
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "[1/4] Generating URDF from xacro..."
ros2 run xacro xacro k9.urdf.xacro > k9.urdf

echo "[2/4] Converting URDF -> SDF..."
gz sdf -p k9.urdf > k9_raw.sdf

echo "[3/4] Patching friction..."
python3 patch_friction.py k9_raw.sdf k9_robot.sdf

echo "[4/4] Done. Output: $SCRIPT_DIR/k9_robot.sdf"
