#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/.." && pwd)"
BAG_DIR="${1:-${WORKSPACE_DIR}/bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${BAG_DIR}/run_${STAMP}.bag"

mkdir -p "${BAG_DIR}"
echo "Recording to ${BAG} ..."

rosbag record \
  -O "${BAG}" \
  /ground_truth/odom \
  /odometry/filtered \
  /warthog_velocity_controller/odom \
  /imu/data \
  /cmd_vel \
  /terrain/surface_type
