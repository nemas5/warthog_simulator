#!/usr/bin/env bash
set -euo pipefail

PACKAGES=(
  build-essential
  cmake
  pybind11-dev
  python3-catkin-tools
  python3-dev
  python3-pip
  python3-venv
  ros-noetic-warthog-simulator
  ros-noetic-warthog-gazebo
  ros-noetic-warthog-description
  ros-noetic-warthog-control
)

if [ "$(id -u)" -eq 0 ]; then
  APT=(apt-get)
elif command -v sudo >/dev/null 2>&1; then
  APT=(sudo apt-get)
else
  echo "Run this script as root or install sudo." >&2
  exit 1
fi

echo "Updating APT package lists..."
"${APT[@]}" update

echo "Installing system and ROS dependencies..."
"${APT[@]}" install -y "${PACKAGES[@]}"

echo
echo "System dependencies are installed."
echo "Load the ROS Noetic environment in the current shell:"
echo "  source /opt/ros/noetic/setup.bash"
echo
echo "Install the remaining package.xml dependencies with:"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo
echo "Then create and activate the Python environment with:"
echo "  source src/scripts/python_venv_setup.sh"
