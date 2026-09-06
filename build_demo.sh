#!/bin/bash
# Always builds from ros2_ws, regardless of where this script is run from -
# running `colcon build` from the project root instead of ros2_ws/ silently
# creates a second, separate install/build/log tree there (colcon searches
# for packages recursively), out of sync with the real one run_demo.sh uses.
set -e
cd "$(dirname "$0")/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
