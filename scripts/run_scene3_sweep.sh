#!/bin/bash
# Runs scene3 (ball_sort_demo3.launch.py) N times with randomized ball/
# basket layouts, recording each run's rosbag + video + ee trajectory into
# scene3_runs/scene3_run{i}/. See run_scene3_sweep.py's docstring for
# details; pass extra args through, e.g. --num-runs 3 --gui --seed 1.
export ROS_DOMAIN_ID=33
source /opt/ros/humble/setup.bash
source /home/anat/code/box_pick_and_place_gazebo_demo/ros2_ws/install/setup.bash
python3 "$(dirname "$0")/run_scene3_sweep.py" "$@"
