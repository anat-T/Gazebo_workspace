#!/bin/bash
# Runs scene4 (ball_sort_demo4.launch.py: all blue, 3 balls in a tight row, 3 baskets in a row + 1 off it) N times with randomized ball/
# basket layouts, recording each run's rosbag + video + ee trajectory into
# scene4_runs/scene4_run{i}/. See run_scene4_sweep.py's docstring for
# details; pass extra args through, e.g. --num-runs 3 --gui --seed 1 --ball-gap 0.002.
export ROS_DOMAIN_ID=33
source /opt/ros/humble/setup.bash
source /home/anat/code/box_pick_and_place_gazebo_demo/ros2_ws/install/setup.bash
python3 "$(dirname "$0")/run_scene4_sweep.py" "$@"
