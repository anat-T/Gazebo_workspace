#!/bin/bash
# Scene 2: same ball-sort setup as scene 1, but one plain gray basket
# instead of the red/green pair - pick the green ball, drop it in the
# (only) basket.
export ROS_DOMAIN_ID=33
source /opt/ros/humble/setup.bash
source /home/anat/code/box_pick_and_place_gazebo_demo/ros2_ws/install/setup.bash
ros2 launch box_pick_place_demo ball_sort_demo2.launch.py
