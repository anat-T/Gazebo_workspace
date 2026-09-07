#!/bin/bash
# Scene 1: single-robot ball-sort demo (pick the green ball, place it in the
# green basket, ignore the red distractor pair).
export ROS_DOMAIN_ID=33
source /opt/ros/humble/setup.bash
source /home/anat/code/box_pick_and_place_gazebo_demo/ros2_ws/install/setup.bash
ros2 launch box_pick_place_demo ball_sort_demo.launch.py
