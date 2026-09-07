#!/bin/bash
# Scene 3: two green balls, two green baskets, same positions as scene 1.
# Color can't tell them apart - the robot picks the ball on its RIGHT and
# places it in the basket on its LEFT (a cross-diagonal move).
export ROS_DOMAIN_ID=33
source /opt/ros/humble/setup.bash
source /home/anat/code/box_pick_and_place_gazebo_demo/ros2_ws/install/setup.bash
ros2 launch box_pick_place_demo ball_sort_demo3.launch.py
