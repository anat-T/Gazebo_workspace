#!/bin/bash
# Scene 1 on REAL hardware: a real UR5e + OnRobot 2FG7 gripper, no Gazebo,
# no vision - fixed coordinates (see real_ball_sort_node.py) and a real
# gripper (see real_grasp_controller.py) instead of the simulated
# DetachableJoint.
#
# To dry-run without touching the real arm first (recommended after any
# code change): ./run_scene1_real.sh --fake
set -e

ROBOT_IP=10.240.10.2
GRIPPER_IP=10.240.10.2    # confirmed reachable at this same host, port 41414
KINEMATICS_FILE=/home/anat/clair_ur5e_2.yaml

USE_FAKE_HARDWARE=false
if [ "$1" = "--fake" ]; then
    USE_FAKE_HARDWARE=true
fi

export ROS_DOMAIN_ID=133
source /opt/ros/humble/setup.bash
source /home/anat/code/box_pick_and_place_gazebo_demo/ros2_ws/install/setup.bash
ros2 launch box_pick_place_demo real_scene1_demo.launch.py \
    robot_ip:="$ROBOT_IP" \
    gripper_ip:="$GRIPPER_IP" \
    kinematics_params_file:="$KINEMATICS_FILE" \
    use_fake_hardware:="$USE_FAKE_HARDWARE" \
    launch_mission:=true
