#!/usr/bin/env python3
"""Replays a real-robot joint configuration on the SIMULATED arm, for a
visual preview before trusting it on real hardware - moves the sim robot to
the exact same joint angles the real UR5e reported, no IK/coordinate
translation involved (joint angles are robot-geometry-independent, so the
same six numbers apply to any UR5e regardless of its mounting or a gripper's
length - unlike a world-frame XYZ target, which is not portable between the
real robot's base_xyz=(0,0,0) and the sim's base_xyz=(0,0,0.75) without
re-deriving it).

Run with the Gazebo sim already up (e.g. ./scripts/run_scene1_demo.sh
launch_mission:=false), matching ball_sort_node.py's own tf_prefix/base_xyz.
"""
import rclpy
from rclpy.node import Node

from box_pick_place_demo.arm_commander import ArmCommander, HOME_Q

MOVE_DURATION_S = 5.0

# shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 - read from
# the real UR5e's /joint_states while its gripper was jogged to touch the
# real box at (-0.66, -0.163, 0.319) (pendant/Base-frame reading).
REAL_ROBOT_Q = [
    0.06797242164611816,
    -1.1307685536197205,
    1.1015585104571741,
    -1.531087690298893,
    -1.628430191670553,
    0.0007288455963134766,
]


def main():
    rclpy.init()
    node = Node("preview_pose_in_sim")
    # Must match ball_sort_node.py's own ArmCommander construction.
    arm = ArmCommander(node, tf_prefix="robot_", base_xyz=(0.0, 0.0, 0.75))

    node.get_logger().info("preview_pose_in_sim: moving to home position...")
    arm.move_home(duration_s=MOVE_DURATION_S)

    node.get_logger().info("preview_pose_in_sim: moving to the real robot's captured pose...")
    arm.move_to_joint_positions(REAL_ROBOT_Q, duration_s=MOVE_DURATION_S)

    node.get_logger().info(
        "preview_pose_in_sim: done - the sim arm is now in the same configuration "
        "the real arm was in when it touched the box.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
