#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from box_pick_place_demo.arm_commander import ArmCommander, HOME_Q

# How far to nudge the shoulder_pan joint, in radians, and how slowly -
# small and slow on purpose for a first-ever real move. 0.1 rad ~= 5.7deg.
NUDGE_RAD = 0.1
MOVE_DURATION_S = 10.0


def main():
    rclpy.init()
    node = Node("test_arm_move")
    # tf_prefix="" matches real_scene1_demo.launch.py's default (no
    # prefix) - must match whatever tf_prefix you actually launched with.
    arm = ArmCommander(node, tf_prefix="", base_xyz=(0.0, 0.0, 0.0))

    node.get_logger().info("test_arm_move: moving to home position...")
    arm.move_home(duration_s=MOVE_DURATION_S)

    nudged_q = np.array(HOME_Q, dtype=float)
    nudged_q[0] += NUDGE_RAD
    node.get_logger().info(f"test_arm_move: nudging shoulder_pan by {NUDGE_RAD} rad...")
    arm.move_to_joint_positions(list(nudged_q), duration_s=MOVE_DURATION_S)

    node.get_logger().info("test_arm_move: returning home...")
    arm.move_home(duration_s=MOVE_DURATION_S)

    node.get_logger().info("test_arm_move: done - if the arm visibly moved and returned home, motion works.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
