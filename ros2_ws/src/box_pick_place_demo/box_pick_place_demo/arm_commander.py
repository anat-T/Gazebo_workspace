"""Publishes joint-space commands as plain topics (per the demo's explicit
requirement: robot commands go over ROS2 topics, not actions/services) to
joint_trajectory_controller's open-loop command topic, and wraps ik_solver
for "go to this world-frame point" convenience.
"""
import time
from typing import Sequence

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from box_pick_place_demo.ik_solver import JOINT_NAMES, solve_ik_pose_down


HOME_Q = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]


class ArmCommander:

    def __init__(self, node: Node, tf_prefix: str, base_xyz=(0.0, 0.0, 0.0)):
        self._node = node
        self._tf_prefix = tf_prefix
        self._base_xyz = base_xyz
        self._joint_names = [f"{tf_prefix}{j}" for j in JOINT_NAMES]
        self._pub = node.create_publisher(
            JointTrajectory, f"/{tf_prefix}joint_trajectory_controller/joint_trajectory", 10)
        self._last_q = np.array(HOME_Q, dtype=float)

    def _wait_for_subscriber(self, timeout_s: float = 10.0) -> bool:
        # self._pub is freshly created in __init__ - ROS2 discovery matching
        # it to joint_trajectory_controller's already-existing subscriber
        # isn't instant, and a publish before that match completes is
        # silently dropped (default QoS is volatile, no history replay).
        # Same class of bug as the DetachableJoint attach/detach race fixed
        # in grasp_controller.py - "sometimes the robot doesn't move at
        # all" on the very first command matches this exactly. Wait for a
        # real subscriber instead of publishing blind.
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self._node, timeout_sec=0.05)
        self._node.get_logger().error(
            f"arm_commander[{self._tf_prefix}]: no subscriber on joint_trajectory "
            f"after {timeout_s:.1f}s - publishing anyway, command will likely be lost")
        return False

    def move_to_joint_positions(self, q: Sequence[float], duration_s: float = 3.0):
        self._wait_for_subscriber()
        msg = JointTrajectory()
        msg.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        sec = int(duration_s)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_s - sec) * 1e9))
        msg.points = [point]
        self._pub.publish(msg)
        self._last_q = np.array(q, dtype=float)
        self._node.get_logger().info(
            f"arm_commander[{self._tf_prefix}]: -> {list(np.round(q, 3))} over {duration_s}s")
        # Open-loop wait: this demo has no feedback controller loop, just
        # gives the trajectory time to finish before the next command.
        time.sleep(duration_s + 0.5)

    def move_home(self, duration_s: float = 3.0):
        self.move_to_joint_positions(HOME_Q, duration_s)

    def move_to_world_point(self, x: float, y: float, z: float, duration_s: float = 3.0) -> bool:
        """x, y, z in world frame. Converted to this robot's own base frame
        (its spawn offset, including height - the robot is mounted on top
        of its table, not on the ground) before solving IK, since
        ik_solver's forward_kinematics() is relative to the robot's
        base_link."""
        local_target = (x - self._base_xyz[0], y - self._base_xyz[1], z - self._base_xyz[2])
        # Position + top-down orientation, not position-only: an
        # unconstrained solve was reaching grasp points from the side
        # rather than straight down, which looked wrong for a pick motion
        # even though the DetachableJoint "gripper" itself doesn't actually
        # need any particular approach angle to attach.
        q = solve_ik_pose_down(local_target, seed=self._last_q)
        if q is None:
            self._node.get_logger().error(
                f"arm_commander[{self._tf_prefix}]: IK failed for world point "
                f"({x:.3f}, {y:.3f}, {z:.3f})"
            )
            return False
        self.move_to_joint_positions(q, duration_s)
        return True
