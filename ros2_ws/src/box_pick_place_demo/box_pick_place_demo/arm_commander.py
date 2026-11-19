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

from box_pick_place_demo.ik_solver import DEFAULT_SEED, JOINT_NAMES, solve_ik_position, solve_ik_pose_down

# Real UR5e joint values (shoulder_pan, shoulder_lift, elbow, wrist_1,
# wrist_2, wrist_3), read off /joint_states with the real arm jogged to a
# pose confirmed clear of the table, mount, and cables - not derived from
# ik_solver's DEFAULT_SEED, which is only a numerical convergence aid with
# no guarantee of being reachable safely (see ik_solver.py's own docstring
# on DEFAULT_SEED).
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

    def move_to_world_point(self, x: float, y: float, z: float, duration_s: float = 3.0,
                             require_pointing_down: bool = True,
                             prefer_pointing_down: bool = False) -> bool:
        """x, y, z in world frame. Converted to this robot's own base frame
        (its spawn offset, including height - the robot is mounted on top
        of its table, not on the ground) before solving IK, since
        ik_solver's forward_kinematics() is relative to the robot's
        base_link."""
        local_target = (x - self._base_xyz[0], y - self._base_xyz[1], z - self._base_xyz[2])
        # This is a local, gradient-based numerical solver, not a global
        # one - it can fail to find a solution that genuinely exists,
        # purely because the seed it started from led it into a bad local
        # minimum. Confirmed on real hardware: solving from self._last_q
        # (this project's usual seed, for continuity between moves) failed
        # for a point later shown to be reachable (and already physically
        # occupied by the real, correctly-oriented gripper) when solved
        # instead from DEFAULT_SEED. Retry once from there before actually
        # giving up - costs nothing when the first attempt succeeds.
        #
        # The solvers also reject (return None for) elbow-down solutions
        # and ones that put a joint at/below the table - see ik_solver's
        # configuration_ok(). When that rejects the solution reached from
        # self._last_q (most likely right after move_home(): HOME_Q is the
        # arm pointing straight up, which sits exactly between the
        # elbow-up and elbow-down families, so it's a coin flip which one
        # the solver walks into), the remaining seeds are elbow-up by
        # construction: DEFAULT_SEED turned to face the target first,
        # then DEFAULT_SEED as-is.
        facing_seed = np.array(DEFAULT_SEED, dtype=float)
        facing_seed[0] = np.arctan2(local_target[1], local_target[0])

        def _solve(solve_fn):
            for seed in (self._last_q, facing_seed, DEFAULT_SEED):
                q = solve_fn(local_target, seed=seed)
                if q is not None:
                    return q
            return None

        if require_pointing_down:
            # Position + top-down orientation, not position-only: an
            # unconstrained solve was reaching grasp points from the side
            # rather than straight down, which looked wrong for a pick
            # motion even though the DetachableJoint "gripper" itself
            # doesn't actually need any particular approach angle to
            # attach. Only the actual pick/place moment needs this -
            # transit/hover waypoints just need to get the arm into
            # position, and constraining orientation there too shrinks the
            # reachable envelope for no benefit (confirmed on real
            # hardware: a transit point well within plain reach failed IK
            # purely because pointing-down wasn't geometrically possible
            # that high/far out, even though *some* valid orientation was).
            q = _solve(solve_ik_pose_down)
        else:
            q = None
            if prefer_pointing_down:
                # Not hard-required, but try pointing-down first anyway:
                # a fully unconstrained position-only solve leaves
                # orientation to whatever the nullspace bias happens to
                # settle on, which can land far from the previous
                # waypoint's orientation even though position barely
                # moved - confirmed on real hardware, consecutive
                # transit/hover waypoints produced 60-90deg swings in
                # individual joints purely from this, even though every
                # single waypoint solved fine on its own. Preferring
                # pointing-down here keeps the whole approach/retreat
                # path oriented consistently whenever that's actually
                # achievable, and only falls back to a fully free
                # orientation when it isn't (e.g. a basket far enough out
                # that pointing-down genuinely isn't geometrically
                # possible there).
                q = _solve(solve_ik_pose_down)
            if q is None:
                q = _solve(solve_ik_position)
        if q is None:
            self._node.get_logger().error(
                f"arm_commander[{self._tf_prefix}]: IK failed for world point "
                f"({x:.3f}, {y:.3f}, {z:.3f})"
            )
            return False
        self.move_to_joint_positions(q, duration_s)
        return True
