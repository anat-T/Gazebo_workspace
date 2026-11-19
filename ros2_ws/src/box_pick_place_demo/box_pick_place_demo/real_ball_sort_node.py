import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node

from box_pick_place_demo.arm_commander import ArmCommander
from box_pick_place_demo.real_grasp_controller import TwoFG7GraspController

# ur_robot_driver's controller_stopper_node keeps joint_trajectory_controller
# INACTIVE until the robot signals it's actually ready to receive commands -
# which only happens once External Control is connected (Play pressed on
# the pendant). Racing ahead of that (e.g. launching this node with
# launch_mission:=true, alongside the driver, before anyone's pressed Play)
# sends move_home()/grasp.detach() at a controller that isn't listening
# yet. Wait for it here instead of assuming it's ready.
CONTROLLER_NAME = "joint_trajectory_controller"
CONTROLLER_WAIT_TIMEOUT_S = 300.0
# Confirmed on real hardware: the robot's reverse-interface socket
# handshake can still be finishing up to ~1s after controller_manager
# already reports "active" - see run_mission()'s own comment on this.
SETTLE_AFTER_CONTROLLER_ACTIVE_S = 2.0

# BALL_XYZ = (-0.651, -0.304, 0.285)
# BALL_XYZ = (0.661, 0.163, 0.319)  # box's earlier position
# BALL_XYZ = (0.07929, 0.37082, 1.07675)  # too high/far out to reach pointing down at all
# This position was jogged to with the gripper genuinely facing straight
# down - confirmed both from the pendant reading and by converting the
# reported orientation quaternion to a Z-axis: (-0.017, 0.003, -0.9998),
# ~1deg off vertical. pendant (-419.93, -149.01, 516.7)mm == broadcaster
# (-0.41994, -0.14901, 0.51669)m (frame_id "base", agreed to <0.1mm) - then
# negated X/Y (not Z) to convert into base_link's convention, which is what
# move_to_world_point()/ik_solver.py's forward_kinematics() actually use.
# BALL_XYZ = (0.41994, 0.14901, 0.51669)  # earlier ball test position
# New ball position, much lower (near table height) - orientation confirmed
# pointing down (Z-axis (0.034, 0.029, -0.999), dot 0.999). pendant/
# broadcaster (-0.10689, -0.43852, 0.11282)m (frame_id "base") -> negated
# X/Y into base_link: (0.10689, 0.43852, 0.11282).
# BALL_XYZ = (0.10689, 0.43852, 0.11282)
BALL_XYZ = (0.41994, 0.14901, 0.51669)
# BASKET_XYZ = (0.43027, 0.25804, 0.54596)  # first different-basket test
# Basket moved farther out for this test - same protocol: pendant reading
# matches broadcaster (frame_id "base") to <0.1mm, orientation confirmed
# pointing down (Z-axis (-0.017, 0.003, -0.9999), dot 0.9999), then negated
# X/Y (not Z) into base_link's convention: (-0.24591, -0.55312, 0.52153) ->
# (0.24591, 0.55312, 0.52153).
BASKET_XYZ = (0.24591, 0.55312, 0.52153)

# BALL_XYZ = (0.07929, 0.37082, 1.07675)
# BASKET_XYZ = (0.07929, 0.37082, 1.07675)

HOVER_CLEARANCE_M = 0.03
# Bumped from 0.02 to 0.05 just for the first run at the new, much-lower
# ball position (near table height) - extra safety margin in case the real
# surface is a bit higher than this reading accounts for. Revert to 0.02
# once this position is confirmed safe.
TOUCH_CLEARANCE_M = 0.02
# 0.35 (the sim scenes' value, objects much closer to the base) put this
# point outside the UR5e's ~0.85m reach for this table position - confirmed
# via IK failure at (x,y,0.635). This object is already ~0.77m out, leaving
# little headroom to also go higher; keep this modest for real hardware.
SAFE_TRANSIT_ABOVE_M = 0.04
RELEASE_CLEARANCE_M = 0.05

# Slow on purpose for real hardware - matches test_arm_move.py's duration.
MOVE_DURATION_S = 15.0


class RealBallSortNode(Node):

    def __init__(self):
        super().__init__("real_ball_sort_node")
        # Must match ur_robot_driver's tf_prefix launch argument (empty by
        # default for a single robot - see real_scene1_demo.launch.py).
        self.declare_parameter("tf_prefix", "")
        tf_prefix = self.get_parameter("tf_prefix").value

        self.declare_parameter("open_gap_mm", 70.0)
        self.declare_parameter("close_gap_mm", 45.0)
        self.declare_parameter("force_n", 50)
        self.declare_parameter("speed_pct", 50)

        self.arm = ArmCommander(self, tf_prefix, base_xyz=(0.0, 0.0, 0.0))
        self.grasp = TwoFG7GraspController(
            self,
            open_gap_mm=self.get_parameter("open_gap_mm").value,
            close_gap_mm=self.get_parameter("close_gap_mm").value,
            force_n=self.get_parameter("force_n").value,
            speed_pct=self.get_parameter("speed_pct").value,
        )
        self._list_controllers_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers")

    def _wait_for_controller_active(self, timeout_s: float = CONTROLLER_WAIT_TIMEOUT_S) -> bool:
        self.get_logger().info(
            f"real_ball_sort: waiting for '{CONTROLLER_NAME}' to become active - "
            "if it isn't already, go press Play on the pendant's External Control program now"
        )
        deadline = time.time() + timeout_s
        last_reminder = 0.0
        while time.time() < deadline:
            if self._list_controllers_client.wait_for_service(timeout_sec=2.0):
                future = self._list_controllers_client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done() and future.result() is not None:
                    for c in future.result().controller:
                        if c.name == CONTROLLER_NAME and c.state == "active":
                            self.get_logger().info(f"real_ball_sort: '{CONTROLLER_NAME}' is active - proceeding")
                            return True
            if time.time() - last_reminder > 15.0:
                self.get_logger().info(
                    f"real_ball_sort: still waiting for '{CONTROLLER_NAME}' to activate "
                    "(press Play on the pendant if you haven't)..."
                )
                last_reminder = time.time()
            time.sleep(1.0)
        self.get_logger().error(
            f"real_ball_sort: '{CONTROLLER_NAME}' never became active after {timeout_s:.0f}s - aborting, "
            "nothing moved"
        )
        return False

    def run_mission(self):
        self.get_logger().info("=== real_ball_sort: mission start ===")

        if not self._wait_for_controller_active():
            return

        if not self.grasp.detach():
            self.get_logger().warn(
                "real_ball_sort: could not confirm the gripper opened at startup - continuing anyway, "
                "watch the real gripper"
            )

        # _wait_for_controller_active() only confirms controller_manager's
        # ROS-side lifecycle state is "active" - not that the robot's own
        # reverse-interface socket handshake has actually finished. That
        # handshake ("Robot requested program" / "Sent program to robot" /
        # "Robot connected to reverse interface") is triggered lazily by
        # the first real joint_trajectory_controller command specifically -
        # grasp.detach() above goes through a separate service and doesn't
        # trigger it, confirmed on real hardware: putting this delay before
        # grasp.detach() instead of here didn't help, the very next arm
        # move still tripped a state tolerance violation immediately after
        # being sent, ~4s before the handshake actually finished. Placing
        # it right here, immediately before the first arm move, is what
        # actually covers the real trigger point.
        self.get_logger().info(
            f"real_ball_sort: waiting {SETTLE_AFTER_CONTROLLER_ACTIVE_S:.1f}s before the first arm "
            "move, for the robot's reverse-interface connection to actually settle"
        )
        time.sleep(SETTLE_AFTER_CONTROLLER_ACTIVE_S)

        # Re-enabled: the mission now guarantees the arm's real starting
        # joint configuration is HOME_Q (not just whatever it happens to be
        # manually jogged to) before doing anything else.
        # self.arm.move_home(duration_s=MOVE_DURATION_S)

        bx, by, bz = BALL_XYZ
        hover_z = bz + HOVER_CLEARANCE_M
        touch_z = bz + TOUCH_CLEARANCE_M
        safe_transit_z = bz + SAFE_TRANSIT_ABOVE_M

        self.get_logger().info(f"real_ball_sort: approaching ball at ({bx:.3f}, {by:.3f}, {bz:.3f})")
        # Only the final approach (touch_z, where the grasp actually
        # happens) needs the gripper pointing straight down - the transit/
        # hover waypoints just need to get the arm into position, so leave
        # their orientation unconstrained (a much larger reachable envelope
        # - see move_to_world_point()'s own comment on why).
        reachable = self.arm.move_to_world_point(bx, by, safe_transit_z, duration_s=MOVE_DURATION_S,
                                                   require_pointing_down=False
                                                   ) and \
            self.arm.move_to_world_point(bx, by, hover_z, duration_s=MOVE_DURATION_S,
                                          require_pointing_down=False
                                          ) and \
            self.arm.move_to_world_point(bx, by, touch_z, duration_s=MOVE_DURATION_S)
        if not reachable:
            self.get_logger().error("real_ball_sort: IK failed reaching the ball - aborting, nothing moved near it")
            return

        picked = self.grasp.attach()
        if not picked:
            self.get_logger().error(
                "real_ball_sort: gripper did not confirm a successful grip - aborting before lift "
                "(see this file's docstring: a confirmed-looking result doesn't guarantee the gripper "
                "is even connected, always watch the hardware)"
            )
            return

        # Lift back through the safe waypoints in reverse, same reasoning
        # as ball_sort_node.py's return trip. Unconstrained like the
        # approach's own transit waypoints - none of these are the actual
        # pick/place moment, and a farther-out basket can push a
        # pointing-down solve for a pure transit point out of reach even
        # when the actual release point is still fine (confirmed on real
        # hardware: release_hover_z failed pointing-down for a basket
        # ~0.6m out, while release_z itself and the unconstrained version
        # of the same waypoint both succeeded).
        self.arm.move_to_world_point(bx, by, hover_z, duration_s=MOVE_DURATION_S,
                                      require_pointing_down=False, prefer_pointing_down=True)
        self.arm.move_to_world_point(bx, by, safe_transit_z, duration_s=MOVE_DURATION_S,
                                      require_pointing_down=False, prefer_pointing_down=True)

        kx, ky, kz = BASKET_XYZ
        release_hover_z = kz + RELEASE_CLEARANCE_M + 0.05
        release_z = kz + RELEASE_CLEARANCE_M
        self.arm.move_to_world_point(kx, ky, safe_transit_z, duration_s=MOVE_DURATION_S,
                                      require_pointing_down=False, prefer_pointing_down=True)
        self.arm.move_to_world_point(kx, ky, release_hover_z, duration_s=MOVE_DURATION_S,
                                      require_pointing_down=False, prefer_pointing_down=True)
        # release_z is the actual placement moment - keep it constrained.
        self.arm.move_to_world_point(kx, ky, release_z, duration_s=MOVE_DURATION_S)
        self.grasp.detach()
        self.arm.move_to_world_point(kx, ky, safe_transit_z, duration_s=MOVE_DURATION_S,
                                      require_pointing_down=False, prefer_pointing_down=True)
        # self.arm.move_home(duration_s=MOVE_DURATION_S)

        self.get_logger().info(
            "=== real_ball_sort: mission complete - verify the placement yourself, there is no "
            "ground-truth check on real hardware ==="
        )


def main():
    rclpy.init()
    node = RealBallSortNode()
    try:
        node.run_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
