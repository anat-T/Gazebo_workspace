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

# BALL_XYZ = (-0.651, -0.304, 0.915)
BALL_XYZ = (0.0, 0.37, 0.915) 
BASKET_XYZ = (0.0, 0.37, 0.915) 

HOVER_CLEARANCE_M = 0.08
TOUCH_CLEARANCE_M = 0.02
# 0.35 (the sim scenes' value, objects much closer to the base) put this
# point outside the UR5e's ~0.85m reach for this table position - confirmed
# via IK failure at (x,y,0.635). This object is already ~0.77m out, leaving
# little headroom to also go higher; keep this modest for real hardware.
SAFE_TRANSIT_ABOVE_M = 0.10
RELEASE_CLEARANCE_M = 0.08

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
        self.arm.move_home(duration_s=MOVE_DURATION_S)

        bx, by, bz = BALL_XYZ
        hover_z = bz + HOVER_CLEARANCE_M
        touch_z = bz + TOUCH_CLEARANCE_M
        safe_transit_z = bz + SAFE_TRANSIT_ABOVE_M

        self.get_logger().info(f"real_ball_sort: approaching ball at ({bx:.3f}, {by:.3f}, {bz:.3f})")
        reachable = self.arm.move_to_world_point(bx, by, safe_transit_z, duration_s=MOVE_DURATION_S) and \
            self.arm.move_to_world_point(bx, by, hover_z, duration_s=MOVE_DURATION_S) and \
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
        # as ball_sort_node.py's return trip.
        self.arm.move_to_world_point(bx, by, hover_z, duration_s=MOVE_DURATION_S)
        self.arm.move_to_world_point(bx, by, safe_transit_z, duration_s=MOVE_DURATION_S)

        kx, ky, kz = BASKET_XYZ
        release_hover_z = kz + RELEASE_CLEARANCE_M + 0.05
        release_z = kz + RELEASE_CLEARANCE_M
        self.arm.move_to_world_point(kx, ky, safe_transit_z, duration_s=MOVE_DURATION_S)
        self.arm.move_to_world_point(kx, ky, release_hover_z, duration_s=MOVE_DURATION_S)
        self.arm.move_to_world_point(kx, ky, release_z, duration_s=MOVE_DURATION_S)
        self.grasp.detach()
        self.arm.move_to_world_point(kx, ky, safe_transit_z, duration_s=MOVE_DURATION_S)
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
