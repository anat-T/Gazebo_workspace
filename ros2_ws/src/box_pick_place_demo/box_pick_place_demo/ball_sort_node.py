"""Single-robot mission: pick the target ball and place it in the basket
(named "the basket" in logs since some scenes have one plain basket, others
a red/green pair - see the basket_x/basket_y parameters below).

Which ball is "the target" is resolved by ball_names (a parameter, see
ball_region_node.py's docstring for the full explanation): one name means
no ambiguity (scenes 1/2, color picks it out); two names (scene 3, both
balls the same color) means the target is whichever is physically on the
robot's right (smaller y) - resolved once, locked in permanently, never
re-evaluated. The other candidate (or red_ball, in scenes with a
color-coded distractor) is never touched - the robot's DetachableJoint
child_model is hardwired to the actual target's model name at spawn time
(see the scene's launch file), so attaching to anything else is
architecturally impossible, not just avoided by mission logic.

Reuses the box-pick demo's proven pieces as-is: ArmCommander (topic-commanded
IK, with its own wait-for-subscriber fix), GraspController (confirmed
attach/detach handshake via the DetachableJoint plugin's own state
feedback), and belief.py's SearchRegion (one random guess in a narrow
"help"-sized belief region around the ball's true position, from
ball_region_node.py).

What's new here, and NOT yet proven by the box demo: attaching and then
LATER detaching again in the same run (pick the ball, carry it, release it
into the basket) - the box demo only ever did one or the other. Treat that
sequence as unverified until checked against the ball's actual final
ground-truth position, not just trusting the attach()/detach() log lines -
see the SUCCESS/FAILED check at the end of run_mission().
"""
import time

import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from box_pick_place_demo.arm_commander import ArmCommander
from box_pick_place_demo.belief import SearchRegion
from box_pick_place_demo.grasp_controller import GraspController

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

REGION_TOPIC = "/robot/search_region"

# Must match the robot's spawn pose in the scene's launch file.
BASE_XYZ = (0.0, 0.0, 0.75)

# Ball is a 0.025m-radius sphere; a little slack beyond that to count a
# guess as "close enough" - same reasoning as GRASP_TOLERANCE_M in
# search_and_pick_node.py.
GRASP_TOLERANCE_M = 0.04
BALL_RADIUS_M = 0.025

# Basket rim height (top of its walls) - same basket geometry (bottom plate
# top at 0.755+0.005=0.76, wall pose z=0.785, half wall height 0.03 -> rim
# at 0.815) is reused across scenes, only its x/y position (basket_x/
# basket_y parameters below) differs. Keep in sync with the SDF if that
# geometry ever changes.
BASKET_RIM_Z = 0.815
# What counts as "the ball ended up in the basket" for the final ground-
# truth check: within the basket's inner footprint (half-extent ~0.05m =
# (0.12m outer - 2*0.01m wall)/2) and resting between its floor and rim.
BASKET_XY_TOLERANCE_M = 0.05
BASKET_Z_RANGE = (0.75, BASKET_RIM_Z + 0.02)


class BallSortNode(Node):

    def __init__(self):
        super().__init__("ball_sort_node")
        # world_name selects which scene's world pose stream to read (each
        # scene's world has its own name, e.g. "ball_sort_demo",
        # "ball_sort_demo2") - must match the <world name="..."> the
        # launching scene actually brought up. basket_x/basket_y is that
        # scene's basket position - static and known at design time, must
        # match its <pose> in that scene's SDF. Unlike the ball, there's no
        # belief region for the basket: nothing to search for, it never
        # moves.
        self.declare_parameter("world_name", "ball_sort_demo")
        self.declare_parameter("basket_x", 0.65)
        self.declare_parameter("basket_y", 0.08)
        self.declare_parameter("ball_names", ["green_ball"])
        world_name = self.get_parameter("world_name").value
        self.basket_xy = (self.get_parameter("basket_x").value, self.get_parameter("basket_y").value)
        self.ball_names = list(self.get_parameter("ball_names").value)
        self.world_pose_topic = f"/world/{world_name}/pose/info"

        self.arm = ArmCommander(self, "robot_", base_xyz=BASE_XYZ)
        self.grasp = GraspController(self, "robot")

        self._region_msg = None
        self.create_subscription(PolygonStamped, REGION_TOPIC, self._on_region, LATCHED)

        # Unlike search_and_pick_node.py's one-shot _true_xy capture, this
        # keeps updating for as long as the node spins - needed at the end
        # to see where the ball actually ended up after being carried and
        # released, not just where it started. target_name is resolved once
        # (see _on_world_poses) and then locked - a later disturbance to
        # the OTHER candidate ball can't flip which one this node tracks.
        self._candidate_xyz = {}
        self._target_name = None
        self._latest_ball_xyz = None
        self.create_subscription(TFMessage, self.world_pose_topic, self._on_world_poses, 10)

    def _on_region(self, msg: PolygonStamped):
        if self._region_msg is None:
            self._region_msg = msg

    def _on_world_poses(self, msg: TFMessage):
        for t in msg.transforms:
            name = t.child_frame_id
            if self._target_name is not None:
                if name == self._target_name:
                    tr = t.transform.translation
                    self._latest_ball_xyz = (tr.x, tr.y, tr.z)
                continue
            if name not in self.ball_names or name in self._candidate_xyz:
                continue
            tr = t.transform.translation
            self._candidate_xyz[name] = (tr.x, tr.y, tr.z)
            if len(self._candidate_xyz) == len(self.ball_names):
                # Lock in the target now, permanently - see module
                # docstring / ball_region_node.py for the left/right
                # convention when there are two candidates.
                self._target_name = min(self._candidate_xyz, key=lambda n: self._candidate_xyz[n][1])
                self._latest_ball_xyz = self._candidate_xyz[self._target_name]
                self.get_logger().info(
                    f"ball_sort: identified '{self._target_name}' as the pick target (of {self.ball_names})"
                )

    def _spin_until(self, predicate, timeout_s: float):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def _spin_for(self, duration_s: float):
        deadline = time.time() + duration_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_mission(self):
        self.get_logger().info("=== ball_sort: mission start ===")

        # DetachableJoint starts attached by default at world load - must
        # detach before any motion. Reliable here (single call) because
        # ball_sort_demo.launch.py gates this node's start on the robot's
        # own controller spawner having already exited, same as
        # search_and_pick_node.py.
        self.grasp.detach()
        self.arm.move_home()

        if not self._spin_until(lambda: self._region_msg is not None, 30.0):
            self.get_logger().error(f"ball_sort: no {REGION_TOPIC} received, aborting")
            return
        region = SearchRegion.from_polygon_stamped(self._region_msg)

        if not self._spin_until(lambda: self._latest_ball_xyz is not None, 10.0):
            self.get_logger().error(f"ball_sort: no ground-truth pose for any of {self.ball_names}, aborting")
            return

        guess_x, guess_y = region.sample_point()
        true_x, true_y, _true_z = self._latest_ball_xyz
        offset = ((guess_x - true_x) ** 2 + (guess_y - true_y) ** 2) ** 0.5
        hit = abs(guess_x - true_x) <= GRASP_TOLERANCE_M and abs(guess_y - true_y) <= GRASP_TOLERANCE_M
        self.get_logger().info(
            f"ball_sort: belief region center=({region.center_x:.3f},{region.center_y:.3f}) "
            f"margin=({region.half_extent_x:.3f},{region.half_extent_y:.3f}) -> guessing "
            f"({guess_x:.3f}, {guess_y:.3f}), ball at ({true_x:.3f},{true_y:.3f}), off by {offset:.3f}m"
        )

        # Same clearance-above-the-object reasoning as search_and_pick_node.py:
        # never command the wrist straight to the object's center (direct
        # collision interpenetration flings it away - proven the hard way on
        # the box demo), approach with real clearance, and route the one big
        # joint-space transit through a high safe point first so no other
        # arm link sweeps through the table on the way.
        hover_z = region.z + BALL_RADIUS_M + 0.05
        grasp_z = region.z + BALL_RADIUS_M + 0.07
        touch_z = region.z + BALL_RADIUS_M + 0.002
        safe_transit_z = region.z + 0.35

        reachable = self.arm.move_to_world_point(guess_x, guess_y, safe_transit_z) and \
            self.arm.move_to_world_point(guess_x, guess_y, hover_z) and \
            self.arm.move_to_world_point(guess_x, guess_y, grasp_z)
        hit = hit and reachable

        if not hit:
            # Safety net, not expected to normally do anything: this margin
            # is narrow ("help"-sized), so a miss should be rare/never, but
            # if it happens there's nothing to place - stay put rather than
            # attempting the basket approach with an empty gripper.
            self.grasp.detach()
            self.get_logger().info(
                f"=== ball_sort: FAILED - guessed ({guess_x:.3f}, {guess_y:.3f})"
                f"{' (unreachable)' if not reachable else ''}, ball was at "
                f"({true_x:.3f}, {true_y:.3f}), off by {offset:.3f}m "
                f"(tolerance {GRASP_TOLERANCE_M:.3f}m) ==="
            )
            return

        touch_z = region.z + BALL_RADIUS_M + 0.002
        self.arm.move_to_world_point(guess_x, guess_y, touch_z)
        self.grasp.attach()
        # Lift back through the safe waypoints in reverse before heading to
        # the basket - same reasoning as search_and_pick_node.py's return
        # trip: go straight up off the table first so the carried ball
        # doesn't get swept through anything on the way.
        self.arm.move_to_world_point(guess_x, guess_y, hover_z)
        self.arm.move_to_world_point(guess_x, guess_y, safe_transit_z)

        basket_x, basket_y = self.basket_xy
        release_hover_z = BASKET_RIM_Z + 0.06
        release_z = BASKET_RIM_Z + 0.02
        self.arm.move_to_world_point(basket_x, basket_y, safe_transit_z)
        self.arm.move_to_world_point(basket_x, basket_y, release_hover_z)
        self.arm.move_to_world_point(basket_x, basket_y, release_z)
        self.grasp.detach()
        self.arm.move_to_world_point(basket_x, basket_y, safe_transit_z)
        self.arm.move_home()

        # Let the released ball fall the last couple cm and settle, then
        # check its ACTUAL final position against the basket's footprint -
        # never just trust that the attach()/detach() sequence did what was
        # intended, the same principle used throughout this project.
        self._spin_for(2.0)
        final_x, final_y, final_z = self._latest_ball_xyz
        placed = (
            abs(final_x - basket_x) <= BASKET_XY_TOLERANCE_M and
            abs(final_y - basket_y) <= BASKET_XY_TOLERANCE_M and
            BASKET_Z_RANGE[0] <= final_z <= BASKET_Z_RANGE[1]
        )
        self.get_logger().info(
            f"=== ball_sort: {'SUCCESS' if placed else 'FAILED'} - ball ended at "
            f"({final_x:.3f}, {final_y:.3f}, {final_z:.3f}), basket is at "
            f"({basket_x:.3f}, {basket_y:.3f}) - this is the demo's punchline ==="
        )


def main():
    rclpy.init()
    node = BallSortNode()
    try:
        node.run_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
