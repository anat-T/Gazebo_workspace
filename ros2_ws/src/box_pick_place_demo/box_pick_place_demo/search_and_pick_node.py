"""Per-robot mission: wait for a belief region, guess ONE point in it, try
to pick there, report success or failure. Launched twice by demo.launch.py
(robot_a / robot_b) with different `use_help` values - the mission logic is
identical either way, only the belief region's margin (set by
belief_region_node) differs, which changes how likely a single random guess
is to actually land on the box.

No camera/vision in the loop: whether a guess "worked" is decided by
comparing it to the box's true position (read from the same ground-truth
world-pose stream belief_region_node uses) within GRASP_TOLERANCE_M - this
stands in for gripper/contact feedback a real robot would get from actually
touching the box (only close enough attempts would physically grab it),
without needing a working depth camera or object detector. The guess point
itself is NOT informed by the true position - only picking whether it counts
as a hit afterward is.

The DetachableJoint plugin (grasp_controller.py) is used for a real pick on
a hit: attach() is called once the arm is hovering at grasp_z (clearance
above the box, not touching it), then the mission stops there - no lift, no
further motion. attach()'s earlier "flings the box away" symptom is now
understood to have been the same two root causes as the box-during-miss
ejection bug: commanding the wrist straight to the box's center caused
direct collision interpenetration (fixed by the grasp_z clearance below),
and there was no reliable way to know the DetachableJoint plugin even
existed/was discovered yet at the time. Neither applies here: grasp_z never
touches the box, and attach() is only ever called several seconds into the
mission, long after spawn. On a miss, only detach() runs (nothing to pick).
"""
import time

import rclpy
from geometry_msgs.msg import PolygonStamped
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from box_pick_place_demo.arm_commander import ArmCommander
from box_pick_place_demo.belief import SearchRegion
from box_pick_place_demo.camera_sensor import SyncedCameraSensor
from box_pick_place_demo.grasp_controller import GraspController
from box_pick_place_demo.se_server_client import check_holding_box

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

WORLD_POSE_TOPIC = "/world/box_pick_demo/pose/info"
BOX_NAME = {"robot_a": "box_a", "robot_b": "box_b"}

# Box is 0.06m across (half-size 0.03m); a little slack beyond that to count
# a guess as "close enough" given the arm's own IK/settling imprecision.
GRASP_TOLERANCE_M = 0.04

# Robots are mounted on top of their table (0.75m, matching the table height
# in worlds/box_pick_demo.sdf), not standing on the ground beside it - so
# base z must match too. robot_name -> base xyz in world frame, must match
# demo.launch.py's ROBOTS list (spawn x/y/z).
BASE_XYZ = {"robot_a": (0.0, 0.0, 0.75), "robot_b": (2.5, 0.0, 0.75)}


class SearchAndPickNode(Node):

    def __init__(self):
        super().__init__("search_and_pick_node")
        self.declare_parameter("robot_name", "robot_a")
        self.declare_parameter("tf_prefix", "robot_a_")
        self.declare_parameter("use_help", False)
        self.declare_parameter("check_se_server", False)
        self.declare_parameter("se_server_url", "http://localhost:8000")

        self.robot_name = self.get_parameter("robot_name").value
        self.tf_prefix = self.get_parameter("tf_prefix").value
        self.use_help = self.get_parameter("use_help").value
        self.check_se_server = self.get_parameter("check_se_server").value
        self.se_server_url = self.get_parameter("se_server_url").value
        self.box_name = BOX_NAME[self.robot_name]

        base_xyz = BASE_XYZ[self.robot_name]
        self.arm = ArmCommander(self, self.tf_prefix, base_xyz=base_xyz)
        self.grasp = GraspController(self, self.robot_name)

        self._region_msg = None
        self.create_subscription(
            PolygonStamped, f"/{self.robot_name}/search_region", self._on_region, LATCHED)

        self._true_xy = None
        self.create_subscription(TFMessage, WORLD_POSE_TOPIC, self._on_world_poses, 10)

    def _on_region(self, msg: PolygonStamped):
        if self._region_msg is None:
            self._region_msg = msg

    def _on_world_poses(self, msg: TFMessage):
        if self._true_xy is not None:
            return
        for t in msg.transforms:
            if t.child_frame_id == self.box_name:
                self._true_xy = (t.transform.translation.x, t.transform.translation.y)
                return

    def _spin_until(self, predicate, timeout_s: float):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def run_mission(self):
        self.get_logger().info(f"=== {self.robot_name}: mission start (use_help={self.use_help}) ===")

        # DetachableJoint starts attached by default when the world loads,
        # so this must run before any arm motion. It's reliable here (a
        # single call, no retry-and-hope) because demo.launch.py gates this
        # whole node's start on THIS robot's own controller spawner having
        # already exited - by the time run_mission() executes at all, this
        # robot's model and its DetachableJoint plugin are guaranteed to
        # already be loaded and discovered by ros_gz_bridge. (An earlier
        # version tried to approximate that guarantee from inside this node
        # via a timing heuristic - waiting on unrelated topics before a
        # second detach() call - which looked reliable in one test but
        # wasn't actually coupled to this robot's own spawn timing, and
        # broke again under different stagger timing. Fixed for real at the
        # launch level instead.)
        self.grasp.detach()
        self.arm.move_home()

        if not self._spin_until(lambda: self._region_msg is not None, 30.0):
            self.get_logger().error(f"{self.robot_name}: no /{self.robot_name}/search_region received, aborting")
            return
        region = SearchRegion.from_polygon_stamped(self._region_msg)

        if not self._spin_until(lambda: self._true_xy is not None, 10.0):
            self.get_logger().error(f"{self.robot_name}: no ground-truth pose for {self.box_name}, aborting")
            return

        guess_x, guess_y = region.sample_point()
        self.get_logger().info(
            f"{self.robot_name}: belief region center=({region.center_x:.3f},{region.center_y:.3f}) "
            f"margin=({region.half_extent_x:.3f},{region.half_extent_y:.3f}) "
            f"-> guessing ({guess_x:.3f}, {guess_y:.3f})"
        )

        true_x, true_y = self._true_xy
        offset = ((guess_x - true_x) ** 2 + (guess_y - true_y) ** 2) ** 0.5
        hit = abs(guess_x - true_x) <= GRASP_TOLERANCE_M and abs(guess_y - true_y) <= GRASP_TOLERANCE_M

        # A guess can land outside the arm's reach, especially for the wide
        # no-help region - that's a legitimate failure outcome (this attempt
        # definitely doesn't have the box), not a reason to abort silently
        # without ever logging the demo's punchline line.
        hover_z = region.z + 0.08
        # Stop above the box's top surface (half-height 0.03m + clearance),
        # NOT at region.z (the box's center height): the wrist_3_link
        # origin has its own real collision geometry, and commanding it
        # straight to the box's center guarantees direct interpenetration
        # with the box's own collision box - the physics engine then
        # violently resolves that overlap, flinging the box away. This is
        # almost certainly what was actually behind the "box ends up a
        # meter away" symptom blamed on DetachableJoint earlier - that
        # plugin was very possibly never the real problem. 0.05m clearance
        # was enough for a direct hit (box stayed exactly at rest,
        # confirmed) but a near-miss still disturbed the box - other arm
        # links (forearm/elbow), not just the wrist, must sweep close to
        # table height near the target during the approach; 0.10m gives
        # more margin for those too.
        grasp_z = region.z + 0.10
        # JointTrajectory interpolates JOINT ANGLES linearly between the
        # arm's current configuration and the target - it has no idea what
        # Cartesian path that traces, so a single big move straight from
        # HOME_Q to (guess_x, guess_y, hover_z) can sweep some other link
        # (forearm/elbow, not just the wrist) straight through the box on
        # the way, even though start and end configurations are both fine.
        # Confirmed by watching the box start drifting away mid-transition,
        # well before hover_z was ever reached and well before attach() was
        # ever called - this has nothing to do with DetachableJoint at all.
        # Routing through a high safe point directly above the guess first
        # means the one big joint-space move (home -> above the guess) ends
        # somewhere well clear of the table, and the two remaining moves
        # (down to hover_z, then to grasp_z) are small, mostly-vertical
        # deltas with much less room to sweep sideways into the box.
        safe_transit_z = region.z + 0.35
        reachable = self.arm.move_to_world_point(guess_x, guess_y, safe_transit_z) and \
            self.arm.move_to_world_point(guess_x, guess_y, hover_z) and \
            self.arm.move_to_world_point(guess_x, guess_y, grasp_z)
        hit = hit and reachable

        if hit:
            # grasp_z's 0.10m clearance (chosen to survive a NEAR-miss
            # approach too, where lateral offset plus tight vertical
            # clearance risks a side-swipe) leaves the wrist visibly
            # floating 7cm above the box - looks like it's not touching
            # anything. Safe to close that gap further here specifically:
            # this is a confirmed hit, so guess_x/guess_y are already
            # within GRASP_TOLERANCE_M of the box's true center, and this
            # is one small, controlled vertical step down from an
            # already-stable hovering pose - not the kind of big
            # open-loop joint-space sweep that caused trouble elsewhere.
            touch_z = region.z + 0.032
            self.arm.move_to_world_point(guess_x, guess_y, touch_z)
            self.grasp.attach()
            # Lift the (now genuinely attached) box back home, retracing
            # the descent waypoints in reverse before the final big
            # joint-space move - same reasoning as safe_transit_z on the
            # way down: go straight up off the table first so the return
            # trip to HOME_Q doesn't sweep the box through the table on
            # the way, then do the one big joint-space move from a
            # position that's already well clear of everything.
            self.arm.move_to_world_point(guess_x, guess_y, hover_z)
            self.arm.move_to_world_point(guess_x, guess_y, safe_transit_z)
            self.arm.move_home()
        else:
            # Safety net, not expected to normally do anything: this
            # mission never calls attach() on a miss, but if the box
            # somehow ended up attached anyway (e.g. a leftover state from
            # a previous run, or the plugin's attached-at-load default if
            # the startup detach() above hadn't actually confirmed yet for
            # some reason), make sure a miss doesn't end with the box
            # wrongly stuck to the wrist.
            self.grasp.detach()

        # On a miss, stay right here rather than retreating and returning
        # home - there's nothing to bring back, and an earlier version of
        # this code found a miss got its (unattached) box disturbed by a
        # retreat+move_home motion through some mechanism that was never
        # fully pinned down (most likely the same blind joint-space-sweep
        # hazard fixed above via safe_transit_z, but that fix hadn't been
        # written yet at the time). On a hit, the lift+move_home above
        # already returned home, now carrying the box for real via
        # DetachableJoint rather than relying on staying put.
        self.get_logger().info(
            f"{self.robot_name}: {'picked the box and returned home' if hit else 'reached the wrong spot, staying put'}")

        self.get_logger().info(
            f"=== {self.robot_name}: {'SUCCESS' if hit else 'FAILED'} - guessed "
            f"({guess_x:.3f}, {guess_y:.3f}){' (unreachable)' if not reachable else ''}, "
            f"box was at ({true_x:.3f}, {true_y:.3f}), off by {offset:.3f}m "
            f"(tolerance {GRASP_TOLERANCE_M:.3f}m) - this is the demo's punchline ==="
        )

        if hit and self.check_se_server:
            camera = SyncedCameraSensor(
                color_topic=f"/{self.robot_name}/camera/image",
                depth_topic=f"/{self.robot_name}/camera/depth_image",
                node_name=f"{self.robot_name}_camera_sensor",
            )
            try:
                frame = camera()
                holding = check_holding_box(PILImage.fromarray(frame["rgb"]), self.se_server_url)
                self.get_logger().info(f"{self.robot_name}: SE_server holding(box) = {holding}")
            finally:
                camera.shutdown()


def main():
    rclpy.init()
    node = SearchAndPickNode()
    try:
        node.run_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
