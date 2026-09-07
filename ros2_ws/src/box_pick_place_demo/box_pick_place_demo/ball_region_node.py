"""Ground-truth oracle for the ball-sort demo: reads the pick target's true
Gazebo pose and publishes a belief region (true position +/- a narrow
margin) for the robot, exactly like belief_region_node.py's "help" case for
robot_b in the box-pick demo - same mechanism, reused as-is via belief.py's
SearchRegion/region_from_true_pose.

ball_names (a parameter, not a constant) names the candidate ball model(s):
- One name (scenes 1/2): that ball is the pick target, no ambiguity.
- Two names (scene 3, both balls green - color can't disambiguate them):
  the pick target is whichever one is physically on the ROBOT'S RIGHT -
  smaller y, using this project's convention that with the robot facing
  +x, +y is its left and -y is its right (right-handed frame, z up). This
  is resolved once, the first time both candidates' poses have been seen,
  and locked in from then on - never re-evaluated later, so a later
  disturbance to either ball can't flip which one is being tracked.

The other candidate (or red_ball, in scenes with a color-coded distractor)
gets no belief region at all: it's a distractor, not a second thing to
search for. The basket's position is static and known at design time (see
ball_sort_node.py's basket_x/basket_y parameters), so it doesn't need a
belief region either - only the moving pick target does.
"""
import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from box_pick_place_demo.belief import region_from_true_pose

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

REGION_TOPIC = "/robot/search_region"


class BallRegionNode(Node):

    def __init__(self):
        super().__init__("ball_region_node")
        # Narrow margin, matching robot_b's "help" case in the box demo -
        # this pick is meant to reliably succeed, the interesting part of
        # this demo is the place (color- or position-based), not a
        # search/guess contrast. world_name selects which scene's world
        # pose stream to read - must match the <world name="..."> the
        # launching scene actually brought up.
        self.declare_parameter("margin", 0.03)
        self.declare_parameter("world_name", "ball_sort_demo")
        self.declare_parameter("ball_names", ["green_ball"])
        world_name = self.get_parameter("world_name").value
        self.ball_names = list(self.get_parameter("ball_names").value)
        self.world_pose_topic = f"/world/{world_name}/pose/info"

        self._candidate_xyz = {}
        self._target_name = None
        self._published = False
        self._pub = self.create_publisher(PolygonStamped, REGION_TOPIC, LATCHED)
        self.create_subscription(TFMessage, self.world_pose_topic, self._on_world_poses, 10)
        self.get_logger().info(
            f"waiting for {self.world_pose_topic} to publish a search region for one of {self.ball_names}"
        )

    def _on_world_poses(self, msg: TFMessage):
        if self._published:
            return
        for transform_stamped in msg.transforms:
            name = transform_stamped.child_frame_id
            if name not in self.ball_names or name in self._candidate_xyz:
                continue
            t = transform_stamped.transform.translation
            self._candidate_xyz[name] = (t.x, t.y, t.z)
            if len(self._candidate_xyz) < len(self.ball_names):
                continue
            # All candidates seen - lock in the target now, permanently.
            # With one name this is trivial; with two (scene 3) this is
            # the "physically on the right" resolution - see module
            # docstring for the left/right convention.
            self._target_name = min(self._candidate_xyz, key=lambda n: self._candidate_xyz[n][1])
            tx, ty, tz = self._candidate_xyz[self._target_name]
            margin = self.get_parameter("margin").value
            region = region_from_true_pose(tx, ty, tz, margin)
            self._pub.publish(region.to_polygon_stamped(frame_id="world"))
            self._published = True
            self.get_logger().info(
                f"identified '{self._target_name}' as the pick target (of {self.ball_names}) - "
                f"published belief region center=({tx:.3f}, {ty:.3f}) margin={margin:.3f}m"
            )
            return


def main():
    rclpy.init()
    node = BallRegionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
