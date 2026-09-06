"""Ground-truth oracle: reads each box's true Gazebo pose and publishes a
belief region (true position +/- margin) for both robots.

This is simulation-only, standing in for "a human pointed roughly at the
box" or "a stronger perception system already narrowed it down" - see the
plan's Context section. Both robots go through the exact same mechanism;
"help" is nothing more than a smaller margin parameter (help_margin <
no_help_margin).
"""
import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from box_pick_place_demo.belief import region_from_true_pose

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

# World pose stream, not /model/<name>/pose: per-model PosePublisher plugins
# were tried first but never actually published for these at-rest dynamic
# bodies (confirmed by echoing the gz-native topic directly) - see
# demo.launch.py's bridge_topics comment. SceneBroadcaster's world pose
# stream, bridged as tf2_msgs/TFMessage, works reliably.
WORLD_POSE_TOPIC = "/world/box_pick_demo/pose/info"

# (box model name / TFMessage child_frame_id, search_region topic, robot's margin parameter name)
BOXES = [
    ("box_a", "/robot_a/search_region", "no_help_margin"),
    ("box_b", "/robot_b/search_region", "help_margin"),
]


class BeliefRegionNode(Node):

    def __init__(self):
        super().__init__("belief_region_node")
        self.declare_parameter("no_help_margin", 0.40)
        self.declare_parameter("help_margin", 0.08)

        self._published = set()
        self._pubs = {}
        self._margin_params = {box_name: margin_param for box_name, _topic, margin_param in BOXES}

        for box_name, topic, _margin_param in BOXES:
            self._pubs[box_name] = self.create_publisher(PolygonStamped, topic, LATCHED)

        self.create_subscription(TFMessage, WORLD_POSE_TOPIC, self._on_world_poses, 10)
        self.get_logger().info(f"waiting for {WORLD_POSE_TOPIC} to publish search regions")

    def _on_world_poses(self, msg: TFMessage):
        for transform_stamped in msg.transforms:
            box_name = transform_stamped.child_frame_id
            if box_name not in self._pubs or box_name in self._published:
                continue
            t = transform_stamped.transform.translation
            margin = self.get_parameter(self._margin_params[box_name]).value
            region = region_from_true_pose(t.x, t.y, t.z, margin)
            self._pubs[box_name].publish(region.to_polygon_stamped(frame_id="world"))
            self._published.add(box_name)
            self.get_logger().info(
                f"{box_name}: published belief region center=({t.x:.3f}, {t.y:.3f}) margin={margin:.3f}m"
            )


def main():
    rclpy.init()
    node = BeliefRegionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
